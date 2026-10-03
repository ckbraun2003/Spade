# Physics requirements for the module API

**Owner:** Physics, as input to Core's module-API spec (`../../core/01-modules-and-scheduler.md`; `../../backlog.md`, step 3). **Status:** requirements, 2026-10-02. Core owns every decision named here. Physics states what each case needs and, where it has one, a recommendation marked as such.

Three cases, in order of weight:
1. SPH, the first field provider that is not a built-in (`PHY-4`, `SL8`).
2. The translation lock, a constraint on both backends (`../../backlog.md`).
3. Jacobi dynamic contact as an alternative contact module (`PHY-5`).

Facts about today's code that the requirements rest on are cited to the file that decides them.

## 1. SPH as a field provider

SPH particles are bodies carrying a `Fluid` component (`ComponentTypeId::fluid`, 9, reserved). They collide with the static world through the existing sphere proxy, and they find their neighbours on the existing sorted grid (`SL8`: no second spatial structure). The provider adds per-particle fluid state and three passes.

| Need | What Physics needs | Why |
|---|---|---|
| **State** | Per-particle rows (density, pressure; kernel radius, rest density, stiffness and viscosity per world), registered so they are snapshotted, digested and mirrored. Capacity is fixed per world at `create()` | Nothing allocates in the step. A restore must resume an SPH run bit for bit |
| **A shared derived structure** | The sorted grid is read by **two** modules: SPH's neighbour search, in Fields, and dynamic contact, in Constraints. Today it is scratch, built inside `CollisionDynamic` and not registered state (`physics/schedule.cpp`; `sim/simulation.cpp` registers no grid array). The API needs a derived structure that one module builds and others read, ordered by declared reads, **and buildable at more than one point in a substep** | `CollisionStatic` moves `pos` (Baumgarte correction) before the contact grid is built. A single build at substep start would put different positions in the contact grid and move the `shower` golden. So SPH gets its own build in Fields and contact keeps its own: one structure (`SL8`), two builds |
| **Component-scoped passes** | A pass iterates the bodies that carry its component. Fluid particles must also be excludable from dynamic contact among themselves, because SPH's pressure force replaces it | Without a participation mask, 50,000 particles (`SL14c`) all enter pairwise contact |
| **Passes and phases** | (i) density and pressure, in **Fields**; (ii) pressure and viscosity forces into `force_acc`, in **Forces**; (iii) Integrate and static contact as for any body | A responder that samples the fluid (buoyancy, aero T3) reads density in Forces, so density must be complete by the end of Fields |
| **A provider with state and a spatial query** | The fluid fields (density, pressure, velocity at an arbitrary point) are kernel sums over neighbours. The field-sampling interface must admit a provider whose sample function reads registered state and the grid, on both backends | Every provider today is constant or position-independent (`Medium::sample`). This is the first that is neither, and the first whose GPU sample function needs buffers bound |
| **Op order** | Neighbour sums run in grid order (cell, then slot), with no atomics, the same discipline as the dynamic-contact gather | fp32 summation order is the parity contract (`engine D2`) |
| **GPU kernel** | Slang density and force kernels over the grid buffers, under the SPIR-V rules and `fp32_math` only | Without a kernel, SPH is CPU-only and a GPU world containing it is refused (`L6`) |
| **Grade, per backend** | CPU: reference, with a corpus golden. Vulkan: banded with a measured band, **or declared absent**. "No GPU path" must be a first-class declaration that the grade check turns into a refusal | A particle pile amplifies one ulp chaotically (`ParityChaos.ShowerPileAmplifiesOneUlpOnTheCpuAlone`). A long-horizon band may not exist, and `SL8` forbids widening one to admit it |
| **Configuration** | Kernel radius, rest density, stiffness and viscosity are per-world parameters in the world description, and so part of the configuration hash. Particle capacity is declared alongside them | Two runs differing only in viscosity must not restore into each other |
| **Publish** | Density and velocity can be requested as fields for camera channels | The pressure-map camera case (`../../01-engine-model.md`, Camera) |

**The reference** is v1's `[SYSTEM]FluidDensity.comp` and `[SYSTEM]FluidForce.comp`, read for the physics, not ported. They are GPU-only, with no determinism story.

## 2. The translation lock

**What it replaces.** The drone sim box holds its quadrotor with two CPU behaviors (`sandbox/drone_sim.hpp`):
- a kinematic behavior sets `pos = 0, vel = 0`;
- a force behavior overwrites `force_acc = −m·g`, so Integrate's `+g` cancels it. That cancellation is exact only because the mass is a power of two.

Behaviors are CPU-only, so a Vulkan step with that registry attached is refused (`CORE-1`).

**What the lock must do:**
- **Hold the body's position at an anchor, and its linear velocity at zero, bitwise, on both backends.** The anchor is the position at lock time. A tolerance would let a held drone drift over a long session.
- **Leave rotation free.** Torques, ω and the orientation integrate as for a free body. The stand exists to fly attitude.
- **Win over every other writer of `pos` and `vel` in the substep.** Contact writes `pos` directly (Baumgarte correction, `physics/contacts.cpp` step 3), and Integrate writes both. So the lock must take effect **after Integrate and before Sensors**.
- **Make the IMU read a held body correctly.** A held body's specific force is `Rᵀ(a − g)` with `a = 0`, so `−Rᵀg`: +1 g "up" in the body frame, whatever the rotors do. Integrate captures specific force from `force_acc / m` (`physics/integrator.cpp`), and a lock that bypasses `force_acc` would report free-fall-plus-thrust instead. The resting-contact IMU debt (`../07-status.md`) is the same defect.
- **Work from any mass,** not only a power of two.
- **Be per body,** set and cleared at step boundaries, and part of registered state so it survives a snapshot.

**Where it can go.** Two shapes meet these needs. Core chooses.
- **(A) Integrate honours a lock.** A body flag (`BodyState::flags` has bits free, `state/layout.hpp`) plus an anchor. For a locked body, Integrate writes `pos = anchor, vel = 0` and `specific_force = −Rᵀg`, on CPU and in `integrate.slang`. A lock module owns the rows that set the flag. Unlocked bodies take the same path as today, so no digest moves.
- **(B) A projection slot after Integrate.** The scheduler gains a slot between Integrate and Sensors for constraint projections, and the lock is a pass there. It must also fix up `specific_force`, so it writes a field Integrate owns.

Physics recommends (A). The hold and the specific force are both established where `pos`, `vel` and `specific_force` are written, which is one site per invariant (`TD-9`), and it needs no new phase. (B) generalises to joints later, but it makes the lock a second writer of three Integrate outputs.

**Grade.** CPU reference, with a golden: a held quadrotor flying a collective-and-attitude script, its IMU reading +g, its position bitwise at the anchor. Vulkan banded. Position and velocity should be bit-exact, because they are written, not integrated. Orientation is banded as `quad_hover`'s already is.

**Done when** the drone sim box runs on Vulkan with its position held and its behaviors gone (`../../backlog.md`).

## 3. Jacobi as a contact module

`resolve_dynamic_contacts_jacobi` and the `collision_fill`/`collision_gather` kernels are built and tested, but not wired (`PHY-5`). They return as an **alternative** to Gauss-Seidel dynamic contact. The likely reason to want them is the GPU, where Gauss-Seidel runs one thread per world.

The module API needs **mutually exclusive alternatives for one role**:
- the module set names exactly one dynamic-contact module, and the choice is part of the configuration hash;
- each alternative declares its own grade and carries its own golden and bands. Jacobi's numbers differ from Gauss-Seidel's by design, so no existing golden is re-pinned;
- a scenario can name the alternative it runs.

This is the aero fidelity-tier mechanism (T0–T3 behind one responder interface, `../02-responders.md`) applied to contact. One mechanism should serve both.
