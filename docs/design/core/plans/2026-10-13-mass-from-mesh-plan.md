# Mass from a mesh part's geometry (plan stub)

**Owner:** Core, joint with Physics, which owns the airframe compile and reviews it. **Status:** a STUB for the lead and Physics, slotted right after the motor offset (the lead, 2026-10-13).
**Why:** under import-only (`CORE-6`, amended 2026-10-12), CAD meshes are how parts arrive. An imported part's mass, centre of mass and inertia must come from its geometry, from the same record as its solid (`TD-9`: one source). Today the compile takes mesh-part mass only from hand-entered records, so mass and geometry can disagree. Found in Task A5 (`ecaa5c4`); B4 had deferred it to A4, and A4 covered only the analytic shapes.
**Builds on:** B3's `mesh_mass_properties()` (volume, centroid and inertia about the centroid, per unit density, in double, bit-identical across MSVC and gcc), B4's mesh parts, and A4's `PartGeometrySpec`.

## The lead's rules (binding)

- A record whose geometry source is a mesh takes its mass, centre of mass and inertia from `mesh_mass_properties(mesh) × density`. It is one record (`TD-9`).
- **No default density.** A mesh-sourced record without a finite density > 0 is refused by name.
- **A hand-entered mass beside a mesh source is a `TD-9` conflict.** It is refused by name, never silently overridden. So is a hand-entered shape or tensor.

## Shape of the change

- `PartGeometrySpec` (`vehicles/model_part.hpp`) gains `GeometrySource::mesh`, plus:
  - `std::shared_ptr<const TriangleMesh> mesh`;
  - `double density` (kg/m³; no default: 0 means "not given" and is refused);
  - `float min_feature` (m; declared, since the compile can't measure a mesh's thinnest feature cheaply; refused if not finite and > 0).
- The compile RESOLVES a mesh-sourced record, before `composite_inertia()` runs, into an explicit record:
  - mass = ρ·V;
  - centre of mass = the record's pose applied to the mesh centroid;
  - `PartShape::tensor` = ρ·I_centroid in the record's local frame.
  So `composite_inertia()` and its tests are unchanged. The same record emits the part (B4's mesh geometry at the record's mount) with the declared `min_feature`.
- Units: mesh positions are metres (B1), density is kg/m³, so ρ·V is kg and ρ·J (m⁵) is kg m². Double throughout, converted once where the model stores float, as today.

## Refusals (by record index, name and field, `L6`)

- A mesh source without a mesh.
- A density that isn't finite and > 0.
- A `min_feature` that isn't finite and > 0.
- A hand-entered `mass`, `shape` or `tensor` beside a mesh source (`TD-9`).
- `make_mesh`'s own refusals pass through unchanged. Nothing is repaired.

## Tasks (outline; RED first, and the RED compiles alone, as in the parts and meshes plan)

- **M1:** `GeometrySource::mesh`, its fields and the refusals.
- **M2:** the compile resolves mesh records. Tests:
  - a mesh box of density ρ equals the analytic box record of the same mass and sides, within B3's stated bound (it's bit-exact when the sides are dyadic);
  - the COM moves with the record's pose;
  - a cavity subtracts;
  - two mesh parts add;
  - the reference quad and every pinned identity and golden are unchanged (it has no mesh records).
- **M3:** prose (the CHANGELOG and consumers line, under "the airframe compile").
- Gates: both presets, `gcc-check.sh --compile`, and a Docker leg at each task's head, with the box rules (the conditional ≥ 6 GB gate, quiet windows, Set-Location to the worktree in each command).

## Open questions (for Physics and the lead)

1. Density per part only for now, with a material table later (the world's `materials` palette has no density)? Recommend per part now.
2. Should an imported multi-object glTF file (B6: one object per mesh node) map one object to one record? Recommend yes; Kat's builder assigns density per object.
3. A mesh part that is an ASSEMBLY of several materials: refused until a per-shell density exists? Recommend one density per record; split the CAD into records.
