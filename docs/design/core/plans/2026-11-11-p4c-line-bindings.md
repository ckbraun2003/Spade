# P4c: the actuator line's set-1 bindings (Core's share), as built

Physics' ask (the actuator line on the GPU, L61/L65) names four buffers; Physics writes the kernels (`airflow_line_{sample,load,spread}`) and the
host packing (`spike/airflow/step_chain.*`). Core owns the bindings and the runner's list. Design rule from the ask: the binding carries DATA, so a
change to the CPU model's smearing or sampling is a host change, not a new binding.

- **Bindings (set 1)**: `airflow_line_elems` 36 and `airflow_line_polar` 37 (read-only on the device, host-written), `airflow_line_vel` 38 and
  `airflow_line_force` 39 (read-write). Declared in `bindings.slang` with their roles; named in `set_one_bindings.hpp` (declared count 34 to 38);
  accepted by `is_declared_set1`. They use the last four spare numbers below the LBM range (40 to 47); 29 to 32 stay reserved for the Physics-named
  parts, posed, vehicle and rotor_model rows.
- **Runner list**: `kAirflowSetBuffers` is unchanged, 0 to 27, contiguous. `kAirflowLineBuffers` in `chain_runner.hpp` is a second contiguous list
  from `kAirflowLineFirstBinding` (36). The runner binds `buffers[i]` at `binding_base + i`, so a runner that uses the line lists the airflow set, a
  4-byte placeholder for each of 28 to 35 it does not use, then the four (no sparse bindings). Kinds: elems and polar resident, vel device-only,
  force staged (the host reads the report back in the chain's own `ChainReadback`, one wait).
- **Capacities**: constants in `chain_runner.hpp`: `kAirflowLineMaxElements` 6144 (D1 at R/dx 38: 4 rotors x 3 blades x 64 x 8 sub-azimuths),
  `kAirflowLinePolarCapacity` 1024, `kAirflowLineForceFloats` 16. The per-run capacities are `GpuFluid`'s create parameters (Physics' file); 0 elements
  is no line. `GpuRecipe` is an enum, so it carries none. Lines run through `GpuFluid`, not `Simulation`; a host that needs them in `Simulation` is its
  own ask.
- **Where the kernels are checked**: the airflow kernels are compiled by `spike/airflow/CMakeLists.txt`, scanned by
  `AirflowRunnerSpirv.KernelsHoldTheDeterminismRules` and pinned by `SpirvPin` rows, not in `kSpirvModules`. The line kernels' rows land after
  Physics hands over the `.slang` files.
- **Later (R1d)**: this takes set 1's last spares below the LBM range, so a second airflow buffer set (the nest) will need a different answer
  (a second descriptor set, or numbers past 47). Nothing to build now.
