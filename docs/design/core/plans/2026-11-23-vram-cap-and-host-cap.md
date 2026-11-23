# The VRAM cap and the host's grid cap (O193), a design note

**Owner:** Core. **Decision:** Cameron via Oversight, "Lift the limit, tight margin". The 16 GiB host number was nodded by Oversight as a POLICY number.

## What changed

- **The device.** The fixed 4 GiB ceiling (`kAirflowVramCap`) is gone. What a run may hold is the heap's budget less the usage and the reserve (`compute::vram_cap`), checked before any allocation (`check_vram`). The per-buffer device checks (maxStorageBufferRange, maxMemoryAllocationSize, descriptor counts) and the 2^31 cell check are unchanged: they protect the shader's 32-bit index and the device's own limits, not a policy.
- **The reserve is a named option**, `vram_reserve_bytes` (signed, so a negative can be refused by name): default 1 GiB (`kRunnerReserveBytes`), a run may LOWER it (the O158 film run asks 0.3 GiB), and anything under `kMinVramReserve` (0.3 GiB) or negative is refused by the option's name.
- **The host.** `kGridByteCap` is now 16 GiB, a stated POLICY number and not a measurement. This PC has 31.9 GB of RAM, so it is about half the box; with the job queue's 4 GB floor and other jobs running that much is not always free. The number is the same on every machine and every run, never the machine's free memory, so a case file validates the same everywhere and a test that asserts a refusal never flakes. It protects one thing: a case that cannot be run is refused before work starts, by name. It is changed only by the lead or Cameron. A run may only lower it (`host_grid_cap`, `check_grid`'s `user_cap`).
- **A passing case whose memory is not there** fails at allocation by name, the grid and the bytes (`assign_zeroed`, used by `make_grid`), never a crash and never a throw out of a function that promises a Result (L6). The test forces it with an injectable allocator, never with a machine that runs out.

## Where, and the proof

`engine/compute/device_limits.hpp`, `engine/physics/airflow/mac_grid.{hpp,cpp}`, `fluid_step.{hpp,cpp}` (`check_grid`'s lower-only parameter), `spike/airflow/chain_runner.{hpp,cpp}` (the runner's option). The tests that asserted 4 GiB changed on purpose, each for the property it pinned, reasons in the RED commit. The RED: a grid over 4 GiB that fits the budget passes; one over the budget less the reserve is refused by name with the cap's make-up; the 2^31 and per-buffer refusals are unchanged.
