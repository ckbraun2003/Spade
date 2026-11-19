# Installer ledger

One row per installer tested by `scripts/package-test.ps1` (run with `-LedgerDir`). The row says what was tested and how it came out; the record it links to
holds every check, the machine, the smoke journal's summary and the job ticket. The record's `.test.md` is copied into `installer-records/` when the row is added.
The stills, the tour video and the full log stay beside the setup.exe in the build folder (not in git).

| Tested (UTC) | Setup | sha256 (12) | Size | Result | Live smoke | Windows; GPU, driver | Job ticket | Record |
|---|---|---|---|---|---|---|---|---|
| 2026-11-17 21:02Z | `Spade-Editor-0.1.0-6f356be-setup` | `4d858df55fea` | 8.9 MB | **FAIL** 18/21 | END verdict=PASS | Microsoft Windows 10 Home 10.0.19045 (build 19045); NVIDIA GeForce RTX 3060 Ti, 572.83 (nvidia-smi: name, driver) | spade-testdocs-installer-6f356be-r4 | [record](installer-records/Spade-Editor-0.1.0-6f356be-setup.test.md) |
| 2026-11-18 02:58Z | `Spade-Editor-0.1.0-eb870a9-setup` | `a213b7a4a95f` | 9.1 MB | **PASS** 21/21 | END verdict=PASS | Microsoft Windows 10 Home 10.0.19045 (build 19045); NVIDIA GeForce RTX 3060 Ti, 572.83 (nvidia-smi: name, driver) | spade-testdocs-installer-eb870a9 | [record](installer-records/Spade-Editor-0.1.0-eb870a9-setup.test.md) |
| 2026-11-18 16:36Z | `Spade-Editor-0.1.0-54b497e-setup` | `445e929eb068` | 9.2 MB | **PASS** 23/23 | END verdict=PASS | Microsoft Windows 10 Home 10.0.19045 (build 19045); NVIDIA GeForce RTX 3060 Ti, 572.83 (nvidia-smi: name, driver) | spade-testdocs-installer-54b497e | [record](installer-records/Spade-Editor-0.1.0-54b497e-setup.test.md) |
| 2026-11-19 00:58Z | `Spade-Editor-0.1.0-e59ab13-setup` | `b877183b8b70` | 9.3 MB | **PASS** 23/23 | END verdict=PASS | Microsoft Windows 10 Home 10.0.19045 (build 19045); NVIDIA GeForce RTX 3060 Ti, 572.83 (nvidia-smi: name, driver) | spade-testdocs-installer-e59ab13 | [record](installer-records/Spade-Editor-0.1.0-e59ab13-setup.test.md) |
| 2026-11-19 14:47Z | `Spade-Editor-0.1.0-fb4d009-setup` | `30542ba9e478` | 9.2 MB | **PASS** 24/24 | END verdict=PASS | Microsoft Windows 10 Home 10.0.19045 (build 19045); NVIDIA GeForce RTX 3060 Ti, 572.83 (nvidia-smi: name, driver) | spade-testdocs-installer-fb4d009 | [record](installer-records/Spade-Editor-0.1.0-fb4d009-setup.test.md) |
| 2026-11-19 17:44Z | `Spade-Editor-0.1.0-f5a30e6-setup` | `183c6d09036c` | 9.3 MB | **PASS** 24/24 | END verdict=PASS | Microsoft Windows 10 Home 10.0.19045 (build 19045); NVIDIA GeForce RTX 3060 Ti, 572.83 (nvidia-smi: name, driver) | spade-testdocs-installer-f5a30e6 | [record](installer-records/Spade-Editor-0.1.0-f5a30e6-setup.test.md) |
| 2026-11-19 21:18Z | `Spade-Editor-0.1.0-d19e4dc-setup` | `99a50bffe0f3` | 9.3 MB | **PASS** 24/24 | END verdict=PASS | Microsoft Windows 10 Home 10.0.19045 (build 19045); NVIDIA GeForce RTX 3060 Ti, 572.83 (nvidia-smi: name, driver) | spade-testdocs-installer-d19e4dc | [record](installer-records/Spade-Editor-0.1.0-d19e4dc-setup.test.md) |
