# Installer test: Spade-Editor-0.1.0-6f356be-setup

**Result: FAIL** (18 of 21 checks passed; failed: installed files present; spade_geo_fetch --help exits 0 and prints its usage; spade_geo_fetch --check on an empty cache is a miss (exit 3), without the network)

| Field | Value |
|---|---|
| Tested (UTC) | 2026-11-17 21:02:38Z |
| Setup | `Spade-Editor-0.1.0-6f356be-setup.exe` |
| Setup path | `%USERPROFILE%\Desktop\Projects\spade-wt\core-2\build-package\Spade-Editor-0.1.0-6f356be-setup.exe` |
| Setup size | 8.9 MB (9294862 bytes) |
| Setup sha256 | `4d858df55feab80d4c4b2689c16a65b73a652ec7dd0889f006d01f0fb6a7701b` |
| Setup built from commit | `6f356be` (the tag in the setup's name; `-dirty` means uncommitted changes) |
| Test script from commit | `master eb870a9a plus branch test-docs/installer-record (run from a copy)` |
| Version the installed editor printed | exit 0: Spade Editor 0.1.0+6f356be (expected `0.1.0+6f356be`) |
| Job ticket | spade-testdocs-installer-6f356be-r4 |
| Total time | 3 min 46 s (226 s) |

## Machine

- Windows: Microsoft Windows 10 Home 10.0.19045 (build 19045)
- CPU: AMD Ryzen 7 3700X 8-Core Processor
- Memory: 31.9 GB
- GPU: NVIDIA GeForce RTX 3060 Ti, 572.83 (nvidia-smi: name, driver)
- GPU: NVIDIA GeForce RTX 3060 Ti, Windows driver 32.0.15.7283

## Checks

| # | Result | Check | Detail |
|---|---|---|---|
| 1 | PASS | install exits 0 | (exit 0; log %LOCALAPPDATA%\Temp\spade-install-test-64d54282\install.log) |
| 2 | PASS | uninstaller registered under HKCU only (per-user, no administrator) | (Microsoft.PowerShell.Core\Registry::HKEY_CURRENT_USER\Software\Microsoft\Windows\CurrentVersion\Uninstall\Spade Editor_is1) |
| 3 | FAIL | installed files present | (missing: spade_geo_fetch.exe) |
| 4 | PASS | the guide PDF is installed |  |
| 5 | PASS | the MSVC runtime is app-local |  |
| 6 | PASS | nothing the editor does not need (no tests, flow_movie, SDK files) | () |
| 7 | PASS | Start menu: Spade Editor, Spade Drone Box, Spade User Guide | (found: Spade User Guide, Spade Editor, Spade Drone Box in %APPDATA%\Microsoft\Windows\Start Menu\Programs\Spade Editor) |
| 8 | PASS | Spade Drone Box runs with --window | (arguments: '--window') |
| 9 | PASS | Spade Editor starts the editor with no arguments, from the install folder | (target %LOCALAPPDATA%\Temp\spade-install-test-64d54282\Spade Editor\SpadeEditor.exe ) |
| 10 | PASS | Spade User Guide opens the installed index.html | (target %LOCALAPPDATA%\Temp\spade-install-test-64d54282\Spade Editor\docs\user-guide\index.html) |
| 11 | PASS | desktop: Spade Editor |  |
| 12 | PASS | --version prints 0.1.0+6f356be | (exit 0: 'Spade Editor 0.1.0+6f356be') |
| 13 | FAIL | spade_geo_fetch --help exits 0 and prints its usage | (exit -1: 'not found: %LOCALAPPDATA%\Temp\spade-install-test-64d54282\Spade Editor\spade_geo_fetch.exe') |
| 14 | FAIL | spade_geo_fetch --check on an empty cache is a miss (exit 3), without the network | (exit -1: 'not found: %LOCALAPPDATA%\Temp\spade-install-test-64d54282\Spade Editor\spade_geo_fetch.exe') |
| 15 | PASS | live smoke from the install folder: exit 0, END verdict=PASS | (exit 0; journal ends 'END verdict=PASS'; %LOCALAPPDATA%\Temp\spade-install-test-64d54282\smoke\journal.txt) |
| 16 | PASS | uninstall: the install folder is gone | (left: ) |
| 17 | PASS | uninstall: the Add/Remove Programs entry is gone |  |
| 18 | PASS | uninstall: the Start-menu folder is gone |  |
| 19 | PASS | uninstall: the desktop shortcut is gone |  |
| 20 | PASS | nothing left under %LOCALAPPDATA%\Spade except logs | (left: ) |
| 21 | PASS | nothing left under %APPDATA%\Spade |  |

## Live smoke

- Exit code: 0
- Journal ends: `END verdict=PASS`
- Journal result line: `RESULT anomalies=0 known_open_fired=0 unmarked=0 expired=0`
- Journal checks counted: 156
- Journal note: NOTE label install-test 0.1.0+6f356be
- Journal note: NOTE chapter at 3.0 s of video, 0.0 s of wall time: 1  The engine
- Journal note: NOTE chapter at 16.5 s of video, 7.6 s of wall time: 2  Airflow
- Journal note: NOTE chapter at 27.7 s of video, 41.2 s of wall time: 3  The editor
- Journal note: NOTE chapter at 91.2 s of video, 138.7 s of wall time: 4  Bodies
- Journal note: NOTE chapter at 105.2 s of video, 158.1 s of wall time: 5  New in this batch · install-test · 0.1.0+6f356be
- Kept in `%USERPROFILE%\Desktop\Projects\spade-wt\core-2\build-package\Spade-Editor-0.1.0-6f356be-setup.smoke`: 01_drone_box.png, 02_airflow.png, 03_editor.png, 04_bodies.png, 05_batch.png, journal.txt, tour.mp4
- A still: `%USERPROFILE%\Desktop\Projects\spade-wt\core-2\build-package\Spade-Editor-0.1.0-6f356be-setup.smoke\01_drone_box.png`

## Log

The full log of this run: `%USERPROFILE%\Desktop\Projects\spade-wt\core-2\build-package\Spade-Editor-0.1.0-6f356be-setup.test.log`. The script exited 1.

