# Installer test: Spade-Editor-0.1.0-eb870a9-setup

**Result: PASS** (21 of 21 checks passed)

| Field | Value |
|---|---|
| Tested (UTC) | 2026-11-18 02:58:13Z |
| Setup | `Spade-Editor-0.1.0-eb870a9-setup.exe` |
| Setup path | `C:\Users\Cameron\Desktop\projects\spade\build-package\Spade-Editor-0.1.0-eb870a9-setup.exe` |
| Setup size | 9.1 MB (9502331 bytes) |
| Setup sha256 | `a213b7a4a95fae7ffc9e69100edccffced63a78d438016b707c2fe0aa2f3c7f8` |
| Setup built from commit | `eb870a9` (the tag in the setup's name; `-dirty` means uncommitted changes) |
| Test script from commit | `65f3c4db (branch test-docs/installer-record; run from an identical copy)` |
| Version the installed editor printed | exit 0: Spade Editor 0.1.0+eb870a9 (expected `0.1.0+eb870a9`) |
| Job ticket | spade-testdocs-installer-eb870a9 |
| Total time | 54 s |

## Machine

- Windows: Microsoft Windows 10 Home 10.0.19045 (build 19045)
- CPU: AMD Ryzen 7 3700X 8-Core Processor
- Memory: 31.9 GB
- GPU: NVIDIA GeForce RTX 3060 Ti, 572.83 (nvidia-smi: name, driver)
- GPU: NVIDIA GeForce RTX 3060 Ti, Windows driver 32.0.15.7283

## Checks

| # | Result | Check | Detail |
|---|---|---|---|
| 1 | PASS | install exits 0 | (exit 0; log C:\Users\Cameron\AppData\Local\Temp\spade-install-test-add414eb\install.log) |
| 2 | PASS | uninstaller registered under HKCU only (per-user, no administrator) | (Microsoft.PowerShell.Core\Registry::HKEY_CURRENT_USER\Software\Microsoft\Windows\CurrentVersion\Uninstall\Spade Editor_is1) |
| 3 | PASS | installed files present | (missing: ) |
| 4 | PASS | the guide PDF is installed |  |
| 5 | PASS | the MSVC runtime is app-local |  |
| 6 | PASS | nothing the editor does not need (no tests, flow_movie, SDK files) | () |
| 7 | PASS | Start menu: Spade Editor, Spade Drone Box, Spade User Guide | (found: Spade User Guide, Spade Editor, Spade Drone Box in C:\Users\Cameron\AppData\Roaming\Microsoft\Windows\Start Menu\Programs\Spade Editor) |
| 8 | PASS | Spade Drone Box runs with --window | (arguments: '--window') |
| 9 | PASS | Spade Editor starts the editor with no arguments, from the install folder | (target C:\Users\Cameron\AppData\Local\Temp\spade-install-test-add414eb\Spade Editor\SpadeEditor.exe ) |
| 10 | PASS | Spade User Guide opens the installed index.html | (target C:\Users\Cameron\AppData\Local\Temp\spade-install-test-add414eb\Spade Editor\docs\user-guide\index.html) |
| 11 | PASS | desktop: Spade Editor |  |
| 12 | PASS | --version prints 0.1.0+eb870a9 | (exit 0: 'Spade Editor 0.1.0+eb870a9') |
| 13 | PASS | spade_geo_fetch --help exits 0 and prints its usage | (exit 0: 'spade_geo_fetch: fetch a map area into the local cache (USGS 3DEP elevation, USGS NAIP image, NOAA NGS geoid height) usage: spade_geo_fetch --lat <deg> --lon <deg> [options]   --lat <deg>, --lon <deg>   the anchor, WGS84 degrees (north and east positive; Bonneville is 40.76 -113.90)   --side <m>                 side of the square area, centred on the anchor: 10 to 1000 (default 1000)   --cache <dir>              the cache directory (default: $SPADE_MAP_CACHE, else the user's cache folder);                              refused inside a git work tree   --cache-limit-mb <n>       size limit for the whole cache (default 512)   --dem-cell <m>             elevation grid cell (default 1; the grid is at most 1024 samples a side)   --image-cell <m>           image pixel (default 0.6; at most 4096 pixels a side)   --check                    look in the cache only, never the network: hit (0) or miss (3)   --refresh                  fetch again even when the area is cached, and replace it   --timeout-s <n>            seconds to wait for each answer after connecting (default 30)   --json                     progress and the result as one JSON object a line   --help                     this text A hit or a fetch prints a summary (geoid, grid, image, box, bytes received) before the result line. An answer the tool refuses is kept as received in <cache>/failed/<service>.bin (ngs, 3dep, naip), outside the cache's entries; nothing reads it back. The last line of standard output names the result. Exit codes: 0 hit or fetched, 2 usage, 3 miss (--check), 10 offline, 11 a service answered and the answer is no use, 12 the request was refused, 13 the cache is full.') |
| 14 | PASS | spade_geo_fetch --check on an empty cache is a miss (exit 3), without the network | (exit 3: 'miss n40760000_w113900000_s1000') |
| 15 | PASS | live smoke from the install folder: exit 0, END verdict=PASS | (exit 0; journal ends 'END verdict=PASS'; C:\Users\Cameron\AppData\Local\Temp\spade-install-test-add414eb\smoke\journal.txt) |
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
- Journal note: NOTE label install-test 0.1.0+eb870a9
- Journal note: NOTE chapter at 3.0 s of video, 0.0 s of wall time: 1  The engine
- Journal note: NOTE chapter at 16.5 s of video, 2.2 s of wall time: 2  Airflow
- Journal note: NOTE chapter at 27.7 s of video, 15.7 s of wall time: 3  The editor
- Journal note: NOTE chapter at 97.1 s of video, 37.6 s of wall time: 4  Bodies
- Journal note: NOTE chapter at 111.1 s of video, 39.4 s of wall time: 5  New in this batch · install-test · 0.1.0+eb870a9
- Kept in `C:\Users\Cameron\Desktop\projects\spade\build-package\Spade-Editor-0.1.0-eb870a9-setup.smoke`: 01_drone_box.png, 02_airflow.png, 03_editor.png, 04_bodies.png, 05_batch.png, journal.txt, tour.mp4
- A still: `C:\Users\Cameron\Desktop\projects\spade\build-package\Spade-Editor-0.1.0-eb870a9-setup.smoke\01_drone_box.png`

## Log

The full log of this run: `C:\Users\Cameron\Desktop\projects\spade\build-package\Spade-Editor-0.1.0-eb870a9-setup.test.log`. The script exited 0.

