# Installer test: Spade-Editor-0.1.0-f5a30e6-setup

**Result: PASS** (24 of 24 checks passed)

| Field | Value |
|---|---|
| Tested (UTC) | 2026-11-19 17:44:57Z |
| Setup | `Spade-Editor-0.1.0-f5a30e6-setup.exe` |
| Setup path | `%USERPROFILE%\Desktop\projects\spade\build-package\Spade-Editor-0.1.0-f5a30e6-setup.exe` |
| Setup size | 9.3 MB (9740825 bytes) |
| Setup sha256 | `183c6d09036cde5c9a43de6e5e511ea5efd229192429c74d8d84d95384c838a7` |
| Setup built from commit | `f5a30e6` (the tag in the setup's name; `-dirty` means uncommitted changes) |
| Test script from commit | `f5a30e61 (master; run from a git archive copy)` |
| Version the installed editor printed | exit 0: Spade Editor 0.1.0+f5a30e6 (expected `0.1.0+f5a30e6`) |
| Job ticket | spade-testdocs-installer-f5a30e6 |
| Total time | 1 min 27 s (87 s) |

## Machine

- Windows: Microsoft Windows 10 Home 10.0.19045 (build 19045)
- CPU: AMD Ryzen 7 3700X 8-Core Processor
- Memory: 31.9 GB
- GPU: NVIDIA GeForce RTX 3060 Ti, 572.83 (nvidia-smi: name, driver)
- GPU: NVIDIA GeForce RTX 3060 Ti, Windows driver 32.0.15.7283

## Checks

| # | Result | Check | Detail |
|---|---|---|---|
| 1 | PASS | install exits 0 | (exit 0; log %LOCALAPPDATA%\Temp\spade-install-test-31da7456\install.log) |
| 2 | PASS | uninstaller registered under HKCU only (per-user, no administrator) | (Microsoft.PowerShell.Core\Registry::HKEY_CURRENT_USER\Software\Microsoft\Windows\CurrentVersion\Uninstall\Spade Editor_is1) |
| 3 | PASS | installed files present | (missing: ) |
| 4 | PASS | the guide PDF is installed |  |
| 5 | PASS | the MSVC runtime is app-local |  |
| 6 | PASS | nothing the editor does not need (no tests, flow_movie, SDK files) | () |
| 7 | PASS | Start menu: Spade Editor, Spade Drone Box, Spade User Guide | (found: Spade User Guide, Spade Editor, Spade Drone Box in %APPDATA%\Microsoft\Windows\Start Menu\Programs\Spade Editor) |
| 8 | PASS | Spade Drone Box runs with --window | (arguments: '--window') |
| 9 | PASS | Spade Editor starts the editor with no arguments, from the install folder | (target %LOCALAPPDATA%\Temp\spade-install-test-31da7456\Spade Editor\SpadeEditor.exe ) |
| 10 | PASS | Spade User Guide opens the installed index.html | (target %LOCALAPPDATA%\Temp\spade-install-test-31da7456\Spade Editor\docs\user-guide\index.html) |
| 11 | PASS | desktop: Spade Editor |  |
| 12 | PASS | --version prints 0.1.0+f5a30e6 | (exit 0: 'Spade Editor 0.1.0+f5a30e6') |
| 13 | PASS | spade_geo_fetch --help exits 0 and prints its usage | (exit 0: 'spade_geo_fetch: fetch a map area into the local cache (USGS 3DEP elevation, USGS NAIP image, NOAA NGS geoid height) usage: spade_geo_fetch --lat <deg> --lon <deg> [options]   --lat <deg>, --lon <deg>   the anchor, WGS84 degrees (north and east positive; Bonneville is 40.76 -113.90)   --side <m>                 side of the square area, centred on the anchor: 10 to 1000 (default 1000)   --cache <dir>              the cache directory (default: $SPADE_MAP_CACHE, else the user's cache folder);                              refused inside a git work tree   --cache-limit-mb <n>       size limit for the whole cache (default 512)   --dem-cell <m>             elevation grid cell (default 1; the grid is at most 1024 samples a side)   --image-cell <m>           image pixel (default 0.6; at most 4096 pixels a side)   --check                    look in the cache only, never the network: hit (0) or miss (3)   --refresh                  fetch again even when the area is cached, and replace it   --timeout-s <n>            seconds to wait for each answer after connecting (default 30)   --json                     progress and the result as one JSON object a line   --help                     this text A hit or a fetch prints a summary (geoid, grid, image, box, bytes received) before the result line. An answer the tool refuses is kept as received in <cache>/failed/<service>.bin (ngs, 3dep, naip), outside the cache's entries; nothing reads it back. The last line of standard output names the result. Exit codes: 0 hit or fetched, 2 usage, 3 miss (--check), 10 offline, 11 a service answered and the answer is no use, 12 the request was refused, 13 the cache is full.') |
| 14 | PASS | spade_geo_fetch --check on an empty cache is a miss (exit 3), without the network | (exit 3: 'miss n40760000_w113900000_s1000') |
| 15 | PASS | spade_part_import --help exits 0 and prints its usage | (exit 0: 'usage: spade_part_import --probe        spade_part_import --inspect <file> [--unit U] [--axes SPEC] [--scale S] [--origin X,Y,Z] [--object NAME]... [--deep --deflection LEN]        spade_part_import --import <file> --store <dir> [--unit U] [--axes SPEC] [--scale S] [--origin X,Y,Z] [--object NAME]...                          [--deflection LEN] [--angle RAD]   --probe            is the STEP converter usable (python, OpenCASCADE, versions), and how to install it when not   --inspect <file>   what the file holds, one gate result per object; writes nothing   --import <file>    the file and its sidecars into the part store (a cache hit when already there)   --store <dir>      the part store (default: $SPADE_PART_STORE); refused inside a git work tree   --unit U           m, mm, cm, um, in or ft: required for STL and OBJ on import; refused for the formats that declare theirs   --axes SPEC        three comma-separated source axes, each with an optional minus, for example -y,x,z: a rotation, not a mirror   --scale S          multiplies every position once, after the unit and the axes   --origin X,Y,Z     a point of the file, in its own unit, that becomes the part's origin (subtracted before the axes and the scale)   --object NAME      one object of a file that holds several; repeat it to pick several (one entry, in any order)   STEP only: --deflection LEN (a length with a unit, no default), --angle RAD (default 0.5), --deep (inspect: tessellate              and run the gate), --step-python PATH, --step-convert PATH Standard output is JSON lines: stage events, then one result line last (only --help prints plain text there, and exit 0). A usage error prints this text on standard error with its result line, exit 2. Exit codes: 0 done, 1 io or internal, 2 usage, 3 unavailable, 4 refused (not a good solid), 5 cancelled, 6 written but flagged.') |
| 16 | PASS | spade_part_import --probe answers one JSON result line (exit 0 with a converter, exit 3 without one) | (exit 3: '{"event":"result","mode":"probe","ok":false,"exit":3,"code":"step_unavailable","message":"no python with step_convert.py was found","advice":"STEP needs the OpenCASCADE converter, which is not installed here. spade_part_import --probe says how to install it.","step":{"available":false,"python":null,"searched":["%LOCALAPPDATA%\\Temp\\spade-install-test-31da7456\\Spade Editor\\.venv-step/Scripts/python.exe","%LOCALAPPDATA%\\Temp\\spade-install-test-31da7456\\Spade Editor\\.venv-step/bin/python","%LOCALAPPDATA%\\Temp\\spade-install-test-31da7456\\Spade Editor\\step_convert.py"],"reason":"no python with step_convert.py was found","install":"python -m venv .venv-step && .venv-step/Scripts/python -m pip install --require-hashes -r tools/step_convert/requirements.txt (bin/python on Linux or macOS); or point SPADE_STEP_PYTHON at a python that has it, and SPADE_STEP_CONVERT at step_convert.py"}}') |
| 17 | PASS | live smoke from the install folder: exit 0, END verdict=PASS | (exit 0; journal ends 'END verdict=PASS'; %LOCALAPPDATA%\Temp\spade-install-test-31da7456\smoke\journal.txt) |
| 18 | PASS | uninstall: the install folder is gone | (left: ) |
| 19 | PASS | uninstall: the Add/Remove Programs entry is gone |  |
| 20 | PASS | uninstall: the Start-menu folder is gone |  |
| 21 | PASS | uninstall: the desktop shortcut is gone |  |
| 22 | PASS | nothing left under %LOCALAPPDATA%\Spade except logs | (left: ) |
| 23 | PASS | nothing left under %APPDATA%\Spade |  |
| 24 | PASS | the test removed its own temp folder (install log, smoke run) | (left: %LOCALAPPDATA%\Temp\spade-install-test-31da7456) |

## Live smoke

- Exit code: 0
- Journal ends: `END verdict=PASS`
- Journal result line: `RESULT anomalies=0 known_open_fired=0 unmarked=0 expired=0`
- Journal checks counted: 156
- Journal note: NOTE label install-test 0.1.0+f5a30e6
- Journal note: NOTE chapter at 3.0 s of video, 0.0 s of wall time: 1  The engine
- Journal note: NOTE chapter at 16.5 s of video, 5.1 s of wall time: 2  Airflow
- Journal note: NOTE chapter at 27.7 s of video, 33.7 s of wall time: 3  The editor
- Journal note: NOTE chapter at 93.1 s of video, 62.5 s of wall time: 4  Bodies
- Journal note: NOTE chapter at 107.1 s of video, 65.8 s of wall time: 5  New in this batch · install-test · 0.1.0+f5a30e6
- Kept in `%LOCALAPPDATA%\Temp\claude\af55eacec662f039\ef52e890-4c8e-4ad0-ab44-686d0f1336ba\scratchpad\pkg-recG\Spade-Editor-0.1.0-f5a30e6-setup.smoke`: 01_drone_box.png, 02_airflow.png, 03_editor.png, 04_bodies.png, 05_batch.png, journal.txt, tour.mp4
- A still: `%LOCALAPPDATA%\Temp\claude\af55eacec662f039\ef52e890-4c8e-4ad0-ab44-686d0f1336ba\scratchpad\pkg-recG\Spade-Editor-0.1.0-f5a30e6-setup.smoke\01_drone_box.png`

## Log

The full log of this run: `%LOCALAPPDATA%\Temp\claude\af55eacec662f039\ef52e890-4c8e-4ad0-ab44-686d0f1336ba\scratchpad\pkg-recG\Spade-Editor-0.1.0-f5a30e6-setup.test.log`. The script exited 0.

