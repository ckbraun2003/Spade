# The default library's sources

Every number in a library entry cites one of these sources by its id, with a grade: `measured`, `published`, `retailer`, `derived`, `fitted` or `assumed` (`engine/scene/library_sources.hpp`). The machine list is `sources.yaml` beside this file. This page is the same list for people. Each source has a heading here naming its id, and a test keeps the two in step.

The repository is public, so it stores no source's own files: no CAD, no mesh and no data file. A table in the library is our own resampling of a cited source. A source whose terms forbid redistribution feeds no number at all. It stays listed so that a citation of it is refused by name.

Licenses:
- `public_domain`: no copyright.
- `facts_cite`: the numbers are facts, used and cited.
- `not_usable`: feeds nothing.

### `nasa_ames_2016`

C. Russell, J. Jung, G. Willink and B. Glasner, "Wind Tunnel and Hover Performance Test Results for Multicopter UAS Vehicles", AHS 72nd Annual Forum, West Palm Beach, FL, May 2016. NTRS 20160007399, <https://ntrs.nasa.gov/citations/20160007399>.

- **License:** `public_domain`. "This is a work of the U.S. Government and is not subject to copyright protection" (p.1). NTRS determines it public use permitted.
- **Used for:** the Phantom 3 and Iris+ rotor diameters, rotor spacing and hover check points. Measured. Its hover and tunnel results are figures only; the library uses its stated numbers (ruling 7).

### `dji_phantom3`

DJI, Phantom 3 Advanced, Professional and Standard specifications, <https://www.dji.com/phantom-3-adv>.

- **License:** `facts_cite`. DJI's site terms were not read.
- **Used for:** the Phantom 3's mass, diagonal and battery. Published.

### `retail_phantom3_parts`

Phantom 3 spare-part listings: the 2312A 960 KV motor (RC Corner, Drone Nerds) and the 9450 propeller (GetFPV, Pictureline).

- **License:** `facts_cite`.
- **Used for:** the Phantom 3's motor KV. Retailer. The listings disagree on the prop's pitch (4.5 or 5.0 in).

### `retail_iris_plus`

3DR Iris+ listings: Adafruit product 2199 and iFixit's 3DR Iris Plus device page.

- **License:** `facts_cite`. No 3DR primary source was read.
- **Used for:** the Iris+'s mass, motor KV and battery. Retailer.

### `uiuc_propdb`

J.B. Brandt, R.W. Deters, G.K. Ananda, O.D. Dantsker, and M.S. Selig (2026-10-21), UIUC Propeller Database, Vols 1-4, University of Illinois at Urbana-Champaign, Department of Aerospace Engineering, retrieved from <https://m-selig.ae.illinois.edu/props/propDB.html>.

- **License:** `facts_cite`. The site states no terms; it asks to be cited as above. We store our own resampled tables, cited, and never the files (ruling 3).
- **Used for:** the props' C_T and C_P against J, and their blade geometry. Measured.

### `apc_data_files`

APC Propellers (Landing Products Inc.), PE0 geometry and PER performance files.

- **License:** `not_usable`. APC's terms say "reproduction, distribution, display or transmission of the content of this site is strictly prohibited". No APC data at all (ruling 4).

### `kat_catalog_apc_shaped`

Kat, `design-specs/integration/drone-builder-catalog.md` (2026-10-03): its propeller J-tables, whose shapes come from APC's PER3 files.

- **License:** `not_usable`, under ruling 4.

### `bitcraze_cf21_datasheet`

Bitcraze, "Datasheet Crazyflie 2.1", Rev 3.

- **License:** `facts_cite`.
- **Used for:** the Crazyflie's mass, size, motors and battery. Published.

### `bitcraze_pwm_thrust`

Bitcraze, "PWM to thrust", crazyflie-firmware documentation (measurements of 2015).

- **License:** `facts_cite`. No licence is stated.
- **Used for:** the Crazyflie's thrust check. Measured.

### `retail_cf_motor`

Pakronics listing, Bitcraze 7x16 mm coreless DC motor (SKU 316030060).

- **License:** `facts_cite`.
- **Used for:** the coreless motor's KV. Retailer.

### `luis_leny_2016`

C. Luis and J. Le Ny, "Design of a Trajectory Tracking Controller for a Nanoquadcopter", Polytechnique Montreal technical report, arXiv:1608.05786, 2016, Table 2.3.1.

- **License:** `facts_cite`. The authors keep copyright.
- **Used for:** the Crazyflie's arm length and inertia. Measured, at second hand.

### `retail_fpv_parts`

FPV part listings: Ready Made RC (iFlight XING2 2207 1855KV; EMAX E3 2306 1700KV), Flying Tech (EMAX ECO II 2306) and GetFPV (HQProp DP 6x3.5x3).

- **License:** `facts_cite`.
- **Used for:** the FPV and basic quads' motors, packs and prop sizes. Retailer.

### `kat_competition_700`

Kat, `content/airframes/competition-700.airframe.json`: the competition drone, with each value's provenance.

- **License:** `facts_cite`. Cameron, 2026-10-21 (ruling 5): "The Grand Prix published nothing, these were derived sim values and require no concern. Have them in the library."
- **Used for:** `basic_quad_competition`. Fitted or assumed: the values are fitted to the AI Grand Prix VQ1 simulator, not hardware.

### `kat_datapack`

Kat, `design-specs/integration/drone-builder-datapack.md` section 7.2: the best candidate parts set for the competition drone.

- **License:** `facts_cite`.
- **Used for:** the competition quad's choice of parts. Nothing in it that took its shape from APC is used.
