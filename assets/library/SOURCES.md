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
- **Used for:** the FPV and basic quads' motors and prop sizes. Retailer. No pack listing is cited yet: a pack's numbers stay `assumed` until one is added here.

### `kat_competition_700`

Kat, `content/airframes/competition-700.airframe.json`: the competition drone, with each value's provenance.

- **License:** `facts_cite`. Cameron, 2026-10-21 (ruling 5): "The Grand Prix published nothing, these were derived sim values and require no concern. Have them in the library."
- **Used for:** `basic_quad_competition`. Fitted or assumed: the values are fitted to the AI Grand Prix VQ1 simulator, not hardware.

### `kat_datapack`

Kat, `design-specs/integration/drone-builder-datapack.md` section 7.4: the candidate pair for the competition drone, 4S and 2450 KV (line 325), which is not withdrawn.

- **License:** `facts_cite`.
- **Used for:** the competition quad's choice of parts. Nothing in it that took its shape from APC is used.
- **Withdrawn:** section 7.2's fit was withdrawn on 2026-10-21, because its inputs included APC-derived rows (the datapack's own note, line 307). A re-run on measured data only is owed, and is not scheduled (Kat, 2026-10-22). Until it lands, nothing fitted cites it: the competition parts that name this source (`comp_2207_2450kv`, `comp_4s`) are `assumed`, and each says so in its note.

### `holybro_x500_v2_kit`

Holybro, PX4 Development Kit X500 V2: the kit page (frame mass) and the docs page with its "3D CAD File" x500v2-frame.step (the kit frame, its motor axes and its plates), <https://docs.holybro.com/drone-development-kit/px4-development-kit-x500v2>, retrieved 2026-11-18.

- **License:** `facts_cite`. The vendor pages carry no terms line. The numbers (mass, dimensions) are facts, used and cited; the CAD file itself stays in the local part store and is not in this repository (L88b). Recorded, not enforced (L88c): docs/design/interface/records/2026-11-18-x500-parts-sources.md section 5.
- **Used for:** the X500 build's frame: its mass (0.610 kg, the bare kit), the poses read from the kit file (motor axes at +-0.1774 m, the plates, the GPS stand, the battery tray) and the kit motor and rotor geometry. Published; the file positions are measured.

### `holybro_pixhawk_6c`

Holybro, Pixhawk 6C technical specification ("Weight (Plastic Case): 34.6g", "Dimensions: 84.8 * 44 * 12.4 mm") and its case CAD, <https://docs.holybro.com/autopilot/pixhawk-6c/technical-specification>, retrieved 2026-11-18.

- **License:** `facts_cite`. The pages and the file carry no terms line; the store terms are the only reuse lines. The numbers are facts, used and cited; the CAD file stays in the local part store (L88b). docs/design/interface/records/2026-11-18-x500-parts-sources.md section 1.
- **Used for:** the 6C controller part's mass (34.6 g, plastic case) and its box. Published.

### `holybro_gps_m9n_standard`

Holybro, M8N/M9N/M10 standard GPS overview ("Dimension (V1)": phi50 x14.4 mm; "Weight (V1)": 32g) and its CAD, <https://docs.holybro.com/gps-and-rtk-system/m8n-m9n-m10-gps/standard-m10-m9n-m8n-gps/overview>, retrieved 2026-11-18.

- **License:** `facts_cite`. The pages and the file carry no terms line. The numbers are facts, used and cited; the CAD file stays in the local part store (L88b). docs/design/interface/records/2026-11-18-x500-parts-sources.md section 2.
- **Used for:** the standard GPS part's mass (32 g) and its shell. Published.

### `raspberry_pi_5_files`

Raspberry Pi, Raspberry Pi 5 3D STEP "No Graphics small file" (design files, MIT licence in the zip), <https://pip.raspberrypi.com/categories/892-raspberry-pi-5>, retrieved 2026-11-18.

- **License:** `facts_cite`. The zip's LICENSE.txt: "This 3D model is provided under the MIT license included at the end of this file." The model is geometry only; it stays in the local part store (L88b) and the repository carries its hash. docs/design/interface/records/2026-11-18-x500-parts-sources.md section 4.
- **Used for:** the Raspberry Pi 5 part's geometry. Measured from the file.

### `retail_raspberry_pi_5`

Retailer listing of the Raspberry Pi 5 8GB board (desertcart.sg), 45 g; Raspberry Pi publishes no weight, <https://www.desertcart.sg/products/593685802-raspberry-pi-5-8gb-ram-board>, retrieved 2026-11-18.

- **License:** `facts_cite`. A retailer listing with no reuse licence; the number is a fact, used and cited. The page was supplied by the lead and not read by Interface (docs/design/interface/records/2026-11-18-x500-parts-sources.md section 4); other sources give 62 g packaged.
- **Used for:** the Raspberry Pi 5 part's mass (0.045 kg). Retailer.

### `raspberry_pi_camera_module_3`

Raspberry Pi, Camera Module 3: product page ("Dimensions: 25 x 24 x 11.5mm") and the design files (camera-module-3-step), <https://www.raspberrypi.com/products/camera-module-3/>, retrieved 2026-11-18.

- **License:** `facts_cite`. raspberrypi.com/licensing: "Most of the design files released and hosted by us are made available openly, with no limitations." The model stays in the local part store (L88b). docs/design/interface/records/2026-11-18-x500-parts-sources.md section 3.
- **Used for:** the Camera Module 3 part's geometry. Measured from the file; its mass is assumed (the vendor publishes none).

### `grepow_tattu_4s_5200`

Grepow, Tattu G-Tech 4S 5200mAh 14.8V 35C LiPo drone battery: "Weight: 436.5g", "Dimensions: 133x45x33.5mm", <https://www.grepow.com/uav-battery/tattu-g-tech-4s-5200mah-14-8v-35c-lipo-drone-battery.html>, retrieved 2026-11-18.

- **License:** `facts_cite`. A maker's product page with no terms line read; the numbers are facts, used and cited. docs/design/interface/records/2026-11-18-x500-parts-sources.md section 6.
- **Used for:** the 4S pack part's mass (0.4365 kg) and box (133 x 33.5 x 45 mm). Retailer.

### `holybro_sik_radio_v3`

Holybro, SiK Telemetry Radio V3 433/915MHz: "Dimension 28 x 53 x 10.7mm (without antenna)", "Weight 23.5g w/ antenna" (the 100 mW and 500 mW columns), <https://docs.holybro.com/radio/sik-telemetry-radio-v3>, retrieved 2026-11-18.

- **License:** `facts_cite`. The datasheet page carries no terms line; the numbers are facts, used and cited. docs/design/interface/records/2026-11-18-x500-parts-sources.md section 7.
- **Used for:** the telemetry radio part's mass (23.5 g) and box (53 x 10.7 x 28 mm). Published.
