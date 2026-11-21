# Real 5 inch FPV kits: source candidates (Interface, for the lead). Written 2026-11-21.

Owner O136, verbatim: "Fpv only, use a real fpv geometry/model like the x500, same realistic fidelity". The film wants a real 5 inch class FPV kit, sourced and graded like the X500 ([2026-11-18-x500-parts-sources.md](2026-11-18-x500-parts-sources.md)): real part meshes, mass summed from parts, real props.

This is a reading record. **Nothing was downloaded**: no CAD, no zip, no data file, no PDF. Every line below comes from a web page or a repo's README/LICENSE/file listing, read by fetch agents on 2026-11-21 and cross-checked against each other; the file names are directory-listing names, no file was opened. Licences are recorded here, not enforced (L88c). Nothing is in the repo or the part store.

Grades follow the X500 record: a figure is as the page states it; "est." is the maker's own word. Nothing here is graded `measured` by Spade.

## Ranking

| Rank | Candidate | Frame CAD | Licence of the CAD | Whole kit |
|---|---|---|---|---|
| 1 | OpenFrame-5F (OpenDrone, Incutec BV) | STEP per part, Onshape source | CERN-OHL-S-2.0, stated in the repo | frame only; FC, ESC, RX as KiCad plus package STEPs; no motor, camera, battery CAD |
| 2 | Source Two and Source V (ps915 repos, from Team BlackSheep) | STEP (one file at the root, not checked whether the full assembly) | GPL-3.0, stated in the README | frame only |
| 3 | Holybro (X500's vendor): Kakute H7 + Tekko32 ESC files, Kopis kit | no 5 inch frame CAD found; Kopis 3D-print archives, contents unknown | none found | FC + ESC files, format unstated |

Ranked by how complete the geometry is and how cleanly it is licensed. The gap that holds for all three: **no maker publishes motor CAD for a 5 inch motor** (see "The motor gap").

## 1. OpenFrame-5F (OpenDrone, Incutec BV, Leuven)

- Repo: https://github.com/OpenDrone-hw/OpenFrame-5F ; maker's page https://opendrone.be/products/openframe . CAD source: Onshape document "OpenDrone-V2" (the link is in the README and in cad/onshape.json; not opened).
- **Geometry published.** README: "exported once to releases/<rev>/ (STEP per part, drawing PDF, manifest.json with SHA-256 per file)". So the frame is STEP per part, not one assembly. Folder https://github.com/OpenDrone-hw/OpenFrame-5F/tree/main/releases/rev2.2 holds drawings/, step/, manifest.json (rev2.2 is on main, not a release). Releases https://github.com/OpenDrone-hw/OpenFrame-5F/releases : rev2 (2026-10-20) and rev1 (2026-07-05), "Assets 3" each, asset names not shown. One STEP name seen: releases/rev2.2/step/anti-slip-pad.step; the others were not listed to us. Parts in bom.csv: carbon arms, Base-Bot, Base-Top, cross, Top; aluminium Standoff-L/R camera side plates; hardware; TPU printed mounts; anti-slip pad. The README says "the ESC is 2.4 mm above the arm screws (from the 2026-10-24 model)", so an ESC sits in their model, but no ESC CAD file was identified.
- **Electronics from the same maker**, all KiCad sources plus component-package 3D models only (QFN, SOT, LED, crystal; no board or enclosure STEP):
  - FC: https://github.com/OpenDrone-hw/OpenFC-Lite (hardware/: .kicad_pcb/.kicad_sch, lib.3dshapes with .step and .wrl, e.g. RP2354A_QFN-60.step).
  - ESC: https://github.com/OpenDrone-hw/OpenESC-30x30 (4in1.kicad_pcb, 4in1ESC-30x30.3dshapes).
  - Receiver: https://github.com/OpenDrone-hw/OpenRX-Mono (lib.3dshapes).
  - Motor "OpenMotor": no CAD. README: "Bell drawing | Not in this repository". Product page https://opendrone.be/products/openmotor?Model=2306&Cells=6S: "there are no design files to publish."
  - Camera and battery: not sold, no CAD. The build page says: "You still need a radio, a camera and video transmitter, goggles, LiPo batteries and a charger, which OpenDrone does not sell."
- **Terms.**
  - Frame README: "Hardware licensed under [CERN-OHL-S-2.0](https://ohwr.org/cern_ohl_s_v2.txt), see [LICENSE]". LICENSE (the file in the repo): "CERN Open Hardware Licence Version 2 - Strongly Reciprocal".
  - Product page https://opendrone.be/products/openframe : "License CERN-OHL-S v2 ↗ Strong reciprocal licence".
  - Site footer (https://opendrone.be): "Flight controller, ESC and receiver designs: CERN-OHL-S. Firmware: GPL/MIT."
  - FC, ESC and RX repo pages: "Hardware licensed under CERN-OHL-S-2.0, see LICENSE." The OpenFC-Lite LICENSE s.1.3 "Source" covers "design materials or digital code", so KiCad and STEP files are inside it (the ESC and RX LICENSE files were not opened).
  - OpenMotor repo: "Everything in this repository is licensed under CERN-OHL-S-2.0... The licence covers OpenDrone's tests and procedures, not the OEM motor design." The bell and stator are an OEM's; the OEM is not named.
  - Docs licence: none found (docs/ holds only .gitkeep).
  - Note: strongly reciprocal. What a film and site that use meshes derived from the files must pass on is not settled here (recorded, not enforced, L88c).
- **Naming.** Brand rules https://github.com/OpenDrone-hw/OpenDrone-Brand/blob/main/BRAND.md : "Ask before merchandise, endorsement-like use or incorporating the identity into another product name or logo." and "The hardware licence does not grant trademark rights." Compatibility statements are allowed. Brand repo: "OpenDrone and Incutec artwork is trademarked; see BRAND.md and LICENSE." Licence s.8.2: "You shall not use any of the name (including acronyms and abbreviations), image, or logo by which the Licensor or CERN is known... Any such permitted use shall be factual and shall not be made so as to suggest any kind of endorsement or implication of involvement by the Licensor or its personnel." So a factual line with no endorsement is allowed; the maker's contact is contact@opendrone.be. No trademark notice was found on "OpenFrame" or "OpenMotor" themselves.
- **Weights.** Frame, README: "Weight | 170 g measured: carbon, aluminium and hardware, without the TPU parts". Product page: "170 g, without TPU parts (est.)". The two pages disagree on measured against estimated; grade it `published`, not `measured`, until the maker answers. No per-part masses (bom.csv has no mass column). FC https://opendrone.be/products/openfc-lite : "Weight | 4 g (est.) | 7 g (est.)" (20x20 | 30x30; size 37.9x37.9 mm). ESC https://opendrone.be/products/openesc : "Weight | 7.5 g (est.) | 15 g (est.)" (size 41.6x42.5 mm, 60 A continuous per channel). RX https://opendrone.be/products/openrx?Model=Mono : 0.8 g (est.), 10.0x17.3 mm. Motor 2306 6S 1950KV: **no mass**. The 11.6 g, 15.8 A, 395 W, 12N14P and 1.5 mm shaft figures on the motor page sit in the 3 inch 1604 2850KV column; do not use 11.6 g for the 5 inch motor. The GitHub README table says "not yet measured" for 2306 6S.
- **Specs** (README): "Wheelbase | 226 mm"; "Top / middle / bottom plate | 2.5 / 3.0 / 3.0 mm carbon"; "Arms, cross | 6 mm carbon"; "Camera side plates | 7 mm CNC aluminium"; "Stack mount | 30.5 x 30.5 M3 (self-clinching nuts)"; "Motor mount | 16 x 16, M3"; "Standoffs | 20 mm, M3 female-female, OD 5". Product page adds "Stack height | 20 mm max" and "Material | T700 carbon, 3K twill".
- **Prop and motor.** The maker's 5 inch build (https://opendrone.be/products/openframe , "5″ build", dated 2026-11-20, pre-order, full build US$372.00, "Ships by 31 March 2027"): OpenFC Lite 30x30 x1, OpenESC 30x30 x1, "OpenMotor 5" 6S (2306, 1950KV)" x4, "OpenRX Mono" x1, "HQProp 5x4.3x3 V2S propeller set, 5 inch" x4. Motor page: stator 2306, KV 1950 (est.), rated cells 6S, props 5". Thrust claim: "Peak thrust is 1652 g at 31283 rpm." and "Square-law fit, 1.68 g/krpm²" (rpm "7 pole pairs assumed"); entry https://opendrone.be/testing#/analyse/openmotor-thrust-vs-rpm could not be read (script-rendered). OpenMotor/test/README.md: "No results are committed yet." The V2S prop is not named in the motor's test or spec text.
- **What it would need downloaded** (only on the lead's word): the rev2.2 STEP files (or the rev2 release assets) and manifest.json for the frame; nothing else is a board or motor model. The Onshape document is a viewer link, not a file.
- **Not read:** the cad/ and step/ file contents, the CHANGELOG, CONTRIBUTING, the rev1 and rev2 release pages, the ESC and RX LICENSE files, the testing page entry above.

## 2. Source Two and Source V (ps915 repos, from Team BlackSheep)

- https://github.com/ps915/source_two : root "Source Two v0.1.step" and "Source Two v0.1.f3d", DXF plates (S2_v0.1_main_plate_3mm.dxf, S2_v0.1_top_plate_2mm.dxf, S2_v0.1_5_inch_arms_4mm_4x.dxf, and more); folder https://github.com/ps915/source_two/tree/master/3D-Print has STEP plus STL pairs for Session, axii, cam, foxeer_lollipop and shark_fin mounts (SMA mount STL only). Not checked whether the root .step is the full assembly.
- https://github.com/ps915/source_v : source_v_V1.step, source_v_V1_upgraded_arms.step, source_v_V1.dxf.
- **Terms.** README (both): "The frame files are released under GNU Public License (GPL) V3." Sidebar "GPL-3.0 license". The LICENSE text itself was not read. It is a software licence applied to frame files; what it requires of a film that uses meshes from them is unsettled here.
- **Naming.** README: "SOURCE TWO is the name of the frame." It says manufacturers must "prefix the name with a brand name" (e.g. "TBS SOURCE TWO") and that forks must take another name. A plain factual mention is not addressed.
- **Weights.** Source Two "Weight: 72 g". Source V 'Weight 5": 63g' (wheelbase 218 mm). Nothing else; no motor, stack, camera or battery named.
- **Prop.** Not named by either repo.
- **Source One V5 (the actual TBS kit):** https://github.com/tbs-trappy/source_one holds V6 files only (SO1-V6-*.stl mounts, So1-V6-2025-AUG-19.dwg/.dxf, So1-V6-7inDC-2025-JUL-07.dwg/.dxf); no STEP, no README or LICENSE seen, Releases empty. Not usable as a source. A search listing says a Printables page for "Source One v0.2" is GPL, text cut off; a Cults3D package for V5 bundles a LICENSE.txt with a STEP_Files folder; neither was read. https://github.com/3dpowerhk/source_two holds partial DXF only and points back at ps915.
- **Needs downloaded:** the one STEP file per repo.
- **Not read:** the Source One V5 product page at GetFPV (HTTP 403), the Printables pages (HTTP 403), TBS's own product page, the LICENSE text of the ps915 repos.

## 3. Holybro (the X500's vendor)

Read 2026-11-21 except where marked. The store pages (holybro.com) returned **HTTP 429 with Retry-After: 60**, on a retry too, so no product, collection or spec page on that host was read. The retry the lead allowed is section 4.

- https://docs.holybro.com/autopilot/kakute-h7-v1-v2-mini/download lists, under "3D CAD Files": a file labelled "Kakute H7 + Tekko32 F4 4in1 50A ESC" (https://docs.holybro.com/files/CQWYXnFM7S12vuCUFlud) and a second with no link text (https://docs.holybro.com/files/cdDGbtLgz5nDw6AISwsV). https://docs.holybro.com/esc-3d-file-downloads lists "Tekko32 F4 Metal 4in1 65A" (https://docs.holybro.com/files/4Ds7oF3s2PZHQWLFA8BA; 65 A, not the 50 A). No page states a format.
- https://holybro.com/pages/downloads lists (cdn.shopify.com, bare binary links, not fetched): "KopisX8-5inch-TPU-3D-Print.rar", "KopisX8_Cinelifter_5in_Duct.zip", "Holybro_Kopis_Freestyle4_-3D_Print_STL_Files.rar", "Holybro_Kopis_Racing3inch_3D_Print.rar", plus assembly guides for "Kopis X8 Cinelifter 5″" (PDFs). Contents unknown; the names say 3D print files, not frame CAD.
- **Terms:** none found on the Kakute download page, the ESC page, the downloads page or the Kakute H7 v1 overview. The store terms https://holybro.com/policies/terms-of-service were recorded in the X500 record as silent on downloads and CAD.
- **Weights.** Kakute H7 v1 (https://docs.holybro.com/autopilot/kakute-h7-v1-v2-mini/kakute-h7-v1/overview, verbatim): "Mounting - 30.5 x 30.5mm/Φ4mm hole with Φ3mm Grommets"; "Dimension - 35x35mm"; "Weight - 8g". PX4 https://docs.px4.io/main/en/flight_controller/kakuteh7.html gives the same values (read through a model-extracted summary, not verbatim). Tekko32 F4 4in1 50A weight and size: not found on any readable page; the spec PDFs "Tekko32 F4 4in1 ESC(50A) SPEC V1.5" and "Tekko32 F4 BigFET 4in1 50A ESC SPEC" are linked from the downloads page and were not fetched.
- **Kit.** No motor or prop is named for any 5 inch kit on the pages read. The docs index https://docs.holybro.com/llms.txt has no 5 inch frame page. The store menu names a "Kopis" collection (https://holybro.com/collections/kopis-fpv-drone; 429).
- **Needs downloaded:** the FC+ESC file(s), format unverified; whichever Kopis archive holds a frame, if any.

## 4. Holybro Kopis, second read (2026-11-21, after the lead's retry)

Pages only, nothing downloaded. **The store still returned HTTP 429** (Retry-After 60): https://holybro.com/collections/kopis-fpv-drone on two attempts and https://holybro.com/collections/kopis-fpv-drone/cinelifter on one. So no Kopis product page was read: no product URL, stock motor, stock prop, stack, battery, price or weight line. The docs host answered.
- Read, HTTP 200: https://holybro.com/pages/downloads , https://docs.holybro.com/llms.txt , https://docs.holybro.com/company/user-manuals/kopis.md .
- 5 inch Kopis files on the downloads page, under "Kopis": "Kopis X8 5" TPU 3D Print" (KopisX8-5inch-TPU-3D-Print.rar) and "KopisX8_Cinelifter_5in_Duct" (KopisX8_Cinelifter_5in_Duct.zip), both on cdn.shopify.com. The page does not say what they contain; "TPU 3D Print" suggests printed parts, which is an inference. No frame plate or arm CAD is named, and no motor, FC/ESC, camera or battery file. Three 5 inch assembly guides are listed as PDFs, not read: "Kopis X8 Cinelifter 5″ – Ducted Assembly Guide", "Kopis X8 Cinelifter 5″ – Caged Assembly Guide", "Kopis_Cinematic_X8_Assembly_Guide". The Kopis docs page shows only 24 unlabelled /files/ attachments (no names), not fetched. The docs index has no 5 inch Kopis or frame page.
- Terms: none found for those files; the downloads page carries only the site-wide footer links (Shipping Policy, Terms of Service, Warranty).
- Not 5 inch, excluded: Kopis Freestyle 4″, Racing 3″, CineWhoop 2.5″, Kopis 2 HDV/6S.
- Answer to "does the frame's CAD exist": **not shown**. The only 5 inch Kopis files offered are printed-part archives of unknown contents. Whether a frame STEP exists, and the stock motor, prop and weights, need the store pages (429) or the assembly-guide PDFs (a download, on the lead's word).

## Others read or seen, not recommended

- **Alpaca** https://github.com/dipree/alpaca : frame/ holds arm.dxf, bottom_plate.dxf, center_plate.dxf, top_plate.dxf; accessories/ has 9 STL; no STEP seen. Sidebar "LGPL-3.0 license", the LICENSE text speaks of "the Library" and "Application" (software terms), the README states no licence for the hardware. README: "Carbon weight: 73.7g"; "the total frame weight is around ~93.4g" (steel screws, aluminium standoffs); "Wheelbase: 225mm (fits 5.1" propellers)"; motor mount 16x16 mm. No motor named. The release link points at a different owner (github.com/derpixeldan/alpaca), not read.
- **The Tropicalist** https://github.com/anardine/the-tropicalist : README: "This project is licensed under the Creative Commons Attribution-ShareAlike 4.0 International Public License."; LICENSE is CC BY-SA 4.0 and says "Patent and trademark rights are not licensed under this Public License." Project/ has folders Carbon Cut Files, FEA Analysis, Fusion 3D Files, STL Files; the contents were not read, so STEP is unknown; "designed for up to 5.1in props". BOM.md and ASSEMBLY.md (weights, motor, prop) not read. The Fusion files are in another repo, github.com/tropicalfpv/the-tropicalist (not read).
- **Pragma 5** https://www.printables.com/model/1009540-pragma-5 : a search snippet says two STEP versions (Analog and HD); the page returned 403 and was not read. Licence unknown.

## The motor gap

No maker of a 5 inch motor was found publishing CAD.
- iFlight: on the XING-E Pro 2207 shop page (https://shop.iflight.com/xing-e-pro-2207-2-6s-fpv-nextgen-motor-pro874) support replies to a request for a motor CAD model that it is an internal document and only the datasheet is available. That is the XING-E Pro, not the XING2, and it shows the maker's policy only. Retailer listings of the XING2 2207 give: stator 22 mm x 7 mm, 16x16 mm mounting, 31.6 g with wire, 5 mm titanium shaft, 12N14P.
- BETAFPV Snow 2306: dimensions given (29.3 x 30.6 mm, 5 mm shaft), no CAD download on the page.
- OpenDrone's OpenMotor: no design files (above).
- Third-party models (formats and terms unverified, none read): a GrabCAD "2306 motor model" (bell and stator separate), CGTrader listings for a Meps Space SZ2306 1950KV (IGES, X_T, STL; no STEP).
- So a motor would be a **stand-in built from published stator and bell dimensions**, its mass graded `retailer` or the maker's "est.", as the X500 record does for the GPS mast.

## Props

- OpenFrame's build names the HQProp 5x4.3x3 V2S. The MiniQuad Test Bench has a table for the older **HQ v1s 5x4.3x3PC** (https://www.miniquadtestbench.com/assets/components/motordata/motorinfo.php?uid=282; thrust, current, RPM at throttle steps; no torque column; the motor is not identified; one row read: 75% throttle about 893 g at 20.16 A and 24,588 RPM). I found no thrust and torque table for the V2S. An IntoFPV forum thread compares the Gemfan Vanover 5136 with the HQ 5x4.3x3 V2S (https://intofpv.com/archive/index.php/thread-29334.html; only the title and header were read). Physics is checking prop data separately.
- Source Two, Source V, Alpaca: no prop named; Alpaca and the Tropicalist say "5.1 in".

## Pages not read

- https://www.getfpv.com/tbs-source-one-v5-5-frame-kit.html : HTTP 403 (not retried).
- https://www.printables.com/model/1009540-pragma-5 : HTTP 403 (not retried).
- every holybro.com store page (Kakute H7, Tekko32, Kopis products and collection): HTTP 429 with Retry-After 60, on both reads (the second after about 25 minutes).
- Not fetched by design: every CAD file, archive and PDF named above.

## 5. OpenFrame-5F rev2.2: the one download (chosen by the owner, O138)

Cameron's pick, verbatim: "OpenFrame-5F (Recommended)". The lead allowed exactly one download: the `releases/rev2.2` folder of the frame repo, nothing else.

- **Source:** https://github.com/OpenDrone-hw/OpenFrame-5F/tree/main/releases/rev2.2 . rev2.2 is a folder on `main`, not a tag or release. Fetched at commit `d14c4b07c80020f39d993c68e25c96eda432dceb` (`main` at the time, read with `git ls-remote`, refs only). Fetch date 2026-11-21.
- **Method:** per-file raw downloads of the folder's listing, each pinned to that commit (`https://raw.githubusercontent.com/OpenDrone-hw/OpenFrame-5F/<commit>/releases/rev2.2/<path>`), a generic client (curl, no headers added). The folder listing was read from the three GitHub pages (rev2.2, step, drawings): `manifest.json`, `drawings/Drawing-1.pdf` and 16 files in `step/`. `manifest.json` was fetched first and the files downloaded are exactly the 17 it lists, which are exactly the 17 the pages listed. No git clone, no other URL, no other path of the repo (the LICENSE and BRAND.md below were read as pages, as in sections 1 to 4).
- **Where:** `<part-store>/_imports/openframe-5f-rev2.2-d14c4b07`, a new folder in the part store's own `_imports` area (first fetched into `part-store-import/` beside the store, then moved on Oversight's word after checking the store code: the only walk of the store is `remove_stale_temps` at `tools/part_import/store.cpp:299`, which looks one level into each store folder and removes only children named `.tmp-*`, so `_imports/<folder>` is not touched; the store refuses a git tree and `_imports` is not one; no test walks the real store). All 17 hashes were re-checked at the new path, 0 mismatches, 18 files. Untrusted data: nothing run from it; paths are passed to tools as arguments.
- **Check:** all 17 files match `manifest.json`'s SHA-256, 0 mismatches. 18 files on disk (17 + `manifest.json`, whose own sha256 is `0cd0bea633716cdfcec6d5a2040e68aeada34f7d97992a9e61825ca84df41647`, 6,885 bytes; the manifest is the maker's own claim, so this checks the transfer, not the maker). Bytes and sha256:

| file | bytes | sha256 |
|---|---|---|
| step/Arm.step | 168,139 | eaa66a41693627d9154bdaeb6282bcb17a4108f0f15fe9cfd97d4fe471597e69 |
| step/Cross.step | 48,708 | 709b3642fbc3c9ace2fdcb6083f23543451abcfbbbeb1cd7baa2cbc33958c4e6 |
| step/Base-Top.step | 398,354 | 28074c26e7a674f75a5c16eab4f30821e275c0c4f9cc434ced0edcbd96dc445a |
| step/Base-Bot.step | 430,839 | 47538aaa7bba939307ca73825236777609665b951f54175ebd48d3ccbad275d5 |
| step/Top.step | 396,351 | d11cf26c7efadbd8619560f5080e52968a60fd9ce4434a31b8cc8a0d6156e8c1 |
| step/Standoff-L.step | 92,441 | 1b01cf10fb440f11400bd72386fb596af89b6235284e05fa21d0124c86725ad8 |
| step/Standoff-M3-F-F-20-OD5.step | 5,900 | a75bb2b867d196ddb3a6e04b7dac1512d3a39ada7504064446ab4870f4e27618 |
| step/Standoff-R.step | 92,781 | ff99e5caa605ba44f68af114b3baa02c94a01014ccbf091a401368489392b925 |
| step/Boot-L.step | 173,505 | de7e7f5202d31a7a6cfd0ee578ce4db29f6b08a53ce587181897f1006c21eb00 |
| step/Boot-R.step | 174,280 | 1dcba107a36caf439f6ebabdcb835e3f6518e2452cd46b30503309b418d24321 |
| step/Cam-Mount-L.step | 25,254 | b14e4e857f68966baa445ecf21971cb9f2a13cddaa6bd6462f3885b12a9c9690 |
| step/Cam-Mount-R.step | 25,372 | a3050c9ce3dbfe49dd0c247517321c0623f3ac1dabcb458b565b53234ebec933 |
| step/Airtag-Antenna-mount.step | 432,376 | 10b1df8bd5be4aab37680e5e4058f91eedb52ab5e9f67816976444c255a46dc8 |
| step/VTX-Mount.step | 381,971 | 3a1d7c9f5d8a3f760f9e0e8451347d473037577a4219a01568f40e42957bbea1 |
| step/anti-slip-pad.step | 184,928 | 059c98304b0ec754a7f764e7652bbc0bc8b0f1131475b5f8d5eb20e3719c7ea4 |
| step/Bumper.step | 296,401 | 9f5c9fe1890025091d6a79b37a75be85ab9791744925929c03cd07c9ac13b547 |
| drawings/Drawing-1.pdf | 784,930 | 01a8427185a49fc566e0127a78bdd33d6ffddedff27bda65d1370f47188d29c4 |

- **What the files are** (manifest.json, read as text): Onshape document "OpenDrone-V2" (https://cad.onshape.com/documents/78e093d02798373a79bc68d0/w/c0ca752de47c1227bb727503), version "OpenFrame-5F rev2.2" created 2026-10-24T21:00Z, exported_at 2026-10-24T21:05:00Z, "onshape_release.py manifest format; exported through the browser session (Onshape API key over quota), founder-made version". Its note: "Re-released 2026-10-24 evening from version OpenFrame-5F rev2.2: middle-plate nut holes for catalogue self-clinching nuts (M3 4.22 + seat 6.7 x 1.55, M4 5.4 + seat 8.1 x 1.55) and camera-mount hole depths; Base-Top, Standoff-L, Standoff-R and the drawing PDF are exported from this version; the other STEP files are the earlier rev2.2 export, checked geometrically identical to this version (boolean difference 0)." Sixteen STEP parts, each a single part (not an assembly): Arm, Cross, Base-Top, Base-Bot, Top, Standoff-L, Standoff-R, Standoff M3 F-F 20 OD5, Boot-L, Boot-R, Cam-Mount-L, Cam-Mount-R, Airtag/Antenna mount, VTX-Mount, anti-slip pad, Bumper. The part placement in the frame is **not** in the folder: no assembly STEP, no pose file. The STEP header (Arm.step): "ISO-10303-21", "Generated by software containing ST-Developer from STEP Tools, Inc.", AP214. The 4 arms and the standoffs repeat, so a part's count and its mount positions must come from the drawing PDF (Drawing-1.pdf, not yet read) or from the frame's own geometry.
- **Licence** (the repo's LICENSE, read as a page, not downloaded): "CERN Open Hardware Licence Version 2 - Strongly Reciprocal"; README: "Hardware licensed under [CERN-OHL-S-2.0](https://ohwr.org/cern_ohl_s_v2.txt), see [LICENSE]"; product page https://opendrone.be/products/openframe: "License CERN-OHL-S v2 ↗ Strong reciprocal licence". Section 8.2: "You shall not use any of the name (including acronyms and abbreviations), image, or logo by which the Licensor or CERN is known... Any such permitted use shall be factual and shall not be made so as to suggest any kind of endorsement or implication of involvement by the Licensor or its personnel." Recorded, not enforced (L88c); the folder stays local (L88b).
- **Brand rule** (https://github.com/OpenDrone-hw/OpenDrone-Brand/blob/main/BRAND.md): "Ask before merchandise, endorsement-like use or incorporating the identity into another product name or logo." and "The hardware licence does not grant trademark rights." Compatibility statements are allowed (a factual mention). The brand repo says "OpenDrone and Incutec artwork is trademarked; see BRAND.md and LICENSE." The site and film may name the frame factually; no endorsement-like use without asking the maker (contact@opendrone.be).

## 6. Import into the part store (Interface, 2026-11-21)

`spade_part_import --import FILE --store <part-store> --deflection 0.05mm` (angle default 0.5 rad), STEP route: converter `tools/step_convert/step_convert.py`, OpenCASCADE 7.9.3, wheel cadquery-ocp 7.9.3.1.1, as the X500 record. Every file declares metres; each file is one object. Entry key `cf16a0ea97d3bea9` for every 0.05 mm import (the same options, so the same key as the Pixhawk's). **15 of 16 imported; every brep-to-mesh volume cross-check unflagged.**

- **Arm: refused at 0.05, 0.07, 0.1, 0.2 and 0.03 mm** ("object 'Arm': 1 self-intersecting triangle pair", a tessellation fault, not a CAD fault), **imported at 0.025 mm** (entry key `6cbe27555ef30ac0`, 2,228 triangles, volume 11.073 cm3, brep-to-mesh difference 0.036 %). The one part at a different tolerance; the entry records it.
- **Airtag/Antenna mount: refused** at 0.05, 0.02, 0.01 and 0.005 mm ("a face was not meshed at deflection ...; use a smaller deflection"): a face the converter will not mesh at any tolerance tried. It is an accessory mount (a 37.7 x 44.8 x 14.2 mm printed part, 4.9 cm3) the kit's masses and film do not need; left out and named. Not repaired.

| part | triangles | store_ref sha256 / file |
|---|---|---|
| Arm (0.025 mm) | 2,228 | 6244ccff87a459d69e3c5bc27e437be4cf0ab3fcac7e48d6d582ad9bcbd770e9 / Arm.spmesh |
| Cross | 888 | 93a5c757de3883dd7a7aa430db36cf4cc2c92fd635f0b0c0d5b4b87a07d933b7 / Cross.spmesh |
| Base-Bot | 4,560 | ba14a6eee949b89b0f42ede8e0b80100a14c4feeef6313b4353f7aa83cbcd58e / Base-Bot.spmesh |
| Base-Top | 6,804 | cb393472dec6f98f684d6e20b16595caa8e7005206cee8d88b96819b4a7a66b1 / Base-Top.spmesh |
| Top | 4,008 | 2b38bd8b6e789e91530fdd4216c45221b2ce594099bc7604569221da9587133b / Top.spmesh |
| Standoff-L | 992 | a80cb66f86290f3cf37942674b16ab77792786edefb65037e67918b1fb950891 / Standoff-L.spmesh |
| Standoff-R | 992 | 21a82b769918749cb5cb78d9650ba267fd7d1e1c081ad054894a66b1d2b8fd19 / Standoff-R.spmesh |
| Standoff M3 F-F 20 OD5 | 208 | 2d2737a3f0ab6d89817358c49f98c5621a69281740b3b07fb9a30d92b21d8787 / Standoff M3 F-F 20 OD5.spmesh |
| Boot-L | 1,784 | e6832261b091e2180ed0c23eee9f412ee96e80c393653b766e13c019cd4a2805 / Boot-L.spmesh |
| Boot-R | 1,784 | 91a8b4d6939c92b4d7b41412737bd68706d303a65ceb2980899f7d0084562109 / Boot-R.spmesh |
| Cam-Mount-L | 376 | 38a9fc2bb586840bbb415f39e5e0336c42ee0b309078ddc2f7a620d39b936c82 / Cam-Mount-L.spmesh |
| Cam-Mount-R | 376 | c0c3549a97cac1f96440e5005e96c847098815495a9b86947a775da8fcc88196 / Cam-Mount-R.spmesh |
| VTX-Mount | 7,412 | 594c9bf10a41273931098fec8bc50f286b9ea6022589e483e53147a0e03cd8a1 / VTX-Mount.spmesh |
| Bumper | 3,232 | dbac8d8bb97f0187fa0660cf53e5a4bc25a5d05411594364a0c475605a030524 / Bumper.spmesh |
| anti-slip pad | 1,608 | e1fa17abb3fa3989ccb000199645489904f3bd2a2b8f6e692208d9e32fe38556 / anti-slip pad.spmesh |

### What the files say about the frame (measured on the STEP files, local read; metres in the file, millimetres below: x across, y along, z up)

Bounding boxes of the files (min to max, mm): Base-Bot x -34.18..34.18, y -86.0..28.96, z -3.0..0; Arm x 0.02..104.05, y 0.02..93.03, z 0..6.0 (one arm, in the +x +y quadrant); Cross z 0..6; Base-Top z 6..9, y -33.25..74.0; Standoff M3 F-F 20 OD5 z 9..29 (one copy, at x -12.75..-7.75, y 67..72); Top z 29..31.5, y -73.5..74.0; Boot-L x 82.35..105.06, y 61.18..94.04; Boot-R the same at y -94.04..-61.18; Bumper y -89.01..-60.97, z -5.97..31.56; Cam-Mount-L/R x +-11.75..17.25, y -74.65..-52.43, z 5.76..29.0; VTX-Mount y 6.25..71.43; Standoff-L/R (camera side plates) x +-11.25..18.25, y -86..-43.82, z 0..29. This agrees with the README's plate thicknesses (bottom 3, middle 3, top 2.5, arms 6 mm) and its 20 mm stack height. **Which end is front: derived**, not stated by the maker: the bumper and the camera mounts are at -y, the VTX mount at +y, so forward is -y (the heading is not in the folder).

**Motor mount centre, measured on Arm.step** (its cylindrical faces, OpenCASCADE, read from the file): a 3.5 mm radius bore at (89.000, 69.534) mm, z 0..6, axis z, with four 1.55 mm radius holes whose centroid is the same point (89.000, 69.535); the holes lie 8.0 mm from the bore's centre (a 16 mm bolt circle). Distance from the frame centre 112.93 mm, so 225.9 mm across the diagonal against the README's "Wheelbase | 226 mm" (0.14 mm short). The other three motors follow by mirror symmetry of the frame (x -> -x, y -> -y): motor centres (+-89.000, +-69.534) mm. So 178.0 mm across and 139.07 mm along: a stretched X, not a square X. The arm's top face (the motor seat) is z 6.0 mm; the underside of Base-Bot z -3.0 mm. The four-arm symmetry is **derived**, from one exported Arm and the paired Boot-L/R, not from a file that places them.

### Frame for the table (design frame as the X500's: +X forward, +Y up, +Z right)

One proper rotation of the studio frame: (x, y, z)_studio -> (X, Y, Z) = (-y, z + 3 mm, -x) mm, determinant +1, origin on the lower face of Base-Bot at the frame's centre (Y 0 there), as the X500 table's origin. The motor centres are then X = -+69.534, Z = -+89.000 mm (front motors X +69.534), the seat Y 9.0 mm, the top face of the arms. The heading is the derived one above.
