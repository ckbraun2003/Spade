# X500 build, real parts: source records (Interface, L88l). Written 2026-11-18.

The CAD stays local (L88b): it lives in the part store, never in git. This record holds only links, hashes, numbers and short quoted
terms. Licences are recorded here, not enforced (L88c). `<part-store>` is the shared part store folder; `<scratchpad>` a throwaway folder.

Every file was fetched once into a fresh `<scratchpad>` folder and read with python -I; nothing is in the repo. All four are imported
into the shared part store `<part-store>` (rev 3 layout `<store>/<sha256 of source>/<key>/`) with
`spade_part_import --import FILE --store ... --deflection 0.05mm` (angle default 0.5 rad), STEP route: converter
tools/step_convert/step_convert.py, step_convert version "step-1", OpenCASCADE 7.9.3, wheel cadquery-ocp 7.9.3.1.1 (hash-pinned,
tools/step_convert/requirements.txt). Every file declares millimetres. Every picked object passed the gate; every cross-check clean.
Entries match the temp-store run exactly (same key, same content hashes, same store_refs).
Sections 1 to 4 are the first four parts; sections 5 to 7 (the kit's own frame, the 4S pack, the radio) were added the same day.
The per-object `store_ref {sha256, file}` and content hashes are in the last section.

## 1. Pixhawk 6C (Holybro)
- Vendor page: https://docs.holybro.com/autopilot/pixhawk-6c/download , label "Pixhawk 6C Case CAD 3D File".
- File link: not recorded. The vendor page links a GitBook file host address that carries a per-link access token, and no token is kept in git; the vendor page and link text above find the file, and the file name and sha256 below identify it.
- File: Pixhawk6C-3D-CAD.stp, 2,170,692 bytes, STEP (header: FILE_NAME 'PIXHAWK6C-XUANRAN_ASM', 2023-02-03, CREO PARAMETRIC BY PTC). sha256 05b838910fd49ec61347a6a18061da2b30fd116fa68c9acae84644b8851f1ef0
- Terms: none found. No licence, copyright or usage line on the download page, the 6C overview/specification/dimensions pages, or in the file. The store terms https://holybro.com/policies/terms-of-service never mention downloads or CAD; the only reuse lines there: "You agree not to reproduce, duplicate, copy, sell, resell or exploit any portion of the Service," (section 2) and "to infringe upon or violate our intellectual property rights or the intellectual property rights of others" (section 12(d)). Mesh conversion and use is UNADDRESSED. No login was asked for the file.
- Datasheet (https://docs.holybro.com/autopilot/pixhawk-6c/technical-specification, "Mechanical data"): "Dimensions: 84.8 * 44 * 12.4 mm"; "Weight (Aluminum Case): 59.3g"; "Weight (Plastic Case): 34.6g". PX4's page https://docs.px4.io/main/en/flight_controller/pixhawk6c.html says "Dimensions: 84.8 * 44 * 12.4 mm" and "Weight: 59.3g" (no case named). The page gives weight, not mass.
- Picked: all 4 solids: MIANKE-PIXHAWK6C-LV-C1 (13,242 tri), DIKE-PIXHAWK6C-LV-C1 (6,152), PCB-PIXHAWK6C-F1 (912), IMU-PIXHAWK6C (392) = 20,698 triangles. The two case halves + PCB + IMU; the file is the assembled controller, not only the case.
- Result: 84.0 x 12.4 x 44.0 mm (the stored axes are file X, then 12.4 high as file Y, then 44 deep as file Z). MISMATCH: length 84.0 against the datasheet 84.8 mm, 0.8 mm short; width 44.0 and height 12.4 match. Volume 19.51 cm3.
- Missing objects: none.

## 2. GPS puck, standard V1 (Holybro M8N/M9N/M10 standard GPS)
- Vendor page: https://docs.holybro.com/gps-and-rtk-system/m8n-m9n-m10-gps/standard-m10-m9n-m8n-gps/download , heading "M8N M9N M10 GPS CAD File", link text GPS-M8N-M9N-M10-full_asm.stp (the page's other files: ...-pcba.stp, ...-Mount.stp, and "M9N & M10 V2" m9n-2-1016.stp, not fetched).
- File link: not recorded. The vendor page links a GitBook file host address that carries a per-link access token, and no token is kept in git; the vendor page and link text above find the file, and the file name and sha256 below identify it.
- File: GPS-M8N-M9N-full_asm.stp (the page labels it GPS-M8N-M9N-M10-full_asm.stp; saved locally under that name), 1,210,026 bytes, STEP (header 'GPS-V5-C1-FULL_ASM_ASM', 2022-06-06, CREO). sha256 42587789183f82d0ab59da80f0149d355ff4df99695bb2ddc071354225197fce
- Terms: none found (same pages and same store terms as section 1; the Holybro "Downloads" page https://holybro.com/pages/downloads was not read; the product pages https://holybro.com/products/m9n-gps returned HTTP 429 and were not read). UNADDRESSED.
- Datasheet (https://docs.holybro.com/gps-and-rtk-system/m8n-m9n-m10-gps/standard-m10-m9n-m8n-gps/overview.md): "Dimension (V1)": "φ50 x14.4 mm"; "Weight (V1)": "32g"; "Dimension (V2)": "φ51 x18.5 mm"; "Weight (V2)": "36.8g"; cable "26cm (42cm cable purchase separately)"; "JST-GH-10P". NOTE: the Holybro X500 V2 kit page (https://docs.holybro.com/drone-development-kit/px4-development-kit-x500v2.md) lists the "M8N GPS Module", not an M9N.
- Mast: NOT on any Holybro page. A retailer listing (found by web search, not re-fetched) says "Fixed Carbon Fiber GPS Mount Pedestal Base (with 70mm & 140mm Carbon Rods)" for the 10-pin set. Grade: retailer. Core writes the mast entry.
- Picked: GPS-V5-SK-C1 (9,346 tri, top shell) and GPS-V5-XK-C1 (4,550, bottom shell) = 13,896 triangles. Left out (all pass the gate, not shown to be visible): GPS-V5-ZDZ-C0 370, GPS-V5-DYDZ-C1 548, GPS-V5-YDZ-C0 496, TAO-XIAN-RTK-F3_3_1 732 (small parts inside the shell), PCB-005-F3 1,518 (44 x 9.4 x 63 mm, longer than the puck: it sticks out in this model, which looks exploded).
- Result: 49.96 x 14.30 x 50.0 mm against the datasheet phi50 x 14.4 mm: diameter matches, height 0.1 mm short. (The whole 7-object assembly is 54.1 x 14.3 x 68.1 mm because of the PCB and small parts; the shell alone is the puck.) Volume 5.74 cm3. Mass 32 g from the datasheet.
- Missing objects: none of the shell; the six objects above are deliberately left out.

## 3. Camera Module 3 (Raspberry Pi), standard variant
- Vendor page: https://pip.raspberrypi.com/categories/1207-design-files (reached from https://pip.raspberrypi.com/categories/786-raspberry-pi-camera-module-3), link text "camera-module-3-step", "9.5 MB ZIP archive", "Updated 6 Oct 2025".
- File link: https://pip-assets.raspberrypi.com/categories/1207-design-files/documents/RP-008154-DS-1-camera-module-3-step.zip (9,971,996 bytes, no sign-in, no redirect). sha256 of the zip b2e2026ea5322bddf1ddcbae97c094822ee9e0ca92d7bc5c748f60a8b5ef6683. Contents: Camera_module_3_std_model_simple.stp (30,848,133 bytes, sha256 of the file ecbc4b091e51f31c6186b81ad68d4205b8dceec0d90c116cd7fd699e1f1c5702; STEP AP214, 'Spatial InterOp 3D', 2024-01-23) and Camera_module_3_wide_model_simple.stp (29,900,214 bytes, not imported); no licence file in the zip.
- Terms (https://www.raspberrypi.com/licensing/): "Most of the design files released and hosted by us are made available openly, with no limitations." and "Permission to use, copy, modify, and/or distribute these designs for any purpose with or without fee is hereby granted." Exceptions on the same page: "some design files that we have shared on our website are from third-party manufacturers" and "Terms and conditions for the use of these remain with the original manufacturer." (it names Diodes, MTCONN, Toby, TRXCOM; the camera file is not named as third-party). Documentation: "All of our online documentation is released under a Creative Commons Attribution-ShareAlike 4.0 International (CC BY-SA) licence." Product PDFs: "is copyright (c) Raspberry Pi Ltd, and is licensed under a Creative Commons Attribution-NoDerivatives 4.0 International (CC BY-ND) licence." The site terms (https://www.raspberrypi.com/terms-and-conditions/, last updated 2 June 2026) 5.1: "All such rights are reserved."; 5.3: "You must not modify the paper or digital copies of any materials you have printed off or downloaded in any way"; 5.5: "You must not use any part of the content on our site for commercial purposes without obtaining a licence"; 5.2 and 5.3 carve out material "specifically identified as being made available under a Creative Commons licence". THE TENSION: the licensing page grants modification and distribution "for any purpose"; the site terms 5.3 and 5.5 restrict modification and commercial use of downloaded material; this STEP's page carries no licence line of its own, so the two cannot be reconciled from the pages. The licensing page names a contact address for questions. Recorded, not enforced (L88c); kept local (L88b).
- Datasheet (https://www.raspberrypi.com/products/camera-module-3/): "Dimensions: 25 x 24 x 11.5mm (12.4mm height for Wide variants)". Mass: not found (the product brief PDF https://pip-assets.raspberrypi.com/categories/786-raspberry-pi-camera-module-3/documents/RP-008151-DS-1-camera-module-3-product-brief.pdf could not be read here).
- Whole file: 590 solids (a full circuit-board model: pads, pins, silkscreen, solder mask, passives).
- Picked (7 objects, 24,364 triangles): Dielectric (the board, 556 tri, 23.9 x 25.0 x 0.7), J1 (7,638, connector 6.5 x 22.9 x 2.6), U4_7 (12,712, 8.5 x 8.5 x 4.7, the tallest part: the lens/sensor housing), U4_3 (340, 18.1 x 11.2 x 1.0), U12 (1,114), U4_6 (1,820, 10.8 x 10.8 x 0.7), X1 (184). Left out on purpose: all pads, pins, silkscreen, solder-mask layers, resistors and capacitors; J2 (3.0 x 8.2 x 1.0 mm, 14,254 triangles for a small part).
- Result: 23.86 x 25.0 x 10.25 mm. MISMATCH: the "simple" model is 1.25 mm lower than the brief's 11.5 mm; the 25 x 24 plan is turned (23.86 x 25.0). Volume 0.90 cm3.
- MISSING OBJECT: U4_4 (10.9 x 10.9 x 3.3 mm, 107 mm3, the second biggest body). Refused by the gate at 0.02, 0.05 and 0.1 mm. First cause: a vertex pair 1.7e-6 mm apart at about 17 mm, different doubles but the same float32 stored point ("1 vertex at the position of an earlier vertex (weld first)"); the converter now merges exactly such duplicates (commit ab3e5600). Then it is refused for "self-intersecting triangle pairs" (a bad tessellation), which is beyond exact coincidence and stays refused by rule. Also refused, not picked anyway: SILKSCREEN_BOTTOM_42 and _54 (14 self-intersecting pairs each).
- Unit note: the file declares its unit as CONVERSION_BASED_UNIT('MILLIMETRE', ...); the converter refused that until commit 34960223.

## 4. Raspberry Pi 5 (Raspberry Pi), "3D STEP - No Graphics small file"
- Vendor page: https://pip.raspberrypi.com/categories/892-raspberry-pi-5 , link text "rpi-5 3D STEP - No Graphics small file", "13 MB ZIP archive", "updated 11 Jun 2026" (the large file with graphics, RP-010082-CA-1, 24 MB, was not fetched; no graphics keeps logos out).
- File link: https://pip-assets.raspberrypi.com/categories/892-raspberry-pi-5/documents/RP-010083-CA-1-rpi-5%203D%20STEP%20-%20No%20Graphics%20small%20file.zip (13,270,704 bytes, no sign-in, no redirect). sha256 of the zip 6841637b4cfa97637bf34b529bfe896f24d9634b2c40b57a156781f55d88f06f. Contents: rpi-5b_no_graphics.step (77,627,469 bytes; sha256 of the file 78164070c1cc7ab854382950d2e85926c6e35432f1e5a146b733397a7c6afb44; written by FreeCAD / Open CASCADE STEP processor 7.8, 2026-05-27) and LICENSE.txt.
- Terms, from LICENSE.txt in the zip, verbatim: "This 3D model is provided under the MIT license included at the end of this file. The 3D model may incorporate individual 3D models that have been provided by third parties with permission to redistribute. No additional license terms are imposed by this use of third party provided 3D models, where applicable." "The overall 3D model and any individual 3D models incorporated into the whole, whether created by first or a third party, are provided here for guidance only and without guarantee of accuracy. ... Any third party intending to use the provided 3D model for appearance and fitment purposes must refer to manufacturer information and conduct suitable measurements of the physical product." "= MIT License = Copyright (c) 2026 Raspberry Pi Ltd. Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, ..." (the standard MIT text). The same Raspberry Pi site-wide statements as in section 3 (licensing page, site terms 5.3/5.5) apply to the site generally, but this file's own MIT licence is the specific grant. The copyright notice must travel with copies.
- Datasheet/dimensions: the product page https://www.raspberrypi.com/products/raspberry-pi-5/ has none; https://www.raspberrypi.com/documentation/computers/sbcs.html says "approximately 85 mm x 56 mm" (read through a page summary, not verbatim). The mechanical drawing https://pip-assets.raspberrypi.com/categories/892-raspberry-pi-5/documents/RP-008347-DS-1-raspberry-pi-5-mechanical-drawing.pdf (174,949 bytes, sha256 5dd680d6c1f5e7aa9c7b020695e315d04aac862df82ff01d4e9041cd0668d7f2) could not be read on this box (no PDF tool), so dimensions come from the CAD: board 85.0 x 56.0 x 1.3 mm, whole model 88.5 x 57.2 x 18.98 mm.
- Mass: Raspberry Pi publishes no weight (no official figure exists). 45 g, graded retailer, from the retailer listing https://www.desertcart.sg/products/593685802-raspberry-pi-5-8gb-ram-board (supplied by the lead; I did not read that page myself). Other sources differ: https://www.ghacks.net/2023/09/28/raspberry-pi-5-specs-price-and-more/ says no official weight is published, and https://www.hifiberry.com/?p=207172 lists 62 g (likely a packaged weight). The figure is a retailer's.
- Whole file: 2,689 solids, 22 parts with no solid (surface-only), skipped and named (code no_solid; the names are in the import result's `skipped` and in import.json): OBJ5048, OBJ5050, OBJ5052 and 19 more.
- Picked (18 objects, 47,302 triangles): board OBJ8572 (1,204 tri, 85.0 x 56.0 x 1.3); Ethernet OBJ4356 (804, 21.2 x 15.6 x 14.6) and OBJ4358 (1,538, 21.6 x 15.9 x 15.3); header OBJ3742 (244, 50.8 x 5.1 x 2.5); SoC OBJ1936 (1,400, 17.0 x 17.0 x 1.4) and OBJ1938 (12); OBJ4366 (4,316, 12 x 12 x 1.0) and OBJ12 (116); OBJ5142 (772, 8.3 x 6.5 x 3.8) and OBJ5144 (580, 8.9 x 7.3 x 4.3); FPC connectors OBJ3526 and OBJ3634 (2,604 each, 2.3 x 16.0 x 3.9), OBJ5006 (2,056), OBJ3532 and OBJ3640 (388 each); OBJ8534 (894) and OBJ8536 (4,202); the USB-A stack OBJ3734 (23,180, 17.5 x 14.5 x 17.5). The file names its objects OBJ nnnn; the roles above are read from sizes and are not certain.
- Result: 88.05 x 57.2 x 18.78 mm (the board matches 85 x 56; ports overhang). Volume 12.74 cm3. Not picked: pads, pins, silkscreen, tiny passives (a pick by size, 2,689 solids in all; 788,720 triangles for the whole file at 0.05 mm).
- MISSING OBJECT: OBJ3726 (17.5 x 14.5 x 17.7 mm, 1,718 mm3, the second USB-A stack). Refused by the gate at 0.05 mm: first for coincident vertices (the float32 case, fixed by the weld, commit ab3e5600), then for "6 duplicate triangles" (the merge made six pairs of triangles identical). Removing duplicate triangles would be a repair, not an exact-duplicate weld, so it stays refused.

## Tool and converter facts (L88/L88l)
- Merged to master with the part-import branch (merge 54b497e8; the gate kinds a5812978). Converter fixes made for these files: 34960223 (CONVERSION_BASED_UNIT named MILLIMETRE etc.), f11e3603 (a part with no solid is skipped and named), ab3e5600 (float32 duplicate weld). Tool: 2a0eeb09 (skipped shown), 384f3f57 (repeatable --object, skipped kept in import.json).
- The `--object` picks name the entry: the cache key is built from the sorted picks, so the same picks in any order are one entry. A different pick set is a different entry of the same source.
- Deflection 0.05 mm for every part (angle 0.5 rad); the entry's tolerance is recorded in each sidecar.

## Store entries and per-object store_ref (sha256 of the sidecar bytes, and its file)
Entry path: `<store>/<source sha256>/<key>/`. Each object also lives at `<store>/<store_ref sha256>/<file>`.

```
== px6c exit 0 cached False source_sha256 05b838910fd49ec61347a6a18061da2b30fd116fa68c9acae84644b8851f1ef0 key cf16a0ea97d3bea9
   entry 05b838910fd49ec61347a6a18061da2b30fd116fa68c9acae84644b8851f1ef0/cf16a0ea97d3bea9 | matches temp store: True | store_ref files missing or wrong: 0 | skipped 0
   PCB-PIXHAWK6C-F1            912 tri  content_hash a13b54a8b7053363  store_ref 6102d3cd46d4dd539e6ba6a37e91567758bcb42b9920e0f01ceb2c86e78b74bb/PCB-PIXHAWK6C-F1.spmesh
   MIANKE-PIXHAWK6C-LV-C1    13242 tri  content_hash 9050cda5131967d9  store_ref b46e9195617796641e590131d93d13bd4f88a790693080820b352f0da3cf4756/MIANKE-PIXHAWK6C-LV-C1.spmesh
   DIKE-PIXHAWK6C-LV-C1       6152 tri  content_hash 0036f4c9370c0bd8  store_ref 0b3807d1ea295a84b5293d23b74430ba3a5d3188a689dee54f554e466a260d32/DIKE-PIXHAWK6C-LV-C1.spmesh
   IMU-PIXHAWK6C               392 tri  content_hash f5583ea92430f583  store_ref 278c17f49e9442bee9db096b6735dafd1febf13b4bec15ee63e3562fd7256c4e/IMU-PIXHAWK6C.spmesh
== gps exit 0 cached False source_sha256 42587789183f82d0ab59da80f0149d355ff4df99695bb2ddc071354225197fce key e8c2181a967e269a
   entry 42587789183f82d0ab59da80f0149d355ff4df99695bb2ddc071354225197fce/e8c2181a967e269a | matches temp store: True | store_ref files missing or wrong: 0 | skipped 0
   GPS-V5-SK-C1               9346 tri  content_hash e59b96e21158ab0b  store_ref 22e4fe75e90d026b30469a1f9fafe72817473bd421cf3e405b12a9dd55b25d27/GPS-V5-SK-C1.spmesh
   GPS-V5-XK-C1               4550 tri  content_hash 5197009088ff0425  store_ref c046eb15dd6db641431fd570085e8339e2dd6200849490f3d21e4b0d82daf7c9/GPS-V5-XK-C1.spmesh
== cam3 exit 0 cached False source_sha256 ecbc4b091e51f31c6186b81ad68d4205b8dceec0d90c116cd7fd699e1f1c5702 key 1a833816caeba9d4
   entry ecbc4b091e51f31c6186b81ad68d4205b8dceec0d90c116cd7fd699e1f1c5702/1a833816caeba9d4 | matches temp store: True | store_ref files missing or wrong: 0 | skipped 0
   Dielectric                  556 tri  content_hash aed84f8ac0bc2356  store_ref 8bdf6c1b1316c3e742e74733e8d23f5e79d75e29ce04f7094fa9d0e339974baa/Dielectric.spmesh
   U4_3                        340 tri  content_hash 162e7189818e1380  store_ref 9e7c82466d157d146fa94f0f66fad9324b75d41a3555d6d0ee05b4ee261aa5e0/U4_3.spmesh
   U4_6                       1820 tri  content_hash 11016dc6950f992e  store_ref b0bd973f942d6790a11f95426610512180e82d7beb8f893035f16b1a5028d22d/U4_6.spmesh
   U4_7                      12712 tri  content_hash e63e0f41f8e1edfe  store_ref c801287e8939cd6f60840e885c6b4fb2f031409584b732aad1bd927947e60ac4/U4_7.spmesh
   U12                        1114 tri  content_hash c9c2c94929b63ea3  store_ref 4eeea849eb668984e6918fa5451ae4647af05faa639944741434b0d1bd65e635/U12.spmesh
   J1                         7638 tri  content_hash a28bfb37fb330632  store_ref 9893fca68b39a86669350063e7e723a312e29816021a8e15d97e36f7966f1588/J1.spmesh
   X1                          184 tri  content_hash 9b3107adbc5d56bf  store_ref c84c1f2c61f0aeba58f6d3e6d5c62c77a12fea093069eae6c3036733edc3b91b/X1.spmesh
== pi5 exit 0 cached False source_sha256 78164070c1cc7ab854382950d2e85926c6e35432f1e5a146b733397a7c6afb44 key c046aa45d58c7a02
   entry 78164070c1cc7ab854382950d2e85926c6e35432f1e5a146b733397a7c6afb44/c046aa45d58c7a02 | matches temp store: True | store_ref files missing or wrong: 0 | skipped 22
   OBJ12                       116 tri  content_hash a55c9d7e789b781b  store_ref 7e031388b64c748d5cfdabf83dca2d1a08bfc18cc5de7c1d3e691d9f957270d9/OBJ12.spmesh
   OBJ1936                    1400 tri  content_hash 3df9c25bf1e25f2f  store_ref f494280926fa6c407968c5bb978cd49fd90fe9c632c7f754f212c702da31455c/OBJ1936.spmesh
   OBJ1938                      12 tri  content_hash 61c6e9750ad965bb  store_ref 45cb69b8b566d99b2484a8f726cc7e9b1a34a8c43b30d53064d5b65aee0001c1/OBJ1938.spmesh
   OBJ3526                    2604 tri  content_hash 4cb7eb33e7ac0010  store_ref 212f56bfd003feebd82f38ae2e948c4e5997dc7578221d0937a09c34dc139f54/OBJ3526.spmesh
   OBJ3532                     388 tri  content_hash eb0f5f5f569f61cb  store_ref 17a6ddd1a61db68976a2f478cc4606b3bf16047148a364d2592ba2ec00018bdf/OBJ3532.spmesh
   OBJ3634                    2604 tri  content_hash d342cd1ce2bb0230  store_ref 0920615ceb2a41ced33c41a5e1dc24e4f0fffbf65efad046ceb6f8f0e9c09f3c/OBJ3634.spmesh
   OBJ3640                     388 tri  content_hash 197c2410f9f5588f  store_ref ad1d005b72c487a37901a5a2287f6ba8652b586068bb4cffa737fae02c0fa2f8/OBJ3640.spmesh
   OBJ3734                   23180 tri  content_hash 9328c5099532db4e  store_ref 0f2c7a6a9d29b58d386adc8f0260bc6b7bbbe30a64ef8a682a6c5e5220e95557/OBJ3734.spmesh
   OBJ3742                     244 tri  content_hash 0d80aa0a0ef16c0b  store_ref 8dd7cae262f69d3e5996cf0c1b8f52370dc8b0b775c146eae9ae37b19e9bf248/OBJ3742.spmesh
   OBJ4356                     804 tri  content_hash 1e6ace467dde0034  store_ref 3edb8d2b9b8dc663ab061b4f3f983e549fb83df6d3a381b9bb1e5cfc9ee29d16/OBJ4356.spmesh
   OBJ4358                    1538 tri  content_hash b8de5b1050c88b3b  store_ref cf3fb4625ac6ceb06a5a0f3da9efd820307f4f3f53dd78753436ea431ba5b36a/OBJ4358.spmesh
   OBJ4366                    4316 tri  content_hash fb9a117b10dd344d  store_ref a453741db902b3a286148a136d54f4d78da336ab06bdea6b15a9c8dcb274898b/OBJ4366.spmesh
   OBJ5006                    2056 tri  content_hash 6c83b0f5779db277  store_ref 552fcd33894a8dd282876d33a7d4ed75ffffde4a55707f88a1f1b74a176323c4/OBJ5006.spmesh
   OBJ5142                     772 tri  content_hash 4f14314d07d7e885  store_ref d4a5ffd33ecee5f77e7792f8cc06fd07ea505369e74ef53d49092128a4413584/OBJ5142.spmesh
   OBJ5144                     580 tri  content_hash 503b68f9ece42d67  store_ref e3ec63431cc620cb5379c77e0687387433742bdc245b7dac8f3a5a52f59e8959/OBJ5144.spmesh
   OBJ8534                     894 tri  content_hash 3e2fb4798507cb30  store_ref 402ce5b3cce3e42d07058212bcde21df52517b69c9c5babd9a3d8151dc49c378/OBJ8534.spmesh
   OBJ8536                    4202 tri  content_hash 9b721f24a38ef955  store_ref 2c20b0afb4fb5767b6dc2b3545d1aa9d4a07addf7ecdf5997fbc6735038164c2/OBJ8536.spmesh
   OBJ8572                    1204 tri  content_hash c5421d666ca8056b  store_ref 1c67333986945f3eabf754755710da4a3a0a54718af01cec0646e17262579d86/OBJ8572.spmesh
```

## 5. Holybro X500 V2 kit frame (Holybro), the kit's own "3D CAD File" (L88r, L88s)
- Vendor page: https://docs.holybro.com/drone-development-kit/px4-development-kit-x500v2 , section "Downloads", heading "3D CAD File"; the page shows the file name x500v2-frame.step as the link text. The page's only other file link is "ESC 3D File Downloads" (a separate page, not read).
- File link: not recorded. The vendor page links a GitBook file host address that carries a per-link access token; the file name and sha256 below identify the file. No sign-in, no redirect, one GET, HTTP 200.
- File: x500v2-frame.step, 24,624,144 bytes, STEP AP214 (AUTOMOTIVE_DESIGN), written 2022-07-19 by ST-Developer v19 through Autodesk Translation Framework 11.7. sha256 e7ccd422cf7825b61ad8270a98d8fbbbc12aced102cc6a20ed17ee66b816c360
- Terms: none found on the kit page or its Downloads section (the store terms of section 1 are the only reuse lines: UNADDRESSED). Recorded, not enforced (L88c); kept local (L88b).
- Datasheet figures (a search summary of Holybro's product page, not read directly): wheelbase 500 mm, frame body 144 x 144 mm and 2 mm thick, 28 mm between the top and bottom plates, landing gear height 215 mm. The docs page lists "Holybro 2216 KV920 Motor (4 pcs) with XT30 Plug".
- Unit: the file names SI millimetre in all 50 of its geometric representation contexts and SI metre only inside DERIVED_UNIT_ELEMENT (a density unit). The converter refused it ("more than one length unit") until its unit rule read the contexts (commit 2b3f492c, branch interface/help-stdout); the four files above read as before.
- Whole file: 244 solid instances under 139 names, 0 skipped, 574.6 cm3; 400.7 x 323.1 x 400.7 mm (Y up, -216.1 to 107.0 mm). Deep inspect at 0.05 mm: 221 pass the gate and 23 are refused, 22 as self_intersecting (the 16 screws GB70-M3-38, the 2 screws GB70-M3-21-DING, and the 4 motor-seat covers HMX5V-DIGAI-DIANJIZUO-MUJU) and 1 as not_closed (the GPS-stand nut GPSV5-ZHIJIA-LUOMAO). 74 solids differ from the B-rep by more than 1% (small hardware and the 4 MAO-JIAO end caps, 1.33%). The 221 passing solids hold 812,828 triangles.
- File frame: millimetres, Y up, X and Z horizontal, origin at the frame centre, the bottom plate's lower face at y=0 (bottom plate y 0 to 2, top plate y 30 to 32). The motor axes are at x = z = +-177.4 mm (radius 250.9 mm; 501.8 mm motor to motor on the diagonal). The GPS stand is at +X and the camera bracket at -X. Which end is forward is NOT stated: the kit's docs pages that exist (Getting Started Build Guide, Propeller Retainer Installation Guide) hold only video and image content, with no front or rear sentence.
- Picked: 65 objects, 176,966 triangles, one entry, key d2a28d276fe74896 (deflection 0.05 mm, angle 0.5 rad, `--object` per pick). Every pick passed the gate and the cross-check, none flagged; the temp-store and real-store entries are identical and every store_ref file was verified. The picks are the frame (plates, arm tubes, motor mounts, arm clamps, motor base plates), the landing gear, the 4 motors (DJ-2216-KV880:1_1 to :1_4), the kit's GPS stand (mast rod 3.9 x 108 x 3.9 mm, tray, base, adapter) and the battery and payload tray (tray plate, pad, two rail tubes, two pylons, 16 rail clamps, platform plate). The list with sizes, centres and store_refs is at the end of this record.
- Motors: the file calls them DJ-2216-KV880 (can 27.7 mm across, 28.7 mm over the body, 51.9 mm tall with the shaft, shaft top at y 64.1 mm, can top at y 52.0 mm); the kit sells the 2216 KV920. Same 2216 can, a different winding, so the geometry holds. The file lists each motor twice (:1_1 to :1_4, and :2 to :5, one content hash per position); the :1_n copies are kept because they sit in the same assembly family and order as the motor mounts and come first in the file's order.
- Left out: the 4 MAO-JIAO end caps (13 x 12.6 x 12.6 mm, flagged by the cross-check, so an exit 6 on every cache hit); the 4 refused motor-seat covers (no repair); the Intel camera bracket ZHIJIA-CAMERA-INTEL (no camera body is in the file), the optical-flow cover GAI-GUANGLIU, the power board PCB-PM06, the XT60 connectors and BM06B-WO; the GPS stand's nut (refused, open); about 160 screws, nuts and washers (15 cm3 in all).
- Against the PX4 drawing mesh x500-body.gltf (a Gazebo-style model, drawing only): the kit's two plates are 143.7 x 143.7 x 2.0 mm where the glTF has a 148.0 mm square zero-thickness sheet and a 148.0 x 12.6 x 148.0 mm shell; the kit's arm tube is hollow, 15.4 mm across (wall 0.7 mm), 177.5 mm long on the diagonal, where the glTF's is a 15 mm prism about 215 mm long and 4 mm lower; the kit's motor axes are +-177.4 mm where the glTF's are +-174.0 mm (3.4 mm per axis, 4.8 mm radially). The rotor glTF's 59 mm "bell" is about twice the real 28.7 mm motor.

## 6. 4S pack, Tattu G-Tech 4S 5200mAh 14.8V 35C (a box; no CAD exists)
- Maker's page: https://www.grepow.com/uav-battery/tattu-g-tech-4s-5200mah-14-8v-35c-lipo-drone-battery.html (Shenzhen Grepow Battery Co., Ltd.). Read as a page, nothing fetched. Quoted: "Minimum Capacity: 5200mAh", "Configuration: 4S / 14.8V / 4 cells", "Max Continuous Discharge: 35C", "Weight: 436.5g", "Dimensions: 133x45x33.5mm".
- Not on the page: a part number, energy in Wh, the connector, whether 436.5 g includes the plug, any tolerance, a datasheet link, any terms line. Retailer listings of the same pack differ (422 to 430 g, 134 x 44 x 32 mm), so the maker's page is the figure cited.
- Holybro does not sell a pack for the kit: the docs say "Recommend 4S 2000-5000mAh Lipo Battery (Not Included)" and give the flight time as tested with a 5000mAh battery.
- The table models 0.4365 kg and Physics' solid is 133 x 33.5 x 45 mm (length, height, width): the same box.

## 7. Telemetry radio, Holybro SiK Telemetry Radio V3 433/915MHz (a box; the only CAD is for another variant)
- Datasheet page: https://docs.holybro.com/radio/sik-telemetry-radio-v3 . 100 mW and 500 mW columns: "Dimension" "28 x 53 x 10.7mm (without antenna)", "Weight" "23.5g w/ antenna", "6-pin JST-GH connector, 3.3V-TTL", "RP-SMA female". The 1W variant is another size ("66.5 x 32 x 16.5mm", "67g w/ antenna"). The kit page lists "SiK Telemetry Radio V3  433/915MHz".
- The downloads pages offer "SiK Long Range Case 3D CAD" (the 1W housing) and "PCBA 3D File" (a bare board); no terms line on either; neither was fetched. The box is the 100 mW datasheet's (L88q).

## Colours the CAD files carry (L88s)
Read with OpenCASCADE's colour tool (STEP colours are sRGB; the values below are 0 to 255 sRGB, the file's own). The meshes in the store carry no colour: this is what a drawing could use instead of chosen ones.
- Kit frame (x500v2-frame.step): one colour for all 244 solids, COLOUR_RGB 'Steel - Satin' = (0.6275, 0.6275, 0.6275) = (160, 160, 160). There is no per-part colour (51 STYLED_ITEMs, one colour).
- Pixhawk 6C and the GPS puck: no colour in the file (no STYLED_ITEM).
- Camera Module 3: 584 of 590 solids carry a colour, 14 distinct. The 7 picks: Dielectric (253, 255, 215); U4_3 (0, 255, 0); U4_6 none; U4_7 none; U12 (196, 196, 196); J1 none; X1 (202, 209, 238). U4_3's (0, 255, 0) is a pure marker green; the lens housing U4_7, U4_6 and the connector J1 have none.
- Raspberry Pi 5: all 2,689 solids carry a colour, 24 distinct. The 18 picks: OBJ8572 (0, 140, 73); OBJ3734 (255, 235, 135); OBJ3742 (34, 34, 34); OBJ4356 (34, 34, 34); OBJ4358 (204, 204, 204); OBJ4366 (51, 51, 51); OBJ1936 (204, 204, 204); OBJ1938 (177, 124, 90); OBJ12 (51, 51, 51); OBJ3526 (245, 245, 222); OBJ3634 (245, 245, 222); OBJ5006 (245, 245, 222); OBJ3532 (105, 84, 70); OBJ3640 (105, 84, 70); OBJ5142 (34, 34, 34); OBJ5144 (204, 204, 204); OBJ8534 (34, 34, 34); OBJ8536 (204, 204, 204).

## Kit frame entry: key d2a28d276fe74896 (65 objects, 176,966 triangles)
Source sha256 e7ccd422cf7825b61ad8270a98d8fbbbc12aced102cc6a20ed17ee66b816c360; entry path `<store>/<source sha256>/d2a28d276fe74896/`; each object also at `<store>/<store_ref sha256>/<file>`. Boxes and centres are in the file frame above, in millimetres, from the stored mesh.

```
## Plates (2)
BOTTOM-PLATE-X500-V5:1 | 7580 tri | box 143.7 x 2.0 x 143.7 | c (0.0, 1.0, 0.0) | eac8e7daf518d4220be2b82c5008dd89823c04e0729ba201d52686051d60f3fc/BOTTOM-PLATE-X500-V5_1.spmesh
TOP-PLATE-X500-V5:1 | 4884 tri | box 143.7 x 2.0 x 143.7 | c (0.0, 31.0, 0.0) | a38be181585fd248aab3e5a3d8c8846b789d4f46d91bca24aa12986eeab923c5/TOP-PLATE-X500-V5_1.spmesh
## Arm tubes (4)
HMX5V-GUAN-DINGWEI:1_1 | 662 tri | box 125.4 x 15.4 x 125.4 | c (-104.8, 16.0, -104.8) | 0ed20d7086190f6507a69c0dfc2f047352e64358bde4ee0537771e1a0afd914d/HMX5V-GUAN-DINGWEI_1_1.spmesh
HMX5V-GUAN-DINGWEI:1_2 | 662 tri | box 125.4 x 15.4 x 125.4 | c (-104.8, 16.0, 104.8) | 86920dafb2d81f59307d3b2c32b735eff5e93229a8520348fb050357077b83ca/HMX5V-GUAN-DINGWEI_1_2.spmesh
HMX5V-GUAN-DINGWEI:1_3 | 662 tri | box 125.4 x 15.4 x 125.4 | c (104.8, 16.0, 104.8) | eb6be5c911fb9fad56ad00f094c69b8743945d17efd66af0b79dc3bacb4fb500/HMX5V-GUAN-DINGWEI_1_3.spmesh
HMX5V-GUAN-DINGWEI:1_4 | 662 tri | box 125.4 x 15.4 x 125.4 | c (104.8, 16.0, -104.8) | b9e52986d35f42220a443b1b8bf536f7cb8e6bf792645d1cc03f52be62cbe1b6/HMX5V-GUAN-DINGWEI_1_4.spmesh
## Motor mounts (4)
HMX5V-ZUO-DJ-MUJU:1_1 | 7456 tri | box 72.0 x 22.0 x 72.0 | c (-164.1, 16.0, -164.1) | a4b1bcf4c19fea89b1c38c6d130f770eedf5d942742ffda29aa72e7596ddbccd/HMX5V-ZUO-DJ-MUJU_1_1.spmesh
HMX5V-ZUO-DJ-MUJU:1_2 | 7456 tri | box 72.0 x 22.0 x 72.0 | c (-164.1, 16.0, 164.1) | 7675bca54b0676d7b86249f34fa8e6a43f324b44cec0bc34b494aa4575bb2995/HMX5V-ZUO-DJ-MUJU_1_2.spmesh
HMX5V-ZUO-DJ-MUJU:1_3 | 7456 tri | box 72.0 x 22.0 x 72.0 | c (164.1, 16.0, 164.1) | a7a5c88bd16842655c645b4a98b8bf0185f23e77b95b83b60899f4a44d166eb7/HMX5V-ZUO-DJ-MUJU_1_3.spmesh
HMX5V-ZUO-DJ-MUJU:1_4 | 7456 tri | box 72.0 x 22.0 x 72.0 | c (164.1, 16.0, -164.1) | 1fbc34c1710e3aa6aa7ff2e232f2e65178890f2ee11c84f0d60cd25ed02c4e79/HMX5V-ZUO-DJ-MUJU_1_4.spmesh
## Arm clamps (8)
HMX5V-JIBI-JIA-MUJU:1_1 | 3742 tri | box 34.1 x 17.9 x 34.1 | c (-55.5, 8.9, -55.5) | 47ed664bc42f065fa8fa5d46c366161166925fafa9c4d2b152a177c54624f677/HMX5V-JIBI-JIA-MUJU_1_1.spmesh
HMX5V-JIBI-JIA-MUJU:1_2 | 3742 tri | box 34.1 x 17.9 x 34.1 | c (-55.5, 8.9, 55.5) | 8249333ef3d6418c1a39dde4714c63ce7ddb92eac4186636f004ea920ccd34c1/HMX5V-JIBI-JIA-MUJU_1_2.spmesh
HMX5V-JIBI-JIA-MUJU:1_3 | 3742 tri | box 34.1 x 17.9 x 34.1 | c (55.5, 8.9, 55.5) | 3ace70925075cea939f0f9742cd05f738d5cdab53b7a6cbba5843aa3ad41a69d/HMX5V-JIBI-JIA-MUJU_1_3.spmesh
HMX5V-JIBI-JIA-MUJU:1_4 | 3742 tri | box 34.1 x 17.9 x 34.1 | c (55.5, 8.9, -55.5) | 4860a0062d4e10e73d942598fae3bae25dcc2b520ec7eadff3206d43d0e6644d/HMX5V-JIBI-JIA-MUJU_1_4.spmesh
HMX5V-JIBI-JIA-MUJU:2_1 | 3742 tri | box 34.1 x 17.9 x 34.1 | c (-55.5, 23.1, -55.5) | f939377052cf21f425b1869827daca9a0723d1b7f9b393e5321df56548601a47/HMX5V-JIBI-JIA-MUJU_2_1.spmesh
HMX5V-JIBI-JIA-MUJU:2_2 | 3742 tri | box 34.1 x 17.9 x 34.1 | c (-55.5, 23.1, 55.5) | b3604da76a0f72acb85ce159d911cd41525ba8513e0f2c5e87c0ea14a6691383/HMX5V-JIBI-JIA-MUJU_2_2.spmesh
HMX5V-JIBI-JIA-MUJU:2_3 | 3742 tri | box 34.1 x 17.9 x 34.1 | c (55.5, 23.1, 55.5) | ed9c7e1307d53c4190c6516345067eede4c274d811c68cec707bd9ec5107914c/HMX5V-JIBI-JIA-MUJU_2_3.spmesh
HMX5V-JIBI-JIA-MUJU:2_4 | 3742 tri | box 34.1 x 17.9 x 34.1 | c (55.5, 23.1, -55.5) | 4a5ea2004adb738a0aa708bc188fc4172cb6c9782899207e376d281961d71320/HMX5V-JIBI-JIA-MUJU_2_4.spmesh
## Motor base plates (4)
BAN-DJ-DIAN-F2:1_1 | 1992 tri | box 53.6 x 2.0 x 53.6 | c (-173.4, 21.0, -173.4) | 0c32b3f93fa068561ac36586369f919d50c9ed6ace1f57618824e3a00ec40f42/BAN-DJ-DIAN-F2_1_1.spmesh
BAN-DJ-DIAN-F2:1_2 | 1992 tri | box 53.6 x 2.0 x 53.6 | c (-173.4, 21.0, 173.4) | 8e06fed000b39d32e3d5c3dec898bf446d19adc2490297ba7661f7ef798bb007/BAN-DJ-DIAN-F2_1_2.spmesh
BAN-DJ-DIAN-F2:1_3 | 1992 tri | box 53.6 x 2.0 x 53.6 | c (173.4, 21.0, 173.4) | a2c8ebc4fe8c5fc02b2a2e026418bf42a2a3953792d8466b325f7a677bfab748/BAN-DJ-DIAN-F2_1_3.spmesh
BAN-DJ-DIAN-F2:1_4 | 1992 tri | box 53.6 x 2.0 x 53.6 | c (173.4, 21.0, -173.4) | e8f9cce30847cae35d2f7f41290617e4bb9f8eee9146e1b00ff46753cdf49d3f/BAN-DJ-DIAN-F2_1_4.spmesh
## Motors (2216, file says KV880) (4)
DJ-2216-KV880:1_1 | 7862 tri | box 28.7 x 51.4 x 28.7 | c (-177.4, 37.8, -177.4) | 8a60bec80f0635d5084f53df812e120869733745abc46e5d9a2274871d129613/DJ-2216-KV880_1_1.spmesh
DJ-2216-KV880:1_2 | 7862 tri | box 28.7 x 51.4 x 28.7 | c (-177.4, 37.8, 177.4) | 3d25dc21a0df32b45dbb754c0e8a23fa20508d049b92bb64ccff5a8783e95bb2/DJ-2216-KV880_1_2.spmesh
DJ-2216-KV880:1_3 | 7862 tri | box 28.7 x 51.4 x 28.7 | c (177.4, 37.8, 177.4) | 61998003dac2847c735a999fd46686c602ee75675126884aef7a7025d7f1a854/DJ-2216-KV880_1_3.spmesh
DJ-2216-KV880:1_4 | 7862 tri | box 28.7 x 51.4 x 28.7 | c (177.4, 37.8, -177.4) | 7a0ab2685bf7e58c75dc7ba9baaf2775349d7bbad43881e7da387ff26e07edf5/DJ-2216-KV880_1_4.spmesh
## Landing gear (12)
CARBON-FIBER-TUBE:1 | 244 tri | box 248.0 x 10.0 x 10.0 | c (0.0, -205.8, -120.0) | f31aa0f05aa92207419bc6cd22a7a5928c244adcde51438143dd98a2f79c5e89/CARBON-FIBER-TUBE_1.spmesh
CARBON-FIBER-TUBE:2 | 244 tri | box 248.0 x 10.0 x 10.0 | c (0.0, -205.8, 120.0) | 4afe0d121aa971508e888f660d340894bdd236dfe81cd27380f5e1239a660b09/CARBON-FIBER-TUBE_2.spmesh
GUAN-CHENG:1 | 588 tri | box 16.0 x 193.6 x 75.1 | c (0.0, -100.9, -86.7) | 70604a87f89d81aab54d257edaa8c59ed40e2361f245e594a8780cdd0e3e5edf/GUAN-CHENG_1.spmesh
GUAN-CHENG:2 | 588 tri | box 16.0 x 193.6 x 75.1 | c (0.0, -100.9, 86.7) | 98b55533125080d937a7b90041deacac63765ac9875643feba32f8f676ac0f91/GUAN-CHENG_2.spmesh
JIA-LIANJIE:1 | 1536 tri | box 33.8 x 22.0 x 32.5 | c (0.0, -11.0, -55.5) | 8a67e5e41f3cb203cf532e7bb38a4f66182d621c8bfa70a07cbdc675e5b5653e/JIA-LIANJIE_1.spmesh
JIA-LIANJIE:2 | 1536 tri | box 33.8 x 22.0 x 32.5 | c (0.0, -11.0, 55.5) | 1f9145cc8b86814f53e7e49ae4de7abbfd25459ababeac09d0533dc6523b2ed0/JIA-LIANJIE_2.spmesh
JIAO-EVA:1 | 3684 tri | box 93.0 x 19.0 x 19.0 | c (-67.5, -205.8, -120.0) | 7c52ba5f54cb1cdaa1276de1048ec524b9cd8efc2c1a322170718fdae28da202/JIAO-EVA_1.spmesh
JIAO-EVA:2 | 3684 tri | box 93.0 x 19.0 x 19.0 | c (67.5, -205.8, -120.0) | 34e2de4dc804c6fab54a7dd77ea522a19361609eb6874ca2b41b69a5e06d072e/JIAO-EVA_2.spmesh
JIAO-EVA:3 | 3684 tri | box 93.0 x 19.0 x 19.0 | c (67.5, -205.8, 120.0) | c5b21680ff18eab836ff913e032bed8adbeefb4b0a9ca35b13693c696cfaaea7/JIAO-EVA_3.spmesh
JIAO-EVA:4 | 3684 tri | box 93.0 x 19.0 x 19.0 | c (-67.5, -205.8, 120.0) | f7530071e9bbefa3ec4d6e028769719f042ca287234e13f8168fda850b63a560/JIAO-EVA_4.spmesh
JIAO-LIANJIE:1 | 1976 tri | box 40.0 x 32.4 x 23.3 | c (0.0, -196.1, -114.8) | 5cd393eb531b13a76f833d53ec56a0df11efd4233cb0a6aa8503ba5efadefea6/JIAO-LIANJIE_1.spmesh
JIAO-LIANJIE:2 | 1976 tri | box 40.0 x 32.4 x 23.3 | c (0.0, -196.1, 114.8) | 6bac83d1e98131d2443c0b8ded56300959467c3f6011093f214d5a944c53d862/JIAO-LIANJIE_2.spmesh
## GPS stand (4)
GAN-GPSV5-ZHIJIA:1 | 112 tri | box 3.9 x 108.0 x 3.9 | c (99.0, 51.5, -0.0) | c03b37ceaafe0be186cd1b20b69a627ab85253c95acd6eef65f0d7ff3937f94a/GAN-GPSV5-ZHIJIA_1.spmesh
GPS-ZHIJIA-ZHUANJIETOU:1 | 668 tri | box 8.0 x 16.5 x 8.0 | c (99.0, -5.8, -0.0) | 5ce1325881a831935684e7027d7c6a13d78d828c18e415420a670f8269dd421c/GPS-ZHIJIA-ZHUANJIETOU_1.spmesh
GPS-ZHIJIA-ZUO:1 | 5736 tri | box 29.7 x 18.7 x 29.7 | c (99.0, -16.7, 0.0) | 4d53360511070a266b91f202f6d017745a71228f3ccc30e8c4952633db59dbf6/GPS-ZHIJIA-ZUO_1.spmesh
GPSV5-ZHIJIA-TUOPAN:1 | 762 tri | box 29.0 x 9.5 x 29.0 | c (99.0, 102.2, 0.0) | 7e3a2ed7958e935168451f0e097de55ece92187a574f9dd09200eccb2be04bff/GPSV5-ZHIJIA-TUOPAN_1.spmesh
## Battery and payload tray (23)
BATTERY-MOUNTING-PLAT:1 | 1996 tri | box 100.0 x 2.0 x 50.3 | c (0.0, -45.5, 0.0) | 78ed5c5daaf5e819de1061e886bccca08ea46e9dd6118f216cc09728fc7c1627/BATTERY-MOUNTING-PLAT_1.spmesh
BATTERY-PAD:1 | 188 tri | box 91.5 x 2.0 x 23.6 | c (0.0, -47.5, 0.0) | 93a6714da957cd55297fa00b70c1acc90cd93121df98bd8f310f0b949436d6d4/BATTERY-PAD_1.spmesh
CARBON-FIBER-TUBE300:1 | 240 tri | box 250.0 x 9.4 x 9.4 | c (5.0, -13.0, 30.0) | f17ba21e5343d25e6a5aa0b65981b7d683f531f1a5a5269be54bb0dede75b53e/CARBON-FIBER-TUBE300_1.spmesh
CARBON-FIBER-TUBE300:2 | 240 tri | box 250.0 x 9.4 x 9.4 | c (5.0, -13.0, -30.0) | 989e961c71b5c0f1e699692a34d3ec73fb25eba18341650e63a877ed5364a528/CARBON-FIBER-TUBE300_2.spmesh
HUAN-GUIJIAO:1_1 | 2216 tri | box 11.4 x 19.4 x 19.4 | c (50.0, -13.0, 30.0) | 4e46fd4af8710cd8726c79693787f441aad9f8a9ba1a2976af41c9df3090a272/HUAN-GUIJIAO_1_1.spmesh
HUAN-GUIJIAO:1_2 | 2216 tri | box 11.4 x 19.4 x 19.4 | c (50.0, -13.0, -30.0) | 71484525a3f052d4cfcef09285c8eca362e14224ce8011d34213439433328619/HUAN-GUIJIAO_1_2.spmesh
HUAN-GUIJIAO:1_3 | 2216 tri | box 11.4 x 19.4 x 19.4 | c (-50.0, -13.0, 30.0) | 1767ac2da34d6de027d9990dbd68f66736fdd91d67a282d7cb04ceafe63f9bf9/HUAN-GUIJIAO_1_3.spmesh
HUAN-GUIJIAO:1_4 | 2216 tri | box 11.4 x 19.4 x 19.4 | c (-50.0, -13.0, -30.0) | 33afed3e13534a51229d3039ab2e3e27811bc6abc5e16d60c72fc84cd484c34a/HUAN-GUIJIAO_1_4.spmesh
HUAN-GUIJIAO:2_1 | 2216 tri | box 11.4 x 19.4 x 19.4 | c (117.0, -13.0, 29.5) | 7ec2188177beefa323c10d6a63ab17464b70bca4e313d68edc666321e706b8a7/HUAN-GUIJIAO_2_1.spmesh
HUAN-GUIJIAO:2_2 | 2216 tri | box 11.4 x 19.4 x 19.4 | c (117.0, -13.0, -29.5) | ae7c072270b833216d632b4bb949eae94bad1e8f3acff836b02870fc4fe6d00a/HUAN-GUIJIAO_2_2.spmesh
HUAN-GUIJIAO:2_3 | 2216 tri | box 11.4 x 19.4 x 19.4 | c (81.0, -13.0, 29.5) | 666f2f2b0d55eed2f8a0c9ed915c502c53e363371319fb46912be1031529f487/HUAN-GUIJIAO_2_3.spmesh
HUAN-GUIJIAO:2_4 | 2216 tri | box 11.4 x 19.4 x 19.4 | c (81.0, -13.0, -29.5) | fe5524a2bf25a0afa519119ddad6db8c0a30aefe4f6c2c656b449a1553493d07/HUAN-GUIJIAO_2_4.spmesh
JIA-GUAN:1_1 | 564 tri | box 6.0 x 23.0 x 21.0 | c (50.0, -11.5, 30.0) | 86ba00502f8ab182cfeab0c1c8ca63a7b3842e923c66497d5ec1883439691e01/JIA-GUAN_1_1.spmesh
JIA-GUAN:1_2 | 564 tri | box 6.0 x 23.0 x 21.0 | c (50.0, -11.5, -30.0) | b2214ccd4ce4d7ddd6c0086c9d3568309904597b6249a0249b9b03ca2ded43e7/JIA-GUAN_1_2.spmesh
JIA-GUAN:1_3 | 564 tri | box 6.0 x 23.0 x 21.0 | c (-50.0, -11.5, 30.0) | 4a9cd7e0af88b0510a70cb6b038fdc625a33f1cbdfef3269d7ca23edf50924fd/JIA-GUAN_1_3.spmesh
JIA-GUAN:1_4 | 564 tri | box 6.0 x 23.0 x 21.0 | c (-50.0, -11.5, -30.0) | 1f1e3076826601fa6ec3b9aac86517e1aae71d23767ee3ae349338efe480f48d/JIA-GUAN_1_4.spmesh
JIA-GUAN:2_1 | 564 tri | box 6.0 x 23.0 x 21.0 | c (117.0, -14.5, 29.5) | a0dd89de70c79b402a91329f37d8cb5e024b5114cd1a461c10eb0efe1e8a08fb/JIA-GUAN_2_1.spmesh
JIA-GUAN:2_2 | 564 tri | box 6.0 x 23.0 x 21.0 | c (117.0, -14.5, -29.5) | a1cab9d5522a5b7a39673fdeb2f851d25d37acc00b09308f6edc09f0870a2426/JIA-GUAN_2_2.spmesh
JIA-GUAN:2_3 | 564 tri | box 6.0 x 23.0 x 21.0 | c (81.0, -14.5, 29.5) | a22affd33d8422e56de2a321688bcf89950c25b31dc704552f8014ebfd018fff/JIA-GUAN_2_3.spmesh
JIA-GUAN:2_4 | 564 tri | box 6.0 x 23.0 x 21.0 | c (81.0, -14.5, -29.5) | 216e7e6fb2f407610b704fce96f3956ad502abb025b2270fdcb573310a52c3d9/JIA-GUAN_2_4.spmesh
PLATFORM-PLAT-X500:1 | 4728 tri | box 65.0 x 2.0 x 93.0 | c (99.0, -27.0, 0.0) | 37a49abff84e397a9207485a92a309c63f5c75dcbbbda231c14e52f1c5b8564a/PLATFORM-PLAT-X500_1.spmesh
PYLONS-X500:1 | 1172 tri | box 9.5 x 38.8 x 69.4 | c (35.0, -25.1, 0.0) | 230bd37b05bafeceeeeb0a301ea95f58390e71ff7693bf5437e259fc283a6a3e/PYLONS-X500_1.spmesh
PYLONS-X500:2 | 1172 tri | box 9.5 x 38.8 x 69.4 | c (-35.0, -25.1, 0.0) | d55a21a70f51eb268cc7a9c076f10d502e64d100372dcc1e67de2444296c49b5/PYLONS-X500_2.spmesh
```
