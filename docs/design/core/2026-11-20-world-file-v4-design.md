# World file v4: the camera capacities and object labels (Phase 3.4)

**Owner:** Core. **Status:** DRAFT for the lead's review; design only, nothing is built. **Inputs:** `../rendering/2026-11-19-camera-sensor-design.md` (section 6 labels, section 11 items 2 and 6, section 11a the keypoint capacities); the merged Phase 3.1 and 3.1b (`Capacities::cameras`, `camera_pixels`, `keypoint_groups`, `keypoints_per_group`); `engine/world/world_file.hpp` (the version rules).
**Settled, not reopened:** Core owns the world-file keys, Rendering owns what a label and a material mean (`RS5`); a label is a `uint16`, 0 means "no labelled object", and Spade attaches no meaning to the number; two nodes or props with one label are one object; the lead's ruling that `world_to_yaml` refuses a camera world until one v4 schema carries cameras together with 3.4.

## 1. What is already there, and what is missing

The world file reads versions 1 to 3 and **writes the lowest version that holds the world** (`world_file.hpp`): a world with no v3 content is version 2 byte for byte, a world with an anchor is version 3. v3 is v2 plus optional keys, and an older build refuses a newer file as `schema_mismatch`, never by dropping a key.

- **Materials are already in the file** (schema v2): the `materials` palette, `sdf.node_materials` (a per-node index, parallel to `nodes`, empty means all `materials[0]`), `props[].material`, and `lighting`. Section 11 item 6 asks for "material resolution as the render scene does it today": that is what the file carries. **No new material key is needed**, and this note adds none.
- **Missing, and the point of v4:** (a) the four camera capacities, which `world_to_yaml` now refuses to write; (b) a per-object label for statics (SDF nodes) and props.

## 2. The v4 content (all optional; a world without any of it is written exactly as before)

1. **Capacities, four optional keys:** `capacities.cameras`, `camera_pixels`, `keypoint_groups`, `keypoints_per_group`. Written only when non-zero (so a camera-free world's `capacities:` block is unchanged); on read an absent key is 0 and a key stated as 0 is accepted. In v2 and v3 files these keys stay unknown, hence an error, as every unknown key is. The pair rules are already `validate_world_desc`'s (they run on a file as on a builder).
2. **Node labels:** `sdf.node_labels`, a list of integers 0..65535, parallel to `nodes`, **same lockstep rule as `node_materials`**: empty (every node unlabelled) or exactly `nodes.size()` long; operator nodes carry whatever the builder last set, and a consumer that labels by leaf is free to ignore them. In code: `SdfProgram::node_labels` (`std::vector<uint16_t>`), `WorldBuilder::label_for_last_node(uint16_t)`, validated for length by `SdfProgram::validate()`. Written only when any label is non-zero.
3. **Prop labels:** `props[].label`, an integer 0..65535, default 0, written only when non-zero (`PropDesc::label`, `WorldBuilder::prop(..., material, label)`).
4. **Version selection.** The writer writes version 4 only when the world has a camera capacity or a non-zero label; otherwise exactly what it writes today (2 or 3). A world with an anchor and labels is 4. `kWorldFileVersion` becomes 4; the file header text says "or 4 for a world with cameras or labels". The refusal added in Phase 3.1 is **lifted by this**: `world_to_yaml` no longer refuses a camera world, it writes it as v4, and its test turns into a round trip.
5. **A v4 key in a v2 or v3 document is an error:** `label`, `node_labels` and the four capacity keys are unknown keys in a v2 or v3 document, so a hand-edited file cannot half-upgrade.

## 3. Semantics (Core's half; the rest is Rendering's)

- **Render-only, like materials:** labels are not in `config_hash`, no pass reads them, no snapshot carries them; two worlds that differ only in labels replay a restored blob identically. The capacity keys **are** in `config_hash`, only when used (as merged in 3.1 and 3.1b). The `WorldDesc` size pins move (the `SdfProgram` gains one vector: 24 bytes in the release shape, 32 with iterator debugging) and the exclusion is written in `world_set.hpp`'s "does not cover" list with its reason.
- **Range:** a label is a `uint16`, so the file's integers are checked 0..65535 and a larger one is refused naming the key and the index.
- **Bodies are not in the file.** A dynamic body's label, and keypoints on any object, travel through the API (3.3, the C ABI) as KAT supplies them as data (`2026-11-19-camera-sensor-design.md` section 6). Per-object keypoints in the file are **not** in v4 (YAGNI: no consumer authors them in a file); they would be one more optional key in a later version.
- **Appearance sets (3.5)** are a separate, render-only block with Rendering's 6a; they ride a version of their own or join v4 only if Rendering's 6a lands first (decided then, not here).

## 4. What a v3 reader does with a v4 file, and what KAT changes

- **A v3 build** reads `world_version: 4` and refuses it as `schema_mismatch`: "world file is schema version 4, this build reads versions 1-3 only", the same loud upgrade path as the v3 anchor. It never drops the cameras or the labels.
- **Nothing changes for an existing file.** A world with no camera capacity and no label keeps its bytes, its version (2 or 3), its hash and its goldens; KAT's pinned world files and `config_hash` expectations hold.
- **KAT has to:** (a) rebuild against the installed headers when it takes this (`Capacities` already grew in 3.1 and 3.1b, and `SdfProgram` and `PropDesc` grow here: `sizeof(WorldDesc)` changes in every consumer build, as the geo anchor's did); (b) accept `world_version` 4 wherever it pins a version at most 3 (the library's `peek.world_version` for a world will say 4 for a camera or labelled world); (c) write its gate labels (its track index plus one) as `label:` on the gate props or `node_labels` entries when it authors a world file, or set them through the API; a camera world authored with `capacities.cameras` and `camera_pixels` is version 4.

## 5. Tests and size

RED first, own test file `tests/test_world_file_v4.cpp`: a camera world round-trips and is written as version 4; a label-only world is version 4; a plain world is version 2 byte for byte and an anchor-only world version 3 (goldens unchanged); canonical re-emission; the optional capacity keys (absent, 0, non-zero); `node_labels` length and range refusals; `label` range refusal; a v3-labelled document with a v4 key is refused naming the key; `world_version: 4` with nothing v4 in it loads (the lowest-version rule is the writer's, not a requirement on a reader); labels excluded from `config_hash`, capacity keys folded; the Phase 3.1 refusal test becomes the camera round trip. Size **S to M** (one branch, `core/world-v4`).

## 6. Questions for the lead

1. Are KAT's gates props (a mesh) or SDF nodes? v4 labels both, so the answer changes nothing built, only which authoring call KAT is told to use.
2. Is "per-object keypoints stay API-only for v4" acceptable, or does Rendering want them in the file?
