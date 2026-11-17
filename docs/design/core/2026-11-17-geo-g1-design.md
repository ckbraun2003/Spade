# Geo G1: geodesy, the world anchor and GNSS coordinates (L85, SP-1 to SP-3)

**Owner:** Core (SP-3 granted to Core by the lead, L85a; Physics reviews that diff). **Status:** DRAFT for the lead's review; design only, nothing is built. Kat's KT-1 and KT-2 wait on this note.
**Inputs:** KAT's `.agents/design/2026-11-17-geo-locations.md` (owner decisions D154 to D159, requirements REQ-G01 to G06, G17 to G19, checks V1 to V3a, "Host path"); `core/detail/math_ops_f64.hpp`, `world/world_file.{hpp,cpp}`, `sim/world_set.cpp` (config_hash), `sim/simulation.{hpp,cpp}` (ReplayConfig, restore), `sensors/gnss.hpp`; `core/plans/2026-10-02-module-api-design.md` (world file v3). Base: master `6b46075a`.
**Settled, not reopened:** the anchor is world (0, 0, 0); axes x east, y up, z south; double precision; no libm; no draw from the noise stream; the host rounds to int32 (KAT's job). Out of G1: the map, terrain and fetch (SP-4, SP-5), KAT's wire type and editor.

## 1. SP-1 geodesy: `core/geodesy.hpp`, `core/geodesy.cpp`

New header in `spade::geo`, INSTALLED (KAT's editor needs metres from typed coordinates, KT-4), built once in `spade_core` under `spade_fp_strict`, like `math_ops_f64.cpp`.

```
struct Geodetic { double lat_deg, lon_deg, height_m; };   // WGS84; height above the ellipsoid
struct Ecef     { double x, y, z; };                      // metres, earth-centred earth-fixed
struct LocalFrame;                                        // the anchor's ECEF origin and east, north, up unit vectors, built once
Ecef        geodetic_to_ecef(const Geodetic&);
Geodetic    ecef_to_geodetic(const Ecef&);
LocalFrame  make_local_frame(const Geodetic& anchor);
glm::dvec3  geodetic_to_world(const LocalFrame&, const Geodetic&);  // world metres: x east, y up, z south
Geodetic    world_to_geodetic(const LocalFrame&, glm::dvec3 world);
```

**The local plane.** World (x, y, z) is the tangent-plane point east = x, north = -z, up = y at the anchor. It maps to ECEF as `p0 + x e + (-z) n + y u`, with `p0` the anchor's ECEF point and (e, n, u) its unit vectors; the back conversion is the exact ECEF to geodetic below. So a point on the plane 1 km out reports a height about 8 cm above the ellipsoid, as Kat's design states; this is the model, not an error.

**Constants** (NGA TR8350.2): a = 6378137.0 m exactly, 1/f = 298.257223563. f, b = a(1 - f) and e^2 = f(2 - f) are computed with `+ - * /` only, so they are the same bits everywhere. The test pins b = 6356752.3142 and e^2 = 0.00669437999014 as published.

**The math, no libm.** Only `+ - * /`, `sqrt` and Spade's own kernels, every expression spelt left to right, no contraction (`spade_fp_strict`).
- **sqrt:** `std::sqrt`. IEEE 754 mandates a correctly rounded sqrt (sqrtsd on x86-64), so it is bit-exact on MSVC and gcc, it is not on the TD-3 canary's banned list, and `core/rng.hpp` and `gnss.hpp` already use it for this reason. The comment in the new file says so.
- **atan2:** `math::atan2_arith` (double, within 1 ulp; exists in `core/detail/math_ops_f64.hpp`). Longitude is `atan2(y, x)`; latitude is `atan2` of the Bowring pair.
- **sin and cos: new, `math::sincos_deg_arith(double deg, double& s, double& c)`** in `math_ops_f64.{hpp,cpp}`, the one home of Spade's transcendentals (TD-9). The angle arrives in degrees: reduce by whole quadrants (k = nearest integer of deg/90; `deg - 90 k` is exact for |deg| < 2^20, since 90 k is an integer and the ulp of deg is at most 1), convert the remainder to radians by a Dekker product with a pinned hi + lo pair for pi/180, evaluate sin and cos on |theta| <= pi/4 by Taylor series (sin to theta^27, cos to theta^26, Horner, fixed order), put the quadrant back. Within 2 ulp of std::sin and std::cos (tested against them in the kernel's test, as `atan2_arith` is in `test_mesh_field.cpp`); the geodesy needs about 1e-16 relative. Domain: |deg| <= 1e6, else NaN. Every caller validates its input first.
- **ECEF to geodetic:** Bowring's method with a FIXED three iterations (no data-dependent loop, so bit-reproducible). It needs no sin or cos: from p = sqrt(x^2 + y^2) the reduced-latitude pair (sin t, cos t) is `(z a, p b) / sqrt((z a)^2 + (p b)^2)` and phi = `atan2(z + e'^2 b sin^3 t, p - e^2 a cos^3 t)`; the next t comes from (1 - f) tan(phi) as a pair, again by one sqrt. Height is `p cos(phi) + z sin(phi) - a sqrt(1 - e^2 sin^2(phi))`, stable at every latitude. Degrees out are `rad * (180/pi)` with the pinned pair.

**Bit-equality is tested, not assumed.** A test pins the exact bit patterns (hex) of both directions at about 24 points (the four cardinal points, both poles' neighbourhood at 89 degrees, mid-latitudes in each hemisphere, heights -100, 0, 1286.5 and 9000 m). It runs on every leg; the Linux leg (gcc-13, the docker leg) failing to match MSVC is the proof of the claim. No tolerance on those checks.

**Tests (V1).**
- **Published points, within 1 mm.** The defining constants fix the cardinal points exactly: (0, 0, 0) gives (a, 0, 0); (0, 90, 0) gives (0, a, 0); (90, any, 0) gives (0, 0, b). Beyond those, two mid-latitude worked examples from a PUBLISHED table, each both ways. **I will not type numbers from memory. This needs one fetch (the lead's word):** the geographic to geocentric worked example in IOGP Geomatics Guidance Note 7-2 (EPSG), and NGA TR8350.2 for the constants. If either document does not give a WGS84 example, I will say so and use the documents' own ellipsoid for that vector, naming it in the test.
- **Round trip over 1 km, within 1e-9 deg:** a grid of world points from -1000 to +1000 m east and north at heights -50, 0 and 200 m, at six anchors (equator, 40.76 N 113.90 W as the Bonneville guess, 60 N, 80 N, 40 S, and a longitude on the antimeridian), world to geodetic to world and geodetic to world to geodetic. Limits: 1e-9 deg (about 0.1 mm) and, in metres, 1e-6 m.
- Kernel tests: `sincos_deg_arith` against `std::sin` and `std::cos` within 2 ulp over 100000 angles, exact at 0, 90, 180, 270 (s and c exactly 0 or +-1), and sin^2 + cos^2 within 2e-16.

## 2. SP-2 the world anchor

**Type and place.** `WorldDesc` (world/builder.hpp) gains `std::optional<geo::Geodetic> anchor`, empty by default. It is world configuration, one anchor per world (so per instance in a `WorldSetDesc`). `validate_world_desc()` is the one validation (file and builder share it): latitude in [-89, 89] (east is undefined at a pole, and the frame degenerates), longitude in [-180, 180], height finite and in [-10000, 100000] m; each refused by field name, `invalid_argument`. -0.0 is stored as +0.0 (`x + 0.0`) so the identity below is a function of the value.

**World file, and the version rule.** v2 is the current writer. The planned v3 (module-API design: required module set, regions, objects) is not built. The rule I propose:
- **v3 is a superset: v2 plus optional keys, `anchor` now and the module API's keys when they land.** A reader of version N reads every version from 1 to N.
- **The writer writes the lowest version that holds the world.** A world with no anchor (and no other v3 content) is written as `world_version: 2` with exactly today's bytes. A world with an anchor is written as `world_version: 3`. `kWorldFileVersion` becomes 3 (the highest this build writes), and a new `kWorldFileLowestWriteVersion`-style helper in `world_file.cpp` picks the number; it is not a second constant a caller must keep in step.
- **An older reader refuses a geo world file by name.** A v2-only build gets `schema_mismatch` ("world_version 3 is not read by this build"), the failure that says "upgrade", and an `anchor:` key in a file that says `world_version: 2` is an unknown-key error, as every key a version does not know is. No silent drop (the file's own rule).
- **Composition with the module API:** its v3 adds its keys to the same schema, also optional, so no v4 and no migration. Whichever lands second adds to the v3 text; the "lowest version" rule needs no change.

**The text.** A top-level mapping, only when set:
```
anchor:
  lat_deg: 40.76
  lon_deg: -113.9
  height_m: 1286.5
```
Doubles are written with `std::to_chars` at 17 significant digits and read with `std::from_chars` (the file's locale-free rule, as its floats at 9), so a world round-trips bit for bit. Where the block sits among the keys is fixed at build time and pinned by a text test. The anchor-less output is the existing text, byte for byte (the existing world-file goldens are that proof).

**Identity (the L82c pattern: only when set).**
- `config_hash` (sim/world_set.cpp) folds the anchor's three doubles after the regions, inside `if (instance.world.anchor)`. A world without one folds exactly what it always did, so every committed hash, digest and snapshot of an anchor-less world is unchanged.
- The refusal by name needs the blob to carry something, because `config_hash` cannot say what differs. `ReplayConfig::_reserved0` (the registered per-world row's growth slot; layout, size, offsets and walk position unchanged) becomes `anchor_identity`: 0 when the world has no anchor, else `geo::anchor_identity(anchor)`, an FNV-1a over the three doubles' bytes under the domain tag "geo.anchor", never 0. Anchor-less snapshots carry the 0 they carry today, so they stay byte-identical. A blob taken under an anchor and restored into an anchor-less simulation (or the reverse, or another anchor) differs in that lane.
- **The refusal:** `check_replay_config` already compares the payload and names the diverged fields. It gains one: **`Code::invalid_argument`**, the code of every restore-config refusal, with context `restore: world anchor differs (blob 0x<id>, this simulation 0x<id>; 0 means no anchor)`. The row index is named when it is not world 0. The identity is a hash, so it names the fact, not the two coordinates; the message says so. An old build restoring an anchored blob refuses too (the lane is non-zero), which is the safe direction.
- A reset keeps the anchor (it is configuration; the replay row is written once at `create()`); another world means another `WorldDesc`, which drops it.

**Access.** `Simulation::world_anchor(uint32_t world_index) -> Result<std::optional<geo::Geodetic>>` (`not_found` for a bad index). `create()` builds each world's `LocalFrame` once, kept in the Simulation (never in state).

**Tests.** Anchor round trip through `world_to_yaml` and `world_from_yaml` bit for bit (awkward values: 40.76, -113.9, 1286.5, 1e-300 refused by range, -0.0); an anchor-less world's text and `config_hash` equal today's (the committed pins); anchor set changes `config_hash`, two anchors differ; v2 text with `anchor:` refused as unknown key naming schema v2; v3 text without an anchor loads; each range refusal named; snapshot restore: same anchor accepted, another anchor refused with the text above, anchor to none and none to anchor refused; a restore of an anchor-less blob into an anchor-less world is byte-identical to today (the existing snapshot goldens and `SnapshotRoundTrip` tests, unedited).

## 3. SP-3 GNSS coordinates

**Contract.** The GNSS ring is unchanged: `GnssFix` keeps its 48 bytes, fp32 position, and its Slang twin is not touched (this is why the conversion is at poll time and not in the pass: a double does not fit the fix, and the GPU path would need a double pass). The fix's `position` is the stored noisy position; the coordinates are computed from that exact float widened to double, so the metres and the coordinates of one fix agree (REQ-G19) to fp32's resolution (0.06 mm at 1 km).

```
Result<GnssPoll> Simulation::poll_gnss_geodetic(GnssSensorRef ref, sensors::SampleIndex since_index,
                                                std::span<sensors::GnssFix> out,
                                                std::span<geo::Geodetic> geodetic) const;
```
Same ring semantics as `poll_gnss`; `geodetic[i]` is the WGS84 latitude and longitude in degrees and the ellipsoid height in metres of `out[i]`, for the n fixes returned. `invalid_argument` when `geodetic.size() < out.size()`; **`unavailable`** ("poll_gnss_geodetic: this world has no anchor") when it has none, so a plain scene can never mistake zeros for coordinates (Kat's `has_coords` is 0 there). `poll_gnss` is untouched. The call is `const` and reads the ring only: it draws nothing from the noise stream, and a test compares the receiver's `noise` stream bytes before and after.

**Tests (V2).** Noise off (sigma 0, no bias): a body at world (0, 0, 0) reports the anchor within 1e-9 deg and 1e-6 m. A body at 100 m east and 100 m north (world x = 100, z = -100, y = 0) reports the coordinates from the first-order formulas lat = phi0 + N/(M + h), lon = lam0 + E/((N + h) cos phi0) (M and N the meridian and prime-vertical radii, written independently in the test) within 1 cm, and a height 1.6 mm above the anchor's (the plane's curvature). The noisy case: with noise on, `geodetic_to_world(geodetic[i])` returns `out[i].position` within 1e-6 m (same noisy position), and the golden digest of a noisy anchor-less scenario is unchanged (no draw). `poll_gnss` and `poll_gnss_geodetic` return the same fixes byte for byte.

**`gnss.hpp` §1 is rewritten when this lands:** the fix stays in the world frame; with an anchor, `poll_gnss_geodetic` adds WGS84 coordinates from that position; the local plane is 8 cm per km off the ellipsoid; the datum is the world's anchor, no longer "the consumer's".

## 4. What Kat gets

- `include/core/geodesy.hpp`: metres to coordinates and back for the editor (KT-4) and for placing the anchor.
- A world file with an `anchor:` key (v3) through the existing `load_world_file` and `world_from_yaml`; KAT's `tools/world_refs.h` accepts the key on its side.
- `Simulation::world_anchor()` and `poll_gnss_geodetic()` (doubles: degrees, degrees, metres). The int32 rounding stays one function in the host (REQ-G01c).
- Refused by name: a restore under another anchor (code above), a geo world file in an older build, a bad latitude, longitude or height.
- Kat's batch API runs separate Simulations and passes each its own `WorldDesc`, so each carries its own anchor with no extra call.

## 5. Order, size and gate

One branch, `core/l85-geo-g1`, three RED and GREEN pairs in this order (each pair on its own commits): SP-1 (kernel + geodesy + tests), SP-2 (anchor, file, identity, restore), SP-3 (poll, gnss.hpp §1). Test files named in READY: `tests/test_geodesy.cpp`, `tests/test_world_anchor.cpp`, `tests/test_gnss_geodetic.cpp`; a CMake line for each and one for `core/geodesy.cpp` (the grant for new tests applies; the engine CMake and the install rules are mine to edit). Gate: the release CPU suite; the debug build; gcc-check on each changed file; the Linux (docker) leg, which is what proves the bit pins; no GPU change, so no GPU leg beyond the existing parity set. The Physics review covers `sensors/gnss.*` and the `gnss.hpp` rewrite only. A second Spade bump for Kat follows.

## 6. Questions for the lead

1. **One fetch**, the vectors' source (section 1): IOGP GN 7-2 and NGA TR8350.2, read-only, nothing cached into the tree.
2. **The version rule** (section 2: v3 superset, lowest version written, anchor-less stays v2 byte for byte). It also fixes how the module API's v3 lands; the lead decides whether that is a law-level change.
3. **Latitude limit +-89** and the height range: change them if Kat's places need more.
4. **Scenes.** A Spade scene file names its world by path and hash, so an anchor written into the world file reaches the scene with it. If the editor wants to own the anchor in the scene, without touching the world file, that is a scene version (v7) and an Interface and Core item; G1 does not need it.
