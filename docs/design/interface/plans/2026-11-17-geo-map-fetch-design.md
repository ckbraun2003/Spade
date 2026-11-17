# SP-5: the map fetch and its local cache (L85, geo G2), DRAFT design note, for review

Interface, 2026-11-17, for the lead. Source: KAT's design `.agents/design/2026-11-17-geo-locations.md` (owner decisions D154 to D159, REQ-G07 to G16, checks V4 and V5). Nothing is built or queued. Every service parameter below is **UNVERIFIED** until one live check (needs the lead's word; section 8).

## 1. Shape of it

Two parts, split so that the engine never sees the network (L85).

| Part | Where | Does |
|---|---|---|
| **Cache reader** | engine side, with Rendering's terrain loader (SP-4) | Reads an entry from the cache directory. A missing or incomplete entry comes back as a named result (`map_missing`, with the area's key and the directory it looked in). No network code, no writes. About 100 lines plus the georef parse. |
| **Fetch tool** | `tools/geo_fetch/` (outside `engine/`), a static library `spade_geo_fetch_lib` and the executable `spade_geo_fetch` | Checks the cache, and on a miss asks the three services, converts, writes the entry, enforces the cache's size limit. The only code in Spade that opens a socket. |

A host or the editor runs `spade_geo_fetch` as a child process on demand and reads its progress lines (section 3). The tool is built with the sandbox target group and installed beside the editor; it is not in `spade_core` or any installed engine library. Tests call the library with a fake transport.

Who writes the reader: it is Rendering's file by SP-4's ownership. I write the layout and the georef spec (section 4) and the tool. If the lead prefers, I write the reader too: it is the same parse.

## 2. Command line

```
spade_geo_fetch --lat <deg> --lon <deg> [options]
```

| Option | Default | Meaning |
|---|---|---|
| `--lat`, `--lon` | required | The anchor, WGS84 degrees (lat north positive, lon east positive: Bonneville is `40.76 -113.90`). Range checked. |
| `--side <m>` | 1000 | Side of the square area, centred on the anchor. At most 1000 (REQ-G07) and at least 10: a refusal by name (`side_too_large`, `side_too_small`), nothing fetched. |
| `--cache <dir>` | see section 4 | Cache root. Refused if it is inside a git work tree (section 4). |
| `--cache-limit-mb <n>` | 512 | Size limit for the whole cache (section 4). |
| `--dem-cell <m>` | 1 | Elevation grid cell. The grid is `ceil(side / cell) + 1` samples a side, at most 1025. |
| `--image-cell <m>` | 0.6 | Image pixel. At most 4096 pixels a side. |
| `--check` | off | Cache only, never the network: reports `hit` or `miss` for the area and exits. This is what an engine host runs to ask "is it there". |
| `--refresh` | off | Fetch again even on a hit, then replace the entry. |
| `--timeout-s <n>` | 30 | Per request, after connect (connect itself 10). |
| `--json` | off | Progress and result as JSON lines on stdout (section 3). Without it, plain text. |

**Result and exit code.** The last line of stdout always names the result; the exit code carries it.

| Result | Exit | Meaning |
|---|---|---|
| `hit` | 0 | The entry was in the cache; no request was made. |
| `fetched` | 0 | Fetched and stored. |
| `miss` | 3 | `--check` only: not in the cache. |
| `offline` | 10 | No network: DNS, connect or TLS setup failed before any service answered (or the machine has no HTTP library, section 6). The message says which. Nothing is written. |
| `service_error` | 11 | A service answered and the answer is no use. Names the service (`3dep`, `naip`, `ngs`), the HTTP status or the reason (wrong size, no-data samples, a compressed TIFF, no geoid height in the JSON). Nothing is written. |
| `refused` | 12 | The request itself: side out of range, coordinates out of range, cache inside a repository. |
| `cache_full` | 13 | One entry alone is bigger than the limit. |
| `usage` | 2 | Bad arguments. |

A failed fetch leaves no partial entry (section 4). "Offline" is only a connection failure at the first request, so a machine that reaches the network but not a service is `service_error`, not `offline`. The tool does not retry an offline result. A timeout or a 5xx is retried once after 2 s.

**Progress.** Three stages in a fixed order, `geoid`, `elevation`, `image`, then `store`. The geoid goes first because it is one tiny request: an offline machine finds out in under 10 s. Plain text, one line per event, for a person:

```
geoid      asking NOAA NGS ... N = -23.412 m (GEOID18)
elevation  USGS 3DEP 1001 x 1001 ... 2.1 of 4.0 MB
image      USGS NAIP 1667 x 1667 ... 3.0 MB
store      /home/x/.cache/spade/maps/v1/n40760000_w113900000_s1000 (7.4 MB)
fetched    entry n40760000_w113900000_s1000
```

With `--json`, one object per line, so KAT's editor can draw a progress bar without parsing prose:

```
{"event":"stage","stage":"elevation","state":"start"}
{"event":"progress","stage":"elevation","bytes":2200000,"total":4200000}
{"event":"stage","stage":"elevation","state":"done"}
{"event":"result","result":"fetched","entry":"n40760000_w113900000_s1000","dir":"...","bytes":7400000,"geoid_height_m":-23.412,"anchor_ellipsoid_height_m":1262.7}
```

`total` is absent when the service sends no length. `progress` is throttled to one per 100 ms and one per stage end. The editor turns `result` into its message: `offline` and `service_error` carry a `message` it can show as is. The `result` line also carries `anchor_ellipsoid_height_m` (section 5), which is what the editor puts in the scene's anchor.

## 3. What one run asks, in order

1. `--check` or a look in the cache: a complete entry is a `hit`. Done.
2. Geoid height at the anchor (NGS).
3. Elevation grid (3DEP).
4. Image (NAIP).
5. Convert, write into a temporary directory in the cache, rename into place, then trim the cache.

The three downloads are independent; the tool does them one after the other (one HTTP handle, simple progress). 5 to 15 MB in all: seconds on a normal link.

## 4. The cache

**Where.** Never in the repository. In order: `--cache`, the environment variable `SPADE_MAP_CACHE`, then the platform's user cache: `%LOCALAPPDATA%\Spade\maps` on Windows, `$XDG_CACHE_HOME/spade/maps` (else `~/.cache/spade/maps`) on Linux. **Guard:** the tool refuses (`refused`, "the cache is inside a git work tree") any directory that has a `.git` in it or in any parent. A test pins it. The reader takes the same three-step lookup, so a host that passes no path finds what the tool wrote.

**Layout** (`v1` is the layout version; a change to the layout makes `v2` beside it, never edits `v1`):

```
<root>/v1/<key>/georef.yaml   the georeference, written last: its presence means "complete"
<root>/v1/<key>/height.f32    the elevation grid
<root>/v1/<key>/image.png     the image
```

**The key** names the area: the anchor rounded to 1e-6 degrees (0.11 m) and the side in metres, as signed integers with the hemisphere as a letter: `n40760000_w113900000_s1000`. Two anchors within 0.11 m share an entry, which is right because the georef, not the key, carries the geometry. The reader computes the same key from the scene's anchor and side.

**`height.f32`.** Raw little-endian IEEE float32, `width * height` samples, row-major, **row 0 is the north edge, column 0 the west edge**. A sample is the height of the middle of its cell, in **metres above the WGS84 ellipsoid** (the NAVD88 step is done by the tool, section 5, so the engine knows no datum). No file header: the size is in the georef. A no-data sample in the service's answer is a `service_error`, not a hole in the file: the entry never holds a NaN.

**`image.png`.** The service's PNG, stored as received (8-bit RGB, no alpha). Same row and column order as the grid, so image pixel `(i, j)` and a point on the ground map by the same georef. The decoder is SP-4's choice; the file is plain PNG, and `miniz` (already vendored for the 3MF reader) inflates it, so no new dependency is needed.

**`georef.yaml`** (version 1; the file is YAML like the world and scene files, written by the tool only):

```yaml
georef_version: 1
key: n40760000_w113900000_s1000
anchor: { lat_deg: 40.76, lon_deg: -113.9 }     # as asked, 9 decimals
side_m: 1000
bbox_deg: { west: ..., south: ..., east: ..., north: ... }   # the area both layers cover
elevation:
  file: height.f32
  width: 1001
  height: 1001
  cell_m: 1.0
  vertical: wgs84_ellipsoid_m
image:
  file: image.png
  width: 1667
  height: 1667
  pixel_m: 0.6
geoid:
  model: GEOID18
  height_m: -23.412        # N at the anchor; ellipsoid height = NAVD88 height + N
anchor_ground:
  ellipsoid_height_m: 1262.7   # the elevation at the anchor (bilinear) + N
sources:
  elevation: USGS 3DEP
  image: USGS NAIP
  geoid: NOAA NGS
fetched_utc: 2026-11-17T01:23:45Z
```

**What the pixel grid means.** Both layers are pixel-is-area: sample `(i, j)` is the middle of its cell, so its centre is at `west + (j + 0.5) * (east - west) / width` degrees of longitude and `north - (i + 0.5) * (north - south) / height` of latitude. `bbox_deg` is where the layer's outer edges are. The layers are regular in longitude and latitude (EPSG:4326), which at 1 km is regular in the local east and north to under a millimetre. The two layers have different sizes; both cover the same `bbox_deg`. To place a sample on the scene's plane the reader uses SP-1 (ENU at the anchor), so the tool and the engine share one geodesy: the tool computes `bbox_deg` from the anchor and `side_m` with SP-1's function (north and east offsets of `side/2` to latitude and longitude). Until SP-1 lands the tool uses the WGS84 radii of curvature in six lines of its own and a test pins both to agree within 1 mm.

**Atomic and incomplete entries.** The tool writes into `<root>/v1/.tmp-<pid>-<key>/`, then renames the directory to `<key>`. A directory with no `georef.yaml` is a miss and is deleted by the next fetch of that key. Two fetches of the same area at once: the second rename fails, the second tool deletes its temp directory and reports `hit`.

**Size limit.** `--cache-limit-mb`, 512 by default: about 40 full areas (one is about 7 to 14 MB). After a successful store the tool trims: while the total of all entries is over the limit, delete the **oldest** entry by `fetched_utc` (never the one just written, never a temp directory). One entry alone over the limit is `cache_full` and is not stored. The reader never touches the cache, so a read does not refresh anything; "oldest" means oldest fetched. (An LRU that the engine updates would make the engine write, which L85 rules out.)

**Never committed.** The root is outside the repository by the guard above, and the repo's `.gitignore` is not asked to carry a map pattern.

## 5. Heights: NAVD88 to the ellipsoid

3DEP heights are orthometric, NAVD88 in the conterminous US (**to be confirmed against the service's metadata in the live check**). GNSS heights are above the ellipsoid. The relation is

```
h (ellipsoid) = H (NAVD88) + N
```

with `N` the geoid height from NOAA NGS at the anchor (GEOID18; over the US it is negative, tens of metres). Over 1 km `N` changes by under 1 cm (KAT's design), so one value serves the whole area: the tool adds `N` to every elevation sample before writing `height.f32`, stores `N` in the georef, and reports the anchor's ground height `anchor_ellipsoid_height_m` = the grid sampled at the anchor (bilinear) + N. The scene's anchor height is that value.

**Flagged, an accepted error the lead may want written into the design:** GEOID18's ellipsoid is NAD83(2011), not WGS84. The two ellipsoid heights differ by up to about 1 to 2 m over the US, a smooth offset that cannot be told from a constant over 1 km. I would do nothing about it: it moves the whole map, not the ground relative to the drone, and the anchor height comes from the same map, so a drone above the map sees consistent heights. A GNSS height checked against a surveyed WGS84 point would be off by that offset. I record it and add no correction.

## 6. The HTTP layer (a tester's PC has no Python)

One small interface, `HttpClient::get(url, timeout) -> { status, bytes, error }`, with `error` one of `none`, `offline`, `timeout`, `tls`, `too_large`. Everything above it is the same on both systems and is what the tests drive through a fake. Responses are capped at 64 MB. Redirects are followed (3DEP and NAIP redirect). Certificates are always verified; there is no insecure option. The User-Agent is `spade-geo-fetch/<version>`, with no name, host or path.

| System | Implementation | New dependency |
|---|---|---|
| **Windows** | **WinHTTP** (`winhttp.dll`, in every Windows 10 and 11). TLS by the OS (Schannel) with TLS 1.2 or newer, the system's certificate store and its proxy settings (`WINHTTP_ACCESS_TYPE_AUTOMATIC_PROXY`). | None; links `winhttp.lib` from the SDK, in the tool only. |
| **Linux** | **libcurl, loaded at run time** with `dlopen("libcurl.so.4")` and the six stable functions the tool needs (`curl_easy_init`, `_setopt`, `_perform`, `_getinfo`, `_cleanup`, `curl_global_init`). The option numbers are declared in the tool (libcurl's ABI keeps them). Nothing links to it, so a machine without libcurl still runs the tool, and the missing library is the named `offline` result ("no HTTP library: install libcurl4"). Proxy and CA bundle come from libcurl and the environment. | None at build time (no curl headers needed, so the Docker leg's image is untouched); needs libcurl on the PC, which most distributions ship. |

I considered fetching curl with CMake or writing TLS ourselves and rejected both: a vendored curl brings a TLS library with it, and hand-rolled TLS is not a thing to ship. Shelling out to a `curl` binary was rejected as fragile (no progress, argument quoting, an absent binary). **Decision for the lead:** the Linux dlopen route needs a sign-off because it is the one place Spade would depend on a system library at run time. The Windows route needs none.

The Linux file is compiled in the Docker leg (gcc-check) but not exercised by a test, since no test touches the network; it is exercised by the live check run on Linux, if the lead wants that run.

## 7. The services' exact request parameters (UNVERIFIED)

Written from what I know of these ArcGIS ImageServers and the NGS API; nothing here has been run. Each line is checked by the live check (section 8) and corrected in the note and in a test if wrong.

All coordinates are lon/lat (EPSG:4326); `W,S,E,N` are `bbox_deg`.

**USGS 3DEP elevation**, an ArcGIS ImageServer:

```
GET https://elevation.nationalmap.gov/arcgis/rest/services/3DEPElevation/ImageServer/exportImage
    ?bbox=W,S,E,N &bboxSR=4326 &imageSR=4326 &size=<cols>,<rows>
    &format=tiff &pixelType=F32 &interpolation=RSP_BilinearInterpolation
    &noData=-9999 &f=image
```

Answer expected: a single-band float32 TIFF of `size` pixels. **Biggest unknown:** whether the TIFF is uncompressed. The tool reads baseline little-endian TIFF with one strip or several, compression 1 (none), and 8 (Deflate, via miniz) if the live check shows the service compresses; any other compression is `service_error` naming it. The tool writes `height.f32` itself, so the engine never parses TIFF. Whether `bbox` and `size` land on the cell edges as assumed in section 4 (pixel-is-area) is the second thing the live check pins, by fetching a 200 m area twice with the bbox shifted by one cell.

**USGS NAIP imagery**, an ArcGIS ImageServer:

```
GET https://imagery.nationalmap.gov/arcgis/rest/services/USGSNAIPImagery/ImageServer/exportImage
    ?bbox=W,S,E,N &bboxSR=4326 &imageSR=4326 &size=<cols>,<rows>
    &format=png &interpolation=RSP_BilinearInterpolation &bandIds=0,1,2 &f=image
```

Answer expected: an 8-bit RGB PNG of `size` pixels. The tool checks the PNG signature and the IHDR width and height against the request, and rejects a 4-band or 16-bit answer by name. NAIP is a mosaic of the newest imagery; the service does not say which year, so the georef carries no capture date (a limit, written in the user's note).

**NOAA NGS geoid height**, the NGS geodetic toolkit's web service:

```
GET https://geodesy.noaa.gov/api/geoid/ght?lat=<deg>&lon=<deg>&model=18
```

Answer expected: JSON with a number `geoidHeight` in metres and the model name. **Unverified:** the sign convention for `lon` (the toolkit may want west as negative, as sent, or as positive degrees west) and the exact JSON key. The live check for Bonneville expects `N` between -35 and -10 m (an unambiguous wrong-sign catch: the wrong longitude sign puts the point in China, where N is about +20 m or more of the opposite sign) and the tool refuses an answer whose model is not GEOID18.

Sources outside these three are not in the tool, and a guard test lists every host string in `tools/geo_fetch/` and fails on any other (REQ-G08, REQ-G09). The three services are US-government public data: no logo or credit line is asked.

## 8. Tests (synthetic fixtures only)

All through the fake `HttpClient`; no test touches the network or the user's real cache (each makes its own temp cache under `tests/unique_temp.hpp`).

| Test | Pins |
|---|---|
| hit | a ready entry: result `hit`, **zero** requests (REQ-G12) |
| miss then fetch | the fake serves a 4x4 float32 TIFF, an 8x8 PNG and a geoid JSON: `fetched`, the files exist, `georef.yaml` has the values, `height.f32` holds `H + N` (checked sample by sample), and a second run is a `hit` with no request |
| offline | every request fails with `offline`: result `offline`, exit 10, **no** directory left, the geoid stage was the one that failed |
| service errors | HTTP 500 (retried once, then `service_error`), a PNG whose IHDR size is wrong, a TIFF with a no-data sample, a compressed TIFF, a geoid JSON without the number: each its own named result and nothing stored |
| refusals | side 1001, side 9, lat 91, a cache inside a git work tree |
| cache limit | a tiny limit evicts the oldest entry but never the new one; one entry bigger than the limit is `cache_full` |
| incomplete entry | a directory with no `georef.yaml` is a miss and is replaced |
| `--check` | a miss with no requests, exit 3 |
| progress | the JSON lines parse, the stages come in order, the last line is the result |
| determinism | the same fake inputs give byte-equal `height.f32`, `image.png` and `georef.yaml` (apart from `fetched_utc`, which the test pins through an injected clock) |
| host guard | the list of host strings in the tool is exactly the three |
| bbox | `bbox_deg` from the SP-1 function and from the tool's own formula agree within 1 mm (when SP-1 is in) |

The two pieces that touch the OS (WinHTTP, libcurl) have no unit test: a thin file each, compiled by the build and by gcc-check.

**The live check.** One command (`spade_geo_fetch --lat 40.76 --lon -113.90 --side 1000 --cache <temp dir>`, run by `scripts/geo-live-check.ps1`), never in ctest, never in a gate, and run only when the lead says so for each run (L85). It does V4's check: the image is the size asked, the elevation spread over the flats is under 1 m, the geoid height is in range. It also checks the TIFF compression, the shifted-bbox cell edge, the NAIP band count and the NGS sign in section 7, and writes what it found into this note.

## 9. Order of work and what I need

1. This note reviewed. Rendering (SP-4) agrees the layout and georef (sections 4 and 5): it is their reader.
2. The library with the fake transport and the tests above (RED, then GREEN), the CLI and progress.
3. The two HTTP files.
4. The live check, when the lead says.

Dependencies and questions for the lead:

- **Q1.** SP-1's geodesy library: who owns it and when does it land? I need one function (offsets to degrees) and use my own six lines until then.
- **Q2.** The Linux libcurl `dlopen` route (section 6): approved?
- **Q3.** The NAD83(2011) against WGS84 offset (section 5): accepted and recorded, or does the owner want it corrected?
- **Q4.** Where does `spade_geo_fetch` install: beside the sandbox executable. Packaging is Interface's; I take that as yes unless told otherwise.
- **Q5.** No new third-party library, so `THIRD_PARTY_NOTICES.md` is unchanged. `miniz` (already there) inflates the PNG and a Deflate TIFF.

Size: library and tests 2.5 days, HTTP files 1, CLI and progress 0.5, scripts and gates 0.5, review 1: about 5 working days, plus the live check.
