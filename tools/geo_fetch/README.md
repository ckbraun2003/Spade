# spade_geo_fetch

Fetches a real-world map area into Spade's local map cache, for a scene that sits at a real place (KAT's geo plan, G2). The design is `docs/design/interface/plans/2026-11-17-geo-map-fetch-design.md`.

The engine never touches the network. This tool is the only program that does, and only when you run it (or the editor runs it for you). It asks three US public-domain services and nothing else: USGS 3DEP (elevation), USGS NAIP (aerial image) and NOAA NGS (geoid height at the anchor). None of them asks for a logo or a credit line.

```
spade_geo_fetch --lat 40.76 --lon -113.90 --side 1000
```

An area is a square, at most 1000 m a side, centred on the anchor; its edges are the points half a side east, west, north and south of the anchor on the anchor's local plane, from `core/geodesy.hpp` (so the anchor must be within 89 degrees of the equator, and an area across the 180th meridian is refused). The result is on the last line of the output and in the exit code: `hit` or `fetched` (0), `miss` (3, with `--check`), `offline` (10), `service_error` (11), `refused` (12), `cache_full` (13). `--json` prints one JSON object a line, so an editor can show progress. `--help` lists the rest.

**Where it is.** The program is `spade_geo_fetch` (`spade_geo_fetch.exe`): built with the rest (target `spade_geo_fetch`), and installed in the Spade Editor's folder beside `spade_sandbox.exe` (`scripts/package.ps1` refuses a package without it, `scripts/package-test.ps1` starts it from the installed folder). It is not in the SDK install. The editor reads what it stores with `read_terrain_map()` (`world/terrain_map.hpp`, in `spade::mesh_import`).

**Where the cache is.** Outside the repository: `--cache <dir>`, else `SPADE_MAP_CACHE`, else `%LOCALAPPDATA%\Spade\maps`. The tool refuses a directory inside a git work tree, so a map is never committed. The cache is trimmed to 512 MB by default (`--cache-limit-mb`), oldest fetch first.

**No network.** With no connection the tool says `offline` and stores nothing; a scene that finds no map in the cache loads with plain ground and the editor shows why. Only Windows has an HTTP layer so far (WinHTTP, which is part of Windows). On any other platform the result is `offline: no HTTP layer on this platform yet`.

**Heights, and an accepted error.** 3DEP heights are above the US vertical datum (NAVD88). The tool adds the geoid height NOAA NGS gives at the anchor (GEOID18) to every sample, so the map holds heights above the ellipsoid, as a GNSS receiver reports them. GEOID18's ellipsoid is NAD83(2011), not WGS84. The two differ by about 1 to 2 m over the US, and the services' coordinates are NAD83 underneath, so the image can also sit 1 to 2 m off WGS84. This is recorded in each entry's `georef.yaml` (`datum_note`) and not corrected. It moves the whole map together, so a drone sees consistent heights over it, but a GNSS height compared with a surveyed WGS84 point can be off by that much.

**Where it works.** The conterminous United States, and nowhere else: a box of 24 to 58 N and 130 to 60 W (the extent NGS lists for its companion DEFLEC18 model, standing in for GEOID18's own grid, whose page I did not find). Outside it the tool refuses by name before any request (exit 12): Hawaii and the Pacific territories (elevations on local mean sea level), Alaska, and Puerto Rico and the US Virgin Islands (GEOID18 covers them, but their elevations are on PRVD02 and VIVD09). The limit that remains: the check is a box, so a point in Canada, Mexico or the Bahamas inside it passes it, and 3DEP's answer of no data there refuses the request after it is made. `--help` says the same.

**What is read.** The elevation answer is an uncompressed float32 TIFF, in strips or in tiles (an ArcGIS ImageServer sends tiles), read to its samples (anything else, a compressed one included, is refused by name); the image is read through to its pixels by `decode_png`, so a PNG that cannot be decoded (not 8-bit RGB, damaged, cut short) is a `service_error` naming `naip`, and nothing is stored. The NGS answer must name `geoidModel` `GEOID18`.

**When a service's answer is refused.** The error quotes the answer's first 120 bytes (printable characters only) and says where the raw answer was kept: `<cache>/failed/<service>.bin` (`ngs`, `3dep` or `naip`), the next refusal of that service replacing it. It is untrusted data that nothing reads back, outside the cache's entries and their size limit. A hit or a fetch prints a summary (geoid height and model, grid, image, box, bytes received) before its result line; `--json` carries the same fields.

**What is checked.** The request parameters are written from the services' documentation; the first live checks (L85f, L85g) found `model` is NGS's numeric ID (14 is GEOID18) and that 3DEP answers in tiles. One more live check, run only when the lead says so, settles the rest. Tests use synthetic answers only.
