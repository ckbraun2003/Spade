# spade_geo_fetch

Fetches a real-world map area into Spade's local map cache, for a scene that sits at a real place (KAT's geo plan, G2). The design is `docs/design/interface/plans/2026-11-17-geo-map-fetch-design.md`.

The engine never touches the network. This tool is the only program that does, and only when you run it (or the editor runs it for you). It asks three US public-domain services and nothing else: USGS 3DEP (elevation), USGS NAIP (aerial image) and NOAA NGS (geoid height at the anchor). None of them asks for a logo or a credit line.

```
spade_geo_fetch --lat 40.76 --lon -113.90 --side 1000
```

An area is a square, at most 1000 m a side, centred on the anchor. The result is on the last line of the output and in the exit code: `hit` or `fetched` (0), `miss` (3, with `--check`), `offline` (10), `service_error` (11), `refused` (12), `cache_full` (13). `--json` prints one JSON object a line, so an editor can show progress. `--help` lists the rest.

**Where the cache is.** Outside the repository: `--cache <dir>`, else `SPADE_MAP_CACHE`, else `%LOCALAPPDATA%\Spade\maps`. The tool refuses a directory inside a git work tree, so a map is never committed. The cache is trimmed to 512 MB by default (`--cache-limit-mb`), oldest fetch first.

**No network.** With no connection the tool says `offline` and stores nothing; a scene that finds no map in the cache loads with plain ground and the editor shows why. Only Windows has an HTTP layer so far (WinHTTP, which is part of Windows). On any other platform the result is `offline: no HTTP layer on this platform yet`.

**Heights, and an accepted error.** 3DEP heights are above the US vertical datum (NAVD88). The tool adds the geoid height NOAA NGS gives at the anchor (GEOID18) to every sample, so the map holds heights above the ellipsoid, as a GNSS receiver reports them. GEOID18's ellipsoid is NAD83(2011), not WGS84. The two differ by about 1 to 2 m over the US, and the services' coordinates are NAD83 underneath, so the image can also sit 1 to 2 m off WGS84. This is recorded in each entry's `georef.yaml` (`datum_note`) and not corrected. It moves the whole map together, so a drone sees consistent heights over it, but a GNSS height compared with a surveyed WGS84 point can be off by that much.

**What is checked.** The request parameters are written from the services' documentation and have not been run: one live check, run only when the lead says so, settles them. Tests use synthetic answers only.
