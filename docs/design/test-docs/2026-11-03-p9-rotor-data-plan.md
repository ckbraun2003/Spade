# P9: Caradonna & Tung's hover data for D3 (plan)

Test/Docs, 2026-11-03, for the lead. Cameron chose real blades: D3 starts with P5's plan and P9's rotor data (`docs/design/backlog.md`, the geometry row; the demonstration plan's P9, `physics/plans/2026-10-12-airflow-drone-demonstration-plan.md`). Nothing is fetched or built yet. Every value and condition named below is read from the report itself. The figures given from memory here are marked "to confirm".

## What P9 delivers

- **`tests/data/airflow/caradonna1981_rotor.hpp`**, header-only like `ghia1982_cavity.hpp`. It holds:
  - SOURCE: the full citation, the report number, and the page and table or figure for each array;
  - METHOD;
  - the condition;
  - the arrays, in the report's own units and coordinates.

  A `static_assert` ties each array's length to its station's point count.
- **A row in `tests/data/airflow/README.md`** for the blade-resolved hovering rotor, the D3 case. It gives the source and the method, and states the caveats below.
- **The review's record:** a short doc in `docs/design/test-docs/` that lists each array's comparison, its count of values, and every difference found and how it was decided.
- **One host test**, which includes the header and checks that the stations and counts match the README. Nothing reads the data until D3.
- **No tolerance.** D3's checks fix those before D3's first run, from P5's measured choice between NS with nests and LBM (`TD-4`, expectations from elsewhere). P9 states only the caveats.

## The data: the low-Mach hover condition only

The demonstration plan's condition is the report's subsonic hover, at tip Mach about 0.44 (to confirm: collective 8°, about 1250 rpm).
1. **The rotor and the condition:**
   - the blades: their count, radius, chord, airfoil, twist and taper;
   - the run: collective, rotor speed, tip Mach and the measured thrust coefficient;
   - the test chamber's notes.
2. **The blade pressures:** the chordwise C_p at each radial station the report measured, upper and lower surfaces, x/c against C_p, as tabulated.
3. **The tip vortex's path:** the radial position r/R and the descent z/R against wake age.
   - Where the report tabulates it, the table is transcribed.
   - Where it only plots it, the plot is digitized, and each array names its digitizing resolution as its uncertainty.

Left out until D3 asks (YAGNI):
- the transonic conditions, which are outside an incompressible solver;
- the other collectives;
- the UIUC propeller data, which is for the actuator line, not the blades.

## The method (as for Ghia)

1. **The source** is the report's scan from NASA's Technical Reports Server, a US Government work in the public domain. The header records its NTRS id and the PDF's SHA-256. This is one download; nothing leaves the box.
2. **The reading:** each value is read from the scan and checked against the scan's OCR layer.
3. **Two independent reproductions**, each named with its own provenance. Candidates, to be found and checked:
   - an open CFD test suite's experimental files for the case (SU2 is said to run it);
   - papers that tabulate the data.

   A disagreement is decided from the scan and noted, as Ghia's Uppsala 0.05188 was.
4. **Values are kept as printed.** A suspected typo is noted, never corrected (Ghia's point 117).
5. **An independent machine comparison** of the header against the scan by a second realm: every value, the count, and no difference. Physics did this for Ghia on 2026-10-12. Test/Docs then reviews the result.

## The caveats the header and README state

- **Compressibility.** At tip Mach 0.44 the Prandtl–Glauert factor 1/√(1 − M²) is 1.11 at the tip, and about 1.03 at r/R 0.5 (M ≈ 0.22). From compressibility alone, an incompressible solver's C_p can differ by up to about 10% near the tip (the demonstration plan's caveat).
- **Reynolds number.** The model rotor's chord Reynolds number is computed from the report's condition (to confirm: about 1e6 at the tip). A drone propeller's is 1e4 to 1e5. The case validates the blade-resolved method, not transition at drone scale.
- **The test chamber.** The report's own notes on the hover chamber's recirculation, as given.

## Steps and owners

1. **Test/Docs, about one session:**
   - fetch the report;
   - read and transcribe it;
   - find the two reproductions;
   - write the header, the README row, the host test and the review doc.

   This goes on a branch, with gcc and the CPU tier.
2. **Physics:** the independent comparison against the scan.
3. **The lead** merges. D3's plan (Physics, P5) cites the header.

## What the lead decides

1. Test/Docs fetches the report from NTRS: one outbound read, nothing sent.
2. Who does the independent second reading. Physics did it for Ghia; the lead verified Ghia first. My recommendation is Physics, since D3 is theirs.
