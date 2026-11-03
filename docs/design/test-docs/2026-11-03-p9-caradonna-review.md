# P9: Caradonna & Tung's hover data, the reading's record

Test/Docs, 2026-11-03. Plan: `2026-11-03-p9-rotor-data-plan.md` (the lead's rulings, 2026-11-03: fetch approved; the second reading is the lead's). The data: `tests/data/airflow/caradonna1981_rotor.hpp`.

## The source

- NASA TM-81232 (USAAVRADCOM TR-81-A-23), September 1981. NTRS 19820004169, accession 82N12042, read once from `https://ntrs.nasa.gov/api/citations/19820004169/downloads/19820004169.pdf`.
- 60 pages, 2,334,139 bytes, SHA-256 `16c14789a5f65892b4ffb4e3628872723195c930f96680068ead7375e57208d2`.
- The local copy is outside the repo, at `C:/Users/Cameron/Desktop/projects/spade/.agents/refs/caradonna1981/19820004169.pdf`, with the page renders this reading used beside it.

## What was read

| Data | Where | Count | How |
|---|---|---|---|
| The rotor and the condition | Section 2 (p. 2 [4]), figure 1 (p. 42 [44]), Table 17's title, figures 4 and 6 | 9 numbers | Read from the scan; the chord (0.1905 m) is derived, R / 6, not printed |
| The blade pressures, θc 8°, 1250 rpm, M_tip 0.439 | Table 17 (p. 26 [28]) | 5 stations; 72 upper and 40 lower taps (224 numbers) and 5 C_L | Read at 300 dpi, every faint cell again at 600 dpi |
| The tip vortex's path | Figure 9 (p. 50 [52]), the 1250 rpm triangles | 3 descent and 5 radius points | Digitized at 600 dpi (below) |

Left out, as the plan says: the other collectives and speeds (Tables 1 to 32, except 17), and the transonic conditions.

## The faint cells, as read

Each was read again at 600 dpi and kept as printed:
- r/R 0.68, upper: the x/c of 0.3812, read 0.39. Its 9 is faint, and 0.39 also stands at r/R 0.96.
- r/R 0.80, upper:
  - x/c 0.00, read 0.3750: the 7 is small and faint.
  - x/c 0.50: the 5 is faint, between 0.42 and 0.56.
  - x/c 0.65, read 0.2000: the middle two digits are faint, and 0.2030 is the other possible reading.
  - x/c 0.24, read 0.5785: the E is broken.
- r/R 0.80, lower:
  - 0.1306: a 6, not an 8, at 600 dpi.
  - x/c 0.24 and 0.34 both print 0.1959. This is kept, a repeat in the original.
  - A lone "1" under the x/c column, below 0.90, with no value: a mark, not a tap, and left out (the lead's reading).
- r/R 0.89, upper: the x/c of 0.1272, read 0.73. It is faint, between 0.65 and 0.80.
- r/R 0.96: the upper x/c of 0.3150 is read 0.39; the lower 0.1467's 1 is faint.
- r/R 0.50, upper, x/c 0.00: −0.8538, the leading edge's stagnation (C_p +0.85). It is kept, although figure 4 doesn't plot that tap.

## The checks

- **Two independent reproductions:** none found. SU2's Caradonna–Tung test case holds the mesh only, with no experimental files. Searches for Table 17's own values found no transcription. This is a departure from the plan, and the lead's reading is the independent one.
- **The OCR layer:** no check. The appendix tables are scans whose text layer is noise.
- **The report's own integral** (`AirflowCaradonnaData.ThePressuresIntegrateNearEachStationsLift`):
  - Each station's −C_p integrates, upper less lower, by trapezoids. Each surface's end taps are held out to the leading edge, and 0 at the trailing edge.
  - Against the printed C_L the ratios are 0.925, 1.055, 1.009, 1.028 and 0.986, from r/R 0.50 to 0.96.
  - The report's scheme isn't printed, so the band is 10%. A misread leading digit moves a station well past it. The check doesn't catch a last-digit slip.
- **The condition:** ΩR = 149.6 m/s from 1250 rpm and 1.143 m. With M_tip 0.439 that implies a speed of sound of 340.8 m/s, air's at about 16 °C.
- **C_T:** figure 4's legend prints 0.00459 and figure 6 prints 0.00460 for the same run. Both are kept.

## Figure 9, digitized

- **The axes:**
  - Calibrated on their own tick marks at 600 dpi. Age runs 0 to 450° at about 6.15 px a degree; z/R 0 to 0.4 at 256 px per 0.1; r/R 0.8 to 1.0 at 253 px per 0.1.
  - Interpolated piecewise between ticks, because the drawing is irregular: the 400 to 450° interval is 8% short.
- **The symbols:**
  - Each triangle was found by matching the legend's own triangle, and every match was looked at.
  - A triangle is kept only where it stands clear of its neighbours. Left out: the overlapped ones near 90° and 372° in z/R, and near 235° in r/R.
- **The points:**
  - z/R (descent, positive down): (50°, 0.007), (162°, 0.081), (295°, 0.212).
  - r/R: (49°, 0.969), (90°, 0.946), (162°, 0.912), (293°, 0.880), (372°, 0.855).
  - At 372° two triangles sit about 0.025 inboard of the other speeds' points. The clearer one is kept.
- **The uncertainty** is a symbol's half-width, ±4° of age and ±0.01 in z/R or r/R. The symbol-centre convention isn't printed.

## For the lead's reading

Compare against the scan:
- every number in the header: the 9 condition numbers, 224 tap numbers and 5 C_L, by machine against your own reading of Table 17;
- each station's tap counts: upper 7, 14, 17, 18, 16; lower 5, 8, 9, 9, 9.

Figure 9's points are a digitizing, not a reading, so they are compared within their ±4° and ±0.01.

## The lead's reading

The lead, 2026-11-03 (decision L7):
- **Table 17:** read from the page's native JBIG2 scan (2496 × 3295, rotated, the faint cells enlarged). Its 229 numbers, compared by machine against `caradonna1981_rotor.hpp` at 0ff8c6c3, differ nowhere. The counts match: upper 7, 14, 17, 18, 16; lower 5, 8, 9, 9, 9.
- **The faint cells:** every one agreed as read above, including 0.2000 (0.2030 the other reading), 0.3750, 0.1306 and the three x/c read 0.39, 0.73 and 0.50.
- **The condition:** the 9 numbers match pp. 2, 42, 45 and 47. The scan's SHA-256 matches.
- **Figure 9:** the axes calibrated on their ticks by pixel scan at 300 dpi, and the 8 points overlaid. Each sits on a 1250 rpm triangle.
- **Kept outside the repo**, beside the PDF: `.agents/refs/caradonna1981/lead_reading_table17.txt` and `lead_fig9_overlay.png`.
- **Three text fixes, no value changes:**
  - Figure 1's 45 in. is a vertical dimension beside the hot-wire traverse, so R is cited as half the 7.5 ft (2.286 m) diameter.
  - The integral is worded as measured, within 7.5% (0.925 to 1.055), with the test's band at 10%.
  - The mark under r/R 0.80's lower x/c column, above.
