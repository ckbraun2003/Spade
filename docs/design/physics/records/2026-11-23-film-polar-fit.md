# The film's own blade polar (polar C): the fit, the choice, the check, and what the caption may say

Date: 2026-11-23 (box clock). Owner: Physics. The FPV pressure film (O207) runs its four lines at a 3-cell kernel (R/dx 51, a 3.70 mm kernel = 2.97 cells, 1.25 mm cell). The library's fit for this propeller is another fit (cd_scale 2.2885 at 2.83 cells, R/dx 15, 6,057 rpm); the film's polar is its own and is declared as such in its part file (`fit_kernel_cells: 2.97`). This record is the film polar's source: `tests/openframe5f_model.hpp` (`kFilmAlpha0Deg`, `kFilmCdScale`, `kFilmFitKernelCells`), the part file, its notes and `tests/test_fit_kernel.cpp` are held to one table.

## What was fitted

The estimated polar's two free numbers, `alpha_0_deg` and `cd_scale`, through the actuator line with the tip correction on, in a reduced single-rotor box (164 x 281 x 164 cells at R/dx 51, 7.56 M cells), against UIUC's static C_T and C_Q of the 5 x 4 in three-blade propeller.

- **Measured point: 6,057 rpm.** C_T 0.18147, C_P 0.11890 (C_Q 0.018924). The scatter of the three-blade sweep is +-2.2% in C_T and +-5.2% in C_P.
- **Scaled point: 9,298 rpm (the film's hover speed).** UIUC's static file stops at 6,396 rpm, so this point is *scaled* from the 6,057 point by the sweep's own trend (x1.0147 in C_T, x0.9751 in C_Q: C_T 0.184138, C_Q 0.0184523). The scaling adds +-2.5%; in quadrature the bands are +-3.3% in C_T and +-5.8% in C_P. It is a scaled target, not a measurement.

The line's response to the two numbers is nearly rank one and nonlinear in alpha_0 (per degree of alpha_0, ln C_T +0.0137 and ln C_Q -0.0656, with curvature -0.0083 per degree squared in ln C_T; per unit of ln cd_scale, -0.0941 and +0.4926). A quadratic-in-alpha_0 model predicted polar E within 0.1 to 0.2 points and polar C's 6,057 result within half a point.

## E and C, on the same bands at both speeds

M = measured; B = box-corrected estimate (below). Cells inside their band are marked; one is outside.

| | 6,057 rpm C_T (+-2.2%) | 6,057 rpm C_Q and C_P (+-5.2%) | 9,298 rpm C_T (+-3.3%) | 9,298 rpm C_Q and C_P (+-5.8%) |
|---|---|---|---|---|
| **E** (alpha_0 -4.0, cd_scale 1.871) | -0.3% M, +0.3% B: inside | -8.3% M, -8.5% B: **outside** | -2.4% M (reduced box), -1.8% M (full film box): inside | -5.0% M (reduced), -5.2% M (full): inside |
| **C** (alpha_0 -4.3344, cd_scale 2.0) | -1.1% M (reduced), -0.5% B: inside | -3.6% M (reduced), -3.8% B: inside | -3.6% M (reduced), -3.0% B: inside only after the correction | +0.2% M (reduced), 0.0% B: inside |

C's 9,298 thrust is 0.3 points past its +-3.3% band in the reduced box that was measured, and 0.3 points inside it after the box correction. That correction is **estimated from E's measurement** and was not run for C: E's full film box read +0.6 points in C_T and -0.2 points in C_Q against its reduced box at 9,298 rpm, and the same two offsets are applied to C. C's full box was not run (the GPU allowance for this fit was spent; accepted).

Polar C was chosen over E because E misses the measured 6,057 rpm power by 8.3% (outside its +-5.2% band) while C is inside every band, the one thrust cell above included only after the estimated correction.

**The 6,057 rpm verification of C** (reduced box, R/dx 51, kernel 3.70 mm = 2.97 cells, 4,928 steps, the last two windows differ by 0.00% in thrust and 0.01% in torque): C_T 0.17948 against 0.18147 (-1.1%), C_Q 0.018248 against 0.018924 (-3.6%), figure of merit 0.529. Its prediction, made before the run, was -1.6% and -3.3%.

## What the caption may say (Physics' draft of Oversight's L-S28 content; Oversight words the final text)

- the polar is this film's own fit at its kernel and cell;
- **measured** at UIUC's 6,057 rpm: thrust -1.1%, power -3.6%, inside the +-2.2% and +-5.2% scatter of the measured sweep (reduced single-rotor box);
- **scaled** to 9,298 rpm from the sweep's trend: thrust -3.0%, power 0.0% (bands +-3.3% and +-5.8%), after a box correction **estimated from E's measurement**; C's scaled thrust is inside its band by 0.3 points only after that correction;
- at equal thrust the fitted blade needs 1.5% more rpm and 4.6% more power than the scaled point; the fit is a calibration to a static point, not a measured section (a large drag scale);
- where the 9,298 rpm point is described, it is "scaled from the measured trend of a different propeller's sweep"; never "measured".

Never "within" for a power figure that is at the edge of its band; the claims about this polar stay held until Oversight words them.

## What was not checked

- **Held-out checks on a 2-blade propeller: not checked.** No measured 2-blade sweep at a comparable speed was used; the fit's transfer to other blade counts is unknown.
- **Grid transfer was measured only for the 5 x 4 at 3 cells.** The same polar at another cell size (the film's polar declares 2.97 cells) is not claimed; a case that asks for another kernel is refused by name.
- **No Reynolds dependence.** The estimated polar has none. This is recorded as the **root cause** of the residual between the two speeds: a Reynolds-dependent drag would move the two speeds' power together where one pair of numbers cannot. It is not proposed now.
- The 4 g propeller mass, the stand-in tables (another propeller's, the kit's has no published data) and the box effect (the film's four-rotor box reads about +8% above an isolated-rotor box) are carried elsewhere (`docs/design/physics/plans/2026-11-21-openframe5f-kit-model.md`).

## GPU ledger for this fit

3.23 h of the 3.3 h cap (per ticket, the earlier reduced-box fits and probes in the earlier ledger; the last six: film confirm of E 2,129 s; E at 6,057 rpm 941 s; C at 6,057 rpm 935 s; fit runs of the three variants 597, 560 and 574 s; the 2-versus-3-cell slice pair 560 and 583 s). The fit's GPU is closed.

## Dated corrections elsewhere

The 2-cell figures in `docs/design/physics/plans/2026-11-21-openframe5f-kit-model.md` (the line's thrust of about 0.9 of a quarter of the weight at 2 cells; the X500's 0.86 to 0.89) were measured at a 2-cell kernel. The film now runs at 2.97 cells. The figure at that kernel is not measured; a dated note there says so.
