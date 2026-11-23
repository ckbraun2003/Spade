# B1: per-prop polar fits through the corrected line (L72k), 2026-11-17

The values live in tools/library_props/polar_fits.json (-> tests/library_polar_fits.gen.hpp) and in the four props' part files (blade.alpha_0_deg, blade.cd_scale, graded `fitted`, citing this record). The run logs (b1-fits.jsonl, the fit driver and the V2b harness env hooks SPADE_V2B_ALPHA0 / SPADE_V2B_CDSCALE) stay in the worktree's .superpowers/sdd/2026-11-10-wake/.

Fitted: the estimated polar's two free parameters (EstimatedPolarShape: alpha_0, cd_scale) per library prop with UIUC static
data, so the actuator line WITH THE TIP CORRECTION ON (A1) reproduces the prop's static C_T and C_Q. Fitted through the
corrected line on the GPU (V2b harness, GpuAirflowLineHover.V2bTheUiuc10x5OnTheGpu), not through the BEMT.
Method: b1_fit.py (two finite-difference evaluations seed a Jacobian, then Newton steps with Broyden updates on
(alpha_0, ln cd_scale) against (ln C_T/ref, ln C_Q/ref); stops when both are within 1%). Every evaluation: b1-fits.jsonl.
Harness: tests/test_gpu_airflow_line_v2b.cpp (SPADE_V2B_ALPHA0, SPADE_V2B_CDSCALE), v2b-fit-run.ps1. Build tip/b1 (60fca87).

| Prop (library id) | alpha_0 deg | cd_scale | Fit point (kernel, grid, rpm) | Line vs UIUC static (C_T, C_Q) | Source of the target |
|---|---|---|---|---|---|
| uiuc_apce_10x5 | -3.0645 | 1.3514 | 12 mm (2.93 cells), R/dx 31, 5000 rpm, D 10 in | 0.09619 vs 0.09636 (-0.2%), 0.005886 vs 0.005870 (+0.3%) | UIUC Vol. 1 static pg0819, library ct_static/cq_static |
| uiuc_apce_9x4.5 | -2.5120 | 1.2907 | 12 mm (2.94 cells), R/dx 28, 5008 rpm, D 9 in | 0.09816 vs 0.09818 (0.0%), 0.006146 vs 0.006129 (+0.3%) | UIUC Vol. 1 static rd0995 (reads ~10% above the run's lowest-J points: library note) |
| uiuc_mit_5x4 | -4.3344 | 2.2885 | 12 mm (2.83 cells), R/dx 15, 6057 rpm, D 5 in, 3 blades | 0.18089 vs 0.18147 (-0.3%), 0.019029 vs 0.018924 (+0.6%) | UIUC Vol. 2 static 0362rd (run 0366rd 6057 rpm) |
| uiuc_cfnq_45t1 | -0.5060 | 1.9354 | 6 mm (2.55 cells; eps/chord 0.67), R/dx 10, 16582 rpm, D 47 mm | 0.15653 vs 0.15586 (+0.4%), 0.016599 vs 0.016764 (-1.0%) | UIUC Vol. 2 static 1320rd at the computed hover rpm (static-only prop) |
| triblade_6inch_scaled | not fitted: stays ESTIMATED (-3, 1) | | the library's scaled stand-in for the 6-inch default; no data of its own | | |

Conditions of validity: each fit holds for the kernel width and resolution it was made at (about 3 cells, eps/chord 0.7 to 1.1)
and with tip_correction ON; do not use the values with the flag off or at another width without re-fitting. cd_scale 2.29 on the
mit 5x4 is a LARGE drag scale (zero-lift C_d 0.069 against the estimate's 0.03): it carries the micro prop's low Reynolds number
and whatever the model leaves out, and it is what the BEMT alone also asked for (about 2.09, L72d). The three bigger props sit at 1.3.

Held-out reruns with the fitted polars (flag ON; bits are the b1 build, 12 mm unless noted) against the same L72d bands:
| Run | flag OFF, estimated (L72d) | flag ON, estimated | flag ON, fitted | band |
|---|---|---|---|---|
| tune 10x5 5000 rpm | 0.10175 / 0.005082 | 0.09659 / 0.005276 | 0.09619 / 0.005886 (fit point) | selection |
| H3 3093 rpm | 0.09785 / 0.005231 | 0.09318 / 0.005373 | 0.09267 / 0.005964 | C_T 0.07965-0.10776; C_Q 0.005019-0.006790: in, in |
| H4 5869 rpm | 0.10283 / 0.005048 | 0.09741 / 0.005252 | 0.09719 / 0.005869 | C_T 0.08304-0.11235; C_Q 0.005046-0.006827: in, in |
| H1/V3 5300 rpm R/dx 29 | 3.149 N | 2.985 N | 2.974 N | 2.305-3.925 N (NASA 3.115): in |
| H5 A mit R/dx 15 | 0.19113 / 0.013502 | 0.17674 / 0.013740 | 0.18089 / 0.019029 (fit point) | now a fit, not held out |
| H5 A pair R/dx 11 | 0.19271 / 0.013568 | 0.17857 / 0.013813 | 0.18273 / 0.019157 | pair: 1.0% C_T, 0.7% C_Q apart |
| H5 B mit 7.8 mm R/dx 24 | 0.18679 / 0.013351 | 0.17788 / 0.013614 | 0.18198 / 0.019111 | +0.3% / +1.0% of UIUC: held out in width and resolution |
Still held out for mit: its width and resolution (form B, R/dx 11), not its rpm (UIUC's mit runs at other rpm were not rerun).
For the 10x5: H3/H4 (rpm), V3 (D 9.4 in, NASA). For the 9x4.5 and the Crazyflie: nothing held out (one static point each).

The D1 quad's stand-in blade with the MIT fit (7649 rpm, R/dx 19, 12 mm, flag off): line thrust 2.035 N, C_T 0.1895: the chain's
k_T (C_T 0.120, 1.29 N at 801 rad/s) is not this blade's, so line rotors on D1 need the rpm (about 6090) or a thrust scale set from
the line's own C_T; the fit does not remove that.

Dated note, 2026-11-23: each fit holds only at the kernel width in cells it was made at; that width is now declared in the part file (`blade.fit_kernel_cells`, `docs/design/physics/records/2026-11-23-fit-kernel-declaration.md`: 2.93, 2.94, 2.83 and 2.55 cells) and a line runs the polar at it. The text above is unchanged; the grid transfer of a polar was measured only for the 5 x 4 at 3 cells, where a different polar fits the film (`2026-11-23-film-polar-fit.md`).
