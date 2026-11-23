# A blade polar declares the kernel width it was fitted at

Date: 2026-11-23 (box clock). Owner: Physics; the part file format is Core's (reviewed there). Branch `physics/fit-kernel`.

## What changed

A propeller part's blade may carry a third polar key, `fit_kernel_cells` (the actuator line's kernel width, in cells, the polar was fitted at), only together with `alpha_0_deg` and `cd_scale`, finite and in [1, 3] as the line's own `eps_cells` is, cited like the other two. A part with no such key is every byte it was (the triblade's pinned hash is a test).

A line's kernel is then decided by one rule, `physics::airflow::resolve_line_kernel(case_cells, fit_cells)`:

| the case asks for | the polar declares | the lines run at |
|---|---|---|
| nothing | nothing | 2 cells (the line's own default, as before) |
| nothing | a width | the polar's width |
| a width | nothing | the case's width |
| a width | the same width | that width |
| a width | another width | **refused**, naming both |

The result reaches `LineRotor::eps_cells` through `LineRotorSpec::eps_cells` and `vehicles::rotor_lines(..., eps_cells)`; a live run states a kernel other than 2 cells in its run label. The `LineRotor` harnesses (the V2b fit and sweep tests) keep their own width; they are how a polar is fitted at one.

## Declared kernels (the polar's fit conditions, `tools/library_props/polar_fits.json`)

| polar | declared cells | fitted at |
|---|---|---|
| uiuc_apce_10x5 | 2.93 | 12 mm kernel, R/dx 31 |
| uiuc_apce_9x4.5 | 2.94 | 12 mm kernel, R/dx 28 |
| uiuc_mit_5x4 (the library's) | 2.83 | 12 mm kernel, R/dx 15 |
| uiuc_cfnq_45t1 | 2.55 | 6 mm kernel, R/dx 10 |
| the OpenFrame-5F film's own copy of the 5 x 4 (polar C) | 2.97 | 3.70 mm kernel, R/dx 51 (`2026-11-23-film-polar-fit.md`) |

The four X500 rotor parts and the fixture part copy the 10x5's polar and its 2.93 declaration. `tests/test_fit_kernel.cpp` holds every copy to the table (the library's to `polar_fits.json` through its generated header, the film's to its own table). The X500 film's props are generated from the library's 10x5 and inherit it.

## Which runs change

Any run that builds its lines from a propeller part that carries a fit and names no kernel now runs at that fit's width, not at 2 cells. That is the point: the polar holds only at the kernel it was fitted at. Runs from a part with no fit (the estimated polar) are unchanged, and the disc runs do not use lines. The only producer of a live run's lines is the flow-movie tool's `--lines` path (`tools/flow_movie/lines.hpp`, which now carries the part's declared kernel into the run), so the cases whose line numbers move are the `--lines` films: the X500 film (its 10x5 polar now at 2.93 cells) and the OpenFrame-5F film (2.97 cells, its own polar). Their figures at the declared kernel are not measured here; the 2-cell figures in the plans are corrected by dated notes, not rewritten.

## What this does not claim

Grid transfer of a fitted polar was measured only for the 5 x 4 at 3 cells (`2026-11-23-film-polar-fit.md`). For the other three polars the declaration says where each was fitted, not that it transfers to another cell size; a case at another kernel is refused until it is fitted there.

`tools/flow_movie/data/x500/parts/x500_13in_estimated.prop.yaml` is an assumed estimate (the 10x5's numbers on a 13 in propeller, graded assumed) with no reader or writer in the tree; it does not declare a kernel and is not held by the agreement test.
