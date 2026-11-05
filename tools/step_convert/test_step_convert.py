#!/usr/bin/env python3
"""Tests of step_convert.py, run by ctest with the label `step` (tools/step_convert/CMakeLists.txt).

    python test_step_convert.py PACK FIXTURES CASE

PACK is the built spade_mesh_pack, FIXTURES the tests/golden/step directory, CASE one test of StepConvert below (ctest
registers one test per case). The converter runs as `python step_convert.py`, with the python named by the environment
variable SPADE_STEP_PYTHON (a venv's, with OCP installed) or else the python running this file.

WITHOUT OCP a case that needs it prints why and exits 77 (ctest's SKIP_RETURN_CODE): a skip, never a pass, and a skip
does not count toward a green gate. The two usage cases need no OCP and run anyway.

The sidecars are read here by hand, from the layout in engine/world/mesh_file.hpp, not by the engine: the C++ packer
already ran make_mesh() on every mesh before it wrote one, so a sidecar existing proves closed, manifold and oriented.
There are no bit goldens: the tessellation is OpenCASCADE's, and a golden would tie these tests to its version.
"""
import math
import os
import struct
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
CONVERT = os.path.join(HERE, "step_convert.py")
PYTHON = os.environ.get("SPADE_STEP_PYTHON") or sys.executable
UNIT_MILLIMETRE, UNIT_CENTIMETRE = 2, 3
PACK = FIXTURES = ""


def have_ocp():
    return subprocess.run([PYTHON, "-I", "-c", "import OCP"], capture_output=True).returncode == 0


def read_spmesh(path):
    """(positions as a list of (x, y, z) in metres, triangles, unit, tolerance, source) of a sidecar."""
    with open(path, "rb") as f:
        d = f.read()
    assert d[:8] == b"SPADEMSH", path
    version, v, t, unit = struct.unpack_from("<IIII", d, 8)
    assert version == 1
    (tolerance,) = struct.unpack_from("<d", d, 24)
    (s,) = struct.unpack_from("<I", d, 32)
    source = d[36:36 + s].decode("utf-8")
    off = 36 + s
    flat = struct.unpack_from(f"<{3 * v}f", d, off)
    off += 12 * v
    idx = struct.unpack_from(f"<{3 * t}I", d, off)
    off += 12 * t
    assert len(d) == off + 8, "size is not 44 + S + 12 V + 12 T"
    return ([flat[i:i + 3] for i in range(0, len(flat), 3)], [idx[i:i + 3] for i in range(0, len(idx), 3)],
            unit, tolerance, source)


def volume(positions, triangles):
    total = 0.0
    for a, b, c in triangles:
        (ax, ay, az), (bx, by, bz), (cx, cy, cz) = positions[a], positions[b], positions[c]
        total += (ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx)) / 6.0
    return total


def bounds(positions):
    return [(min(p[i] for p in positions), max(p[i] for p in positions)) for i in range(3)]


class StepConvert(unittest.TestCase):
    def convert(self, fixture, out, *options, expect=0):
        run = subprocess.run([PYTHON, "-I", CONVERT, os.path.join(FIXTURES, fixture), "--out", out, "--pack", PACK,
                              *options], capture_output=True, text=True, encoding="utf-8")
        self.assertEqual(run.returncode, expect, f"stdout:\n{run.stdout}\nstderr:\n{run.stderr}")
        return run

    def sidecar(self, out, name):
        return read_spmesh(os.path.join(out, name + ".spmesh"))

    # -- the usage cases: no OCP needed ---------------------------------------------------------------------------

    def test_a_deflection_is_required_and_needs_a_unit(self):
        with tempfile.TemporaryDirectory() as out:
            for options in ([], ["--deflection", "0.1"], ["--deflection", "0.1km"], ["--deflection", "-1mm"],
                            ["--deflection", "0mm"]):
                run = self.convert("cube_10mm.step", out, *options, expect=2)
                self.assertIn("--deflection", run.stderr)
            self.assertEqual(os.listdir(out), [], "a usage error reads and writes nothing")

    def test_the_packer_must_be_found(self):
        run = subprocess.run([PYTHON, "-I", CONVERT, os.path.join(FIXTURES, "cube_10mm.step"), "--out", "x",
                              "--deflection", "0.1mm", "--pack", os.path.join(HERE, "no_such_packer")],
                             capture_output=True, text=True)
        self.assertEqual(run.returncode, 2, run.stderr)
        self.assertIn("spade_mesh_pack", run.stderr)

    # -- the conversions: OCP needed ------------------------------------------------------------------------------

    def test_a_cube_is_closed_exact_and_carries_its_provenance(self):
        with tempfile.TemporaryDirectory() as out:
            self.convert("cube_10mm.step", out, "--deflection", "0.05mm")
            self.assertEqual(os.listdir(out), ["Cube.spmesh"])
            positions, triangles, unit, tolerance, source = self.sidecar(out, "Cube")
            self.assertEqual((len(positions), len(triangles)), (8, 12))
            self.assertAlmostEqual(volume(positions, triangles), 1e-6, delta=1e-11)
            for low, high in bounds(positions):
                self.assertAlmostEqual(low, 0.0, delta=1e-9)
                self.assertAlmostEqual(high, 0.01, delta=1e-9)
            self.assertEqual(unit, UNIT_MILLIMETRE)
            self.assertAlmostEqual(tolerance, 5e-5, delta=1e-12)
            for part in ("cube_10mm.step", "Cube", "STEP", "OCCT 7.9.3", "cadquery-ocp", "0.05mm", "0.5 rad"):
                self.assertIn(part, source)

    def test_a_cylinder_with_a_hole_is_genus_one_and_within_the_deflection_bound(self):
        d = 0.02  # mm
        with tempfile.TemporaryDirectory() as out:
            self.convert("holed_cylinder.step", out, "--deflection", f"{d}mm")
            positions, triangles, unit, tolerance, _ = self.sidecar(out, "Ring")
            # Euler: V - E + F = 0 for a torus, E = 3T/2.
            self.assertEqual(len(positions) - len(triangles) // 2, 0)
            exact = math.pi * (10.0 ** 2 - 4.0 ** 2) * 20.0 * 1e-9  # m^3
            bound = (2.0 / 3.0) * d * 1e-3 * 2.0 * math.pi * (10.0 + 4.0) * 1e-3 * 20e-3  # chord error, outer + hole
            self.assertLessEqual(abs(volume(positions, triangles) - exact), bound * 1.05)
            for low, high in bounds(positions)[:2]:
                self.assertAlmostEqual(high, 0.01, delta=1e-5)
                self.assertAlmostEqual(low, -0.01, delta=1e-5)
            self.assertAlmostEqual(bounds(positions)[2][1], 0.02, delta=1e-9)

    def test_an_assembly_is_placed_by_its_transforms(self):
        with tempfile.TemporaryDirectory() as out:
            self.convert("rotated_pair.step", out, "--deflection", "0.05mm")
            self.assertEqual(sorted(os.listdir(out)), ["Block.spmesh", "Pin.spmesh"])
            block = self.sidecar(out, "Block")
            pin = self.sidecar(out, "Pin")
            self.assertAlmostEqual(volume(block[0], block[1]), 20e-3 * 10e-3 * 5e-3, delta=1e-12)
            self.assertAlmostEqual(volume(pin[0], pin[1]), 125e-9, delta=1e-13)  # positive: still outward-wound
            for (low, high), (want_low, want_high) in zip(bounds(block[0]), ((0, 0.02), (0, 0.01), (0, 0.005))):
                self.assertAlmostEqual(low, want_low, delta=1e-8)
                self.assertAlmostEqual(high, want_high, delta=1e-8)
            # The pin: placed at (30, 0, 0) mm with its x axis along +y, so (a, b, c) goes to (30 - b, a, c).
            for (low, high), (want_low, want_high) in zip(bounds(pin[0]), ((0.025, 0.030), (0, 0.005), (0, 0.005))):
                self.assertAlmostEqual(low, want_low, delta=1e-8)
                self.assertAlmostEqual(high, want_high, delta=1e-8)

    def test_the_files_own_unit_is_kept(self):
        with open(os.path.join(FIXTURES, "cube_10mm.step"), encoding="utf-8") as f:
            text = f.read()
        self.assertIn("SI_UNIT(.MILLI.,.METRE.)", text)
        with tempfile.TemporaryDirectory() as work:
            centi = os.path.join(work, "cube_10cm.step")
            with open(centi, "w", encoding="utf-8", newline="\n") as f:
                f.write(text.replace("SI_UNIT(.MILLI.,.METRE.)", "SI_UNIT(.CENTI.,.METRE.)"))
            out = os.path.join(work, "out")
            run = subprocess.run([PYTHON, "-I", CONVERT, centi, "--out", out, "--deflection", "0.5mm", "--pack", PACK],
                                 capture_output=True, text=True, encoding="utf-8")
            self.assertEqual(run.returncode, 0, run.stderr)
            positions, triangles, unit, tolerance, _ = self.sidecar(out, "Cube")
            self.assertEqual(unit, UNIT_CENTIMETRE)
            self.assertAlmostEqual(tolerance, 5e-4, delta=1e-12)  # the 0.5mm asked, in metres, whatever the file's unit
            self.assertAlmostEqual(bounds(positions)[0][1], 0.1, delta=1e-8)  # ten centimetres
            self.assertAlmostEqual(volume(positions, triangles), 1e-3, delta=1e-8)

    def test_a_coarse_deflection_is_flagged_by_the_cross_check_and_still_written(self):
        with tempfile.TemporaryDirectory() as out:
            run = self.convert("holed_cylinder.step", out, "--deflection", "3mm", "--angle", "1.2", expect=5)
            self.assertIn("FLAGGED", run.stdout)
            self.assertIn("Ring", run.stderr)
            self.assertEqual(os.listdir(out), ["Ring.spmesh"])

    def test_a_file_that_is_not_step_is_refused_by_name(self):
        with tempfile.TemporaryDirectory() as work:
            bad = os.path.join(work, "not_a_step.step")
            with open(bad, "w", encoding="utf-8") as f:
                f.write("this is not a STEP file\n")
            run = subprocess.run([PYTHON, "-I", CONVERT, bad, "--out", os.path.join(work, "out"), "--deflection",
                                  "0.1mm", "--pack", PACK], capture_output=True, text=True, encoding="utf-8")
            self.assertEqual(run.returncode, 3, run.stderr)
            self.assertIn("not_a_step.step", run.stderr)

    def test_a_named_solid_can_be_picked_and_an_unknown_one_is_a_usage_error(self):
        with tempfile.TemporaryDirectory() as out:
            self.convert("rotated_pair.step", out, "--deflection", "0.05mm", "--solid", "Pin")
            self.assertEqual(os.listdir(out), ["Pin.spmesh"])
        with tempfile.TemporaryDirectory() as out:
            run = self.convert("rotated_pair.step", out, "--deflection", "0.05mm", "--solid", "Nope", expect=2)
            self.assertIn("Nope", run.stderr)


NO_OCP_NEEDED = {"test_a_deflection_is_required_and_needs_a_unit", "test_the_packer_must_be_found"}


def main():
    global PACK, FIXTURES
    if len(sys.argv) != 4:
        sys.stderr.write(__doc__)
        return 2
    PACK, FIXTURES, case = sys.argv[1:]
    if not hasattr(StepConvert, case):
        sys.stderr.write(f"no such case: {case}\n")
        return 2
    if case not in NO_OCP_NEEDED and not have_ocp():
        sys.stdout.write(f"SKIPPED {case}: OCP is not importable by {PYTHON}; install tools/step_convert/"
                         "requirements.txt in a venv and name its python in SPADE_STEP_PYTHON\n")
        return 77
    suite = unittest.TestSuite([StepConvert(case)])
    result = unittest.TextTestRunner(verbosity=1).run(suite)
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    sys.exit(main())
