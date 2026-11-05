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
import json
import math
import os
import struct
import subprocess
import sys
import tempfile
import unittest
from collections import Counter

HERE =os.path.dirname(os.path.abspath(__file__))
CONVERT = os.path.join(HERE, "step_convert.py")
PYTHON = os.environ.get("SPADE_STEP_PYTHON") or sys.executable
UNIT_MILLIMETRE, UNIT_CENTIMETRE = 2, 3
PACK = FIXTURES = ""


def have_ocp():
    return subprocess.run([PYTHON, "-B", "-I", "-c", "import OCP"], capture_output=True).returncode == 0


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


def occt_vertex_counts(step_path, deflection_mm, angle=0.5):
    """{solid name: the number of distinct nodes OpenCASCADE's own SHARED-EDGE discretisation gives}, for a STEP in mm.

    Derived from topology, independently of step_convert.py's position weld: the distinct topological vertices, plus
    for each edge the nodes strictly inside its polygon (every face on the edge must agree on how many), plus for each
    face the nodes of its triangulation that no edge polygon of that face holds. A weld that misses a seam would
    leave the mesh with more vertices than this."""
    sys.path.insert(0, HERE)
    import step_convert as sc
    from OCP.BRep import BRep_Tool
    from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_VERTEX
    from OCP.TopExp import TopExp, TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS
    from OCP.TopTools import TopTools_IndexedDataMapOfShapeListOfShape, TopTools_IndexedMapOfShape

    counts = {}
    for name, solid, _ in sc.read_solids(step_path, "mm", set()):
        sc.tessellate(solid, deflection_mm, angle, name)  # meshes `solid` in place, as the converter does
        vertices = TopTools_IndexedMapOfShape()
        TopExp.MapShapes_s(solid, TopAbs_VERTEX, vertices)
        edge_faces = TopTools_IndexedDataMapOfShapeListOfShape()
        TopExp.MapShapesAndAncestors_s(solid, TopAbs_EDGE, TopAbs_FACE, edge_faces)
        total = vertices.Extent()
        for i in range(1, edge_faces.Extent() + 1):
            edge = TopoDS.Edge_s(edge_faces.FindKey(i))
            if BRep_Tool.Degenerated_s(edge):
                continue
            nodes_seen = set()
            for face in edge_faces.FindFromIndex(i):
                location = TopLoc_Location()
                triangulation = BRep_Tool.Triangulation_s(TopoDS.Face_s(face), location)
                nodes_seen.add(BRep_Tool.PolygonOnTriangulation_s(edge, triangulation, location).NbNodes())
            if len(nodes_seen) != 1:
                raise AssertionError(f"{name}: the faces of an edge disagree on its discretisation {nodes_seen}")
            total += nodes_seen.pop() - 2
        faces = TopTools_IndexedMapOfShape()
        TopExp.MapShapes_s(solid, TopAbs_FACE, faces)
        for i in range(1, faces.Extent() + 1):
            face = TopoDS.Face_s(faces.FindKey(i))
            location = TopLoc_Location()
            triangulation = BRep_Tool.Triangulation_s(face, location)
            on_edges = set()
            edges = TopExp_Explorer(face, TopAbs_EDGE)  # a seam shows twice, once per side
            while edges.More():
                edge = TopoDS.Edge_s(edges.Current())
                if not BRep_Tool.Degenerated_s(edge):
                    nodes = BRep_Tool.PolygonOnTriangulation_s(edge, triangulation, location).Nodes()
                    on_edges.update(nodes.Value(k) for k in range(nodes.Lower(), nodes.Upper() + 1))
                edges.Next()
            total += triangulation.NbNodes() - len(on_edges)
        counts[name] = total
    return counts


class StepConvert(unittest.TestCase):
    def convert(self, fixture, out, *options, expect=0):
        run = subprocess.run([PYTHON, "-B", "-I", CONVERT, os.path.join(FIXTURES, fixture), "--out", out, "--pack", PACK,
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
        run = subprocess.run([PYTHON, "-B", "-I", CONVERT, os.path.join(FIXTURES, "cube_10mm.step"), "--out", "x",
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

    def test_a_part_placed_several_times_is_named_apart_in_walk_order(self):
        # One Motor placed four times, the occurrences carrying no names of their own (a drone's STEP): the four
        # instances would all be "Motor", so each gets _1.._4 in walk order, and --solid can pick one.
        with tempfile.TemporaryDirectory() as out:
            self.convert("quad_motors.step", out, "--deflection", "0.05mm")
            self.assertEqual(sorted(os.listdir(out)), [f"Motor_{k}.spmesh" for k in (1, 2, 3, 4)])
            # A point (a, b, c) of the motor (10 x 6 x 4 mm) goes to: 1 itself; 2 (a + 50, b, c); 3 (-b, a + 40, c);
            # 4 (50 - a, 40 - b, c).
            want = {1: ((0, 10), (0, 6), (0, 4)), 2: ((50, 60), (0, 6), (0, 4)), 3: ((-6, 0), (40, 50), (0, 4)),
                    4: ((40, 50), (34, 40), (0, 4))}
            for k, box in want.items():
                positions, triangles, _, _, source = self.sidecar(out, f"Motor_{k}")
                self.assertTrue(source.endswith(f": Motor_{k}"), source)
                self.assertAlmostEqual(volume(positions, triangles), 240e-9, delta=1e-13)
                for (low, high), (want_low, want_high) in zip(bounds(positions), box):
                    self.assertAlmostEqual(low, want_low * 1e-3, delta=1e-8)
                    self.assertAlmostEqual(high, want_high * 1e-3, delta=1e-8)
        with tempfile.TemporaryDirectory() as out:
            self.convert("quad_motors.step", out, "--deflection", "0.05mm", "--solid", "Motor_3")
            self.assertEqual(os.listdir(out), ["Motor_3.spmesh"])
            self.assertAlmostEqual(bounds(self.sidecar(out, "Motor_3")[0])[1][1], 0.05, delta=1e-8)
        with tempfile.TemporaryDirectory() as out:  # the bare name no longer exists: it names four things
            run = self.convert("quad_motors.step", out, "--deflection", "0.05mm", "--solid", "Motor", expect=2)
            self.assertIn("Motor", run.stderr)

    def test_a_suffixed_name_that_meets_a_real_name_is_refused_naming_both(self):
        # Occurrences Motor, Motor_2, Motor, Motor: the three Motors would be Motor_1, Motor_2, Motor_3, and the real
        # Motor_2 is already there. Nothing is guessed.
        with open(os.path.join(FIXTURES, "quad_motors.step"), encoding="utf-8") as f:
            text = f.read()
        marker = "NEXT_ASSEMBLY_USAGE_OCCURRENCE('','','',"
        first = text.index(marker) + len(marker)
        second = text.index(marker, first)
        text = text[:second] + "NEXT_ASSEMBLY_USAGE_OCCURRENCE('','Motor_2',''," + text[second + len(marker):]
        with tempfile.TemporaryDirectory() as work:
            clash = os.path.join(work, "clash.step")
            with open(clash, "w", encoding="utf-8", newline="\n") as f:
                f.write(text)
            out = os.path.join(work, "out")
            run = subprocess.run([PYTHON, "-B", "-I", CONVERT, clash, "--out", out, "--deflection", "0.1mm", "--pack", PACK],
                                 capture_output=True, text=True, encoding="utf-8")
            self.assertEqual(run.returncode, 3, run.stderr)
            self.assertIn("'Motor_2' and 'Motor'", run.stderr)
            self.assertFalse(os.path.exists(out), "a refusal writes nothing")

    def test_the_position_weld_is_watertight_and_matches_occts_shared_edge_count(self):
        # Every faces' nodes on a shared edge must come out as ONE vertex. Two things pin it, on faces of every kind the
        # fixtures have (planar, cylindrical with a seam, closed circles): each directed edge of the sidecar appears
        # once and its reverse once (so every edge is shared by exactly two triangles, oppositely wound); and the
        # vertex count is the one OpenCASCADE's own shared-edge discretisation gives (occt_vertex_counts), so a seam
        # the weld missed would show as extra vertices. (A mesh with an open edge could not be a sidecar at all:
        # make_mesh() refuses it, so a missed seam is a refusal, exit 4, never a silent hole.)
        for fixture, deflection, names in (("cube_10mm.step", 0.05, ["Cube"]), ("holed_cylinder.step", 0.02, ["Ring"]),
                                           ("holed_cylinder.step", 0.1, ["Ring"]),
                                           ("rotated_pair.step", 0.05, ["Block", "Pin"])):
            with self.subTest(fixture=fixture, deflection=deflection), tempfile.TemporaryDirectory() as out:
                self.convert(fixture, out, "--deflection", f"{deflection}mm")
                run = subprocess.run([PYTHON, "-B", "-I", os.path.join(HERE, "test_step_convert.py"), "--occt-vertex-counts",
                                      os.path.join(FIXTURES, fixture), str(deflection)],
                                     capture_output=True, text=True, encoding="utf-8")
                self.assertEqual(run.returncode, 0, run.stderr)
                expected = json.loads(run.stdout)
                for name in names:
                    positions, triangles, *_ = self.sidecar(out, name)
                    directed = Counter()
                    for a, b, c in triangles:
                        directed.update([(a, b), (b, c), (c, a)])
                    self.assertTrue(all(n == 1 for n in directed.values()), f"{name}: a directed edge is used twice")
                    self.assertTrue(all(directed.get((b, a)) == 1 for a, b in directed), f"{name}: an edge is open")
                    self.assertEqual(len(positions), expected[name], f"{name}: the weld left seams unmerged")

    def test_the_files_own_unit_is_kept(self):
        with open(os.path.join(FIXTURES, "cube_10mm.step"), encoding="utf-8") as f:
            text = f.read()
        self.assertIn("SI_UNIT(.MILLI.,.METRE.)", text)
        with tempfile.TemporaryDirectory() as work:
            centi = os.path.join(work, "cube_10cm.step")
            with open(centi, "w", encoding="utf-8", newline="\n") as f:
                f.write(text.replace("SI_UNIT(.MILLI.,.METRE.)", "SI_UNIT(.CENTI.,.METRE.)"))
            out = os.path.join(work, "out")
            run = subprocess.run([PYTHON, "-B", "-I", CONVERT, centi, "--out", out, "--deflection", "0.5mm", "--pack", PACK],
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
            run = subprocess.run([PYTHON, "-B", "-I", CONVERT, bad, "--out", os.path.join(work, "out"), "--deflection",
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
    if len(sys.argv) == 4 and sys.argv[1] == "--occt-vertex-counts":  # a helper the weld test runs under the OCP python
        sys.stdout.write(json.dumps(occt_vertex_counts(sys.argv[2], float(sys.argv[3]))))
        return 0
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
