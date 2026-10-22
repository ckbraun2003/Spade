"""Tests for resample_uiuc.py (L4a of the default-drone library plan).

Run by hand whenever the tool or its input changes (the lead's ruling,
2026-10-22), and name it with its output in READY:

    python tools/library_props/test_resample_uiuc.py

With SPADE_LIBRARY_WRITE=1 the synthetic expected header is rewritten
instead of compared, as the repository's other generators do. The input
under testdata/ is our own numbers in UIUC's file format, never UIUC's,
and every value checked by hand below was computed from it on paper.
"""

import hashlib
import math
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import resample_uiuc as tool  # noqa: E402

TESTDATA = os.path.join(HERE, "testdata")
PROPS = os.path.join(TESTDATA, "props.json")
UIUC = os.path.join(TESTDATA, "uiuc")
EXPECTED = os.path.join(TESTDATA, "expected.gen.hpp")


def hover_of(r, ct):
    """The hover rpm the sim flies at with C_T0 = ct: n = sqrt(m g / (N rho ct D^4))."""
    d = r["drone"]
    return 60.0 * math.sqrt(d["mass_kg"] * tool.G / (d["rotors"] * tool.RHO * ct * r["diameter"] ** 4))


def by_id(results, prop_id):
    for r in results:
        if r["id"] == prop_id:
            return r
    raise AssertionError("no prop " + prop_id)


class TheGeneratedTablesAreWhatTheRuleGives(unittest.TestCase):
    def test_the_synthetic_header_is_the_expected_one(self):
        text, _ = tool.generate(PROPS, UIUC)
        if os.environ.get("SPADE_LIBRARY_WRITE"):
            with open(EXPECTED, "w", encoding="utf-8", newline="\n") as f:
                f.write(text)
            print("wrote " + EXPECTED)
            return
        with open(EXPECTED, "rb") as f:
            expected = f.read().decode("utf-8")
        self.assertEqual(text, expected, "re-run with SPADE_LIBRARY_WRITE=1 after a reviewed change")

    def test_the_rule_by_hand(self):
        results, _ = tool.resample_all(PROPS, UIUC)
        a = by_id(results, "syn_a")
        # Hover: n = sqrt(m g / (N rho C_T0 D^4)) with C_T0 = 0.10 at the
        # static's middle rpm (6000): 4999.6 rpm, so the 5000 rpm run.
        self.assertEqual(a["run_rpm"], 5000.0)
        self.assertAlmostEqual(a["first_pass_rpm"], 4999.6, delta=0.1)
        # The reported hover rpm is the sim's: recomputed with the shipped
        # ct_static, 0.11 at the run's rpm (Physics, 2026-10-22).
        self.assertAlmostEqual(a["hover_rpm"], 4999.6 * math.sqrt(0.10 / 0.11), delta=0.1)
        self.assertEqual(a["hover_rpm"], hover_of(a, a["ct_static"]))
        # J = 0 is the static run at 5000 rpm: C_T 0.11, C_P 0.055.
        self.assertEqual(a["ct_table"][0][0], 0.0)
        self.assertAlmostEqual(a["ct_table"][0][1], 0.11, places=15)
        self.assertAlmostEqual(a["cq_table"][0][1], 0.055 / (2 * math.pi), places=15)
        # k from 1 on the 0.05 grid, through the run's last measured J
        # (0.42): 0, 0.05, ..., 0.40.
        self.assertEqual([j for j, _ in a["ct_table"]], [k / 20 for k in range(0, 9)])
        self.assertEqual([j for j, _ in a["cq_table"]], [k / 20 for k in range(0, 9)])
        ct = dict(a["ct_table"])
        # Below the first measured J (0.12): the static point to that point.
        self.assertAlmostEqual(ct[0.05], 0.11 - 0.01 * 0.05 / 0.12, places=15)
        self.assertAlmostEqual(ct[0.10], 0.11 - 0.01 * 0.10 / 0.12, places=15)
        # Between measured points.
        self.assertAlmostEqual(ct[0.15], 0.097, places=15)
        self.assertAlmostEqual(ct[0.25], 0.0855, places=15)
        self.assertAlmostEqual(ct[0.40], 0.063, places=15)
        cq = dict(a["cq_table"])
        self.assertAlmostEqual(cq[0.40], 0.0426 / (2 * math.pi), places=15)
        self.assertEqual(a["first_measured_j"], 0.12)
        self.assertEqual(a["last_measured_j"], 0.42)
        # The step check, never written into the table: the first two run
        # points extrapolated to J = 0 give C_T 0.112 and C_P 0.0524.
        self.assertAlmostEqual(a["step_ct"], (0.112 - 0.11) / 0.11, places=12)
        self.assertAlmostEqual(a["step_cp"], (0.0524 - 0.055) / 0.055, places=12)
        # The blade: the hub station, the 0.05 grid above it, the tip.
        self.assertEqual(a["r_over_r"], [0.15] + [k / 20 for k in range(4, 20)] + [1.0])
        self.assertEqual(a["hub_cutout"], 0.15)
        blade = dict(zip(a["r_over_r"], a["c_over_r"]))
        self.assertAlmostEqual(blade[0.30], 0.10 + 0.05 * 0.15 / 0.35, places=15)
        self.assertEqual(a["beta_deg"][-1], 12.0)
        self.assertEqual(a["blades"], 2)

        b = by_id(results, "syn_b")
        # 3398 rpm computed: the 3500 rpm run.
        self.assertEqual(b["run_rpm"], 3500.0)
        self.assertAlmostEqual(b["hover_rpm"], 3398.4, delta=0.2)
        self.assertEqual([j for j, _ in b["ct_table"]], [k / 20 for k in range(0, 7)])
        self.assertAlmostEqual(dict(b["ct_table"])[0.15], 0.07, places=15)
        # Hub 0.18 drops 0.20 (closer than 0.025); tip 0.91 drops 0.90.
        self.assertEqual(b["r_over_r"], [0.18] + [k / 20 for k in range(5, 18)] + [0.91])
        self.assertEqual(b["c_over_r"][0], 0.10)
        self.assertEqual(b["c_over_r"][-1], 0.05)

    def test_the_checks_are_reported_not_applied(self):
        results, warnings = tool.resample_all(PROPS, UIUC)
        b = by_id(results, "syn_b")
        self.assertAlmostEqual(b["step_ct"], -0.2, places=12)
        self.assertAlmostEqual(b["step_cp"], -0.12, places=12)
        self.assertTrue(any("syn_b" in w and "step" in w for w in warnings), warnings)
        self.assertTrue(any("syn_b" in w and "published" in w for w in warnings), warnings)
        self.assertFalse(any("syn_a" in w for w in warnings), warnings)
        # The table keeps the static point: no blend.
        self.assertEqual(b["ct_table"][0][1], 0.10)
        text, _ = tool.generate(PROPS, UIUC)
        for needle in ("step C_T -20.0%", "step C_P -12.0%", "first measured J 0.2",
                       "ends at r/R 0.91", "differs from the published 2500 rpm"):
            self.assertIn(needle, text)

    def test_a_static_only_prop_iterates_to_its_fixed_point(self):
        results, _ = tool.resample_all(PROPS, UIUC)
        c = by_id(results, "syn_c")
        self.assertIsNone(c["run_file"])
        self.assertEqual(c["ct_table"], [])
        self.assertEqual(c["cq_table"], [])
        # C_T = 1e-5 rpm on syn_c's line, so the fixed point rpm = 60
        # sqrt(K / (1e-5 rpm)) is rpm^3 = 3600 K / 1e-5: by hand, 10163.
        k = 0.00711 * tool.G / (4 * tool.RHO * 0.047**4)
        fixed = (3600 * k / 1e-5) ** (1 / 3)
        self.assertAlmostEqual(c["hover_rpm"], fixed, delta=0.5)
        # Self-consistent: the shipped ct_static is C_T at the hover rpm it
        # gives, to the iteration's 0.5 rpm, and the reported rpm is its own.
        self.assertAlmostEqual(c["ct_static"], 1e-5 * c["hover_rpm"], delta=1e-5 * 0.5)
        self.assertEqual(c["hover_rpm"], hover_of(c, c["ct_static"]))
        self.assertGreater(c["passes"], 3)
        self.assertLessEqual(c["passes"], 20)
        # A tabled prop's static values are its table's J = 0 row.
        a = by_id(results, "syn_a")
        self.assertEqual(a["ct_static"], a["ct_table"][0][1])
        self.assertEqual(a["cq_static"], a["cq_table"][0][1])
        text, _ = tool.generate(PROPS, UIUC)
        self.assertIn("static only: no wind-tunnel run", text)

    def test_no_published_hover_rpm_is_no_cross_check(self):
        results, warnings = tool.resample_all(PROPS, UIUC)
        self.assertIsNone(by_id(results, "syn_c")["hover_diff"])
        self.assertFalse(any("syn_c" in w for w in warnings), warnings)
        text, _ = tool.generate(PROPS, UIUC)
        self.assertIn("no published hover rpm to cross-check", text)


class EveryInputCarriesItsProvenance(unittest.TestCase):
    def test_url_date_and_sha256_of_each_file_read(self):
        results, _ = tool.resample_all(PROPS, UIUC)
        a = by_id(results, "syn_a")
        names = [i["file"] for i in a["inputs"]]
        self.assertEqual(names, ["syn_a_static.txt", "syn_a_run_5000.txt", "syn_a_geom.txt"])
        for i in a["inputs"]:
            with open(os.path.join(UIUC, i["file"]), "rb") as f:
                body = f.read()
            self.assertEqual(i["sha256"], hashlib.sha256(body).hexdigest())
            self.assertEqual(i["fnv1a64"], tool.fnv1a64(body))
            self.assertEqual(i["url"], "synthetic:" + i["file"])
            self.assertEqual(i["retrieved_utc"], "2026-10-22T00:00:00Z")
        text, _ = tool.generate(PROPS, UIUC)
        self.assertIn('"synthetic:syn_a_run_5000.txt"', text)
        self.assertIn('"2026-10-22T00:00:00Z"', text)
        # A static-only prop reads no run.
        self.assertEqual([i["file"] for i in by_id(results, "syn_c")["inputs"]], ["syn_c_static.txt", "syn_c_geom.txt"])


class TheReviewsMinors(unittest.TestCase):
    def test_a_tie_takes_the_lower_rpm(self):
        self.assertEqual(tool.nearest_run([(6000.0, "b"), (4000.0, "a")], 5000.0), (4000.0, "a"))
        self.assertEqual(tool.nearest_run([(4000.0, "a"), (6000.0, "b")], 5000.0), (4000.0, "a"))
        self.assertEqual(tool.nearest_run([(4000.0, "a"), (6000.0, "b")], 5001.0), (6000.0, "b"))

    def test_a_comment_never_splits_a_hyphenated_word(self):
        lines = tool.comment("word " * 13 + "tri-blade tables: mit_5x4 (three-blade, 5 in)")
        self.assertTrue(any("(three-blade," in ln for ln in lines), lines)

    def test_the_header_is_ascii_and_within_120_columns(self):
        # MSVC reads a source without /utf-8 in the local code page, so the
        # header stays ASCII; a long note becomes adjacent literals.
        long_note = "a note " * 40
        lines = tool.literal_lines(long_note, "        ")
        self.assertGreater(len(lines), 1)
        self.assertTrue(all(len(ln) <= 120 for ln in lines), lines)
        joined = "".join(ln.strip()[1:-1] for ln in lines)
        self.assertEqual(joined, long_note)
        with self.assertRaisesRegex(tool.ToolError, r"not ASCII.*line"):
            tool.ascii_only("ok\nbad \u00b1 here\n")

    def test_each_prop_carries_its_grade_note(self):
        text, _ = tool.generate(PROPS, UIUC)
        self.assertIn('        "derived from syn_a\'s shape, assumed",\n', text)
        self.assertIn('        "",\n', text)  # syn_a has none


class TheToolRefusesByName(unittest.TestCase):
    def _with(self, files, props):
        d = tempfile.mkdtemp()
        for name, body in files.items():
            with open(os.path.join(d, name), "w", encoding="utf-8") as f:
                f.write(body)
        p = os.path.join(d, "props.json")
        with open(p, "w", encoding="utf-8") as f:
            f.write(props)
        return p, d

    def _props(self, runs):
        return ('{"props": [{"id": "x", "volume": "v", "retrieved": "r", "blades": 2, '
                '"diameter_m": 0.254, "static": "s.txt", "runs": ' + runs + ', "geometry": "g.txt", '
                '"drone": {"name": "d", "mass_kg": 1.444, "rotors": 4, "published_hover_rpm": 5000, '
                '"published_source": "s"}}], "sources": {' + ", ".join(
                    f'"{n}": {{"url": "u", "retrieved_utc": "d"}}'
                    for n in ("s.txt", "g.txt", "r_5000.txt", "r_9000.txt", "run.txt")) + '}}')

    STATIC = "RPM CT CP\n4000 0.12 0.06\n8000 0.09 0.045\n"
    GEOM = "r/R c/R beta\n0.15 0.1 30\n1.0 0.05 10\n"

    def test_a_run_out_of_order(self):
        p, d = self._with({"s.txt": self.STATIC, "g.txt": self.GEOM,
                           "r_5000.txt": "J CT CP eta\n0.3 0.1 0.05 0.3\n0.2 0.09 0.04 0.4\n"},
                          self._props('["r_5000.txt"]'))
        with self.assertRaisesRegex(tool.ToolError, r"r_5000\.txt.*strictly increasing"):
            tool.resample_all(p, d)

    def test_a_run_outside_the_static_range(self):
        p, d = self._with({"s.txt": self.STATIC, "g.txt": self.GEOM,
                           "r_9000.txt": "J CT CP eta\n0.1 0.1 0.05 0.3\n0.2 0.09 0.04 0.4\n"},
                          self._props('["r_9000.txt"]'))
        with self.assertRaisesRegex(tool.ToolError, r"9000.*outside.*s\.txt"):
            tool.resample_all(p, d)

    def test_a_missing_file(self):
        p, d = self._with({"s.txt": self.STATIC, "g.txt": self.GEOM}, self._props('["r_5000.txt"]'))
        with self.assertRaisesRegex(tool.ToolError, r"r_5000\.txt"):
            tool.resample_all(p, d)

    def test_a_file_without_a_source(self):
        props = self._props('["r_5000.txt"]').replace('"g.txt": {"url": "u", "retrieved_utc": "d"}, ', "")
        p, d = self._with({"s.txt": self.STATIC, "g.txt": self.GEOM,
                           "r_5000.txt": "J CT CP eta\n0.1 0.1 0.05 0.3\n0.2 0.09 0.04 0.4\n"}, props)
        with self.assertRaisesRegex(tool.ToolError, r"g\.txt.*no source"):
            tool.resample_all(p, d)

    def test_a_static_only_hover_that_does_not_settle(self):
        # C_T so steep in rpm (-0.2 + 3e-5 rpm) that n = sqrt(K / C_T(n))
        # moves away from its fixed point on each pass.
        props = self._props("[]").replace('"mass_kg": 1.444', '"mass_kg": 1.7')
        p, d = self._with({"s.txt": "RPM CT CP\n7000 0.01 0.01\n12000 0.16 0.05\n", "g.txt": self.GEOM}, props)
        with self.assertRaisesRegex(tool.ToolError, r"x:.*(settle|outside)"):
            tool.resample_all(p, d)

    def test_a_run_name_without_its_rpm(self):
        p, d = self._with({"s.txt": self.STATIC, "g.txt": self.GEOM,
                           "run.txt": "J CT CP eta\n0.1 0.1 0.05 0.3\n0.2 0.09 0.04 0.4\n"},
                          self._props('["run.txt"]'))
        with self.assertRaisesRegex(tool.ToolError, r"run\.txt.*rpm"):
            tool.resample_all(p, d)


class TheHashIsFnv1a64OfLfText(unittest.TestCase):
    def test_known_values_and_crlf(self):
        self.assertEqual(tool.fnv1a64(b""), 0xCBF29CE484222325)
        self.assertEqual(tool.fnv1a64(b"a"), 0xAF63DC4C8601EC8C)
        self.assertEqual(tool.fnv1a64(b"a\r\nb\r\n"), tool.fnv1a64(b"a\nb\n"))


if __name__ == "__main__":
    unittest.main()
