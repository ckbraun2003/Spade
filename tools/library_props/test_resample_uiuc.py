"""Tests for resample_uiuc.py (L4a of the default-drone library plan).

Run by hand whenever the tool or its input changes (the lead's ruling,
2026-10-22), and name it with its output in READY:

    python tools/library_props/test_resample_uiuc.py

With SPADE_LIBRARY_WRITE=1 the synthetic expected header is rewritten
instead of compared, as the repository's other generators do. The input
under testdata/ is our own numbers in UIUC's file format, never UIUC's,
and every value checked by hand below was computed from it on paper.
"""

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
        self.assertAlmostEqual(a["hover_rpm"], 4999.6, delta=0.1)
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
                '"published_source": "s"}}]}')

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
