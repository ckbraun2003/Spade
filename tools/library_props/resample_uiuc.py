"""resample_uiuc.py -- the default library's propeller tables and blade
definitions, resampled from the UIUC Propeller Database (Task L4a of
docs/design/core/plans/2026-10-21-default-drone-library-plan.md).

    python tools/library_props/resample_uiuc.py --props tools/library_props/props.json
        --uiuc <the folder of UIUC files> --out tests/library_props.gen.hpp

The UIUC files are downloaded by hand and never committed (ruling 3). This
writes our own resampling of them as C++ constants, in shortest round-trip
doubles, so the C++ side never recomputes a number. Standard library only.
Tests: test_resample_uiuc.py beside it, run by hand whenever this file or its
input changes (the lead, 2026-10-22).

THE RULE (reviewed by Physics, 2026-10-22; the plan's Rulings):
  run     Per prop, the wind-tunnel run whose rpm (its file name's trailing
          _<rpm>.txt) is nearest the drone's hover rpm, n = sqrt(m g /
          (N rho C_T0 D^4)) with rho 1.225, where C_T0 is the static C_T at
          the static run's middle rpm. A tie takes the lower rpm.
  hover   The hover rpm reported is the sim's: n again, with the shipped
          static C_T0. A static-only prop iterates n = sqrt(m g / (N rho
          C_T(n) D^4)) to a fixed point (|dn| < 0.5 rpm, at most 20 passes,
          refused by name if unsettled), so its static point is C_T at the
          hover it gives.
  J = 0   C_T and C_P from the static run, linear in rpm at the run's rpm.
  table   Linear in J at J = k/20 for k = 1, 2, ... through the run's last
          measured J, never past it. Below its first measured J the line runs
          from the static point to that point: interpolation between two
          measurements. C_Q = C_P / 2 pi.
  blade   The geometry file's first station, the hub cut-out, exactly; the
          grid r/R = 0.15, 0.20, ..., 1.00 above it; and its last station,
          dropping any grid point closer than 0.025 to either. c/R and beta
          are linear in r/R. The blade count is the props table's, from the
          prop's name.
  checks  Reported, never applied: the run's first two points extrapolated to
          J = 0 against the static C_T and C_P (a warning above 10%), and the
          computed hover rpm against the published one (above 20%), where one
          is published (null where none is).

HASHES AND PROVENANCE. FNV-1a 64 of each file read, CRLF read as LF so a
Windows checkout and the leg agree. The header stamps the tool's and the props
table's, which tests/test_library_props.cpp recomputes from the source tree.
For each UIUC input it records the FNV-1a 64 and the SHA-256, with the URL and
UTC retrieval date from the props table's `sources` (the lead, 2026-10-22).
The inputs themselves are not committed, so these are recorded, not rechecked;
a file with no source is refused.

STATIC ONLY. A prop with no runs (UIUC measured some static only) gets its
static C_T and C_Q at the hover rpm and empty tables; the header says so.
"""

import argparse
import hashlib
import json
import math
import os
import re
import sys
import textwrap

RHO = 1.225  # kg/m^3, the hover rpm's air
G = 9.80665  # m/s^2
STEP_WARN = 0.10
HOVER_WARN = 0.20
EDGE = 0.025  # a blade grid point closer than this to the hub or tip is dropped
SETTLE_RPM = 0.5  # a static-only hover's fixed point, to this
MAX_PASSES = 20
EPS = 1e-9


class ToolError(Exception):
    """A refusal, naming the prop or the file."""


def fnv1a64(data):
    """FNV-1a 64 of `data`, CRLF read as LF."""
    h = 0xCBF29CE484222325
    for byte in data.replace(b"\r\n", b"\n"):
        h ^= byte
        h = (h * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return h


def hash_file(path):
    with open(path, "rb") as f:
        return fnv1a64(f.read())


def read_columns(path, wanted):
    """The `wanted` columns (by header name, ignoring case) of a UIUC text
    file: a header line, then rows of numbers, the first wanted column
    strictly increasing."""
    name = os.path.basename(path)
    try:
        with open(path, encoding="utf-8") as f:
            lines = [(n, ln.split()) for n, ln in enumerate(f, start=1) if ln.strip()]
    except OSError as e:
        raise ToolError(f"{name}: cannot be read ({e.strerror})") from None
    if not lines:
        raise ToolError(f"{name}: is empty")
    header = [h.lower() for h in lines[0][1]]
    columns = []
    for w in wanted:
        if w not in header:
            raise ToolError(f"{name}: has no column '{w}' in its header {' '.join(lines[0][1])}")
        columns.append(header.index(w))
    rows = []
    for n, fields in lines[1:]:
        try:
            values = [float(fields[c]) for c in columns]
        except (IndexError, ValueError):
            raise ToolError(f"{name}: line {n} is not a row of numbers: {' '.join(fields)}") from None
        if not all(math.isfinite(v) for v in values):
            raise ToolError(f"{name}: line {n} has a value that is not finite")
        rows.append(values)
    if len(rows) < 2:
        raise ToolError(f"{name}: needs at least 2 rows, has {len(rows)}")
    for a, b in zip(rows, rows[1:]):
        if not b[0] > a[0]:
            raise ToolError(f"{name}: its {wanted[0]} column must be strictly increasing: {a[0]!r} then {b[0]!r}")
    return rows


def interp(points, x):
    """Linear in x between the bracketing (x, y) points; x within them."""
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        if x0 <= x <= x1:
            if x == x0:
                return y0
            if x == x1:
                return y1
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    raise ValueError(f"{x!r} is outside {points[0][0]!r} to {points[-1][0]!r}")


def run_rpm_of(name):
    m = re.search(r"_(\d+)\.txt$", name)
    if not m:
        raise ToolError(f"{name}: a run's file name must end in _<rpm>.txt, naming its rpm")
    return float(m.group(1))


def nearest_run(runs, rpm):
    """The (rpm, file) run nearest `rpm`; a tie takes the lower rpm."""
    return min(runs, key=lambda r: (abs(r[0] - rpm), r[0]))


def field(entry, key, where):
    if key not in entry:
        raise ToolError(f"{where}: has no '{key}'")
    return entry[key]


def provenance(name, path, sources):
    """A file the tool read: its name, its hashes and where it came from."""
    if name not in sources:
        raise ToolError(f"{name}: has no source in the props table's sources (its URL and UTC retrieval date)")
    src = sources[name]
    with open(path, "rb") as f:
        body = f.read()
    return {
        "file": name,
        "fnv1a64": fnv1a64(body),
        "sha256": hashlib.sha256(body).hexdigest(),
        "url": field(src, "url", name),
        "retrieved_utc": field(src, "retrieved_utc", name),
    }


def resample(prop, uiuc_dir, sources):
    """One prop's tables, blade and checks, by THE RULE."""
    pid = field(prop, "id", "a prop")
    drone = field(prop, "drone", pid)
    diameter = float(field(prop, "diameter_m", pid))
    mass = float(field(drone, "mass_kg", pid + ".drone"))
    rotors = int(field(drone, "rotors", pid + ".drone"))
    # The cross-check figure, or null where none is published.
    published = field(drone, "published_hover_rpm", pid + ".drone")
    published = None if published is None else float(published)

    static_name = field(prop, "static", pid)
    static_path = os.path.join(uiuc_dir, static_name)
    static = read_columns(static_path, ["rpm", "ct", "cp"])
    static_ct = [(r[0], r[1]) for r in static]
    static_cp = [(r[0], r[2]) for r in static]
    lo, hi = static[0][0], static[-1][0]

    # n = sqrt(m g / (N rho C_T0 D^4)), in rpm.
    def hover_at(ct):
        return 60.0 * math.sqrt(mass * G / (rotors * RHO * ct * diameter**4))

    # The first pass, with C_T0 at the static's middle rpm, picks the run.
    first_pass_rpm = hover_at(interp(static_ct, (lo + hi) / 2))
    runs = [(run_rpm_of(r), r) for r in field(prop, "runs", pid)]
    paths = [(static_name, static_path)]
    passes = 1

    if not runs:
        # Static only: its static point IS the hover point, so iterate n =
        # hover_at(C_T(n)) to its fixed point (Physics, 2026-10-22).
        rpm = first_pass_rpm
        while True:
            if not lo <= rpm <= hi:
                raise ToolError(f"{pid}: has no run, and its hover rpm {rpm:.0f} is outside {static_name}'s "
                                f"{lo:g} to {hi:g} rpm")
            settled = hover_at(interp(static_ct, rpm))
            if abs(settled - rpm) < SETTLE_RPM:
                break
            if passes == MAX_PASSES:
                raise ToolError(f"{pid}: its static-only hover rpm did not settle in {MAX_PASSES} passes "
                                f"({rpm:.1f}, then {settled:.1f})")
            rpm = settled
            passes += 1
        if not lo <= settled <= hi:
            raise ToolError(f"{pid}: has no run, and its hover rpm {settled:.0f} is outside {static_name}'s "
                            f"{lo:g} to {hi:g} rpm")
        run_name = run_rpm = first_j = last_j = step_ct = step_cp = None
        ct0 = interp(static_ct, settled)
        cp0 = interp(static_cp, settled)
        ct_table, cq_table = [], []
    else:
        run_rpm, run_name = nearest_run(runs, first_pass_rpm)
        if not lo <= run_rpm <= hi:
            raise ToolError(f"{pid}: the run {run_name}'s {run_rpm:g} rpm is outside {static_name}'s "
                            f"{lo:g} to {hi:g} rpm")
        run_path = os.path.join(uiuc_dir, run_name)
        run = read_columns(run_path, ["j", "ct", "cp"])
        paths.append((run_name, run_path))
        if not run[0][0] > 0.0:
            raise ToolError(f"{run_name}: its first J must be above 0: J = 0 is the static run's")
        ct0 = interp(static_ct, run_rpm)
        cp0 = interp(static_cp, run_rpm)
        ct_points = [(0.0, ct0)] + [(r[0], r[1]) for r in run]
        cp_points = [(0.0, cp0)] + [(r[0], r[2]) for r in run]
        first_j, last_j = run[0][0], run[-1][0]
        grid = [0.0]
        k = 1
        while k / 20 <= last_j:
            grid.append(k / 20)
            k += 1
        ct_table = [(j, interp(ct_points, j)) for j in grid]
        cq_table = [(j, interp(cp_points, j) / (2 * math.pi)) for j in grid]
        # The step check: the run's first two points, extrapolated to J = 0.
        (j0, t0, p0), (j1, t1, p1) = run[0], run[1]
        step_ct = (t0 - (t1 - t0) * j0 / (j1 - j0) - ct0) / ct0
        step_cp = (p0 - (p1 - p0) * j0 / (j1 - j0) - cp0) / cp0

    geom_name = field(prop, "geometry", pid)
    geom_path = os.path.join(uiuc_dir, geom_name)
    geom = read_columns(geom_path, ["r/r", "c/r", "beta"])
    paths.append((geom_name, geom_path))
    hub, tip = geom[0][0], geom[-1][0]
    if not (0.0 <= hub and tip <= 1.0):
        raise ToolError(f"{geom_name}: its stations must lie in [0, 1], found {hub!r} to {tip!r}")
    nodes = [hub]
    nodes += [k / 20 for k in range(3, 21) if k / 20 - hub >= EDGE - EPS and tip - k / 20 >= EDGE - EPS]
    nodes.append(tip)
    chord = [(r[0], r[1]) for r in geom]
    twist = [(r[0], r[2]) for r in geom]

    return {
        "id": pid,
        "volume": field(prop, "volume", pid),
        "retrieved": field(prop, "retrieved", pid),
        "blades": int(field(prop, "blades", pid)),
        "diameter": diameter,
        "drone": drone,
        "note": prop.get("note", ""),
        "run_file": run_name,
        "run_rpm": run_rpm,
        # The rpm the sim hovers at: recomputed with the shipped ct_static.
        "hover_rpm": hover_at(ct0),
        "first_pass_rpm": first_pass_rpm,
        "passes": passes,
        "published_hover_rpm": published,
        "hover_diff": None if published is None else (hover_at(ct0) - published) / published,
        "ct_static": ct0,
        "cq_static": cp0 / (2 * math.pi),
        "ct_table": ct_table,
        "cq_table": cq_table,
        "first_measured_j": first_j,
        "last_measured_j": last_j,
        "step_ct": step_ct,
        "step_cp": step_cp,
        "r_over_r": nodes,
        "c_over_r": [interp(chord, r) for r in nodes],
        "beta_deg": [interp(twist, r) for r in nodes],
        "hub_cutout": hub,
        "tip": tip,
        "inputs": [provenance(name, path, sources) for name, path in paths],
    }


def pct(x):
    return f"{100 * x:+.1f}%"


def stepped(r):
    return r["step_ct"] is not None and (abs(r["step_ct"]) > STEP_WARN or abs(r["step_cp"]) > STEP_WARN)


def warnings_of(r):
    out = []
    if stepped(r):
        out.append(f"{r['id']}: the static-to-run step at J = 0 is {pct(r['step_ct'])} on C_T and "
                   f"{pct(r['step_cp'])} on C_P (above 10%): review it")
    if r["hover_diff"] is not None and abs(r["hover_diff"]) > HOVER_WARN:
        out.append(f"{r['id']}: the computed hover rpm {r['hover_rpm']:.0f} differs from the published "
                   f"{r['published_hover_rpm']:g} rpm ({r['drone'].get('published_source', '')}) by "
                   f"{pct(r['hover_diff'])}: check the mass, the prop or the figure")
    return out


def resample_all(props_path, uiuc_dir):
    """Every prop of the props table: (results, warnings)."""
    try:
        with open(props_path, encoding="utf-8") as f:
            table = json.load(f)
    except (OSError, ValueError) as e:
        raise ToolError(f"{os.path.basename(props_path)}: cannot be read as JSON ({e})") from None
    where = os.path.basename(props_path)
    sources = field(table, "sources", where)
    results = [resample(p, uiuc_dir, sources) for p in field(table, "props", where)]
    warnings = [w for r in results for w in warnings_of(r)]
    return results, warnings


def number(x):
    """A C++ double literal that reads back to x bit for bit."""
    if not math.isfinite(x):
        raise ToolError(f"{x!r} is not finite")
    return repr(float(x))


def literal_lines(s, indent):
    """`s` as one C++ string literal, or as adjacent literals of at most 90
    characters each when it would pass 120 columns; they join to `s`."""
    if len(indent) + len(text_literal(s)) + 1 <= 120:
        return [indent + text_literal(s)]
    pieces = textwrap.wrap(s, 90, drop_whitespace=False, break_on_hyphens=False, break_long_words=False,
                           replace_whitespace=False)
    if "".join(pieces) != s:
        raise ToolError(f"cannot split a note into literals: {s[:40]!r}...")
    return [indent + text_literal(p) for p in pieces]


def ascii_only(text):
    """`text`, refused by line if it is not ASCII: MSVC reads a source
    without /utf-8 in the local code page."""
    for n, line in enumerate(text.split("\n"), start=1):
        if not line.isascii():
            raise ToolError(f"the header is not ASCII at line {n}: {line.strip()[:60]!r}")
    return text


def text_literal(s):
    return '"' + s.replace("\\", "\\\\").replace('"', '\\"') + '"'


def comment(text, indent="//   "):
    return [indent + line for line in textwrap.wrap(text, 76, break_on_hyphens=False)]


def array_lines(declaration, rows):
    """`declaration{` rows `};`, or `declaration{};` for none."""
    if not rows:
        return [declaration + "{};"]
    return [declaration + "{{"] + rows + ["}};"]


def emit_prop(r):
    ident = re.sub(r"[^0-9A-Za-z]", "_", r["id"])
    drone = r["drone"]
    who = (f"{drone.get('name', '')} ({drone['mass_kg']:g} kg, {drone['rotors']} rotors, "
           f"D {r['diameter']:g} m)")
    lines = [f"// {r['id']} -- {r['volume']}, retrieved {r['retrieved']}, {r['blades']} blades."]
    if r["run_file"] is None:
        lines += comment(f"static only: no wind-tunnel run. C_T and C_Q are the static run's at the hover rpm "
                         f"{r['hover_rpm']:.0f} computed for {who}, its fixed point in {r['passes']} passes; "
                         f"the tables are empty.")
    else:
        lines += comment(f"run {r['run_file']} at {r['run_rpm']:g} rpm, the nearest to the first-pass hover rpm "
                         f"{r['first_pass_rpm']:.0f} computed for {who}. The sim hovers at "
                         f"{r['hover_rpm']:.0f} rpm on the shipped ct_static: a Reynolds ratio run/hover of "
                         f"{r['run_rpm'] / r['hover_rpm']:.2f}.")
    if r["hover_diff"] is None:
        hover = "no published hover rpm to cross-check"
    else:
        hover = f"published hover {r['published_hover_rpm']:g} rpm ({drone.get('published_source', '')})"
    if r["hover_diff"] is not None and abs(r["hover_diff"]) > HOVER_WARN:
        hover = (f"FLAG: the computed hover rpm differs from the published {r['published_hover_rpm']:g} rpm "
                 f"({drone.get('published_source', '')}) by {pct(r['hover_diff'])}")
    lines += comment(hover + ".")
    if r["run_file"] is not None:
        lines += comment(f"step C_T {pct(r['step_ct'])}, step C_P {pct(r['step_cp'])} at J = 0 (report only"
                         + (", above 10%: review it" if stepped(r) else "")
                         + f"); first measured J {r['first_measured_j']!r}, last {r['last_measured_j']!r}.")
    if r["tip"] < 1.0:
        lines += comment(f"the blade ends at r/R {r['tip']!r}, the file's last station (no extrapolation).")
    if r["note"]:
        lines += comment("grade note: " + r["note"])
    n = len(r["ct_table"])
    for key, label in (("ct_table", "ct"), ("cq_table", "cq")):
        lines += array_lines(f"inline constexpr std::array<std::array<double, 2>, {n}> {ident}_{label}",
                             [f"    {{{number(j)}, {number(v)}}}," for j, v in r[key]])
    m = len(r["r_over_r"])
    for key in ("r_over_r", "c_over_r", "beta_deg"):
        lines.append(f"inline constexpr std::array<double, {m}> {ident}_{key}{{")
        lines += [f"    {number(v)}," for v in r[key]]
        lines.append("};")
    rows = []
    for i in r["inputs"]:
        rows += ["    Input{",
                 f"        {text_literal(i['file'])},",
                 f"        0x{i['fnv1a64']:016x}ull,",
                 f"        {text_literal(i['sha256'])},",
                 f"        {text_literal(i['url'])},",
                 f"        {text_literal(i['retrieved_utc'])},",
                 "    },"]
    lines += array_lines(f"inline constexpr std::array<Input, {len(r['inputs'])}> {ident}_inputs", rows)
    zero_if_none = lambda v: number(0.0 if v is None else v)  # noqa: E731
    notes = literal_lines(r["note"], "        ")
    entry = [
        "    Prop{",
        f"        {text_literal(r['id'])},",
        f"        {text_literal(r['volume'])},",
        f"        {text_literal(r['retrieved'])},",
        f"        {r['blades']},",
        f"        {zero_if_none(r['run_rpm'])},",
        f"        {number(r['hover_rpm'])},",
        f"        {number(r['ct_static'])},",
        f"        {number(r['cq_static'])},",
        f"        {zero_if_none(r['first_measured_j'])},",
        f"        {zero_if_none(r['last_measured_j'])},",
        f"        detail::{ident}_ct,",
        f"        detail::{ident}_cq,",
        f"        detail::{ident}_r_over_r,",
        f"        detail::{ident}_c_over_r,",
        f"        detail::{ident}_beta_deg,",
        f"        {number(r['hub_cutout'])},",
        f"        detail::{ident}_inputs,",
        *[ln + "," if i == len(notes) - 1 else ln for i, ln in enumerate(notes)],
        "    },",
    ]
    return lines, entry


HEADER_TOP = """\
// =============================================================================
// GENERATED by tools/library_props/resample_uiuc.py -- do not edit; re-run it.
// Our own resampling of the UIUC Propeller Database's runs and geometries
// (ruling 3: their files are never committed). THE RULE and its checks are
// the tool's docstring (Physics' review, 2026-10-22).
// =============================================================================
#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <string_view>

namespace spade::test::library_props {

// FNV-1a 64 of the tool and of its props table, CRLF read as LF.
// tests/test_library_props.cpp recomputes both from the source tree, so an
// edit to either without re-running the tool fails the suite.
"""

HEADER_TYPES = """
// A file the tool read: its name, FNV-1a 64 (CRLF read as LF) and SHA-256,
// and where it came from. The UIUC files are not committed, so these are
// recorded, not rechecked.
struct Input {
    std::string_view file;
    std::uint64_t hash;
    std::string_view sha256;
    std::string_view url;
    std::string_view retrieved_utc;
};

// One prop: its static point, its tables (the static point at J = 0 first,
// then J = k/20; empty for a static-only prop), its blade stations (the hub
// cut-out first) and where they came from.
struct Prop {
    std::string_view id;
    std::string_view volume;
    std::string_view retrieved;
    int blades;
    double run_rpm;  // 0 for a static-only prop
    double hover_rpm;  // computed from the drone
    double ct_static;
    double cq_static;  // C_P / 2 pi
    double first_measured_j;  // 0 for a static-only prop
    double last_measured_j;   // 0 for a static-only prop
    std::span<const std::array<double, 2>> ct_table;  // [J, C_T]
    std::span<const std::array<double, 2>> cq_table;  // [J, C_Q], C_Q = C_P / 2 pi
    std::span<const double> r_over_r;
    std::span<const double> c_over_r;
    std::span<const double> beta_deg;
    double hub_cutout;
    std::span<const Input> inputs;
    std::string_view note;  // the grade note, for the part file's citation (L4)
};

namespace detail {
"""


def generate(props_path, uiuc_dir):
    """The header's text, and the warnings."""
    results, warnings = resample_all(props_path, uiuc_dir)
    tool_hash = hash_file(os.path.abspath(__file__))
    props_hash = hash_file(props_path)
    out = [HEADER_TOP.rstrip("\n")]
    out.append(f"inline constexpr std::uint64_t kToolHash = 0x{tool_hash:016x}ull;   // resample_uiuc.py")
    out.append(f"inline constexpr std::uint64_t kPropsHash = 0x{props_hash:016x}ull;  // "
               f"{os.path.basename(props_path)}")
    out.append(HEADER_TYPES.rstrip("\n"))
    entries = []
    for r in results:
        lines, entry = emit_prop(r)
        out.append("")
        out += lines
        entries += entry
    out.append("")
    out.append("}  // namespace detail")
    out.append("")
    out.append(f"inline constexpr std::array<Prop, {len(results)}> kProps{{{{")
    out += entries
    out.append("}};")
    out.append("")
    out.append("}  // namespace spade::test::library_props")
    return ascii_only("\n".join(out) + "\n"), warnings


def main(argv=None):
    parser = argparse.ArgumentParser(description="Resample UIUC propeller data into tests/library_props.gen.hpp.")
    parser.add_argument("--props", required=True, help="the props table (tools/library_props/props.json)")
    parser.add_argument("--uiuc", required=True, help="the folder of hand-downloaded UIUC files")
    parser.add_argument("--out", required=True, help="the header to write")
    args = parser.parse_args(argv)
    try:
        text, warnings = generate(args.props, args.uiuc)
    except ToolError as e:
        print("resample_uiuc: " + str(e), file=sys.stderr)
        return 1
    for w in warnings:
        print("warning: " + w, file=sys.stderr)
    with open(args.out, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
