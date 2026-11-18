#!/usr/bin/env python3
"""A stand-in for tools/step_convert/step_convert.py with the same new modes (--probe, --inspect, --emit-3mf), for
tests/test_part_import_step.cpp: it needs no OpenCASCADE and reads no STEP (the input only has to exist). It "holds" two
solids, `arm` (a 10 mm cube) and `hub` (a 20 mm cube at x = 40 mm), in millimetres.

FAKE_STEP_MODE (environment) picks a misbehaviour: slow (--emit-3mf sleeps a minute: the cancel test), unreadable (exit 3,
a message), refuse (exit 4), garbage (exit 0 with stdout that is not JSON), flag (the cross-check flags `arm`: exit 5, the
3MF is still written), noocp (--probe says OpenCASCADE is not installed; every other mode exits 3).
Every run appends its argument list as one JSON line to FAKE_STEP_LOG when that is set.
"""
import json
import os
import sys
import time
import zipfile

SOLIDS = {"arm": (10.0, 0.0), "hub": (20.0, 40.0)}  # name: (edge, x offset), millimetres
MODE = os.environ.get("FAKE_STEP_MODE", "")
SKIPPED = [{"name": "cover", "code": "no_solid"}] if MODE == "sheet" else []  # a surface-only part the real converter leaves out


def cube(edge, dx):
    v = [(dx + x * edge, y * edge, z * edge) for x in (0, 1) for y in (0, 1) for z in (0, 1)]
    t = [(0, 2, 3), (0, 3, 1), (4, 5, 7), (4, 7, 6), (0, 1, 5), (0, 5, 4), (2, 6, 7), (2, 7, 3), (0, 4, 6), (0, 6, 2),
         (1, 3, 7), (1, 7, 5)]
    return v, [(a, c, b) for a, b, c in t]  # outward-wound


def write_3mf(path, names):
    rows = ['<?xml version="1.0" encoding="UTF-8"?>',
            '<model unit="millimeter" xml:lang="en-US" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">',
            "<resources>"]
    for n, name in enumerate(names, 1):
        v, t = cube(*SOLIDS[name])
        rows.append(f'<object id="{n}" name="{name}" type="model"><mesh><vertices>')
        rows += [f'<vertex x="{x!r}" y="{y!r}" z="{z!r}"/>' for x, y, z in v]
        rows.append("</vertices><triangles>")
        rows += [f'<triangle v1="{a}" v2="{b}" v3="{c}"/>' for a, b, c in t]
        rows.append("</triangles></mesh></object>")
    rows.append("</resources><build>")
    rows += [f'<item objectid="{n}"/>' for n in range(1, len(names) + 1)]
    rows.append("</build></model>")
    types = ('<?xml version="1.0" encoding="UTF-8"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/'
             'content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.'
             'relationships+xml"/><Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-'
             '3dmodel+xml"/></Types>')
    rels = ('<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.openxmlformats.org/package/'
            '2006/relationships"><Relationship Id="rel0" Target="/3D/3dmodel.model" Type="http://schemas.'
            'microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", types)
        z.writestr("_rels/.rels", rels)
        z.writestr("3D/3dmodel.model", "\n".join(rows).encode("utf-8"))


def say(obj):
    sys.stdout.write(json.dumps(obj) + "\n")


def main(argv):
    log = os.environ.get("FAKE_STEP_LOG")
    if log:
        with open(log, "a", encoding="utf-8") as f:
            f.write(json.dumps(argv) + "\n")
    if "--probe" in argv:
        if MODE == "noocp":
            say({"ocp": False, "python": sys.version.split()[0], "reason": "OCP is not installed"})
            return 0
        say({"ocp": True, "python": sys.version.split()[0], "occt": "7.8.1-fake", "wheel": "fake-1"})
        return 0
    if MODE == "noocp":
        sys.stderr.write("step_convert: OCP is not installed\n")
        return 3
    if MODE == "unreadable":
        sys.stderr.write("step_convert: part.step: not a STEP file OpenCASCADE reads\n")
        return 3
    if MODE == "refuse":
        sys.stderr.write("step_convert: arm: make_mesh refused the mesh (not closed)\n")
        return 4
    if MODE == "garbage":
        sys.stdout.write("this is not json\n")
        return 0
    args = [a for a in argv]
    source = next((a for a in args if a.lower().endswith((".step", ".stp"))), None)
    if source is None or not os.path.isfile(source):
        sys.stderr.write("step_convert: cannot read the input\n")
        return 1
    wanted = [args[i + 1] for i, a in enumerate(args) if a == "--solid"]
    names = [n for n in SOLIDS if not wanted or n in wanted]
    if "--inspect" in args:
        say({"unit": "mm", "occt": "7.8.1-fake", "wheel": "fake-1", "skipped": SKIPPED,
             "solids": [{"name": n, "volume": SOLIDS[n][0] ** 3, "bbox_min": [SOLIDS[n][1], 0.0, 0.0],
                         "bbox_max": [SOLIDS[n][1] + SOLIDS[n][0], SOLIDS[n][0], SOLIDS[n][0]]} for n in names]})
        return 0
    if "--emit-3mf" in args:
        out = args[args.index("--emit-3mf") + 1]
        if MODE == "slow":
            time.sleep(60)
        write_3mf(out, names)
        flagged = ["arm"] if MODE == "flag" and "arm" in names else []
        say({"unit": "mm", "occt": "7.8.1-fake", "wheel": "fake-1",
             "cross_check": [{"name": n, "brep_volume": SOLIDS[n][0] ** 3,
                              "mesh_volume": SOLIDS[n][0] ** 3 * (0.9 if n in flagged else 1.0),
                              "difference": 0.1 if n in flagged else 0.0} for n in names],
             "flagged": flagged, "skipped": SKIPPED})
        return 5 if flagged else 0
    sys.stderr.write("step_convert: no mode\n")
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
