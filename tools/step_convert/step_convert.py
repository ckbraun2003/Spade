#!/usr/bin/env python3
"""step_convert.py -- a STEP file to one Spade mesh sidecar (.spmesh) per solid. Offline; CORE-6 D.

    python step_convert.py IN.step --out DIR --deflection 0.1mm [--angle 0.5] [--solid NAME]...
                           [--pack PATH/spade_mesh_pack]

WHAT IT DOES. Reads the STEP with OpenCASCADE (OCP, from the pinned cadquery-ocp wheels in requirements.txt), keeping
the part names, the assembly placements and the file's length unit. Tessellates each solid at the deflection given,
writes a core-specification 3MF (one object per solid instance, named, in the file's unit, its placement as the build
item's transform), and runs spade_mesh_pack on it, so every mesh passes Spade's own reader and make_mesh() before a
sidecar exists. OpenCASCADE is LGPL-2.1 with an exception; this script runs as its own process and Spade never links,
loads or ships it.

THE DEFLECTION IS REQUIRED. It is the chord error of the curved faces, a geometric choice that depends on the part and
on the grid it will meet (docs: d at most a tenth of the smallest feature that matters). It takes a unit suffix
(m, mm, cm, um, in, ft); a bare number is refused, so a unit is never guessed. It is recorded in each sidecar as
its tolerance, in metres. --angle (radians, default 0.5) is the angular deflection.

NOTHING IS HEALED. The mesher is single-threaded, so a run repeats. A solid make_mesh() refuses is reported by name with
its message and the exit code says 4; change the deflection or fix the CAD. Shells, surfaces and wires are not read: a
product with no solid is refused by name.

THE CROSS-CHECK. OpenCASCADE's own B-rep volume of each solid is printed beside the mesh's, with the relative
difference. A difference above 1% is flagged, and the exit code says 5 (the sidecars are written all the same).

EXIT CODES. 0 written; 1 an input, output or the packer failed in a way not below; 2 a usage error (nothing read);
3 OCP is not installed, or the STEP cannot be read or holds what this tool does not read (named); 4 a solid was
refused by make_mesh() (the packer's message); 5 written, but the cross-check flagged a solid.

NODES ARE STITCHED BY POSITION. BRepMesh triangulates face by face, and the nodes it puts on a shared edge need not be
bit-equal between the two faces; Spade's weld is exact. Nodes within 1e-6 of the deflection of one another (and the
deflection is at least a thousand times that: a node is never merged with a neighbour that is really apart) are one
vertex, and a triangle that collapses is dropped.
"""
import argparse
import importlib.metadata
import math
import os
import re
import subprocess
import sys
import tempfile
import zipfile
from xml.sax.saxutils import quoteattr

# Metres per unit, and the 3MF unit name and OCCT unit name of each length unit this tool reads.
UNITS = {
    "m": (1.0, "meter", "M"),
    "mm": (1e-3, "millimeter", "MM"),
    "cm": (1e-2, "centimeter", "CM"),
    "um": (1e-6, "micron", "UM"),
    "in": (0.0254, "inch", "INCH"),
    "ft": (0.3048, "foot", "FT"),
}
SI_PREFIX = {"$": "m", "MILLI": "mm", "CENTI": "cm", "MICRO": "um"}
CROSS_CHECK_LIMIT = 0.01
STITCH_FRACTION = 1e-6


class Refusal(Exception):
    """A run that stops with an exit code and a message."""

    def __init__(self, code, message):
        super().__init__(message)
        self.code = code
        self.message = message


def parse_length(text, flag):
    """'0.1mm' -> (value, unit). A bare number, an unknown unit or a negative or non-finite value is a usage error."""
    match = re.fullmatch(r"([0-9.eE+-]+)([a-z]+)", text.strip())
    if not match or match.group(2) not in UNITS:
        raise Refusal(2, f"{flag} '{text}' is not a number and a unit ({', '.join(UNITS)}); a unit is never guessed")
    try:
        value = float(match.group(1))
    except ValueError:
        raise Refusal(2, f"{flag} '{text}' is not a number and a unit") from None
    if not math.isfinite(value) or value <= 0.0:
        raise Refusal(2, f"{flag} '{text}' must be finite and above zero")
    return value, match.group(2)


def file_length_unit(path):
    """The length unit the Part 21 text declares, or a refusal: the STEP must name exactly one."""
    with open(path, "rb") as f:
        text = f.read().decode("latin-1")
    found = set()
    si_length = r"LENGTH_UNIT\(\)\s*NAMED_UNIT\(\*\)\s*SI_UNIT\(\s*(\.[A-Z]+\.|\$)\s*,\s*\.METRE\.\s*\)"
    for m in re.finditer(si_length, text):
        prefix = m.group(1).strip(".")
        if prefix not in SI_PREFIX:
            raise Refusal(3, f"{os.path.basename(path)}: the length unit prefix '{prefix}' is not read "
                             f"(m, mm, cm, um, in, ft)")
        found.add(SI_PREFIX[prefix])
    for m in re.finditer(r"CONVERSION_BASED_UNIT\(\s*'([A-Za-z ]+)'", text):
        name = m.group(1).strip().upper()
        if name in ("INCH", "FOOT"):
            found.add("in" if name == "INCH" else "ft")
        elif name not in ("DEGREE", "RADIAN"):
            raise Refusal(3, f"{os.path.basename(path)}: the unit '{name}' is not read (m, mm, cm, um, in, ft)")
    if len(found) != 1:
        raise Refusal(3, f"{os.path.basename(path)}: {'no length unit' if not found else 'more than one length unit'}"
                         f" is declared{'' if not found else ' (' + ', '.join(sorted(found)) + ')'}; "
                         f"a unit is never guessed")
    return found.pop()


def import_ocp():
    try:
        import OCP  # noqa: F401  (the import is the test)
    except ImportError as e:
        raise Refusal(3, "OCP is not installed (" + str(e) + "); install tools/step_convert/requirements.txt with "
                         "pip --require-hashes") from None


def ocp_versions():
    wheel = importlib.metadata.version("cadquery-ocp")
    return ".".join(wheel.split(".")[:3]), wheel


class Part:
    """One solid instance: a name, its mesh in the part's own frame, the placement and the B-rep volume."""

    def __init__(self, name, vertices, triangles, transform, brep_volume):
        self.name = name
        self.vertices = vertices
        self.triangles = triangles
        self.transform = transform  # 12 numbers, the 3MF row-vector form
        self.brep_volume = brep_volume


def mesh_volume(vertices, triangles):
    """Signed volume of a closed triangle mesh by the divergence theorem (positive for outward winding)."""
    total = 0.0
    for a, b, c in triangles:
        (ax, ay, az), (bx, by, bz), (cx, cy, cz) = vertices[a], vertices[b], vertices[c]
        total += (ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx)) / 6.0
    return total


class Stitcher:
    """Vertices by position: a point within `tolerance` of an existing vertex is that vertex."""

    def __init__(self, tolerance):
        self.tolerance = tolerance
        self.vertices = []
        self.cells = {}

    def index(self, p):
        q = self.tolerance
        base = tuple(math.floor(c / q) for c in p)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    for i in self.cells.get((base[0] + dx, base[1] + dy, base[2] + dz), ()):
                        v = self.vertices[i]
                        if (v[0] - p[0]) ** 2 + (v[1] - p[1]) ** 2 + (v[2] - p[2]) ** 2 <= q * q:
                            return i
        self.vertices.append(p)
        self.cells.setdefault(base, []).append(len(self.vertices) - 1)
        return len(self.vertices) - 1


def tessellate(solid, deflection, angle, name):
    """(vertices, triangles) of `solid`, outward-wound, in the solid's own frame, in the file's unit."""
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS

    mesher = BRepMesh_IncrementalMesh(solid, deflection, False, angle, False)
    mesher.Perform()
    stitch = Stitcher(deflection * STITCH_FRACTION)
    triangles = []
    faces = TopExp_Explorer(solid, TopAbs_FACE)
    while faces.More():
        # The explorer composes the parents' orientation and location into the face it returns.
        face = TopoDS.Face_s(faces.Current())
        location = TopLoc_Location()
        triangulation = BRep_Tool.Triangulation_s(face, location)
        if triangulation is None:
            raise Refusal(4, f"{name}: a face was not meshed at deflection {deflection:g}; use a smaller deflection")
        trsf = location.Transformation()
        reverse = face.Orientation() == TopAbs_REVERSED
        index = []
        for i in range(1, triangulation.NbNodes() + 1):
            p = triangulation.Node(i).Transformed(trsf)
            index.append(stitch.index((p.X(), p.Y(), p.Z())))
        for i in range(1, triangulation.NbTriangles() + 1):
            a, b, c = triangulation.Triangle(i).Get()
            a, b, c = index[a - 1], index[b - 1], index[c - 1]
            if reverse:
                b, c = c, b
            if a != b and b != c and a != c:
                triangles.append((a, b, c))
        faces.Next()
    return stitch.vertices, triangles


def name_of(label):
    from OCP.TDataStd import TDataStd_Name

    attribute = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), attribute):
        return str(attribute.Get().ToExtString()).strip()
    return ""


def transform_of(location):
    """The 3MF 12-number form (row vectors: the rotation transposed, then the translation) of a TopLoc_Location."""
    t = location.Transformation()
    rows = [[t.Value(c + 1, r + 1) for c in range(3)] for r in range(3)]
    return [rows[r][c] for r in range(3) for c in range(3)] + [t.Value(r + 1, 4) for r in range(3)]


def read_parts(path, unit, deflection, angle, wanted):
    """The solid instances of the STEP, each meshed and measured. `deflection` is in the file's unit."""
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.Interface import Interface_Static
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDF import TDF_LabelSequence
    from OCP.TDocStd import TDocStd_Document
    from OCP.TopAbs import TopAbs_SOLID
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.XCAFApp import XCAFApp_Application
    from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool

    app = XCAFApp_Application.GetApplication_s()
    doc = TDocStd_Document(TCollection_ExtendedString("MDTV-XCAF"))
    app.InitDocument(doc)
    reader = STEPCAFControl_Reader()  # constructing a reader registers the STEP statics the next line sets
    # The geometry stays in the file's own unit, so the 3MF says that unit and no scaling happens anywhere.
    if not Interface_Static.SetCVal_s("xstep.cascade.unit", UNITS[unit][2]):
        raise Refusal(1, f"OpenCASCADE would not take the unit {UNITS[unit][2]}")
    reader.SetNameMode(True)
    base = os.path.basename(path)
    if reader.ReadFile(path) != IFSelect_RetDone:
        raise Refusal(3, f"{base}: not a STEP file OpenCASCADE reads")
    if not reader.Transfer(doc):
        raise Refusal(3, f"{base}: OpenCASCADE could not transfer the STEP's shapes")

    shapes = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots = TDF_LabelSequence()
    shapes.GetFreeShapes(roots)
    parts = []

    def leaf(label, location, instance_name):
        shape = XCAFDoc_ShapeTool.GetShape_s(label)
        solids = []
        explorer = TopExp_Explorer(shape, TopAbs_SOLID)
        while explorer.More():
            solids.append(explorer.Current())
            explorer.Next()
        label_name = instance_name or name_of(label) or "solid"
        if not solids:
            raise Refusal(3, f"{label_name}: no solid (shells, surfaces and wires are not read)")
        for k, solid in enumerate(solids):
            name = label_name if len(solids) == 1 else f"{label_name}_{k + 1}"
            if wanted and name not in wanted:
                continue
            vertices, triangles = tessellate(solid, deflection, angle, name)
            props = GProp_GProps()
            BRepGProp.VolumeProperties_s(solid, props)
            parts.append(Part(name, vertices, triangles, transform_of(location), props.Mass()))

    def walk(label, location, instance_name):
        if XCAFDoc_ShapeTool.IsAssembly_s(label):
            components = TDF_LabelSequence()
            XCAFDoc_ShapeTool.GetComponents_s(label, components)
            for i in range(1, components.Length() + 1):
                component = components.Value(i)
                referred = component
                if XCAFDoc_ShapeTool.IsReference_s(component):
                    from OCP.TDF import TDF_Label

                    referred = TDF_Label()
                    XCAFDoc_ShapeTool.GetReferredShape_s(component, referred)
                placed = location.Multiplied(XCAFDoc_ShapeTool.GetLocation_s(component))
                walk(referred, placed, name_of(component) or name_of(referred))
        else:
            leaf(label, location, instance_name)

    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location(), name_of(roots.Value(i)))
    if not parts:
        named = f" named {sorted(wanted)}" if wanted else ""
        raise Refusal(2 if wanted else 3, f"{base}: no solid to convert{named}")
    return parts


def fnum(x):
    return repr(float(x))


def write_3mf(path, parts, unit_name):
    """A core-specification 3MF: one object per part, the placement as the build item's transform."""
    rows = ['<?xml version="1.0" encoding="UTF-8"?>',
            f'<model unit="{unit_name}" xml:lang="en-US" '
            'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">',
            "<resources>"]
    for n, part in enumerate(parts, 1):
        rows.append(f"<object id=\"{n}\" name={quoteattr(part.name)} type=\"model\"><mesh><vertices>")
        rows += [f'<vertex x="{fnum(x)}" y="{fnum(y)}" z="{fnum(z)}"/>' for x, y, z in part.vertices]
        rows.append("</vertices><triangles>")
        rows += [f'<triangle v1="{a}" v2="{b}" v3="{c}"/>' for a, b, c in part.triangles]
        rows.append("</triangles></mesh></object>")
    rows.append("</resources><build>")
    for n, part in enumerate(parts, 1):
        rows.append(f'<item objectid="{n}" transform="{" ".join(fnum(v) for v in part.transform)}"/>')
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


def find_packer(given):
    candidate = given or os.environ.get("SPADE_MESH_PACK")
    if not candidate:
        raise Refusal(2, "spade_mesh_pack is needed: give --pack PATH or set SPADE_MESH_PACK")
    if not (os.path.isfile(candidate) and os.access(candidate, os.X_OK)):
        raise Refusal(2, f"spade_mesh_pack not found at '{candidate}'")
    return candidate


def run(argv):
    parser = argparse.ArgumentParser(prog="step_convert.py", add_help=False)
    parser.add_argument("input", nargs="?")
    parser.add_argument("--out")
    parser.add_argument("--deflection")
    parser.add_argument("--angle", default="0.5")
    parser.add_argument("--solid", action="append", default=[])
    parser.add_argument("--pack")
    parser.error = lambda message: (_ for _ in ()).throw(Refusal(2, message))  # noqa: E731
    args, extra = parser.parse_known_args(argv)
    if extra:
        raise Refusal(2, "unknown argument '" + extra[0] + "'")
    if not args.input:
        raise Refusal(2, "no input file")
    if not args.out:
        raise Refusal(2, "--out DIR is required")
    if not args.deflection:
        raise Refusal(2, "--deflection is required (for example 0.1mm): the chord error is a choice, with no default")
    d_value, d_unit = parse_length(args.deflection, "--deflection")
    try:
        angle = float(args.angle)
    except ValueError:
        raise Refusal(2, f"--angle '{args.angle}' is not a number of radians") from None
    if not math.isfinite(angle) or angle <= 0.0:
        raise Refusal(2, "--angle must be finite and above zero")
    packer = find_packer(args.pack)
    if not os.path.isfile(args.input):
        raise Refusal(1, f"cannot read '{args.input}'")

    import_ocp()
    unit = file_length_unit(args.input)
    deflection = d_value * UNITS[d_unit][0] / UNITS[unit][0]  # in the file's unit
    parts = read_parts(args.input, unit, deflection, angle, set(args.solid))

    lines = []
    flagged = []
    for part in parts:
        volume = mesh_volume(part.vertices, part.triangles)
        difference = abs(volume - part.brep_volume) / abs(part.brep_volume) if part.brep_volume else math.inf
        mark = ""
        if difference > CROSS_CHECK_LIMIT or volume <= 0.0:
            mark = "  FLAGGED"
            flagged.append(part.name)
        lines.append(f"{part.name:<24} {part.brep_volume:>16.6g} {volume:>16.6g} {difference:>10.3%}{mark}")
    occt, wheel = ocp_versions()
    base = os.path.basename(args.input)
    source = f"{base} [STEP; OCCT {occt} via cadquery-ocp {wheel}; deflection {args.deflection}, angle {angle:g} rad]"

    with tempfile.TemporaryDirectory(prefix="step_convert_") as work:
        model = os.path.join(work, "model.3mf")
        write_3mf(model, parts, UNITS[unit][1])
        pack = subprocess.run([packer, model, "--out", args.out, "--tolerance", args.deflection, "--source", source],
                              capture_output=True, text=True, encoding="utf-8", errors="replace")
    sys.stdout.write(pack.stdout)
    sys.stderr.write(pack.stderr)
    if pack.returncode != 0:
        return pack.returncode
    sys.stdout.write(f"cross-check ({unit}^3):  {'name':<22} {'B-rep volume':>16} {'mesh volume':>16} "
                     f"{'difference':>10}\n")
    for line in lines:
        sys.stdout.write("  " + line + "\n")
    if flagged:
        sys.stderr.write("step_convert: the mesh volume differs from the B-rep's by more than "
                         f"{CROSS_CHECK_LIMIT:.0%} for {', '.join(flagged)}; use a smaller --deflection or --angle\n")
        return 5
    return 0


def main(argv):
    try:
        return run(argv)
    except Refusal as r:
        sys.stderr.write("step_convert: " + r.message + "\n")
        return r.code
    except MemoryError:
        sys.stderr.write("step_convert: out of memory\n")
        return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
