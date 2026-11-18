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

THE MODES spade_part_import USES (L88; it runs this script as its own process and reads one JSON object from standard output):

    step_convert.py --probe                      {"ocp": bool, "python", "occt", "wheel", "reason"}; always exit 0
    step_convert.py IN.step --inspect            {"unit", "occt", "wheel", "solids": [{"name", "volume", "bbox_min", "bbox_max"}]}
                                                 in the file's unit, placements applied; nothing is tessellated
    step_convert.py IN.step --emit-3mf OUT.3mf --deflection D [--angle A] [--solid NAME]...
                                                 the core-specification 3MF only (no packer, no sidecars), and
                                                 {"unit", "occt", "wheel", "cross_check": [{"name", "brep_volume",
                                                 "mesh_volume", "difference"}], "flagged": [names]}; exit 0, or 5 when a
                                                 solid is flagged (the 3MF is written all the same)

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
vertex, and a triangle that collapses is dropped. The nodes are compared in double, as OpenCASCADE returns them, not
in float. On the fixtures OpenCASCADE's nodes on a shared edge are bit-equal (the merge distance is 0), and the
packer's reader welds bit-equal positions anyway; the tolerance is for rounding (a face's location applied to its
nodes). A pair it misses is a seam left open, which make_mesh() refuses (exit 4, the solid named): never a silent hole.
test_step_convert.py pins this against OpenCASCADE's own shared-edge node count.
"""
import argparse
import importlib.metadata
import math
import os
import re
import subprocess
import sys
import tempfile
import struct
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
# The units a CONVERSION_BASED_UNIT may name (either spelling): the name is the unit, its conversion factor is the unit itself.
CONVERSION_UNIT_NAMES = {
    "INCH": "in", "FOOT": "ft", "METRE": "m", "METER": "m", "MILLIMETRE": "mm", "MILLIMETER": "mm",
    "CENTIMETRE": "cm", "CENTIMETER": "cm", "MICROMETRE": "um", "MICROMETER": "um", "MICRON": "um",
}
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
    """The length unit of the STEP's geometric representation contexts (GLOBAL_UNIT_ASSIGNED_CONTEXT), or a refusal.

    The unit is what the file's contexts name, never a guess. A length unit declared elsewhere, for instance inside a derived unit
    (a density in kg per cubic metre), is not the geometry's unit and is ignored; contexts that name different length units
    refuse, and so does a file whose contexts name none."""
    with open(path, "rb") as f:
        text = f.read().decode("latin-1")
    base = os.path.basename(path)
    declared = {}  # entity id -> the unit key, or None with the reason it is not read
    for m in re.finditer(r"#(\d+)\s*=\s*\(([^;]*?LENGTH_UNIT\(\)[^;]*?)\)\s*;", text):
        body = m.group(2)
        si = re.search(r"SI_UNIT\(\s*(\.[A-Z]+\.|\$)\s*,\s*\.METRE\.\s*\)", body)
        conversion = re.search(r"CONVERSION_BASED_UNIT\(\s*'([A-Za-z ]+)'", body)
        if si:
            prefix = si.group(1).strip(".")
            declared[m.group(1)] = (SI_PREFIX.get(prefix), f"{base}: the length unit prefix '{prefix}' is not read "
                                                           f"(m, mm, cm, um, in, ft)")
        elif conversion:  # Spatial InterOp's exports say MILLIMETRE this way rather than with an SI_UNIT
            name = conversion.group(1).strip().upper()
            declared[m.group(1)] = (CONVERSION_UNIT_NAMES.get(name), f"{base}: the unit '{name}' is not read (m, mm, cm, um, in, ft)")
    used = set()  # the length units the contexts name
    for context in re.finditer(r"GLOBAL_UNIT_ASSIGNED_CONTEXT\(\s*\(([^)]*)\)", text):
        for ref in re.findall(r"#(\d+)", context.group(1)):
            if ref in declared:
                unit, why = declared[ref]
                if unit is None:
                    raise Refusal(3, why)
                used.add(unit)
    if len(used) != 1:
        raise Refusal(3, f"{base}: {'no length unit' if not used else 'more than one length unit'}"
                         f" is named by the geometric representation contexts{'' if not used else ' (' + ', '.join(sorted(used)) + ')'}; "
                         f"a unit is never guessed")
    return next(iter(used))


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


def weld_to_stored_precision(vertices, triangles, scale, transform=None):
    """The mesh with every vertex that is the same point once stored merged into the first of them.

    The part files store float32 metres, and the engine's gate refuses a vertex at the position of an earlier one. Two
    tessellation vertices a few nanometres apart (the stitcher's grid is finer than a float32 at a few centimetres) are
    different doubles but one stored point, so merging exactly those changes nothing that is stored: it is no repair. A
    pair that is still two points when stored is left alone. `scale` is metres per file unit; `transform` is the 3MF
    12-number placement applied before the rounding, as the engine applies it. Triangles that become a line are dropped."""
    def stored(p):
        x, y, z = p
        if transform is not None:
            m = transform
            x, y, z = (x * m[0] + y * m[3] + z * m[6] + m[9], x * m[1] + y * m[4] + z * m[7] + m[10],
                       x * m[2] + y * m[5] + z * m[8] + m[11])
        return tuple(struct.unpack("f", struct.pack("f", c * scale))[0] for c in (x, y, z))

    first = {}
    remap = []
    kept = []
    for p in vertices:
        key = stored(p)
        if key not in first:
            first[key] = len(kept)
            kept.append(p)
        remap.append(first[key])
    if len(kept) == len(vertices):
        return vertices, triangles
    welded = []
    for a, b, c in triangles:
        a, b, c = remap[a], remap[b], remap[c]
        if a != b and b != c and a != c:
            welded.append((a, b, c))
    return kept, welded


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


def unique_names(base, found):
    """`found` with every name that more than one entry carries (a part placed several times, whose instances the
    exporter did not name apart) suffixed `_1`, `_2`, ... in walk order: the rule a part of several solids already
    follows. Names that are already unique are untouched. If a suffixed name lands on another entry's name the
    result would not be unique, and nothing is guessed: the two are named in a refusal."""
    count = {}
    for name, _, _ in found:
        count[name] = count.get(name, 0) + 1
    seen, owner, out = {}, {}, []
    for name, solid, location in found:
        if count[name] > 1:
            seen[name] = seen.get(name, 0) + 1
            new = f"{name}_{seen[name]}"
        else:
            new = name
        if new in owner:
            raise Refusal(3, f"{base}: the names '{owner[new]}' and '{name}' both come out as '{new}' (a name used "
                             "more than once is suffixed _1, _2, ...); rename one in the CAD")
        owner[new] = name
        out.append((new, solid, location))
    return out


def read_solids(path, unit, wanted, skipped=None):
    """[(name, solid, location)] for every solid instance of the STEP: the solid in its own frame, and where the
    assembly puts it. `wanted`, if not empty, names the ones to keep. A part with no solid (a shell, surface or wire) is a
    refusal of the file unless `skipped` is a list: then its name is appended there and the rest is read."""
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
    found = []

    def leaf(label, location, instance_name):
        shape = XCAFDoc_ShapeTool.GetShape_s(label)
        solids = []
        explorer = TopExp_Explorer(shape, TopAbs_SOLID)
        while explorer.More():
            solids.append(explorer.Current())
            explorer.Next()
        label_name = instance_name or name_of(label) or "solid"
        if not solids:
            if skipped is None:
                raise Refusal(3, f"{label_name}: no solid (shells, surfaces and wires are not read)")
            skipped.append(label_name)
            return
        for k, solid in enumerate(solids):
            found.append((label_name if len(solids) == 1 else f"{label_name}_{k + 1}", solid, location))

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
    found = unique_names(base, found)
    if wanted:
        found = [f for f in found if f[0] in wanted]
    if not found:
        named = f" named {sorted(wanted)}" if wanted else ""
        raise Refusal(2 if wanted else 3, f"{base}: no solid to convert{named}")
    return found


def no_solid_entries(names):
    """The parts left out because they hold no solid, as the answers list them."""
    return [{"name": n, "code": "no_solid"} for n in names]


def read_parts(path, unit, deflection, angle, wanted, skipped=None):
    """The solid instances of the STEP, each meshed and measured. `deflection` is in the file's unit."""
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps

    parts = []
    for name, solid, location in read_solids(path, unit, wanted, skipped):
        vertices, triangles = tessellate(solid, deflection, angle, name)
        placement = transform_of(location)
        vertices, triangles = weld_to_stored_precision(vertices, triangles, UNITS[unit][0], placement)
        props = GProp_GProps()
        BRepGProp.VolumeProperties_s(solid, props)
        parts.append(Part(name, vertices, triangles, placement, props.Mass()))
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


def say(obj):
    """One JSON object on standard output, one line: what spade_part_import reads."""
    import json

    sys.stdout.write(json.dumps(obj) + "\n")


def probe():
    """Is OpenCASCADE importable here, and which versions. Always exit 0: the answer is in the JSON."""
    answer = {"ocp": False, "python": ".".join(str(v) for v in sys.version_info[:3]), "occt": "", "wheel": "", "reason": ""}
    try:
        import_ocp()
        answer["occt"], answer["wheel"] = ocp_versions()
        answer["ocp"] = True
    except Refusal as r:
        answer["reason"] = r.message
    except Exception as e:  # noqa: BLE001 -- a broken install is an answer, not a crash
        answer["reason"] = "OCP is not installed (" + str(e) + ")"
    say(answer)
    return 0


def inspect_solids(path, wanted):
    """The solids of the STEP with name, B-rep volume and box (placements applied), in the file's unit. Nothing is meshed."""
    from OCP.BRepBndLib import BRepBndLib
    from OCP.BRepGProp import BRepGProp
    from OCP.Bnd import Bnd_Box
    from OCP.GProp import GProp_GProps

    import_ocp()
    unit = file_length_unit(path)
    solids = []
    skipped = []
    for name, solid, location in read_solids(path, unit, wanted, skipped):
        props = GProp_GProps()
        BRepGProp.VolumeProperties_s(solid, props)
        box = Bnd_Box()
        BRepBndLib.Add_s(solid.Moved(location), box, False)
        x0, y0, z0, x1, y1, z1 = box.Get()
        solids.append({"name": name, "volume": props.Mass(), "bbox_min": [x0, y0, z0], "bbox_max": [x1, y1, z1]})
    occt, wheel = ocp_versions()
    say({"unit": unit, "occt": occt, "wheel": wheel, "solids": solids, "skipped": no_solid_entries(skipped)})
    return 0


def emit_3mf(path, out, deflection_text, angle, wanted):
    """The 3MF of the STEP at `out` and the cross-check as JSON. Exit 5 when a solid is flagged; the 3MF is written either way."""
    d_value, d_unit = parse_length(deflection_text, "--deflection")
    import_ocp()
    unit = file_length_unit(path)
    deflection = d_value * UNITS[d_unit][0] / UNITS[unit][0]  # in the file's unit
    skipped = []
    parts = read_parts(path, unit, deflection, angle, wanted, skipped)
    checks, flagged = [], []
    for part in parts:
        volume = mesh_volume(part.vertices, part.triangles)
        difference = abs(volume - part.brep_volume) / abs(part.brep_volume) if part.brep_volume else math.inf
        if difference > CROSS_CHECK_LIMIT or volume <= 0.0:
            flagged.append(part.name)
        checks.append({"name": part.name, "brep_volume": part.brep_volume, "mesh_volume": volume,
                       "difference": difference if math.isfinite(difference) else 1.0})
    write_3mf(out, parts, UNITS[unit][1])
    occt, wheel = ocp_versions()
    say({"unit": unit, "occt": occt, "wheel": wheel, "cross_check": checks, "flagged": flagged,
         "skipped": no_solid_entries(skipped)})
    return 5 if flagged else 0


def run(argv):
    # --help / -h is argparse's own: the usage and this file's description on standard output, exit 0 (a request, not an error).
    parser = argparse.ArgumentParser(prog="step_convert.py", description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", nargs="?", help="the STEP file")
    parser.add_argument("--probe", action="store_true", help="say whether OpenCASCADE can be used, as JSON; reads nothing")
    parser.add_argument("--inspect", action="store_true", help="list the solids and what is skipped, as JSON; writes nothing")
    parser.add_argument("--emit-3mf", dest="emit_3mf", help="tessellate and write one 3MF here (needs --deflection)")
    parser.add_argument("--out", help="the directory the .spmesh sidecars are written to")
    parser.add_argument("--deflection", help="chord error with a unit, for example 0.1mm; no default")
    parser.add_argument("--angle", default="0.5", help="angular deflection in radians (default 0.5)")
    parser.add_argument("--solid", action="append", default=[], help="keep only this solid; repeat it to keep several")
    parser.add_argument("--pack", help="the spade_mesh_pack program")
    parser.error = lambda message: (_ for _ in ()).throw(Refusal(2, message))  # noqa: E731
    args, extra = parser.parse_known_args(argv)
    if extra:
        raise Refusal(2, "unknown argument '" + extra[0] + "'")
    if args.probe:
        return probe()
    if not args.input:
        raise Refusal(2, "no input file")
    if args.inspect or args.emit_3mf:
        if not os.path.isfile(args.input):
            raise Refusal(1, f"cannot read '{args.input}'")
        if args.inspect:
            return inspect_solids(args.input, set(args.solid))
        if not args.deflection:
            raise Refusal(2, "--deflection is required (for example 0.1mm): the chord error is a choice, with no default")
        try:
            angle = float(args.angle)
        except ValueError:
            raise Refusal(2, f"--angle '{args.angle}' is not a number of radians") from None
        if not math.isfinite(angle) or angle <= 0.0:
            raise Refusal(2, "--angle must be finite and above zero")
        return emit_3mf(args.input, args.emit_3mf, args.deflection, angle, set(args.solid))
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
