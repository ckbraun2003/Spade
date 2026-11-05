#!/usr/bin/env python3
"""Writes the three STEP fixtures of tools/step_convert's tests, as plain-text ISO 10303-21 (Part 21).

NO CAD KERNEL MADE THESE FILES. Every entity is written by hand-coded rules below (the AP214 product structure,
the planar and cylindrical B-rep faces, the assembly placement), so a test that converts them with OpenCASCADE
checks OpenCASCADE's reading against the specification and against analytic geometry, not against itself.
Run `python gen_fixtures.py` to rewrite the .step files beside this script; the output is deterministic.

  cube_10mm.step        one product "Cube": a 10 mm cube, six planar faces.
  holed_cylinder.step   one product "Ring": a cylinder (R 10 mm, h 20 mm) with a through hole (r 4 mm); two planar
                        annulus faces and two cylindrical faces, whose circles each have a seam edge.
  rotated_pair.step     an assembly "Pair" of "Block" (20 x 10 x 5 mm at the origin) and "Pin" (a 5 mm cube placed
                        at (30, 0, 0) with its x axis along +y: a quarter turn about z), through
                        NEXT_ASSEMBLY_USAGE_OCCURRENCE and an ITEM_DEFINED_TRANSFORMATION.

All lengths are millimetres (SI_UNIT(.MILLI.,.METRE.)).
"""
import math
import os
import sys


def fmt(x):
    s = repr(float(x))
    if "e" in s:
        mant, exp = s.split("e")
        if "." not in mant:
            mant += "."
        return mant + "E" + exp
    if "." not in s:
        s += "."
    return s


def unit(v):
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v)


class P21:
    def __init__(self):
        self.lines = []

    def add(self, text):
        self.lines.append(text)
        return len(self.lines)

    def point(self, p):
        return self.add("CARTESIAN_POINT('',(%s,%s,%s))" % tuple(fmt(c) for c in p))

    def direction(self, d):
        return self.add("DIRECTION('',(%s,%s,%s))" % tuple(fmt(c) for c in unit(d)))

    def axis(self, origin, z=(0, 0, 1), x=(1, 0, 0)):
        return self.add("AXIS2_PLACEMENT_3D('',#%d,#%d,#%d)" %
                        (self.point(origin), self.direction(z), self.direction(x)))

    def line(self, origin, direction):
        v = self.add("VECTOR('',#%d,1.)" % self.direction(direction))
        return self.add("LINE('',#%d,#%d)" % (self.point(origin), v))

    def circle(self, centre, radius):
        return self.add("CIRCLE('',#%d,%s)" % (self.axis(centre), fmt(radius)))

    def vertex(self, p):
        return self.add("VERTEX_POINT('',#%d)" % self.point(p))

    def edge(self, v0, v1, curve):
        return self.add("EDGE_CURVE('',#%d,#%d,#%d,.T.)" % (v0, v1, curve))

    def loop(self, oriented):
        parts = ",".join("#%d" % self.add("ORIENTED_EDGE('',*,*,#%d,.%s.)" % (e, "T" if fwd else "F"))
                         for e, fwd in oriented)
        return self.add("EDGE_LOOP('',(%s))" % parts)

    def face(self, bounds, surface, same_sense=True):
        """bounds: [(loop, is_outer)]."""
        parts = ",".join("#%d" % self.add("%s('',#%d,.T.)" % ("FACE_OUTER_BOUND" if outer else "FACE_BOUND", lp))
                         for lp, outer in bounds)
        return self.add("ADVANCED_FACE('',(%s),#%d,.%s.)" % (parts, surface, "T" if same_sense else "F"))

    def solid(self, name, faces):
        shell = self.add("CLOSED_SHELL('',(%s))" % ",".join("#%d" % f for f in faces))
        return self.add("MANIFOLD_SOLID_BREP('%s',#%d)" % (name, shell))


# The cube's corners are numbered x + 2y + 4z; each face's cycle runs counter-clockwise seen from outside, so the
# right-hand normal (v1 - v0) x (v2 - v1) is the outward normal.
CUBE_FACES = [((0, 2, 3, 1), (0, 0, -1)), ((4, 5, 7, 6), (0, 0, 1)), ((0, 1, 5, 4), (0, -1, 0)),
              ((2, 6, 7, 3), (0, 1, 0)), ((0, 4, 6, 2), (-1, 0, 0)), ((1, 3, 7, 5), (1, 0, 0))]


def box(p, name, lo, size):
    """A MANIFOLD_SOLID_BREP of the box from `lo` of extent `size`."""
    corners = [tuple(lo[a] + (size[a] if (i >> a) & 1 else 0.0) for a in range(3)) for i in range(8)]
    vertices = [p.vertex(c) for c in corners]
    edges = {}
    faces = []
    for cycle, normal in CUBE_FACES:
        oriented = []
        for k in range(4):
            a, b = cycle[k], cycle[(k + 1) % 4]
            lo_i, hi_i = min(a, b), max(a, b)
            if (lo_i, hi_i) not in edges:
                d = tuple(corners[hi_i][c] - corners[lo_i][c] for c in range(3))
                edges[(lo_i, hi_i)] = p.edge(vertices[lo_i], vertices[hi_i], p.line(corners[lo_i], d))
            oriented.append((edges[(lo_i, hi_i)], a < b))
        v0, v1 = corners[cycle[0]], corners[cycle[1]]
        plane = p.add("PLANE('',#%d)" % p.axis(v0, normal, tuple(v1[c] - v0[c] for c in range(3))))
        faces.append(p.face([(p.loop(oriented), True)], plane))
    return p.solid(name, faces)


def build_ring(p, name, outer_r, inner_r, height):
    verts = {}
    for key, r, z in (("ob", outer_r, 0.0), ("ot", outer_r, height), ("ib", inner_r, 0.0), ("it", inner_r, height)):
        verts[key] = p.vertex((r, 0.0, z))
    circ = {k: p.edge(verts[k], verts[k], p.circle((0.0, 0.0, 0.0 if k[1] == "b" else height),
                                                   outer_r if k[0] == "o" else inner_r)) for k in verts}
    seam = {"o": p.edge(verts["ob"], verts["ot"], p.line((outer_r, 0.0, 0.0), (0, 0, 1))),
            "i": p.edge(verts["ib"], verts["it"], p.line((inner_r, 0.0, 0.0), (0, 0, 1)))}
    faces = []
    # Bottom (normal -z): the outer circle runs clockwise about +z (reversed), the hole's counter-clockwise.
    bottom = p.add("PLANE('',#%d)" % p.axis((0.0, 0.0, 0.0), (0, 0, -1), (1, 0, 0)))
    faces.append(p.face([(p.loop([(circ["ob"], False)]), True), (p.loop([(circ["ib"], True)]), False)], bottom))
    # Top (normal +z): the other way round.
    top = p.add("PLANE('',#%d)" % p.axis((0.0, 0.0, height), (0, 0, 1), (1, 0, 0)))
    faces.append(p.face([(p.loop([(circ["ot"], True)]), True), (p.loop([(circ["it"], False)]), False)], top))
    # The outer wall, normal radially out: bottom circle forward, seam up, top circle reversed, seam down.
    wall = p.add("CYLINDRICAL_SURFACE('',#%d,%s)" % (p.axis((0.0, 0.0, 0.0)), fmt(outer_r)))
    faces.append(p.face([(p.loop([(circ["ob"], True), (seam["o"], True), (circ["ot"], False), (seam["o"], False)]),
                          True)], wall))
    # The hole's wall, normal toward the axis (out of the material): the same surface type, sense reversed, and
    # the loop run the other way.
    hole = p.add("CYLINDRICAL_SURFACE('',#%d,%s)" % (p.axis((0.0, 0.0, 0.0)), fmt(inner_r)))
    faces.append(p.face([(p.loop([(seam["i"], True), (circ["it"], True), (seam["i"], False), (circ["ib"], False)]),
                          True)], hole, same_sense=False))
    return p.solid(name, faces)


HEADER = """ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('Spade test fixture: %(what)s'),'2;1');
FILE_NAME('%(file)s','2026-11-04T00:00:00',('Spade tests/golden/step/gen_fixtures.py'),('Spade'),
 'plain-text Part 21 writer, no CAD kernel','none','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));
ENDSEC;
DATA;
"""


def context(p):
    lu = p.add("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.))")
    au = p.add("(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.))")
    su = p.add("(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT())")
    unc = p.add("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-05),#%d,'distance_accuracy_value',"
                "'confusion accuracy')" % lu)
    return p.add("(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#%d))"
                 "GLOBAL_UNIT_ASSIGNED_CONTEXT((#%d,#%d,#%d))REPRESENTATION_CONTEXT('ID1','3D'))" % (unc, lu, au, su))


def application(p):
    ctx = p.add("APPLICATION_CONTEXT('core data for automotive mechanical design processes')")
    p.add("APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#%d)" % ctx)
    return ctx


def product(p, app, name):
    """PRODUCT, its formation and definition; returns the PRODUCT_DEFINITION."""
    pctx = p.add("PRODUCT_CONTEXT('',#%d,'mechanical')" % app)
    prod = p.add("PRODUCT('%s','%s','',(#%d))" % (name, name, pctx))
    form = p.add("PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE('','',#%d,.NOT_KNOWN.)" % prod)
    dctx = p.add("PRODUCT_DEFINITION_CONTEXT('part definition',#%d,'design')" % app)
    return p.add("PRODUCT_DEFINITION('design','',#%d,#%d)" % (form, dctx))


def shape_of(p, definition, representation):
    pds = p.add("PRODUCT_DEFINITION_SHAPE('','',#%d)" % definition)
    p.add("SHAPE_DEFINITION_REPRESENTATION(#%d,#%d)" % (pds, representation))


def part(p, app, ctx, name, make_solid):
    """A product whose shape is one solid; returns (definition, shape representation, its axis placement)."""
    definition = product(p, app, name)
    solid = make_solid(name)
    axis = p.axis((0.0, 0.0, 0.0))
    rep = p.add("ADVANCED_BREP_SHAPE_REPRESENTATION('',(#%d,#%d),#%d)" % (solid, axis, ctx))
    shape_of(p, definition, rep)
    return definition, rep, axis


def single(path, what, name, make_solid):
    p = P21()
    app = application(p)
    ctx = context(p)
    part(p, app, ctx, name, lambda n: make_solid(p, n))
    return render(path, what, p)


def pair(path):
    p = P21()
    app = application(p)
    ctx = context(p)
    block_def, block_rep, block_axis = part(p, app, ctx, "Block",
                                            lambda n: box(p, n, (0.0, 0.0, 0.0), (20.0, 10.0, 5.0)))
    pin_def, pin_rep, pin_axis = part(p, app, ctx, "Pin", lambda n: box(p, n, (0.0, 0.0, 0.0), (5.0, 5.0, 5.0)))
    pair_def = product(p, app, "Pair")
    # Where each part's own axis lands in the assembly: the block unmoved; the pin at (30, 0, 0) with its x axis
    # along +y (so its y axis is along -x), a quarter turn about z. A point (a, b, c) of the pin goes to
    # (30 - b, a, c).
    asm_origin = p.axis((0.0, 0.0, 0.0))
    block_at = p.axis((0.0, 0.0, 0.0))
    pin_at = p.axis((30.0, 0.0, 0.0), (0, 0, 1), (0, 1, 0))
    asm_rep = p.add("SHAPE_REPRESENTATION('',(#%d,#%d,#%d),#%d)" % (asm_origin, block_at, pin_at, ctx))
    shape_of(p, pair_def, asm_rep)
    for k, (child_def, child_rep, child_axis, placed, label) in enumerate(
            ((block_def, block_rep, block_axis, block_at, "Block"), (pin_def, pin_rep, pin_axis, pin_at, "Pin"))):
        nauo = p.add("NEXT_ASSEMBLY_USAGE_OCCURRENCE('%d','%s','',#%d,#%d,$)" % (k + 1, label, pair_def, child_def))
        usage = p.add("PRODUCT_DEFINITION_SHAPE('','',#%d)" % nauo)
        itd = p.add("ITEM_DEFINED_TRANSFORMATION('','',#%d,#%d)" % (child_axis, placed))
        rel = p.add("(REPRESENTATION_RELATIONSHIP('','',#%d,#%d)REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#%d)"
                    "SHAPE_REPRESENTATION_RELATIONSHIP())" % (child_rep, asm_rep, itd))
        p.add("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#%d,#%d)" % (rel, usage))
    return render(path, "a block and a pin placed by a quarter turn about z", p)


def render(path, what, p):
    text = HEADER % {"what": what, "file": os.path.basename(path)}
    text += "".join("#%d=%s;\n" % (i + 1, line) for i, line in enumerate(p.lines))
    text += "ENDSEC;\nEND-ISO-10303-21;\n"
    with open(path, "w", newline="\n") as f:
        f.write(text)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    single(os.path.join(here, "cube_10mm.step"), "a 10 mm cube", "Cube",
           lambda p, n: box(p, n, (0.0, 0.0, 0.0), (10.0, 10.0, 10.0)))
    single(os.path.join(here, "holed_cylinder.step"), "a cylinder with a through hole", "Ring",
           lambda p, n: build_ring(p, n, 10.0, 4.0, 20.0))
    pair(os.path.join(here, "rotated_pair.step"))
    print("wrote 3 fixtures to", here)


if __name__ == "__main__":
    sys.exit(main())
