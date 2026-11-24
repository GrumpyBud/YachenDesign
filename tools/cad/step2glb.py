"""Mesh the stowed STEP assembly into AdvantageScope GLB models (robot frame, meters).

Usage: step2glb.py STOWED.step SCORING.step OUTDIR. Parts are grouped by how they move between
the stowed and scoring exports: fixed (base), first stage, carriage, or rotating (arm and claw).

Robot frame: +X forward (= CAD -Y, the claw side), +Y left (= CAD +X), +Z up (floor = 0).
"""
import json, struct, sys, os, math
from array import array
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool, XCAFDoc_ColorType, XCAFDoc_ColorTool
from OCP.TDF import TDF_Label
from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.Quantity import Quantity_Color
from OCP.TopLoc import TopLoc_Location
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool
from OCP.IFSelect import IFSelect_RetDone

src, moved_src, outdir = sys.argv[1], sys.argv[2], sys.argv[3]


def read(path):
    doc = TDocStd_Document(TCollection_ExtendedString("doc"))
    r = STEPCAFControl_Reader()
    r.SetColorMode(True)
    r.SetNameMode(True)
    assert r.ReadFile(path) == IFSelect_RetDone
    r.Transfer(doc)
    return doc


doc = read(src)
st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
ct = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())


def name(label):
    a = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), a):
        return a.Get().ToExtString()
    return "?"


def color(label, default):
    c = Quantity_Color()
    for t in (XCAFDoc_ColorType.XCAFDoc_ColorSurf, XCAFDoc_ColorType.XCAFDoc_ColorGen):
        cl = TDF_Label()
        if XCAFDoc_ColorTool.GetColor_s(label, t, cl) and XCAFDoc_ColorTool.GetColor_s(cl, c):
            return (c.Red(), c.Green(), c.Blue())
        try:
            if ct.GetColor(XCAFDoc_ShapeTool.GetShape_s(label), t, c):
                return (c.Red(), c.Green(), c.Blue())
        except Exception:
            pass
    return default


def group_of(stow_loc, moved_loc, path):
    """Classify a part by its rigid motion from the stowed export to the scoring export (mm)."""
    rel = moved_loc.Multiplied(stow_loc.Inverted()).Transformation()
    q = rel.GetRotation()
    angle = 2 * math.degrees(math.acos(min(1.0, abs(q.W()))))
    travel = rel.TranslationPart().Modulus()
    if angle > 1.0:
        return "arm"
    if travel < 5.0:
        return "base"
    if 600.0 < travel < 800.0:
        return "stage1"
    if travel > 1200.0:
        return "carriage"
    raise ValueError(f"Unclassified motion {travel:.1f} mm for {'/'.join(path)}")


# group -> color -> (positions array, indices array)
meshes = {}
counts = {}
mn = [1e9] * 3
mx = [-1e9] * 3


def add_shape(shape, grp, rgb):
    BRepMesh_IncrementalMesh(shape, 2.5, False, 0.7, True)
    pos, idx = meshes.setdefault(grp, {}).setdefault(
        tuple(round(v, 3) for v in rgb), (array("f"), array("I"))
    )
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = TopoDS.Face(exp.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            trsf = loc.Transformation()
            base = len(pos) // 3
            for i in range(1, tri.NbNodes() + 1):
                p = tri.Node(i).Transformed(trsf)
                # CAD -> robot frame
                x, y, z = -p.Y() / 1000.0, p.X() / 1000.0, p.Z() / 1000.0
                pos.extend((x, y, z))
                for k, v in enumerate((x, y, z)):
                    mn[k] = min(mn[k], v)
                    mx[k] = max(mx[k], v)
            rev = face.Orientation() == TopAbs_REVERSED
            for i in range(1, tri.NbTriangles() + 1):
                a, b, c = tri.Triangle(i).Get()
                if rev:
                    b, c = c, b
                idx.extend((base + a - 1, base + b - 1, base + c - 1))
        exp.Next()
    counts[grp] = counts.get(grp, 0) + 1


def walk(label, loc, path, rgb, out):
    """Collect leaf parts (ref label, world location, path, color) in document order."""
    comps = TDF_LabelSequence()
    XCAFDoc_ShapeTool.GetComponents_s(label, comps)
    for i in range(1, comps.Length() + 1):
        comp = comps.Value(i)
        cloc = loc.Multiplied(XCAFDoc_ShapeTool.GetLocation_s(comp))
        ref = TDF_Label()
        XCAFDoc_ShapeTool.GetReferredShape_s(comp, ref)
        p = path + [name(comp)]
        c = color(comp, color(ref, rgb)) if rgb is not None else None
        if XCAFDoc_ShapeTool.IsAssembly_s(ref):
            walk(ref, cloc, p, c, out)
        else:
            out.append((ref, cloc, p, c))


def leaves(document, with_color):
    tool = XCAFDoc_DocumentTool.ShapeTool_s(document.Main())
    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)
    out = []
    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location(), [name(roots.Value(i))],
             (0.6, 0.6, 0.6) if with_color else None, out)
    return out


stow_leaves = leaves(doc, True)
moved_leaves = leaves(read(moved_src), False)
assert len(stow_leaves) == len(moved_leaves), "exports differ in structure"
for (ref, loc, p, c), (_, moved_loc, moved_p, _) in zip(stow_leaves, moved_leaves):
    assert p == moved_p, (p, moved_p)
    shape = XCAFDoc_ShapeTool.GetShape_s(ref).Moved(loc)
    add_shape(shape, group_of(loc, moved_loc, p), c)

print("leaf counts", counts)
print("bounds robot frame (before floor shift)", mn, mx)
floor = mn[2]


def vertex_normals(pos, idx):
    """Area-weighted per-vertex normals. Each CAD face has its own vertices, so edges stay sharp."""
    acc = [0.0] * len(pos)
    for t in range(0, len(idx), 3):
        a, b, c = idx[t] * 3, idx[t + 1] * 3, idx[t + 2] * 3
        ux, uy, uz = pos[b] - pos[a], pos[b + 1] - pos[a + 1], pos[b + 2] - pos[a + 2]
        vx, vy, vz = pos[c] - pos[a], pos[c + 1] - pos[a + 1], pos[c + 2] - pos[a + 2]
        nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
        for v in (a, b, c):
            acc[v] += nx
            acc[v + 1] += ny
            acc[v + 2] += nz
    out = array("f", bytes(4 * len(pos)))
    for v in range(0, len(pos), 3):
        n = math.sqrt(acc[v] ** 2 + acc[v + 1] ** 2 + acc[v + 2] ** 2)
        if n > 0:
            out[v], out[v + 1], out[v + 2] = acc[v] / n, acc[v + 1] / n, acc[v + 2] / n
        else:
            out[v + 2] = 1.0
    return out


def write_glb(path, groups):
    bin_ = bytearray()
    accessors, views, prims, materials = [], [], [], []
    for rgb, (pos, idx) in groups.items():
        if not idx:
            continue
        for k in range(2, len(pos), 3):
            pos[k] -= floor
        pmin = [min(pos[k::3]) for k in range(3)]
        pmax = [max(pos[k::3]) for k in range(3)]
        off = len(bin_)
        bin_ += pos.tobytes()
        views.append({"buffer": 0, "byteOffset": off, "byteLength": len(pos) * 4, "target": 34962})
        accessors.append({"bufferView": len(views) - 1, "componentType": 5126, "count": len(pos) // 3,
                          "type": "VEC3", "min": pmin, "max": pmax})
        pa = len(accessors) - 1
        normals = vertex_normals(pos, idx)
        off = len(bin_)
        bin_ += normals.tobytes()
        views.append({"buffer": 0, "byteOffset": off, "byteLength": len(normals) * 4, "target": 34962})
        accessors.append({"bufferView": len(views) - 1, "componentType": 5126, "count": len(normals) // 3,
                          "type": "VEC3"})
        na = len(accessors) - 1
        off = len(bin_)
        bin_ += idx.tobytes()
        views.append({"buffer": 0, "byteOffset": off, "byteLength": len(idx) * 4, "target": 34963})
        accessors.append({"bufferView": len(views) - 1, "componentType": 5125, "count": len(idx), "type": "SCALAR"})
        materials.append({"pbrMetallicRoughness": {"baseColorFactor": [*rgb, 1.0], "metallicFactor": 0.1,
                                                   "roughnessFactor": 0.7}, "doubleSided": True})
        prims.append({"attributes": {"POSITION": pa, "NORMAL": na}, "indices": len(accessors) - 1,
                      "material": len(materials) - 1})
    gltf = {"asset": {"version": "2.0", "generator": "step2glb"}, "scene": 0, "scenes": [{"nodes": [0]}],
            "nodes": [{"mesh": 0}], "meshes": [{"primitives": prims}], "materials": materials,
            "accessors": accessors, "bufferViews": views, "buffers": [{"byteLength": len(bin_)}]}
    js = json.dumps(gltf, separators=(",", ":")).encode()
    js += b" " * (-len(js) % 4)
    bin_ += b"\0" * (-len(bin_) % 4)
    total = 12 + 8 + len(js) + 8 + len(bin_)
    with open(path, "wb") as f:
        f.write(struct.pack("<III", 0x46546C67, 2, total))
        f.write(struct.pack("<II", len(js), 0x4E4F534A) + js)
        f.write(struct.pack("<II", len(bin_), 0x004E4942) + bin_)
    print(path, round(total / 1e6, 2), "MB")


os.makedirs(outdir, exist_ok=True)
for grp, fname in [("base", "model.glb"), ("stage1", "model_0.glb"), ("carriage", "model_1.glb"), ("arm", "model_2.glb")]:
    write_glb(os.path.join(outdir, fname), meshes.get(grp, {}))
json.dump({"floorCadZ": floor, "bounds": [mn, mx]}, open(os.path.join(outdir, "..", "meshinfo.json"), "w"))
