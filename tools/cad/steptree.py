"""Parse STEP assembly tree and world transforms of each occurrence (pure python)."""
import re, sys, json, math

ENT = re.compile(r"#(\d+)\s*=\s*(.*?);\s*$", re.S)
REF = re.compile(r"#(\d+)")


def load(path):
    text = open(path, encoding="latin-1").read()
    data = text[text.index("DATA;") + 5 :]
    ents = {}
    buf = []
    # split on ';' at end of lines
    for stmt in re.split(r";\s*\n", data):
        stmt = stmt.strip()
        m = re.match(r"#(\d+)\s*=\s*(.*)", stmt, re.S)
        if m:
            ents[int(m.group(1))] = m.group(2).replace("\n", "")
    return ents


def nums(s):
    return [float(x) for x in re.findall(r"[-+]?\d*\.?\d+(?:E[-+]?\d+)?", s)]


def placement(ents, ax):
    body = ents[ax]
    refs = [int(r) for r in REF.findall(body)]
    p = nums(ents[refs[0]].split("(", 2)[2])
    z = nums(ents[refs[1]].split("(", 2)[2]) if len(refs) > 1 else [0, 0, 1]
    x = nums(ents[refs[2]].split("(", 2)[2]) if len(refs) > 2 else [1, 0, 0]
    y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]]
    return [[x[0], y[0], z[0], p[0]], [x[1], y[1], z[1], p[1]], [x[2], y[2], z[2], p[2]], [0, 0, 0, 1]]


def mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def inv(m):
    r = [[m[j][i] for j in range(3)] for i in range(3)]
    t = [-sum(r[i][k] * m[k][3] for k in range(3)) for i in range(3)]
    return [r[0] + [t[0]], r[1] + [t[1]], r[2] + [t[2]], [0, 0, 0, 1]]


def tree(path):
    ents = load(path)
    pd_name = {}
    prod = {}
    pdf = {}
    for i, b in ents.items():
        if b.startswith("PRODUCT("):
            prod[i] = re.findall(r"'([^']*)'", b)[1]
        elif b.startswith("PRODUCT_DEFINITION_FORMATION"):
            pdf[i] = int(REF.findall(b)[-1])
    for i, b in ents.items():
        if b.startswith("PRODUCT_DEFINITION("):
            r = REF.findall(b)
            pd_name[i] = prod.get(pdf.get(int(r[0])), "?")
    nauo = {}
    for i, b in ents.items():
        if b.startswith("NEXT_ASSEMBLY_USAGE_OCCURRENCE"):
            r = [int(x) for x in REF.findall(b)]
            name = re.findall(r"'([^']*)'", b)[1]
            nauo[i] = (name, r[0], r[1])
    pds_of = {}
    for i, b in ents.items():
        if b.startswith("PRODUCT_DEFINITION_SHAPE"):
            r = REF.findall(b)
            if r:
                pds_of[i] = int(r[0])
    xf = {}
    for i, b in ents.items():
        if b.startswith("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION"):
            rr, pds = [int(x) for x in REF.findall(b)]
            n = pds_of.get(pds)
            rel = ents[rr]
            idt = int(re.search(r"WITH_TRANSFORMATION\(#(\d+)", rel).group(1))
            a1, a2 = [int(x) for x in REF.findall(ents[idt])]
            xf[n] = mul(placement(ents, a2), inv(placement(ents, a1)))
    children = {}
    for i, (name, par, ch) in nauo.items():
        children.setdefault(par, []).append(i)
    allch = {c for _, _, c in nauo.values()}
    roots = [p for p in children if p not in allch]
    out = []

    def walk(pd, M, pathn):
        for i in children.get(pd, []):
            name, par, ch = nauo[i]
            W = mul(M, xf.get(i, [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]))
            p = pathn + [name]
            out.append({"path": "/".join(p), "product": pd_name.get(ch), "leaf": ch not in children, "M": W})
            walk(ch, W, p)

    I = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    for r in roots:
        walk(r, I, [pd_name.get(r, "root")])
    return out


if __name__ == "__main__":
    res = {}
    for name, p in [("stow", sys.argv[1]), ("score", sys.argv[2]), ("intake", sys.argv[3])]:
        res[name] = tree(p)
    json.dump(res, open(sys.argv[4], "w"))
    print({k: len(v) for k, v in res.items()})
