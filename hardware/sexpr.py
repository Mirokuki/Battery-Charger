"""Minimal S-expression reader/writer for KiCad files."""


class Sym(str):
    """Bare (unquoted) atom."""


def parse(text):
    pos = 0
    n = len(text)
    stack = [[]]
    while pos < n:
        c = text[pos]
        if c in " \t\r\n":
            pos += 1
        elif c == "(":
            stack.append([])
            pos += 1
        elif c == ")":
            done = stack.pop()
            stack[-1].append(done)
            pos += 1
        elif c == '"':
            pos += 1
            out = []
            while text[pos] != '"':
                if text[pos] == "\\":
                    pos += 1
                    esc = text[pos]
                    out.append({"n": "\n", "t": "\t"}.get(esc, esc))
                else:
                    out.append(text[pos])
                pos += 1
            pos += 1
            stack[-1].append("".join(out))
        else:
            start = pos
            while pos < n and text[pos] not in " \t\r\n()":
                pos += 1
            stack[-1].append(Sym(text[start:pos]))
    return stack[0][0]


def dump(node, indent=0):
    if isinstance(node, list):
        simple = all(not isinstance(x, list) for x in node)
        if simple:
            return "(" + " ".join(dump(x) for x in node) + ")"
        pad = "\t" * (indent + 1)
        head = []
        rest = list(node)
        while rest and not isinstance(rest[0], list):
            head.append(dump(rest.pop(0)))
        body = "".join("\n" + pad + dump(x, indent + 1) for x in rest)
        return "(" + " ".join(head) + body + "\n" + "\t" * indent + ")"
    if isinstance(node, Sym):
        return str(node)
    if isinstance(node, str):
        return '"' + node.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n") + '"'
    if isinstance(node, float):
        return ("%.4f" % node).rstrip("0").rstrip(".")
    return str(node)


def find(node, key):
    return [x for x in node if isinstance(x, list) and x and x[0] == key]


def first(node, key):
    got = find(node, key)
    return got[0] if got else None
