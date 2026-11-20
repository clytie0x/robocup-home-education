"""base/jetson/ports.py is the one port table (base/docs/communication.md, finding 2)."""
import ast
import re
import unittest
from pathlib import Path

import ports

BASE = Path(__file__).resolve().parents[2]  # base/ on the laptop; on the Pi only hmc-base/ is there
# an address with a port in a string ("http://127.0.0.1:8081/", "<jetson>:8082") or a bind tuple ('0.0.0.0', 8082)
ADDRESS = re.compile(r"(?:localhost|\d+\.\d+\.\d+\.\d+|<[\w -]+>):(\d{4,5})\b"
                     r"""|\(\s*['"](?:0\.0\.0\.0|localhost|127\.0\.0\.1)?['"]\s*,\s*(\d{4,5})\s*\)""")
OUTSIDE = ("arm", "voice")  # code that cannot import ports.py (other workspaces on the Jetson): numbers only


def numbers():
    return {k: v for k, v in vars(ports).items() if k.isupper() and isinstance(v, int)}


def literal_ports(tree, table):
    """(line, text) for each table number assigned to a *PORT* name, a port= default or keyword, or --port default."""
    found = []

    def hits(node):
        return [n for n in ast.walk(node) if isinstance(n, ast.Constant) and type(n.value) is int and n.value in table]

    for node in ast.walk(tree):
        if isinstance(node, (ast.Assign, ast.AnnAssign)):
            targets = node.targets if isinstance(node, ast.Assign) else [node.target]
            names = [n.id for t in targets for n in ast.walk(t) if isinstance(n, ast.Name)]
            if node.value is not None and any("PORT" in n.upper() for n in names):
                found += [(c.lineno, "%s = %d" % (names[0], c.value)) for c in hits(node.value)]
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda)):
            args = node.args.posonlyargs + node.args.args
            pairs = list(zip(args[len(args) - len(node.args.defaults):], node.args.defaults))
            pairs += [(a, d) for a, d in zip(node.args.kwonlyargs, node.args.kw_defaults) if d is not None]
            found += [(c.lineno, "%s=%d" % (a.arg, c.value)) for a, d in pairs if "port" in a.arg.lower()
                      for c in hits(d)]
        elif isinstance(node, ast.Call):
            flag = node.args and isinstance(node.args[0], ast.Constant) and "port" in str(node.args[0].value).lower()
            for k in node.keywords:
                if k.arg and ("port" in k.arg.lower() or (flag and k.arg == "default")):
                    found += [(c.lineno, "%s=%d" % (k.arg, c.value)) for c in hits(k.value)]
    return found


class PortsTest(unittest.TestCase):
    def test_every_port_belongs_to_one_host(self):
        listed = sorted(name for group in ports.HOSTS.values() for name in group)
        self.assertEqual(listed, sorted(numbers()), "each port name in exactly one HOSTS group")

    def test_no_number_twice_on_one_host(self):
        for host, group in ports.HOSTS.items():
            values = [getattr(ports, name) for name in group]
            self.assertEqual(len(values), len(set(values)), "%s uses a port twice: %s" % (host, sorted(values)))

    def test_the_table_has_the_known_ports(self):
        pi, jetson = ([getattr(ports, n) for n in ports.HOSTS[h]] for h in ("pi", "jetson"))
        self.assertEqual(sorted(pi), [5603, 5604, 5610, 8080, 8081])
        self.assertEqual(sorted(jetson), [5601, 5602, 5605, 5606, 5607, 5608, 8080, 8081, 8082, 8090, 8091])

    def test_scan_finds_a_literal(self):
        table = set(numbers().values())
        tree = ast.parse("A_PORT = 5604\ndef f(host, port=5601): pass\nlink(port=8090)\n"
                         "ap.add_argument('--port', type=int, default=8081)\nN = 5604\n")
        self.assertEqual([t for _, t in literal_ports(tree, table)],
                         ["A_PORT = 5604", "port=5601", "port=8090", "default=8081"])

    @unittest.skipUnless((BASE / "jetson" / "ports.py").exists(), "base/jetson is not here")
    def test_no_module_keeps_its_own_port_number(self):
        table = set(numbers().values())
        left = []
        for folder in (BASE / "pi", BASE / "jetson", BASE / "pc"):
            for path in sorted(folder.rglob("*.py")):
                rel = path.relative_to(BASE)
                if "tests" in rel.parts or path.name == "ports.py":
                    continue
                tree = ast.parse(path.read_text(encoding="utf-8"), str(path))
                left += ["%s:%d %s" % (rel.as_posix(), line, text) for line, text in literal_ports(tree, table)]
        self.assertEqual(left, [], "use ports.py instead")

    def test_address_scan(self):
        found = [m.group(1) or m.group(2) for m in ADDRESS.finditer(
            "u = 'http://127.0.0.1:8081/x'\nv = '<jetson-ip>:8091'\nS(('0.0.0.0', 8082), H)\np('rate', 16000)\n")]
        self.assertEqual(found, ["8081", "8091", "8082"])

    @unittest.skipUnless((BASE.parent / "arm").exists(), "arm/ and voice/ are not here")
    def test_other_teams_use_only_table_ports(self):  # a stale or new number there is a port nobody planned
        table = set(numbers().values())
        left = []
        for folder in OUTSIDE:
            for path in sorted((BASE.parent / folder).rglob("*.py")):
                if "test" in path.parts or path.name.startswith("test_"):
                    continue
                text = path.read_text(encoding="utf-8", errors="replace")
                for m in ADDRESS.finditer(text):
                    if int(m.group(1) or m.group(2)) not in table:
                        left.append("%s:%d %s" % (path.relative_to(BASE.parent).as_posix(),
                                                  text.count("\n", 0, m.start()) + 1, m.group(0)))
        self.assertEqual(left, [], "add the port to base/jetson/ports.py (and communication.md) or use one there")


if __name__ == "__main__":
    unittest.main()
