"""__init__ 에서 아직 만들지 않은 self 값을 먼저 읽는 실수를 잡는다.

2026-09-30: base_node 에 self.pair = ... and self.fine 을 self.fine 보다 먼저
넣어 노드가 시작하자마자 죽기를 1분간 되풀이했다. 함수 단위 시험은 __init__ 을
돌리지 않아 못 잡았다. 실제 노드를 띄우지 않고 소스 구조로 본다.
"""
import ast
import pathlib

SRC = pathlib.Path(__file__).resolve().parent.parent / "und_amr_bringup"


def _init_order_problems(path, cls_name):
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == cls_name)
    init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "__init__")
    methods = {n.name for n in cls.body if isinstance(n, ast.FunctionDef)}
    consts = {t.id for n in cls.body if isinstance(n, ast.Assign)
              for t in n.targets if isinstance(t, ast.Name)}
    # __init__ 안에서 한 번이라도 대입되는 값들
    assigned_somewhere = set()
    for node in ast.walk(init):
        if isinstance(node, ast.Attribute) and isinstance(node.ctx, ast.Store) \
                and isinstance(node.value, ast.Name) and node.value.id == "self":
            assigned_somewhere.add(node.attr)
    seen, problems = set(), []
    for stmt in init.body:
        loads = [n for n in ast.walk(stmt) if isinstance(n, ast.Attribute)
                 and isinstance(n.ctx, ast.Load) and isinstance(n.value, ast.Name)
                 and n.value.id == "self"]
        # 콜백(lambda) 안에서 읽는 것은 나중에 불린다
        lam = {id(x) for l in ast.walk(stmt) if isinstance(l, ast.Lambda)
               for x in ast.walk(l)}
        for n in loads:
            a = n.attr
            if (id(n) in lam or a in methods or a in consts or a in seen
                    or a not in assigned_somewhere):
                continue
            problems.append("%s:%d self.%s 를 대입하기 전에 읽음" % (path.name, n.lineno, a))
        for n in ast.walk(stmt):
            if isinstance(n, ast.Attribute) and isinstance(n.ctx, ast.Store) \
                    and isinstance(n.value, ast.Name) and n.value.id == "self":
                seen.add(n.attr)
    return problems


def test_base_node_init_order():
    assert _init_order_problems(SRC / "base_node.py", "BaseNode") == []
