"""静态审查：所有跨文件的函数调用，关键字参数必须在被调函数的签名里。

2026-10-29 的 V17_4 就是因为 `run_live` 给 `main_entry` 传了
`last_good_traffic_hold=`，而 `main_entry` 的签名漏了它，整场 458 轮全部
TypeError。300 项单元测试一条都没抓到，因为它们都直接调 `main_entry`，
没人走 `run_live` 那条真实调用链。

这个脚本不在乎逻辑，只保证"名字对得上"。
"""
import ast
import sys
from pathlib import Path

ROOT = Path(sys.argv[1] if len(sys.argv) > 1 else '.').resolve()
FILES = sorted(p for p in ROOT.glob('*.py'))


def signatures():
    """name -> set(参数名)，覆盖本工程内的顶层函数与方法。"""
    out = {}
    for p in FILES:
        try:
            tree = ast.parse(p.read_text(encoding='utf-8'))
        except Exception:
            continue
        for node in ast.walk(tree):
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                a = node.args
                names = {x.arg for x in a.posonlyargs + a.args + a.kwonlyargs}
                if a.vararg or a.kwarg:
                    names.add('*')          # 接受任意关键字，不检查
                out.setdefault(node.name, set()).update(names)
    return out


def methods():
    """方法名 -> set(参数名)。同名方法取并集；有 **kwargs 的记为 '*'。

    2026-10-29 审查发现第一版只检查 `ast.Name`（裸函数调用），
    漏掉了所有 `obj.method(...)`。而 `detectors.lane_mask(...)`、
    `self.hold.expire(...)` 这类方法调用断链一样会整场 TypeError。
    """
    out = {}
    for p in FILES:
        try:
            tree = ast.parse(p.read_text(encoding='utf-8'))
        except Exception:
            continue
        for node in ast.walk(tree):
            if isinstance(node, ast.FunctionDef):
                a = node.args
                names = {x.arg for x in a.posonlyargs + a.args + a.kwonlyargs}
                if a.vararg or a.kwarg:
                    names.add('*')
                out.setdefault(node.name, set()).update(names)
    return out


def calls():
    for p in FILES:
        try:
            src = p.read_text(encoding='utf-8')
            tree = ast.parse(src)
        except Exception:
            continue
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            kws = [k.arg for k in node.keywords if k.arg]
            if not kws:
                continue
            if isinstance(node.func, ast.Name):
                yield p, node.lineno, node.func.id, kws
            elif isinstance(node.func, ast.Attribute):
                # obj.method(...) —— 按方法名匹配
                yield p, node.lineno, node.func.attr, kws


def main():
    sig = signatures()
    for name, params in methods().items():
        sig.setdefault(name, set()).update(params)
    bad = 0
    checked = 0
    for path, line, name, kws in calls():
        if name not in sig:
            continue
        params = sig[name]
        if '*' in params:
            continue
        checked += 1
        extra = [k for k in kws if k not in params]
        if extra:
            bad += 1
            rel = path.relative_to(ROOT)
            print(f'{rel}:{line}  {name}() 不认识的参数: {", ".join(extra)}')
    print(f'\n检查了 {checked} 个带关键字参数的调用，问题 {bad} 处')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
