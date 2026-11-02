"""打包 RK3568 工程为 zip —— 统一行尾、校验后再打包。

为什么需要它（2026-10-31 上板事故）
------------------------------------
V25 包里的 `run.sh` 等 30 个文件是 CRLF。Linux 的 bash **在执行前就要解析
整个脚本**，`set -uo pipefail\r` 里的 `\r` 被当成选项名的一部分：

    : invalid option name pipefail
    run.sh: line 11: cd: $'...\\r': No such file or directory
    run.sh: line 36: syntax error near unexpected token `$'do\\r''

`run.sh` 自己第 2 步就 `sed -i 's/\\r$//' ./*.sh`，但那一步**永远到不了** ——
脚本先要能被解析。所以**只能在打包时保证 LF**，不能指望板上自救。

规则
----
* `.sh` 一律 LF（硬要求，打包前校验，不通过直接失败）
* `.py` / `.md` / `.json` / `.txt` / `.csv` / `.yaml` 也统一成 LF（无坏处）
* 二进制（.rknn / .pt / .onnx / .png …）一个字节都不碰
* 不许有 UTF-8 BOM（会让 `#!/usr/bin/env bash` 失效）
* 打包后再把 zip 读回来复验一次
"""
from __future__ import annotations

import argparse
import sys
import zipfile
from pathlib import Path

TEXT_SUFFIXES = {'.sh', '.py', '.md', '.json', '.txt', '.csv', '.yaml', '.yml',
                 '.cfg', '.ini', '.toml', '.html', '.js', '.css'}
EXCLUDE_DIRS = {'__pycache__', 'diag_frames', 'captures', '.git'}
EXCLUDE_SUFFIXES = {'.pyc', '.pyo', '.npy'}


def _text_files(root: Path):
    for p in sorted(root.rglob('*')):
        if not p.is_file():
            continue
        if any(part in EXCLUDE_DIRS for part in p.relative_to(root).parts[:-1]):
            continue
        if p.suffix in EXCLUDE_SUFFIXES:
            continue
        yield p


def normalize(root: Path, dry=False):
    """把文本文件的 CRLF/BOM 去掉。返回改动清单。"""
    changed = []
    for p in _text_files(root):
        if p.suffix not in TEXT_SUFFIXES:
            continue
        raw = p.read_bytes()
        new = raw
        if new.startswith(b'\xef\xbb\xbf'):
            new = new[3:]
        if b'\r\n' in new:
            new = new.replace(b'\r\n', b'\n')
        if new != raw:
            changed.append((p.relative_to(root).as_posix(),
                            raw.count(b'\r\n'), len(raw) - len(new)))
            if not dry:
                p.write_bytes(new)
    return changed


def verify(root: Path):
    """打包前校验：任何 .sh 有 CR 或任何文本文件有 BOM 都算失败。"""
    problems = []
    for p in _text_files(root):
        if p.suffix != '.sh':
            continue
        raw = p.read_bytes()
        if b'\r' in raw:
            problems.append(f'{p.relative_to(root).as_posix()}: 仍含 CR')
        if raw.startswith(b'\xef\xbb\xbf'):
            problems.append(f'{p.relative_to(root).as_posix()}: 有 UTF-8 BOM')
        if not raw.startswith(b'#!'):
            problems.append(f'{p.relative_to(root).as_posix()}: 缺少 shebang')
    return problems


def verify_zip(zip_path: Path, expected_len=120):
    """打包后把 zip 读回来，确认里面的 .sh 是 LF 且有执行位。"""
    problems = []
    with zipfile.ZipFile(zip_path) as z:
        names = z.namelist()
        for info in z.infolist():
            n = info.filename
            if not n.endswith('.sh'):
                continue
            data = z.read(n)
            if b'\r' in data:
                problems.append(f'{n}: zip 内仍含 CR')
            if data.startswith(b'\xef\xbb\xbf'):
                problems.append(f'{n}: zip 内有 BOM')
            if not (info.external_attr >> 16) & 0o111:
                problems.append(f'{n}: zip 内没有执行位')
    return names, problems


def build(root: Path, out: Path, prefix_len=64):
    """root 的**父目录**作为 zip 内的顶层，保持 <ROOT_NAME>/... 结构。

    `.sh` / `.py` 存 0o755：`run.sh` 里最后一行是 `exec "./start_v12.sh"`，
    那要求 start_v12.sh 本身可执行。python 的 zipfile 默认写 0o600，
    解压后就是 644 —— run.sh 第 1 步虽然有 chmod，但不能指望它兜底。
    """
    n = 0
    with zipfile.ZipFile(out, 'w', zipfile.ZIP_DEFLATED, compresslevel=6) as z:
        for p in _text_files(root):
            zi = zipfile.ZipInfo.from_file(p, p.relative_to(root.parent).as_posix())
            zi.compress_type = zipfile.ZIP_DEFLATED
            if p.suffix in ('.sh', '.py'):
                zi.external_attr = (0o755 << 16) | 0o100000
            else:
                zi.external_attr = (0o644 << 16) | 0o100000
            with open(p, 'rb') as fh:
                z.writestr(zi, fh.read())
            n += 1
    return n


def main():
    ap = argparse.ArgumentParser(description='行尾归一 + 校验 + 打包')
    ap.add_argument('--src', type=Path, required=True, help='工程目录（如 .../RK3568_FPGA_V17）')
    ap.add_argument('--out', type=Path, required=True, help='输出 zip 路径')
    ap.add_argument('--normalize', action='store_true', help='先就地统一行尾')
    ap.add_argument('--dry-run', action='store_true', help='只报告，不改不打包')
    a = ap.parse_args()

    src = a.src.resolve()
    if not src.is_dir():
        raise SystemExit(f'不是目录：{src}')

    if a.normalize:
        changed = normalize(src, dry=a.dry_run)
        print(f'行尾归一：{len(changed)} 个文件')
        for name, crlf, saved in changed:
            print(f'  {name}: CRLF×{crlf} (-{saved}B)')
        if a.dry_run:
            return

    problems = verify(src)
    if problems:
        print('[FAIL] 打包前校验失败：')
        for x in problems:
            print('  ', x)
        raise SystemExit(1)
    print('[OK] 打包前校验通过（.sh 全部 LF、无 BOM、都有 shebang）')

    n = build(src, a.out)
    names, zproblems = verify_zip(a.out)
    if zproblems:
        print('[FAIL] 打包后复验失败：')
        for x in zproblems:
            print('  ', x)
        a.out.unlink(missing_ok=True)
        raise SystemExit(1)
    size = a.out.stat().st_size
    print(f'[OK] 打包后复验通过：{n} 个文件，{a.out.name} = {size:,} B ({size/1048576:.2f} MB)')


if __name__ == '__main__':
    main()
