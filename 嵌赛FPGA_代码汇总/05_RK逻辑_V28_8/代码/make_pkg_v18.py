#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""打包 V18：单包，LF 换行 + 可执行位。

全灰度版（G640）已于 2026-10-28 淘汰，不再出包。

用法：python make_pkg_v17_4.py <工程目录> <输出目录>
"""
import stat
import sys
import zipfile
from pathlib import Path

SKIP_DIRS = {'__pycache__', 'captures', '.git',
             # 工程目录里有一个同名旧副本（V15.7 时代的整份拷贝），
             # 打进去会让包多 31 MB 且混淆"哪份 main.py 在跑"。
             'RK3568_FPGA_V15_7_FP16_SCAN',
             'reference'}
SKIP_SUFFIX = ('.pyc', '.log', '.zip')
EXEC_EXT = {'.sh'}


def iter_files(root: Path):
    for p in sorted(root.rglob('*')):
        if p.is_dir():
            continue
        rel = p.relative_to(root)
        if any(part in SKIP_DIRS for part in rel.parts):
            continue
        if p.suffix in SKIP_SUFFIX:
            continue
        yield p, rel


def write_zip(root: Path, out: Path, top: str = 'RK3568_FPGA_V17'):
    count = 0
    total = 0
    with zipfile.ZipFile(out, 'w', zipfile.ZIP_DEFLATED, compresslevel=6) as z:
        for path, rel in iter_files(root):
            data = path.read_bytes()
            if path.suffix in {'.py', '.sh', '.md', '.txt', '.json', '.csv'}:
                data = data.replace(b'\r\n', b'\n')
            info = zipfile.ZipInfo(str(Path(top) / rel).replace('\\', '/'))
            perm = 0o755 if path.suffix in EXEC_EXT else 0o644
            info.external_attr = (stat.S_IFREG | perm) << 16
            info.create_system = 3
            info.compress_type = zipfile.ZIP_DEFLATED
            z.writestr(info, data)
            count += 1
            total += len(data)
    print(f'{out.name}: {count} files, {total/1e6:.1f} MB raw')


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    root = Path(sys.argv[1]).resolve()
    outdir = Path(sys.argv[2]).resolve()
    outdir.mkdir(parents=True, exist_ok=True)
    write_zip(root, outdir / 'RK3568_FPGA_V21_CROSSWALK_TRAFFIC_EVIDENCE.zip')
    return 0


if __name__ == '__main__':
    sys.exit(main())
