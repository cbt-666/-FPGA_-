"""启动前校验 models/ 下是否齐了 models_manifest.json 要求的文件。

用法：python3 check_models.py <额外的车道模型文件名>
缺什么打印什么，一行一个；全齐则无输出、退出码 0。
"""
import json
import os
import sys

MODELS = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'models')
MANIFEST = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        'models_manifest.json')


def main(argv):
    try:
        with open(MANIFEST, encoding='utf-8') as fh:
            manifest = json.load(fh)
    except Exception as exc:
        print(f'无法读取 models_manifest.json: {exc}')
        return 1
    wanted = set(manifest.get('models', {}))
    wanted.update(argv[1:])
    missing = [n for n in sorted(wanted) if not os.path.isfile(os.path.join(MODELS, n))]
    for name in missing:
        print(name)
    return 0 if not missing else 1


if __name__ == '__main__':
    raise SystemExit(main(sys.argv))
