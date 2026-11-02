import os
import shutil

ROOT = os.path.dirname(os.path.abspath(__file__))
RAW = os.path.join(ROOT, "raw")
DS = os.path.join(ROOT, "dataset")
CLASSES = {"video1": "1", "video2": "2"}


def main():
    if os.path.isdir(DS):
        shutil.rmtree(DS)
    for mod in ("train", "val"):
        for tag, cls in CLASSES.items():
            dst = os.path.join(DS, mod, cls)
            os.makedirs(dst, exist_ok=True)
            src_dir = os.path.join(RAW, tag)
            files = sorted(f for f in os.listdir(src_dir) if f.endswith(".jpg"))
            for f in files:
                shutil.copy2(os.path.join(src_dir, f), os.path.join(dst, f))
            print(f"{mod}/{cls} <- {tag}: {len(files)} imgs")
    total = sum(len(os.listdir(os.path.join(DS, m, c))) for m in ("train", "val") for c in ("1", "2"))
    print(f"total files: {total}")


if __name__ == "__main__":
    main()
