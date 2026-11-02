import os
import sys
import time

import numpy as np
import onnx
import onnxruntime as ort
import torch
import importlib.util

ROOT = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(ROOT, "out")

spec = importlib.util.spec_from_file_location("t12", os.path.join(ROOT, "12_train_yrg.py"))
t12 = importlib.util.module_from_spec(spec)
sys.modules["t12"] = t12
spec.loader.exec_module(t12)


def export(tag, n_cls):
    ck = torch.load(os.path.join(OUT, f"cls_{tag}.pt"), map_location="cpu", weights_only=False)
    img = ck["img"]
    model = t12.Net(n_cls)
    model.load_state_dict(ck["state_dict"])
    model.eval()
    p = os.path.join(OUT, f"cls_{tag}.onnx")
    torch.onnx.export(model, torch.randn(1, 3, img, img), p,
                      input_names=["images"], output_names=["logits"],
                      opset_version=12, do_constant_folding=True, dynamo=False)
    onnx.checker.check_model(onnx.load(p))

    sess = ort.InferenceSession(p, providers=["CPUExecutionProvider"])
    x = np.random.randn(1, 3, img, img).astype(np.float32)
    for _ in range(5):
        sess.run(None, {"images": x})
    t = time.time()
    for _ in range(50):
        sess.run(None, {"images": x})
    dt = (time.time() - t) / 50

    with torch.no_grad():
        ox = sess.run(None, {"images": x})[0][0]
        tx = model(torch.from_numpy(x)).numpy()[0]
    diff = np.abs(ox - tx).max()

    print(f"{tag}: input 1x3x{img}x{img}  .onnx={os.path.getsize(p)/1e6:.2f}MB  "
          f"ORT_CPU={dt*1000:.2f}ms/frame  onnx-vs-torch maxdiff={diff:.2e}  "
          f"params={sum(q.numel() for q in model.parameters()):,}")


for tag, n in [("yrg2", 2), ("yrg3", 3)]:
    export(tag, n)
