import os
import time

import numpy as np
import onnx
import onnxruntime as ort
import torch

import nn_def

ROOT = os.path.dirname(os.path.abspath(__file__))
CKPT = os.path.join(ROOT, "out", "cls2_small.pt")
ONNX = os.path.join(ROOT, "out", "cls2_small.onnx")

ck = torch.load(CKPT, map_location="cpu", weights_only=False)
IMG = ck["img"]
model = nn_def.build()
model.load_state_dict(ck["state_dict"])
model.eval()

dummy = torch.randn(1, 3, IMG, IMG)
torch.onnx.export(
    model, dummy, ONNX,
    input_names=["images"], output_names=["logits"],
    opset_version=12, do_constant_folding=True, dynamo=False,
)
onnx.checker.check_model(onnx.load(ONNX))
print(f"onnx saved {os.path.getsize(ONNX)/1e6:.2f}MB  input 1x3x{IMG}x{IMG}")

sess = ort.InferenceSession(ONNX, providers=["CPUExecutionProvider"])
x = np.random.randn(1, 3, IMG, IMG).astype(np.float32)
for _ in range(5):
    sess.run(None, {"images": x})
t = time.time()
for _ in range(50):
    out = sess.run(None, {"images": x})
dt = (time.time() - t) / 50
print(f"onnxruntime CPU: {dt*1000:.2f}ms/frame @1x3x{IMG}x{IMG}")
print("output shape", out[0].shape)

pt_sz = os.path.getsize(CKPT) / 1e6
print(f"\nmodel size: .pt={pt_sz:.2f}MB  .onnx={os.path.getsize(ONNX)/1e6:.2f}MB")
print(f"params: {sum(p.numel() for p in model.parameters()):,}")
