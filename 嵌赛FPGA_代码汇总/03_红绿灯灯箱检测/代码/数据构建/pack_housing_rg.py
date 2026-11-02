"""新灯箱模型交付包 (纯红绿域版).

输出到 桌面\\嵌赛FPGA\\Rk3568\\红绿灯灯箱\\v2_纯红绿域\\ , 不覆盖 v1 文件。
"""
import os
import shutil
import sys
import zipfile

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

D = r'C:\Users\cbt\Desktop\嵌赛FPGA\Rk3568\红绿灯灯箱'
SUB = os.path.join(D, 'v2_纯红绿域')
SRC = r'D:\ADAS_Datasets\_code'
RK = os.path.join(SRC, 'rknn_out_rg')
DEMO = r'D:\AI\CLaude_workplace\临时文件\demo'

os.makedirs(os.path.join(SUB, 'demo'), exist_ok=True)

ITEMS = [
    (os.path.join(RK, 'housing_3568_fp16.rknn'), 'housing_3568_fp16.rknn'),
    (os.path.join(RK, 'housing_3576_fp16.rknn'), 'housing_3576_fp16.rknn'),
    (os.path.join(SRC, 'housing_rg_split.onnx'), 'housing_split.onnx'),
    (r'D:\ADAS_Datasets\_runs\housing_rg_n1280\weights\best.pt', 'housing_best.pt'),
    (os.path.join(SRC, 'conv_housing_fp16.py'), 'conv_housing_fp16.py'),
    (os.path.join(SRC, 'export_housing_split.py'), 'export_housing_split.py'),
    (os.path.join(SRC, 'build_housing_rg.py'), 'build_housing_rg.py'),
    (os.path.join(SRC, 'train_housing_rg.py'), 'train_housing_rg.py'),
    (os.path.join(SRC, 'verify_housing_rg_rknn.py'), 'decode_verify_example.py'),
    (os.path.join(SRC, 'board_housing_rules.py'), 'board_filter_rules.py'),
]

print('== 复制文件 ==')
for s, name in ITEMS:
    if os.path.isfile(s):
        shutil.copy2(s, os.path.join(SUB, name))
        print('  ✓ %-28s %.2f MB' % (name, os.path.getsize(s) / 1e6))
    else:
        print('  ✗ MISS %s' % s)

print('\n== 效果图 ==')
for f in ('09_新旧模型对比.png', '10_板端_新旧对照.png', '11_板端放大_新模型.png'):
    p = os.path.join(DEMO, f)
    if os.path.isfile(p):
        shutil.copy2(p, os.path.join(SUB, 'demo', f))
        print('  ✓ demo/%s' % f)
    else:
        print('  ✗ MISS %s' % f)

# ---- 打 zip ----
zp = os.path.join(D, '红绿灯灯箱_v2_纯红绿域.zip')
if os.path.exists(zp):
    os.remove(zp)
with zipfile.ZipFile(zp, 'w', zipfile.ZIP_DEFLATED, compresslevel=6) as z:
    for root, _, files in os.walk(SUB):
        for f in files:
            full = os.path.join(root, f)
            arc = os.path.join('红绿灯灯箱_v2_纯红绿域',
                               os.path.relpath(full, SUB))
            z.write(full, arc)
print('\n==> %s  %.2f MB' % (zp, os.path.getsize(zp) / 1e6))
print('==> %s' % SUB)
