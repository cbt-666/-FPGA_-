#!/usr/bin/env python3
"""车道线虚实 —— 3类语义分割 (背景 / 白虚线 / 实线)

为什么是 3 类而不是 4 类：
  赛题只考"能否变道"。黄虚线跨过去是逆行，本就不该变道 ->
  "禁止跨越"的东西(白实/黄虚/黄实/双黄/导流线)全部并成一类就够了。
  于是判据塌缩为：该侧边界是【白色虚线】-> 可变道；否则直行。

输入：FPGA 灰度 Y = (77R+150G+29B)>>8，复制成 3 通道喂 ImageNet 预训练骨干
网络：MobileNetV3-Small encoder + 轻量 FPN-lite decoder (~2.7M 参数)
损失：加权 CE + Dice（细线类像素占比极低，必须加权）

用法:
  python train_lane_seg.py --epochs 60
"""
import os
import csv
import random
import argparse
import collections
import numpy as np
import cv2
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader
import torchvision.models as tvm

ROOT = r'D:\ADAS_Datasets\06_车道线_lane\_seg_cache'   # 预处理缓存，见 cache_seg_data.py
OUT = r'D:\ADAS_Datasets\06_车道线_lane\_runs\seg'
WIN = (512, 288)          # (W, H)  网络输入
BIN_TH = 130              # 与队友 gray_binarization 一致（可由 --th 覆盖）
BIN_JIT = 18              # 二值化模式下的阈值抖动幅度
GEO = 0.0                 # 几何增强强度（透视/缩放/长宽比），0 = 关闭
MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)
MODE = 'gray'             # 'gray' | 'binary'，由 main() 按命令行覆盖


def fpga_gray(bgr):
    """与队友 FPGA 完全一致的灰度公式"""
    b = bgr[:, :, 0].astype(np.int32)
    g = bgr[:, :, 1].astype(np.int32)
    r = bgr[:, :, 2].astype(np.int32)
    return ((77 * r + 150 * g + 29 * b) >> 8).astype(np.uint8)


def binarize_pool(g, out_wh, th=BIN_TH):
    """在缓存分辨率(1024x576)二值化，再池化到 out_wh

    顺序很关键：车道线在 1080p 只有约 3px 宽，若先缩小再二值化，
    插值会把细线糊掉导致断裂。先二值化再池化，细线能完整保留。
    池化倍数取 2 或 4（最大池化，白像素不会被平均掉）。
    """
    b = np.where(g > th, 255, 0).astype(np.uint8)
    f = max(int(round(g.shape[1] / out_wh[0])), 1)
    h, w = b.shape
    b = b[:h // f * f, :w // f * f]
    return b.reshape(h // f, f, w // f, f).max(axis=(1, 3))


# 相对阈值参数：核宽 = 0.157*图宽，偏移 10（见 relative_threshold.py 的扫描结果）
REL_K = 161          # 1024 宽下的等价核宽 (1920 下是 301)
REL_OFF = 10


def binarize_rel_pool(g, out_wh, off=REL_OFF, k=REL_K):
    """相对阈值二值化 + 池化

    二分 = (原图 - 水平均值) > offset
    水平均值当"局部路面基准"，抵消路面绝对亮度差异 ——
    固定阈值 130 在亮灰色沥青上会把近一半路面变白，车道线被淹。
    FPGA 实现只要一条 300 级行缓冲 + 滑动求和，比中值滤波便宜得多。
    """
    ref = cv2.blur(g, (k, 1))
    b = np.where(g.astype(np.int16) - ref.astype(np.int16) > off, 255, 0).astype(np.uint8)
    f = max(int(round(g.shape[1] / out_wh[0])), 1)
    h, w = b.shape
    b = b[:h // f * f, :w // f * f]
    return b.reshape(h // f, f, w // f, f).max(axis=(1, 3))


def _fit(g, out_wh, nearest=False):
    """把 g 贴到 out_wh

    · 整数倍降采样 -> 最大池化（保住细线：平均/最近邻会把 1~2px 的车道线抽没）
    · 其它情况（升采样/非整数比）-> 插值
    binarize_pool 只能降采样，升采样必须在这里补。
    """
    ow, oh = int(out_wh[0]), int(out_wh[1])
    h, w = g.shape[:2]
    if (w, h) == (ow, oh):
        return g
    fx, fy = w / ow, h / oh
    if (abs(fx - round(fx)) < 1e-6 and abs(fy - round(fy)) < 1e-6
            and round(fx) >= 2):
        f = int(round(fx))
        hh, ww = h // f * f, w // f * f
        return g[:hh, :ww].reshape(hh // f, f, ww // f, f).max(axis=(1, 3))
    return cv2.resize(g, (ow, oh),
                      interpolation=cv2.INTER_NEAREST if nearest
                      else cv2.INTER_AREA)


def geo_aug(img, msk, rng, s=1.0):
    """图像与掩码共用的随机几何增强（透视 / 长宽比 / 缩放 / 平移）

    为什么需要：实测模型对【视角不正】极不鲁棒 —— 一张斜视角低分辨率图会让
    它吐出一大块横跨画面中心的假阳性，盖住真实车道线。
    训练里没有斜视角样本，模型就没见过消失点偏离画面中心的情形。

    做法：在**归一化坐标(0~1)**上构造单应矩阵，图像和掩码各自用自己的
    尺寸去 warp —— 这样即使两者分辨率不同（缓存图 1024x576 / 掩码 768x432）
    也是**同一个物理变换**，不会错位。
    """
    if s <= 0:
        return img, msk
    j = lambda a: float(rng.uniform(-a, a))                        # noqa: E731
    # 目标四角（归一化），绕标准四角做有界扰动
    dst = np.float32([[0, 0], [1, 0], [1, 1], [0, 1]])
    # 透视：四角独立微扰，位移量随 s 放大
    persp = 0.06 * s
    dst = dst + np.float32([[j(persp), j(persp * 0.6)],
                            [j(persp), j(persp * 0.6)],
                            [j(persp), j(persp * 0.6)],
                            [j(persp), j(persp * 0.6)]])
    # 消失点水平平移（整幅左右偏置，模拟车不在画面正中）
    shift = 0.10 * s
    dst[:, 0] += j(shift)
    # 长宽比独立伸缩（各向异性），模拟非 16:9 输入
    asp = 0.10 * s
    ax, ay = 1.0 + j(asp), 1.0 + j(asp)
    dst = (dst - 0.5) * np.float32([ax, ay]) + 0.5
    # 随机缩放（围绕中心），缩放 <1 会引入需要填充的边框
    z = 1.0 - 0.18 * s * rng.random()
    dst = (dst - 0.5) * z + 0.5

    src = np.float32([[0, 0], [1, 0], [1, 1], [0, 1]])

    def warp(a, interp):
        h, w = a.shape[:2]
        # ⚠️ src 和 dst 必须在**同一坐标系**里。曾经 src 用归一化(0~1)、
        # dst 用像素(0~1024)，解出的 H 带了 ×1024 的尺度，作用到真实像素上
        # 全部飞出图像 -> 输出常量图。两边都乘 [w,h] 才对。
        S = np.float32([w, h])
        H = cv2.getPerspectiveTransform(src * S, dst * S)
        return cv2.warpPerspective(a, H, (w, h), flags=interp,
                                   borderMode=cv2.BORDER_CONSTANT, borderValue=0)

    return warp(img, cv2.INTER_LINEAR), warp(msk, cv2.INTER_NEAREST)


def preprocess(bgr, mode, th=None):
    """bgr 原图 -> uint8 单通道网络输入（推理用，走完整分辨率）

    ⚠️ th 不能用 `th=BIN_TH` 这种默认参数写法：默认值在**函数定义时**就绑定，
    之后改 BIN_TH 全局量对它无效（曾因此把 th=190 的评估偷偷跑成了 th=130）。

    ⚠️ 二值化顺序必须是「先二值化再缩放」：车道线在 1080p 只有约 3px 宽，
    先缩放会把细线插值糊掉导致断裂。所以这里在**原分辨率**上二值化，
    再用最近邻贴到 WIN（最近邻不产生灰边，保持二值语义）。
    """
    if th is None:
        th = BIN_TH
    g = fpga_gray(bgr)
    if mode == 'gray':
        return cv2.resize(g, tuple(WIN), interpolation=cv2.INTER_AREA)
    # 原分辨率二值化 -> 贴到 WIN
    if mode == 'binary':
        b = np.where(g > th, 255, 0).astype(np.uint8)
    else:
        ref = cv2.blur(g, (REL_K, 1))
        b = np.where(g.astype(np.int16) - ref.astype(np.int16) > REL_OFF,
                     255, 0).astype(np.uint8)
    return _fit(b, WIN, nearest=True)


def collect(official=False, only_official=False):
    rows = []
    srcs = (('OF',) if only_official else
            (('OF', 'A', 'B') if official else ('A', 'B')))
    for src in srcs:
        for split in ('train', 'valid'):
            d = os.path.join(ROOT, src, split)
            gd, md = os.path.join(d, 'gray'), os.path.join(d, 'mask')
            if not os.path.isdir(gd):
                continue
            for n in sorted(os.listdir(gd)):
                mp = os.path.join(md, n)
                if os.path.exists(mp):
                    rows.append((os.path.join(gd, n), mp, src, split))
    return rows


class DS(Dataset):
    def __init__(self, rows, train=True, mode=None):
        self.rows = rows
        self.train = train
        self.mode = mode or MODE

    def __len__(self):
        return len(self.rows)

    def __getitem__(self, i):
        ip, mp, _s, _sp = self.rows[i]
        # 缓存图：灰度 1024x576(FPGA公式) / 掩码尺寸可能不同 -> 先对齐
        g0 = cv2.imdecode(np.fromfile(ip, np.uint8), cv2.IMREAD_GRAYSCALE)
        mk = cv2.imdecode(np.fromfile(mp, np.uint8), cv2.IMREAD_GRAYSCALE)
        if g0 is None:
            g0 = np.zeros((576, 1024), np.uint8)
        if mk is None:
            mk = np.zeros(g0.shape, np.uint8)
        if mk.shape != g0.shape:
            mk = cv2.resize(mk, (g0.shape[1], g0.shape[0]),
                            interpolation=cv2.INTER_NEAREST)
        mk = np.clip(mk, 0, 2).astype(np.uint8)

        if self.train and self.mode == 'gray':
            a = random.uniform(0.75, 1.30)
            b = random.uniform(-22, 22)
            g0 = np.clip(g0.astype(np.float32) * a + b, 0, 255).astype(np.uint8)
            if random.random() < 0.25:
                g0 = np.clip(g0.astype(np.float32) +
                             np.random.normal(0, random.uniform(2, 8), g0.shape),
                             0, 255).astype(np.uint8)
        if self.train and random.random() < 0.5:
            g0 = g0[:, ::-1].copy()
            mk = mk[:, ::-1].copy()

        # 几何增强：此时 g0 与 mk 已同尺寸，一次透视变换同时作用于两者。
        # （曾让两者各按自己尺寸 warp，等于变成两次全尺寸变换，慢一倍）
        if self.train and GEO > 0:
            g0, mk = geo_aug(g0, mk, random, GEO)

        if self.mode == 'gray':
            g = cv2.resize(g0, tuple(WIN), interpolation=cv2.INTER_AREA)
        else:
            # 二值化：先二值化再缩放（细线才不断），再最近邻贴到 WIN
            if self.mode == 'binary':
                # 阈值抖动幅度随 BIN_TH 等比放大：190±22 和 130±18 的相对宽度
                # 差很多，而不同域的"最佳阈值"能差到 130 vs 190，必须扛得住。
                th = BIN_TH if not self.train else int(
                    random.uniform(BIN_TH - BIN_JIT, BIN_TH + BIN_JIT))
                b = np.where(g0 > th, 255, 0).astype(np.uint8)
            else:
                off = REL_OFF if not self.train else int(random.uniform(6, 15))
                ref = cv2.blur(g0, (REL_K, 1))
                b = np.where(g0.astype(np.int16) - ref.astype(np.int16) > off,
                             255, 0).astype(np.uint8)
            g = _fit(b, WIN, nearest=True)

        if mk.shape[::-1] != WIN:
            mk = cv2.resize(mk, tuple(WIN), interpolation=cv2.INTER_NEAREST)

        x = torch.from_numpy(g.astype(np.float32) / 255.0).unsqueeze(0).repeat(3, 1, 1)
        x = (x - torch.from_numpy(MEAN).view(3, 1, 1)) / torch.from_numpy(STD).view(3, 1, 1)
        return x, torch.from_numpy(mk.astype(np.int64))


class Net(nn.Module):
    """MobileNetV3-Small 骨干 + FPN-lite 解码器

    实测各段输出 (输入 320x640): idx0->16ch@H/2, idx1->16ch@H/4, idx3->24ch@H/8,
    idx8->48ch@H/16, idx12->576ch@H/32。逐级上采样融合，保住细线锐度。
    """

    def __init__(self, nclass=3, pretrained=True):
        super().__init__()
        try:
            w = tvm.MobileNet_V3_Small_Weights.IMAGENET1K_V1 if pretrained else None
            m = tvm.mobilenet_v3_small(weights=w)
        except Exception:
            m = tvm.mobilenet_v3_small(weights=None)
        f = m.features
        self.st0 = f[0:1]     # 16ch  @ H/2
        self.st1 = f[1:2]     # 16ch  @ H/4
        self.st2 = f[2:4]     # 24ch  @ H/8
        self.st3 = f[4:9]     # 48ch  @ H/16
        self.st4 = f[9:13]    # 576ch @ H/32

        def lat(cin, cout):
            return nn.Sequential(nn.Conv2d(cin, cout, 1, bias=False),
                                 nn.BatchNorm2d(cout), nn.ReLU(inplace=True))

        self.l4 = lat(576, 96)
        self.l3 = lat(48, 64)
        self.l2 = lat(24, 48)
        self.l1 = lat(16, 32)
        self.l0 = lat(16, 32)

        def merge(cin, cout):
            return nn.Sequential(nn.Conv2d(cin, cout, 3, padding=1, bias=False),
                                 nn.BatchNorm2d(cout), nn.ReLU(inplace=True))

        self.m3 = merge(96 + 64, 96)
        self.m2 = merge(96 + 48, 64)
        self.m1 = merge(64 + 32, 48)
        self.m0 = merge(48 + 32, 32)
        self.head = nn.Conv2d(32, nclass, 1)

    def forward(self, x):
        c0 = self.st0(x)
        c1 = self.st1(c0)
        c2 = self.st2(c1)
        c3 = self.st3(c2)
        c4 = self.st4(c3)

        def up(t, ref):
            return F.interpolate(t, size=ref.shape[-2:], mode='bilinear',
                                 align_corners=False)

        u = self.m3(torch.cat([up(self.l4(c4), c3), self.l3(c3)], 1))
        u = self.m2(torch.cat([up(u, c2), self.l2(c2)], 1))
        u = self.m1(torch.cat([up(u, c1), self.l1(c1)], 1))
        u = self.m0(torch.cat([up(u, c0), self.l0(c0)], 1))
        u = F.interpolate(u, scale_factor=2, mode='bilinear', align_corners=False)
        return self.head(u)


def dice_loss(logit, tgt, nclass=3, eps=1.0):
    p = torch.softmax(logit, 1)
    oh = F.one_hot(tgt, nclass).permute(0, 3, 1, 2).float()
    dims = (0, 2, 3)
    inter = (p * oh).sum(dims)
    card = p.sum(dims) + oh.sum(dims)
    return 1.0 - ((2 * inter + eps) / (card + eps)).mean()


@torch.no_grad()
def evaluate(net, rows, dev, mode=None):
    net.eval()
    cm = np.zeros((3, 3), np.int64)
    for x, y in DataLoader(DS(rows, False, mode), batch_size=4):
        p = net(x.to(dev)).argmax(1).cpu().numpy()
        t = y.numpy()
        for c in range(3):
            for d in range(3):
                cm[c, d] += int(((t == c) & (p == d)).sum())
    iou, rec = [], []
    for c in (1, 2):
        tp = cm[c, c]
        fn = cm[c, :].sum() - tp
        fp = cm[:, c].sum() - tp
        iou.append(tp / max(tp + fn + fp, 1))
        rec.append(tp / max(tp + fn, 1))
    return iou, rec, cm


def main():
    global MODE, WIN, BIN_TH, BIN_JIT
    ap = argparse.ArgumentParser()
    ap.add_argument('--epochs', type=int, default=60)
    ap.add_argument('--bs', type=int, default=8)
    ap.add_argument('--lr', type=float, default=3e-4)
    ap.add_argument('--mode', choices=['gray', 'binary', 'binrel'], default='binrel',
                    help='binrel = 相对阈值二值化(水平均值基准，与队友链路兼容且不淹线)')
    ap.add_argument('--workers', type=int, default=0)
    ap.add_argument('--res', default='512x288', help='网络输入分辨率 WxH')
    ap.add_argument('--tag', default=None, help='权重文件名后缀')
    ap.add_argument('--init', default=None, help='从已有权重热启动(微调)')
    ap.add_argument('--official', action='store_true',
                    help='训练集里加入官方同域帧(OF)')
    ap.add_argument('--only-official', action='store_true',
                    help='只用官方同域帧训练')
    ap.add_argument('--th', type=int, default=None, help='二值化阈值(覆盖 BIN_TH)')
    ap.add_argument('--thjit', type=int, default=None, help='阈值抖动幅度(覆盖 BIN_JIT)')
    ap.add_argument('--geo', type=float, default=None,
                    help='几何增强强度(透视/缩放/长宽比/平移)，0=关闭')
    a = ap.parse_args()
    MODE = a.mode
    global BIN_TH, BIN_JIT, GEO
    if a.th is not None:
        BIN_TH = a.th
    if a.thjit is not None:
        BIN_JIT = a.thjit
    if a.geo is not None:
        GEO = a.geo
    if MODE == 'binary':
        print('二值化阈值 %d ± %d' % (BIN_TH, BIN_JIT))
    if GEO > 0:
        print('几何增强强度 %.2f (透视/长宽比/缩放/平移)' % GEO)
    w, h = a.res.lower().split('x')
    WIN = (int(w), int(h))
    print('输入分辨率', WIN)

    rows = collect(a.official or a.only_official, a.only_official)
    tr = [r for r in rows if r[3] == 'train']
    va = [r for r in rows if r[3] == 'valid']
    print('模式: %s  |  训练 %d / 验证 %d' % (MODE, len(tr), len(va)))
    c = collections.Counter(r[2] for r in rows)
    print('来源:', dict(c))

    dev = 'cuda' if torch.cuda.is_available() else 'cpu'
    net = Net().to(dev)
    if a.init:
        net.load_state_dict(torch.load(a.init, map_location=dev))
        print('热启动自 %s' % a.init)
    npar = sum(p.numel() for p in net.parameters())
    print('参数量 %.2fM' % (npar / 1e6))

    # 类别权重：细线类像素占比极低，用 sqrt 反频率加权
    cnt = np.zeros(3, np.float64)
    for r in tr[:300]:
        mk = cv2.imdecode(np.fromfile(r[1], np.uint8), cv2.IMREAD_GRAYSCALE)
        if mk is not None:
            mk = np.clip(mk, 0, 2)
            for k in range(3):
                cnt[k] += int((mk == k).sum())
    freq = cnt / max(cnt.sum(), 1)
    w = (1.0 / np.sqrt(np.maximum(freq, 1e-7)))
    w = w / w.sum() * 3
    print('类别频率', np.round(freq, 5), '-> 权重', np.round(w, 3))
    wt = torch.tensor(w, dtype=torch.float32, device=dev)

    opt = torch.optim.AdamW(net.parameters(), lr=a.lr, weight_decay=0.02)
    dl = DataLoader(DS(tr, True), batch_size=a.bs, shuffle=True,
                    num_workers=a.workers, persistent_workers=a.workers > 0,
                    drop_last=True)
    sch = torch.optim.lr_scheduler.OneCycleLR(
        opt, max_lr=a.lr, total_steps=a.epochs * max(len(dl), 1), pct_start=0.25)

    best = -1
    step = 0
    os.makedirs(OUT, exist_ok=True)
    ck = os.path.join(OUT, 'lane_seg_%s%s.pt' % (MODE, '_' + a.tag if a.tag else ''))
    for ep in range(a.epochs):
        net.train()
        tot = 0.0
        for x, y in dl:
            x, y = x.to(dev), y.to(dev)
            opt.zero_grad()
            o = net(x)
            loss = F.cross_entropy(o, y, weight=wt) + 0.5 * dice_loss(o, y)
            loss.backward()
            opt.step()
            sch.step()
            tot += float(loss.item())
            if step % 200 == 0:
                print('  ep%d step%-4d loss %.4f' % (ep + 1, step, loss.item()),
                      flush=True)
            step += 1
        # 末轮：OneCycleLR 已退火到 0，直接用最后一个 epoch 的权重。
        # --only-official 时验证集为空（留出帧被隔离），自选模型无从谈起。
        if ep == a.epochs - 1:
            torch.save(net.state_dict(), ck)
            print('末轮权重已保存（OneCycle 退火到 0）')
        if (ep + 1) % 5 == 0 or ep == a.epochs - 1:
            if not va:
                print('ep%3d loss %.4f | 验证集为空，跳过评估' % (ep + 1, tot / max(len(dl), 1)))
                continue
            iou, rec, _ = evaluate(net, va, dev, MODE)
            print('ep%3d loss %.4f | 虚线 IoU %.3f 召回 %.3f | 实线 IoU %.3f 召回 %.3f'
                  % (ep + 1, tot / max(len(dl), 1), iou[0], rec[0], iou[1], rec[1]))
            score = iou[0] * 0.6 + iou[1] * 0.4
            if score > best:
                best = score
                torch.save(net.state_dict(), ck)
    print('最佳分数 %.4f  已保存 %s' % (best, ck))


if __name__ == '__main__':
    main()
