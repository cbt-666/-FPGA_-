#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""中国红绿灯【灯箱】标注工具 v2

⚠️ 输入是【红绿灰度化后】的图（队友 FPGA 输出域），不是彩色原图！
   灰度化规则: rg_gray_cn  (G>=80 & G-R>=55 & G>B | R>=96 & R*5>G*8 & R+16>=B)

任务: 给每个"灯箱"(三个灯位合成的大框)画一个框, 标注 = 亮着那颗灯的颜色。
     灯箱全灭(图中看不到任何颜色)时标 off。

三类标签: 红 / 绿 / off
     红 = 红灯亮(红圆灯 或 红箭头)
     绿 = 绿灯亮
     off = 黄灯亮(灰度化后变灰) / 灯熄灭 / 整个灯箱全灭
     ⇒ 下游: 红 和 off 一律判"停止", 绿判"通行"

为什么这么标: 下游用【框数量】判方向
     1 框 = 圆灯(全向)      2 框 = 左箭头 + 圆(直行)
     3 框 = 左 + 直 + 右箭头

界面: 左=整帧(拉框/缩放/平移)  右=当前框放大(核对手感)

操作:
  鼠标左键拖空白   = 画新框 (用当前激活的类别)
  鼠标左键点已有框 = 选中它
  鼠标右键点框     = 直接删掉
  鼠标中键拖       = 平移画面
  滚轮             = 以光标为中心缩放
  1 / 2 / 3        = 设激活类别(红/绿/off); 若已选中某框则改它的类别
  Del / Backspace  = 删除选中框
  A                = 显示/隐藏预填框(对比用)
  Z = 适应窗口   N = 下一张   P = 上一张   Esc = 保存退出

自动存盘; 重开断点续标。
产物: 36_中国灯箱池/_labels.json
      {文件名: {"boxes":[[cls,x1,y1,x2,y2],...], "done":true}}
"""
import os
import sys
import json
import collections

import numpy as np
import cv2
import tkinter as tk
from tkinter import messagebox

DST = r"D:\ADAS_Datasets\36_中国灯箱池"
IMG = os.path.join(DST, "images_png")
PRE = os.path.join(DST, "_prefill.json")
LAB = os.path.join(DST, "_labels.json")
HINT = os.path.join(DST, "_haslight.json")     # v13 门控: 该帧是否可能有红绿灯

CLS = {1: "box"}                     # 单类: 只标"这是不是灯箱", 颜色交给下游算法
CLS_BGR = {"box": (0, 255, 255)}
CLS_HEX = {"box": "#30ffff"}
# 旧数据里的红/绿/off 仍能显示(复看历史标注用), 但新画的框一律是 box
for _k, _v in {"red": (0, 0, 255), "green": (0, 255, 0), "off": (200, 200, 200)}.items():
    CLS_BGR.setdefault(_k, _v)
for _k, _v in {"red": "#ff3030", "green": "#30ff30", "off": "#cccccc"}.items():
    CLS_HEX.setdefault(_k, _v)
NO_LIGHT = "nolite"          # 键 4 的"伪类别": 本图没有红绿灯
PRE_HEX = "#777777"          # 预填框(未确认) 灰色
SEL_HEX = "#ffffff"          # 选中框 白色

CANW = 980                   # 画布宽度
CANH = 760                   # 画布高度
ZOOM_MIN, ZOOM_MAX = 0.10, 6.0


def rd(p):
    return cv2.imdecode(np.fromfile(p, np.uint8), cv2.IMREAD_COLOR)


def bgr2photo(bgr):
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    h, w = rgb.shape[:2]
    return tk.PhotoImage(data=b'P6\n%d %d\n255\n' % (w, h) + rgb.tobytes())


def auto_cls(img, x1, y1, x2, y2):
    """按框内纯红/纯绿像素数自动判色 (36池实测 96.5% 准)。
    红绿都有 -> 取较多的那个; 都没有 -> off。"""
    H, W = img.shape[:2]
    a, b = max(0, int(x1)), max(0, int(y1))
    c, d = min(W, int(x2)), min(H, int(y2))
    if c <= a or d <= b:
        return "off", 0, 0
    R = img[b:d, a:c, 2].astype(np.int16)
    G = img[b:d, a:c, 1].astype(np.int16)
    B = img[b:d, a:c, 0].astype(np.int16)
    nR = int(((R > 150) & (G < 110) & (B < 110)).sum())
    nG = int(((G > 150) & (R < 110) & (B < 110)).sum())
    if nR < 5 and nG < 5:
        return "off", nR, nG
    return ("red" if nR >= nG else "green"), nR, nG


class App:
    def __init__(self, root, files, labels, prefill, hint=None):
        self.root = root
        self.files = files
        self.lab = labels
        self.pre = prefill                 # 现在是空的(不再自动标注)
        self.hint = hint or {}             # v13 门控: True=可能有红绿灯
        self.i = 0
        self.zoom = 1.0
        self.ox = 0.0          # 图像左上角在画布中的位置
        self.oy = 0.0
        self.img = None
        self.active = "box"
        self.sel = None        # 选中框索引
        self.drag = None       # ('new', x0,y0) | ('pan', x0,y0,ox,oy)
        self.rubber = None
        self.show_pre = True       # 参考框默认显示(A键切换); 36池无参考框故无影响
        self.auto_cls = True       # 画框后按框内像素自动判红/绿/off (1/2/3 仍可改)
        self.n_total = len(files)
        self.icache = {}
        self._build()
        self.load(self.i)

    # ---------------- UI ----------------
    def _build(self):
        self.bar = tk.Label(self.root, anchor='w', font=('Consolas', 12),
                            bg='#111', fg='#0ff')
        self.bar.pack(fill='x')

        # ---- 类别按钮条: 鼠标直接点, 不用记键盘 ----
        bf = tk.Frame(self.root, bg='#111')
        bf.pack(fill='x', pady=4)
        self.btns = {}
        tk.Button(bf, text='虚框正确\n(M) 下一张', font=('Consolas', 13, 'bold'),
                  width=13, height=2, bg='#30ffff', fg='#000', relief='raised', bd=3,
                  command=self.accept_refs).pack(side='left', padx=4)
        tk.Button(bf, text='丢弃此图\n(B)', font=('Consolas', 13, 'bold'),
                  width=11, height=2, bg='#7a2a2a', fg='#fff', relief='raised', bd=3,
                  command=self.reject_img).pack(side='left', padx=4)
        tk.Button(bf, text='无红绿灯\n(4)', font=('Consolas', 13, 'bold'),
                  width=11, height=2, bg='#404060', fg='#fff', relief='raised',
                  bd=3, command=self.toggle_no_light).pack(side='left', padx=4)
        tk.Button(bf, text='参考框\n(A)', font=('Consolas', 12), width=9, height=2,
                  bg='#3a3a3a', fg='#fff', relief='raised', bd=3,
                  command=lambda: (setattr(self, 'show_pre', not self.show_pre),
                                   self.render())).pack(side='left', padx=4)
        for txt, cmd, col, w in (
                ('删除\n(Del)', self.del_sel, '#555555', 8),
                ('适应\n(Z)', lambda: (self.fit(), self.render()), '#2a5a2a', 8),
                ('◀ 上一张\n(P)', self.prev_img, '#2a3a6a', 10),
                ('下一张 ▶\n(N)', self.next_img, '#2a3a6a', 10),
                ('存盘退出\n(Esc)', self.quit, '#6a2a2a', 11)):
            tk.Button(bf, text=txt, font=('Consolas', 12), width=w, height=2,
                      bg=col, fg='#fff', relief='raised', bd=3,
                      command=cmd).pack(side='left', padx=4)

        body = tk.Frame(self.root, bg='#111')
        body.pack(fill='both', expand=True)

        self.can = tk.Canvas(body, width=CANW, height=CANH, bg='#000',
                             highlightthickness=1, highlightbackground='#333')
        self.can.grid(row=0, column=0, padx=4, pady=4)
        self.can.bind('<Button-1>', self.m_down)
        self.can.bind('<B1-Motion>', self.m_move)
        self.can.bind('<ButtonRelease-1>', self.m_up)
        self.can.bind('<Button-3>', self.m_right)
        self.can.bind('<Button-2>', self.p_down)
        self.can.bind('<B2-Motion>', self.p_move)
        self.can.bind('<ButtonRelease-2>', self.p_up)
        self.can.bind('<MouseWheel>', self.wheel)

        side = tk.Frame(body, bg='#111')
        side.grid(row=0, column=1, sticky='n', padx=6, pady=4)
        tk.Label(side, text='当前框放大', font=('Consolas', 10),
                 bg='#111', fg='#ccc').pack()
        self.im_zoom = tk.Label(side, bg='#000')
        self.im_zoom.pack()
        self.info = tk.Label(side, justify='left', font=('Consolas', 10),
                             bg='#111', fg='#aaa', anchor='nw')
        self.info.pack(fill='x', pady=8)

        tk.Label(self.root, justify='left', font=('Consolas', 11), bg='#111',
                 fg='#eee',
                 text='鼠标: 左键拖=画框   左键点框=选中   右键点框=删除   中键拖=平移\n'
                      '滚轮: 下=下一张图  上=上一张图        Ctrl+滚轮 = 缩放\n'
                      'vim:  j=下一个标记   f=上一个标记   h=删掉该标记\n'
                      '1=红  2=绿  3=off  4=本图没有红绿灯(再按一次可恢复)\n'
                      'Z=适应窗口   N/空格=下一张图   P=上一张图   Esc=存盘退出'
                 ).pack(fill='x', pady=4)
        self.root.bind('<Key>', self.on_key)
        self.root.bind('<Escape>', lambda e: self.quit())

    # ---------------- 坐标 ----------------
    def w2c(self, x, y):
        """图像坐标 -> 画布坐标"""
        return x * self.zoom + self.ox, y * self.zoom + self.oy

    def c2w(self, cx, cy):
        """画布坐标 -> 图像坐标"""
        return (cx - self.ox) / self.zoom, (cy - self.oy) / self.zoom

    # ---------------- 载入 ----------------
    def rec(self, fn=None):
        fn = fn or self.files[self.i]
        r = self.lab.setdefault(fn, {"boxes": [], "done": False,
                                     "_seeded": True, "no_light": False})
        # ⚠️ 不再灌任何自动标注框 —— 从空白开始, 由用户自己画
        r.setdefault("boxes", [])
        r.setdefault("no_light", False)
        r["_seeded"] = True
        return r

    def load(self, i):
        self.i = max(0, min(i, len(self.files) - 1))
        fn = self.files[self.i]
        if fn not in self.icache:
            if len(self.icache) > 4:
                self.icache.clear()
            im = rd(os.path.join(IMG, fn))
            if im is None:
                im = np.zeros((100, 100, 3), np.uint8)
            self.icache[fn] = im
        self.img = self.icache[fn]
        self.fit()
        # 打开就选中第一个框 -> h/j/f 立刻可用
        self.sel = 0 if self.boxes() else None
        self.render()

    def fit(self):
        h, w = self.img.shape[:2]
        self.zoom = min(CANW / w, CANH / h) * 0.98
        self.ox = (CANW - w * self.zoom) / 2
        self.oy = (CANH - h * self.zoom) / 2

    def boxes(self):
        return self.rec()["boxes"]

    # ---------------- 渲染 ----------------
    def render(self):
        h, w = self.img.shape[:2]
        dw, dh = max(1, int(w * self.zoom)), max(1, int(h * self.zoom))
        interp = cv2.INTER_AREA if self.zoom < 1 else cv2.INTER_NEAREST
        disp = cv2.resize(self.img, (dw, dh), interpolation=interp)
        self.photo = bgr2photo(disp)
        self.can.delete('all')
        self.can.create_image(self.ox, self.oy, anchor='nw', image=self.photo)

        # 参考框 (虚线, 按类别着色; A 键切换)
        if self.show_pre:
            for b in self.pre.get(self.files[self.i], []):
                x1, y1 = self.w2c(b["x1"], b["y1"])
                x2, y2 = self.w2c(b["x2"], b["y2"])
                self.can.create_rectangle(x1, y1, x2, y2,
                                          outline=CLS_HEX.get(b["cls"], PRE_HEX),
                                          dash=(4, 3), width=1)
        # 已确认框
        for k, b in enumerate(self.boxes()):
            _, x1, y1, x2, y2 = b
            a = self.w2c(x1, y1); c = self.w2c(x2, y2)
            sel = (k == self.sel)
            self.can.create_rectangle(
                a[0], a[1], c[0], c[1],
                outline=SEL_HEX if sel else CLS_HEX.get(b[0], '#fff'),
                width=4 if sel else 2)
            if sel:                       # 选中框加个高亮角标, 一眼看到"光标"
                for (px, py, dx, dy) in ((a[0], a[1], 1, 1), (c[0], a[1], -1, 1),
                                         (a[0], c[1], 1, -1), (c[0], c[1], -1, -1)):
                    self.can.create_line(px, py, px + 14 * dx, py,
                                         fill='#ffff00', width=4)
                    self.can.create_line(px, py, px, py + 14 * dy,
                                         fill='#ffff00', width=4)
            self.can.create_text(a[0] + 3, a[1] - 8, anchor='w',
                                 text=f'{k+1}.{b[0]}' if sel else b[0],
                                 fill='#ffff00' if sel else CLS_HEX.get(b[0], '#fff'),
                                 font=('Consolas', 11, 'bold'))
        if self.rubber:
            x1, y1, x2, y2 = self.rubber
            self.can.create_rectangle(x1, y1, x2, y2,
                                      outline=CLS_HEX[self.active], width=2)
        self.upd_bar()
        self.upd_zoompane()

    def upd_bar(self):
        fn = self.files[self.i]
        r = self.rec()
        cnt = collections.Counter(b[0] for b in r["boxes"])
        done = sum(1 for f in self.files if self.lab.get(f, {}).get("done"))
        h = self.hint.get(fn)
        tag = ''
        if r.get("no_light"):
            tag = '   [本图无红绿灯 *]'
        elif h is False:
            tag = '   !! 疑似无红绿灯 (确无请按 V)'
        n = len(r["boxes"])
        if n == 0:
            selinfo = '   (无框: 拖鼠标画新框)'
        elif self.sel is None:
            selinfo = '   按 j 选第一个框'
        else:
            b = r["boxes"][self.sel]
            selinfo = ('   >>> 第%d/%d个 [%s]  j下一个 f上一个 h删除'
                       % (self.sel + 1, n, b[0]))
        self.bar.config(
            text='%4d/%4d  %-22s  虚框=%d  已标=%d  已完成=%d/%d%s%s'
                 % (self.i + 1, len(self.files), fn,
                    len(self.pre.get(fn, [])), n, done, self.n_total,
                    '  [本帧已标]' if r.get("done") else '',
                    tag),
            fg='#888' if r.get("no_light") else ('#ff0' if h is False else
                                                 ('#0f0' if r["done"] else '#0ff')))

    def upd_zoompane(self):
        """右侧: 放大当前选中框 (或用预填第一个)"""
        box = None
        if self.sel is not None and self.sel < len(self.boxes()):
            box = self.boxes()[self.sel]
        if box is None:
            self.im_zoom.config(image=''); self.im_zoom.image = None
            self.info.config(text='(没选中框)\n左键点框可放大查看')
            return
        _, x1, y1, x2, y2 = box
        H, W = self.img.shape[:2]
        px = max(20, (x2 - x1) * 0.6); py = max(20, (y2 - y1) * 0.6)
        X1 = int(max(0, x1 - px)); Y1 = int(max(0, y1 - py))
        X2 = int(min(W, x2 + px)); Y2 = int(min(H, y2 + py))
        c = self.img[Y1:Y2, X1:X2].copy()
        if c.size == 0:
            return
        s = min(320 / c.shape[1], 320 / c.shape[0])
        c = cv2.resize(c, (max(1, int(c.shape[1] * s)), max(1, int(c.shape[0] * s))),
                       interpolation=cv2.INTER_NEAREST)
        cv2.rectangle(c, (int((x1 - X1) * s), int((y1 - Y1) * s)),
                      (int((x2 - X1) * s), int((y2 - Y1) * s)),
                      CLS_BGR.get(box[0], (255, 255, 255)), 2)
        p = bgr2photo(c)
        self.im_zoom.config(image=p); self.im_zoom.image = p
        self.info.config(text='%s  %dx%d  AR=%.2f\n红像素=%d 绿像素=%d' %
                              (box[0], x2 - x1, y2 - y1,
                               (x2 - x1) / max(1, y2 - y1),
                               auto_cls(self.img, x1, y1, x2, y2)[1],
                               auto_cls(self.img, x1, y1, x2, y2)[2]))

    # ---------------- 鼠标 ----------------
    def hit(self, wx, wy):
        """返回命中的框索引(后面的优先, 因为后画的在上层)"""
        for k in range(len(self.boxes()) - 1, -1, -1):
            _, x1, y1, x2, y2 = self.boxes()[k]
            if x1 <= wx <= x2 and y1 <= wy <= y2:
                return k
        return None

    def m_down(self, e):
        wx, wy = self.c2w(e.x, e.y)
        k = self.hit(wx, wy)
        if k is not None:
            self.sel = k
            self.render()
        else:
            self.sel = None
            self.drag = ('new', e.x, e.y)
            self.rubber = (e.x, e.y, e.x, e.y)
            self.render()

    def m_move(self, e):
        if not self.drag:
            return
        kind = self.drag[0]
        if kind == 'new':
            self.rubber = (self.drag[1], self.drag[2], e.x, e.y)
            self.render()
        elif kind == 'pan':
            _, x0, y0, ox, oy = self.drag
            self.ox = ox + (e.x - x0); self.oy = oy + (e.y - y0)
            self.render()

    def m_up(self, e):
        if not self.drag:
            return
        if self.drag[0] == 'new' and self.rubber:
            x1, y1 = self.c2w(self.rubber[0], self.rubber[1])
            x2, y2 = self.c2w(self.rubber[2], self.rubber[3])
            a, b = min(x1, x2), min(y1, y2)
            c, d = max(x1, x2), max(y1, y2)
            W, H = self.img.shape[1], self.img.shape[0]
            a = max(0, a); b = max(0, b); c = min(W, c); d = min(H, d)
            if (c - a) >= 6 and (d - b) >= 6:
                self.boxes().append(["box", float(a), float(b),
                                     float(c), float(d)])
                self.sel = len(self.boxes()) - 1
                self.rec()["done"] = True
                self.rec()["no_light"] = False
                self.save()
        self.drag = None
        self.rubber = None
        self.render()

    def m_right(self, e):
        wx, wy = self.c2w(e.x, e.y)
        k = self.hit(wx, wy)
        if k is not None:
            self.boxes().pop(k)
            self.sel = None
            self.rec()["done"] = True
            self.save()
            self.render()

    def p_down(self, e):
        self.drag = ('pan', e.x, e.y, self.ox, self.oy)

    def p_move(self, e):
        self.m_move(e)

    def p_up(self, e):
        self.drag = None

    def wheel(self, e):
        """滚轮: 下=下一张图, 上=上一张图 ; Ctrl+滚轮 = 缩放"""
        if e.state & 0x0004:                     # Ctrl 按下 -> 缩放
            wx, wy = self.c2w(e.x, e.y)
            f = 1.15 if e.delta > 0 else 1 / 1.15
            nz = max(ZOOM_MIN, min(ZOOM_MAX, self.zoom * f))
            if nz == self.zoom:
                return
            self.ox = e.x - wx * nz
            self.oy = e.y - wy * nz
            self.zoom = nz
            self.render()
            return
        if e.delta < 0:
            self.next_img()
        else:
            self.prev_img()

    # ---------------- 按钮/键盘动作 ----------------
    def pick(self, cls):
        """点类别按钮: 有选中框则改它, 否则设为当前激活类别"""
        if self.sel is not None and 0 <= self.sel < len(self.boxes()):
            self.boxes()[self.sel][0] = cls
            self.rec()["done"] = True
            self.rec()["no_light"] = False
            self.save()
        self.active = cls
        self.render()

    def del_sel(self):
        """删除选中框, 删完把'光标'留在原位置 -> 可以连着按 h 连删"""
        if self.sel is not None and 0 <= self.sel < len(self.boxes()):
            k = self.sel
            self.boxes().pop(k)
            n = len(self.boxes())
            self.sel = min(k, n - 1) if n else None
            self.rec()["done"] = True
            self.save()
            self.render()

    def sel_box(self, d):
        """在当前图的标注框之间移动 (j 下一个 / f 上一个)"""
        n = len(self.boxes())
        if n == 0:
            self.sel = None
            self.render()
            return
        if self.sel is None:
            self.sel = 0 if d > 0 else n - 1
        else:
            self.sel = (self.sel + d) % n
        self.center_on_sel()
        self.render()

    def center_on_sel(self):
        """把选中的框挪到视野中心(若已可见则不动)"""
        if self.sel is None or self.sel >= len(self.boxes()):
            return
        _, x1, y1, x2, y2 = self.boxes()[self.sel]
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        sx, sy = self.w2c(cx, cy)
        if not (60 < sx < CANW - 60 and 60 < sy < CANH - 60):
            self.ox += CANW / 2 - sx
            self.oy += CANH / 2 - sy

    def toggle_no_light(self):
        """按键3: 标/取消"本图没有红绿灯"。误按可再按一次恢复原来的框。"""
        r = self.rec()
        if not r.get("no_light"):
            r["_boxes_bak"] = [list(b) for b in r["boxes"]]   # 暂存, 便于撤回
            r["no_light"] = True
            r["boxes"] = []
            self.sel = None
        else:
            r["no_light"] = False
            bak = r.pop("_boxes_bak", None)
            if bak:
                r["boxes"] = bak
            self.sel = 0 if r["boxes"] else None
        r["done"] = True
        self.save()
        self.render()

    def adopt_refs(self):
        """(旧接口) 采纳参考框但不跳转; 现在由 accept_refs 取代"""
        self.accept_refs()

    def next_img(self):
        r = self.rec()
        # 没画框也没标"无红绿灯" -> 不算标完, 下次还能回到这张
        if r.get("no_light") or r.get("boxes"):
            r["done"] = True
        else:
            r["done"] = False
        r.pop("rejected", None)
        self.save()
        self.skip_to(self.i + 1, +1)

    def reject_img(self):
        """b = 丢弃这张图: 不进训练集; 清掉框, 免得回退时误用"""
        r = self.rec()
        r["boxes"] = []
        r["no_light"] = False
        r["done"] = False
        r["rejected"] = True
        self.sel = None
        self.save()
        self.skip_to(self.i + 1, +1)

    def skip_to(self, i, step):
        i = max(0, min(i, len(self.files) - 1))
        while 0 <= i < len(self.files):
            if step > 0 and i == len(self.files) - 1:
                break
            if step < 0 and i == 0:
                break
            if not self.lab.get(self.files[i], {}).get("rejected"):
                break
            i += step
        self.load(i)

    def accept_refs(self):
        """m = 把本帧虚框(参考框)直接作为标注, 跳到下一张"""
        fn = self.files[self.i]
        r = self.rec()
        refs = self.pre.get(fn, [])
        r["boxes"] = []
        H, W = self.img.shape[:2]
        for b in refs:
            x1 = max(0.0, min(W, float(b["x1"]))); x2 = max(0.0, min(W, float(b["x2"])))
            y1 = max(0.0, min(H, float(b["y1"]))); y2 = max(0.0, min(H, float(b["y2"])))
            if x2 - x1 < 6 or y2 - y1 < 6:
                continue
            r["boxes"].append(["box", x1, y1, x2, y2])
        r["done"] = True
        r["no_light"] = False
        r.pop("rejected", None)
        self.save()
        self.skip_to(self.i + 1, +1)

    def prev_img(self):
        self.save()
        self.skip_to(self.i - 1, -1)

    def on_key(self, e):
        k = e.keysym
        if k == '4':                        # 4 = 本图没有红绿灯
            self.toggle_no_light()
            return
        if k in ('Delete', 'BackSpace'):
            self.del_sel()
        # ---- vim 风格 ----
        elif k == 'j':                      # 下一处标记
            self.sel_box(+1)
        elif k == 'f':                      # 上一处标记
            self.sel_box(-1)
        elif k == 'h':                      # 删掉当前标记
            self.del_sel()
        elif k in ('n', 'space', 'Right', 'Return'):   # 下一张(保留本帧已有的框)
            self.next_img()
        elif k in ('b', 'B'):               # 丢弃这一张: 不进训练集
            self.next_img()
        elif k in ('m', 'M'):               # 采纳虚框(=参考框)并跳到下一张
            self.accept_refs()
        elif k in ('p', 'Left'):            # 上一张图
            self.prev_img()
        elif k == 'v' or k == 'V':          # 兼容旧键
            self.toggle_no_light()
        elif k in ('y', 'Y'):               # 同 M
            self.accept_refs()
        elif k == 'a':
            self.show_pre = not self.show_pre
            self.render()
        elif k == 'z':
            self.fit(); self.render()

    # ---------------- 存盘 ----------------
    def save(self):
        tmp = LAB + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump(self.lab, f, ensure_ascii=False, indent=1)
        os.replace(tmp, LAB)

    def quit(self):
        self.save()
        self.root.destroy()


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--pool', default=None,
                    help='标注池目录 (默认 36_中国灯箱池)')
    a = ap.parse_args()
    global DST, IMG, PRE, LAB, HINT
    if a.pool:
        DST = a.pool
        IMG = os.path.join(DST, 'images_png')
        PRE = os.path.join(DST, '_prefill.json')
        LAB = os.path.join(DST, '_labels.json')
        HINT = os.path.join(DST, '_haslight.json')
    if not os.path.isfile(PRE):
        # 无参考框的池: 以 images_png 里的全部图片为准
        _img = IMG
        if os.path.isdir(_img):
            json.dump({f: [] for f in sorted(os.listdir(_img))
                       if os.path.splitext(f)[1].lower() in ('.png', '.jpg', '.jpeg')},
                      open(PRE, 'w', encoding='utf-8'), ensure_ascii=False)

    pre = json.load(open(PRE, encoding='utf-8'))
    files = sorted(pre.keys())
    hint = {}
    if os.path.exists(HINT):
        hint = json.load(open(HINT, encoding='utf-8'))
    labels = {}
    if os.path.exists(LAB):
        labels = json.load(open(LAB, encoding='utf-8'))
        print('已有标注 %d 图 (断点续标)' % len(labels))
    for f in files:
        labels.setdefault(f, {"boxes": [], "done": False, "no_light": False})
    # 跳到第一张没标过的
    start = 0
    for n, f in enumerate(files):
        if not labels[f].get("done"):
            start = n
            break
    else:
        start = 0
    done = sum(1 for f in files if labels[f].get("done"))
    nref = sum(len(v) for v in pre.values())
    print('池: %s' % DST)
    print('共 %d 帧, 已确认 %d 帧, 从第 %d 帧开始' % (len(files), done, start + 1))
    if nref:
        print('  本池含 %d 个【参考框】, 按 A 键显示虚线参考' % nref)
    else:
        print('  提示: 状态栏出现 "疑似无红绿灯" 的图, 大概率真没有灯')
    root = tk.Tk()
    root.title('红绿灯灯箱标注 v7  [%s]' % os.path.basename(DST))
    root.configure(bg='#111')
    app = App(root, files, labels, pre, hint)
    app.load(start)
    root.mainloop()


if __name__ == '__main__':
    main()
