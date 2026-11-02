#!/usr/bin/env python3
"""导航箭头 补距离位数 —— 在上一版 193 个框上给每个框加 1/2/3

数字 = 导航卡上剩余距离的【位数】, 只有两档:
    3 位  >= 100 米        (如 200米 / 893米)
    2 位  < 100 米         (如 50米)

读图: 旧标注 D:\\ADAS_Datasets\\42_导航箭头\\annotations.json
写盘: D:\\ADAS_Datasets\\42_导航箭头_距离\\annotations_digit.json
图片: 仍用旧目录 images/, 不改动任何原文件

键盘:
    1 / 2 / 3    给选中框设 1/2/3 位 (未选中则给本图所有框设)
                 某图所有框都有数字后自动跳到下一张
    0            清除选中框的数字
    R / L / U    改类别: 右转 / 左转 / 掉头 (选中框)
    N / P        下一张 / 上一张 (有框的帧)
    Q            视图对准箭头 (1.6x)
    W            视图对准箭头右边的距离文字 (2.2x)
    Z            缩放到适应
    鼠标左键     选中框 (拖空白处可新增框)
    鼠标右键     删除框
    滚轮         缩放      中键拖   平移
"""
import os
import sys
import json
import glob
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk, ImageDraw

OLD = r'D:\ADAS_Datasets\42_导航箭头'
ROOT = r'D:\ADAS_Datasets\42_导航箭头_距离'
ANN = os.path.join(ROOT, 'annotations_digit.json')
IMGDIR = os.path.join(OLD, 'images')

CLASSES = ['right', 'left', 'uturn']
CN = {'right': '右转', 'left': '左转', 'uturn': '掉头'}
TK = {'right': '#ff40ff', 'left': '#ffc000', 'uturn': '#00b0ff'}
PL = {'right': (255, 64, 255), 'left': (255, 192, 0), 'uturn': (0, 176, 255)}
HANDLE = 4
MAG = 3.0
MAG_W, MAG_H = 320, 210


def seed():
    os.makedirs(ROOT, exist_ok=True)
    src = os.path.join(OLD, 'annotations.json')
    ann = json.load(open(src, encoding='utf-8'))
    for it in ann['images']:
        it['done'] = bool(it['boxes'])
        for b in it.get('boxes', []):
            if b.get('cls') == 'straight':
                b['cls'] = 'uturn'
            b['digit'] = None
    with open(ANN, 'w', encoding='utf-8') as f:
        json.dump(ann, f, ensure_ascii=False, indent=1)
    return ann


class App:
    def __init__(self, root):
        self.root = root
        if os.path.exists(ANN):
            self.ann = json.load(open(ANN, encoding='utf-8'))
        else:
            self.ann = seed()
        self.imgs = self.ann['images']
        for it in self.imgs:
            for b in it.get('boxes', []):
                b.setdefault('digit', None)

        self.idx = 0
        self.scale = 1.0
        self.ox = 0.0
        self.oy = 0.0
        self.pil = None
        self.tkimg = None
        self.magimg = None
        self.sel = None
        self.drag = None
        self.panning = None
        self.mag_pt = None
        self.skip_done = tk.BooleanVar(value=True)

        root.title('导航箭头 · 补距离位数 (3=百米 / 2=十米 / 1=个米)')
        root.geometry('1560x980')

        left = ttk.Frame(root)
        left.pack(side='left', fill='both', expand=True)
        self.canvas = tk.Canvas(left, bg='#202020', highlightthickness=0)
        self.canvas.pack(fill='both', expand=True)
        self.canvas.bind('<Button-1>', self.on_press)
        self.canvas.bind('<B1-Motion>', self.on_drag)
        self.canvas.bind('<ButtonRelease-1>', self.on_release)
        self.canvas.bind('<Button-3>', self.on_right)
        self.canvas.bind('<Button-2>', self.on_pan_start)
        self.canvas.bind('<B2-Motion>', self.on_pan)
        self.canvas.bind('<ButtonRelease-2>', self.on_pan_end)
        self.canvas.bind('<MouseWheel>', self.on_wheel)
        self.canvas.bind('<Motion>', self.on_motion)
        self.canvas.bind('<Configure>', lambda e: self.redraw())

        right = ttk.Frame(root, width=360)
        right.pack(side='right', fill='y')
        right.pack_propagate(False)

        self.lbl_file = ttk.Label(right, text='', font=('Consolas', 10, 'bold'))
        self.lbl_file.pack(padx=8, pady=(10, 2), anchor='w')
        self.lbl_pos = ttk.Label(right, text='', font=('Consolas', 9))
        self.lbl_pos.pack(padx=8, anchor='w')

        ttk.Separator(right).pack(fill='x', pady=6)
        ttk.Label(right, text='放大镜 (跟随鼠标 %.1fx) —— 用它读"886米"' % MAG,
                  font=('', 9, 'bold')).pack(padx=8, anchor='w')
        self.mag = tk.Canvas(right, width=MAG_W, height=MAG_H, bg='#000',
                             highlightthickness=1, highlightbackground='#888')
        self.mag.pack(padx=8, pady=4)

        nav = ttk.Frame(right)
        nav.pack(padx=8, pady=6, fill='x')
        ttk.Button(nav, text='◀ 上一张 (P)', command=self.prev).pack(side='left', fill='x', expand=True)
        ttk.Button(nav, text='下一张 (N) ▶', command=self.next).pack(side='left', fill='x', expand=True)
        ttk.Checkbutton(right, text='跳过已补完的帧', variable=self.skip_done).pack(padx=8, anchor='w')

        ttk.Separator(right).pack(fill='x', pady=6)
        ttk.Label(right, text='距离位数  (目标框未选中 = 整帧生效)',
                  font=('', 10, 'bold')).pack(padx=8, anchor='w')
        df = ttk.Frame(right)
        df.pack(padx=8, pady=4, fill='x')
        digits = [('3', '3 位  ≥100米', '#c0392b'),
                  ('2', '2 位  <100米', '#d35400')]
        for d, txt, col in digits:
            tk.Button(df, text=txt, bg=col, fg='white', font=('', 11, 'bold'),
                      activebackground=col, command=lambda dd=d: self.apply_digit(dd)
                      ).pack(fill='x', pady=2)
        ttk.Button(right, text='清除数字 (0)', command=lambda: self.apply_digit(None)
                   ).pack(padx=8, fill='x', pady=2)

        ttk.Separator(right).pack(fill='x', pady=6)
        ttk.Label(right, text='类别  (选中框后按 R/L/U)',
                  font=('', 9, 'bold')).pack(padx=8, anchor='w')
        cf = ttk.Frame(right)
        cf.pack(padx=8, pady=4, fill='x')
        for c in CLASSES:
            tk.Button(cf, text=CN[c], bg=TK[c], fg='white',
                      activebackground=TK[c], command=lambda cc=c: self.set_class(cc)
                      ).pack(fill='x', pady=1)

        ttk.Separator(right).pack(fill='x', pady=6)
        ttk.Label(right, text='本图框  (点选, Del 删除)', font=('', 9, 'bold')).pack(padx=8, anchor='w')
        self.lst = tk.Listbox(right, height=5, font=('Consolas', 10),
                              selectmode='browse', exportselection=False)
        self.lst.pack(padx=8, pady=4, fill='x')
        self.lst.bind('<<ListboxSelect>>', self.on_list)

        ttk.Separator(right).pack(fill='x', pady=4)
        ttk.Button(right, text='视图对准箭头 (Q)', command=self.goto_arrow).pack(padx=8, fill='x', pady=2)
        ttk.Button(right, text='对准距离文字 (W)', command=self.goto_dist).pack(padx=8, fill='x', pady=2)
        ttk.Button(right, text='适应窗口 (Z)', command=self.fit).pack(padx=8, fill='x', pady=2)

        self.lbl_stat = ttk.Label(right, text='', font=('Consolas', 9), foreground='#555',
                                  justify='left')
        self.lbl_stat.pack(padx=8, pady=8, anchor='w')

        for d in ('2', '3'):
            root.bind(d, lambda e, dd=d: self.apply_digit(dd))
        root.bind('0', lambda e: self.apply_digit(None))
        root.bind('r', lambda e: self.set_class('right'))
        root.bind('l', lambda e: self.set_class('left'))
        root.bind('u', lambda e: self.set_class('uturn'))
        root.bind('<Delete>', lambda e: self.delete_sel())
        root.bind('<BackSpace>', lambda e: self.delete_sel())
        root.bind('n', lambda e: self.next())
        root.bind('p', lambda e: self.prev())
        root.bind('z', lambda e: self.fit())
        root.bind('q', lambda e: self.goto_arrow())
        root.bind('w', lambda e: self.goto_dist())
        root.bind('s', lambda e: self.save(True))
        root.bind('<plus>', lambda e: self.zoom_at(1.25, None))
        root.bind('<equal>', lambda e: self.zoom_at(1.25, None))
        root.bind('<minus>', lambda e: self.zoom_at(0.8, None))
        root.protocol('WM_DELETE_WINDOW', self.on_close)

        self.load(0)

    # ---------------- 帧筛选 ----------------
    def labeled(self):
        return [i for i, it in enumerate(self.imgs) if it['boxes']]

    def complete(self, it):
        return bool(it['boxes']) and all(b.get('digit') for b in it['boxes'])

    def step(self, d):
        order = self.labeled()
        if not order:
            return
        pos = order.index(self.idx) if self.idx in order else -1
        if pos < 0:
            pos = 0 if d > 0 else len(order) - 1
            self.load(order[pos])
            return
        k = pos + d
        while 0 <= k < len(order):
            if not (self.skip_done.get() and self.complete(self.imgs[order[k]])):
                self.load(order[k])
                return
            k += d
        # 到头了: 停在原地, 只刷新
        self.refresh_list()
        self.redraw()

    def next(self):
        self.step(1)

    def prev(self):
        self.step(-1)

    # ---------------- 坐标 ----------------
    def to_image(self, cx, cy):
        return (cx - self.ox) / self.scale, (cy - self.oy) / self.scale

    # ---------------- 载入 / 绘制 ----------------
    def load(self, i):
        self.save()
        self.idx = max(0, min(i, len(self.imgs) - 1))
        it = self.imgs[self.idx]
        p = os.path.join(IMGDIR, it['file'])
        if not os.path.exists(p):
            messagebox.showerror('缺图', p)
            return
        img = Image.open(p).convert('RGB')
        it['w'], it['h'] = img.size
        self.pil = img
        self.sel = 0 if it['boxes'] else None
        self.drag = None
        self.lbl_file.config(text='[%d/%d]  %s' % (self.idx + 1, len(self.imgs), it['file']))
        self.fit()
        self.refresh_list()

    def fit(self):
        cw = self.canvas.winfo_width()
        ch = self.canvas.winfo_height()
        if cw < 50 or ch < 50:
            cw, ch = 1150, 900
        if not self.pil:
            return
        w, h = self.pil.size
        self.scale = min(cw / w, ch / h) * 0.98
        self.ox = (cw - w * self.scale) / 2
        self.oy = (ch - h * self.scale) / 2
        self.redraw()

    def focus(self, ix, iy, scale):
        cw = self.canvas.winfo_width()
        ch = self.canvas.winfo_height()
        if cw < 50 or ch < 50:
            cw, ch = 1150, 900
        self.scale = scale
        self.ox = cw / 2 - ix * self.scale
        self.oy = ch / 2 - iy * self.scale
        self.redraw()

    def cur_box(self):
        it = self.imgs[self.idx]
        if self.sel is not None and self.sel < len(it['boxes']):
            return it['boxes'][self.sel]
        return it['boxes'][0] if it['boxes'] else None

    def goto_arrow(self):
        b = self.cur_box()
        if not b:
            self.focus(205, 660, 1.6)
            return
        x1, y1, x2, y2 = b['box']
        self.focus((x1 + x2) / 2, (y1 + y2) / 2, 1.6)

    def goto_dist(self):
        b = self.cur_box()
        if not b:
            self.focus(420, 660, 2.2)
            return
        x1, y1, x2, y2 = b['box']
        self.focus(x2 + 120, (y1 + y2) / 2, 2.2)

    def redraw(self):
        if not self.pil:
            return
        it = self.imgs[self.idx]
        disp = self.pil.resize((max(1, int(it['w'] * self.scale)),
                                max(1, int(it['h'] * self.scale))), Image.LANCZOS)
        d = ImageDraw.Draw(disp)
        for k, b in enumerate(it['boxes']):
            x1, y1, x2, y2 = b['box']
            bx = [x1 * self.scale, y1 * self.scale, x2 * self.scale, y2 * self.scale]
            col = PL[b['cls']]
            lw = 3 if k == self.sel else 2
            d.rectangle(bx, outline=col, width=lw)
            dig = b.get('digit')
            tag = '%s%s' % (CN[b['cls']], dig if dig else '?')
            tw = 13 * len(tag) + 6
            ty = max(0, bx[1] - 14)
            d.rectangle([bx[0], ty, bx[0] + tw, ty + 14], fill=col)
            d.text((bx[0] + 3, ty + 1), tag, fill=(0, 0, 0))
        if self.drag:
            x1, y1, x2, y2 = self.drag
            d.rectangle([x1 * self.scale, y1 * self.scale,
                         x2 * self.scale, y2 * self.scale],
                        outline=(255, 255, 0), width=2)

        self.tkimg = ImageTk.PhotoImage(disp)
        self.canvas.delete('all')
        self.canvas.create_image(self.ox, self.oy, anchor='nw', image=self.tkimg)
        self.lbl_pos.config(text='缩放 %.2fx   框数 %d   已补 %d'
                            % (self.scale, len(it['boxes']),
                               sum(1 for b in it['boxes'] if b.get('digit'))))
        self.draw_mag()

    def draw_mag(self):
        if self.mag_pt is None or not self.pil:
            self.mag.delete('all')
            return
        ix, iy = self.mag_pt
        w, h = self.pil.size
        hw, hh = MAG_W / (2 * MAG), MAG_H / (2 * MAG)
        x1 = max(0, int(ix - hw)); y1 = max(0, int(iy - hh))
        x2 = min(w, int(ix + hw)); y2 = min(h, int(iy + hh))
        if x2 - x1 < 4 or y2 - y1 < 4:
            return
        crop = self.pil.crop((x1, y1, x2, y2))
        crop = crop.resize((int(crop.width * MAG), int(crop.height * MAG)), Image.LANCZOS)
        d = ImageDraw.Draw(crop)
        for b in self.imgs[self.idx]['boxes']:
            bx1, by1, bx2, by2 = b['box']
            d.rectangle([(bx1 - x1) * MAG, (by1 - y1) * MAG,
                         (bx2 - x1) * MAG, (by2 - y1) * MAG],
                        outline=PL[b['cls']], width=2)
        cx, cy = (ix - x1) * MAG, (iy - y1) * MAG
        d.line([cx - 8, cy, cx + 8, cy], fill=(255, 0, 0), width=1)
        d.line([cx, cy - 8, cx, cy + 8], fill=(255, 0, 0), width=1)
        self.magimg = ImageTk.PhotoImage(crop)
        self.mag.delete('all')
        self.mag.create_image(0, 0, anchor='nw', image=self.magimg)

    def refresh_list(self):
        self.lst.delete(0, 'end')
        it = self.imgs[self.idx]
        for k, b in enumerate(it['boxes']):
            self.lst.insert('end', '%d %s%s' % (k, CN[b['cls']], b.get('digit') or '?'))
        if self.sel is not None and self.sel < self.lst.size():
            self.lst.selection_clear(0, 'end')
            self.lst.selection_set(self.sel)
        boxes = [b for i in self.imgs for b in i['boxes']]
        done = sum(1 for b in boxes if b.get('digit'))
        cnt = {d: sum(1 for b in boxes if b.get('digit') == d) for d in '23'}
        self.lbl_stat.config(
            text='框 %d/%d 已补 (%.0f%%)\n3位 %d | 2位 %d | 未补 %d'
                 % (done, len(boxes), 100.0 * done / max(len(boxes), 1),
                    cnt['3'], cnt['2'], len(boxes) - done))

    # ---------------- 交互 ----------------
    def hit(self, ix, iy):
        it = self.imgs[self.idx]
        best, bk = None, -1
        for k in range(len(it['boxes']) - 1, -1, -1):
            x1, y1, x2, y2 = it['boxes'][k]['box']
            if x1 - HANDLE <= ix <= x2 + HANDLE and y1 - HANDLE <= iy <= y2 + HANDLE:
                area = (x2 - x1) * (y2 - y1)
                if best is None or area < best:
                    best, bk = area, k
        return bk

    def on_motion(self, e):
        self.mag_pt = self.to_image(e.x, e.y)
        self.draw_mag()

    def on_press(self, e):
        ix, iy = self.to_image(e.x, e.y)
        k = self.hit(ix, iy)
        if k >= 0:
            self.sel = k
            self.lst.selection_clear(0, 'end')
            self.lst.selection_set(k)
        else:
            self.sel = None
            self.drag = [ix, iy, ix, iy]
        self.redraw()

    def on_drag(self, e):
        if self.drag:
            ix, iy = self.to_image(e.x, e.y)
            self.drag[2], self.drag[3] = ix, iy
            self.redraw()

    def on_release(self, e):
        if not self.drag:
            return
        x1, y1, x2, y2 = self.drag
        self.drag = None
        x1, x2 = sorted((x1, x2))
        y1, y2 = sorted((y1, y2))
        it = self.imgs[self.idx]
        x1 = max(0, min(x1, it['w'] - 1)); x2 = max(1, min(x2, it['w']))
        y1 = max(0, min(y1, it['h'] - 1)); y2 = max(1, min(y2, it['h']))
        if (x2 - x1) < 4 or (y2 - y1) < 4:
            self.redraw()
            return
        last = it['boxes'][-1] if it['boxes'] else None
        it['boxes'].append({'cls': last['cls'] if last else 'right',
                            'box': [round(v, 1) for v in (x1, y1, x2, y2)],
                            'manual': True, 'digit': None})
        self.sel = len(it['boxes']) - 1
        self.save()
        self.refresh_list()
        self.redraw()

    def on_right(self, e):
        ix, iy = self.to_image(e.x, e.y)
        k = self.hit(ix, iy)
        if k >= 0:
            del self.imgs[self.idx]['boxes'][k]
            self.sel = None
            self.save()
            self.refresh_list()
            self.redraw()

    def on_pan_start(self, e):
        self.panning = (e.x, e.y, self.ox, self.oy)

    def on_pan(self, e):
        if self.panning:
            sx, sy, ox, oy = self.panning
            self.ox = ox + (e.x - sx)
            self.oy = oy + (e.y - sy)
            self.redraw()

    def on_pan_end(self, e):
        self.panning = None

    def on_wheel(self, e):
        self.zoom_at(1.15 if e.delta > 0 else 1 / 1.15, (e.x, e.y))

    def zoom_at(self, f, pt):
        cw = self.canvas.winfo_width()
        ch = self.canvas.winfo_height()
        if cw < 50 or ch < 50:
            cw, ch = 1150, 900
        if pt is None:
            pt = (cw / 2, ch / 2)
        ix, iy = self.to_image(*pt)
        self.scale = max(0.05, min(20.0, self.scale * f))
        self.ox = pt[0] - ix * self.scale
        self.oy = pt[1] - iy * self.scale
        self.redraw()

    def on_list(self, e):
        s = self.lst.curselection()
        if s:
            self.sel = int(s[0])
            self.redraw()

    # ---------------- 编辑 ----------------
    def set_class(self, c):
        it = self.imgs[self.idx]
        if self.sel is None or self.sel >= len(it['boxes']):
            return
        it['boxes'][self.sel]['cls'] = c
        self.save()
        self.refresh_list()
        self.redraw()

    def apply_digit(self, d):
        it = self.imgs[self.idx]
        if not it['boxes']:
            return
        if self.sel is not None and self.sel < len(it['boxes']):
            targets = [self.sel]
        else:
            targets = range(len(it['boxes']))
        for k in targets:
            it['boxes'][k]['digit'] = d
        self.save()
        self.refresh_list()
        self.redraw()
        if d and self.complete(it):
            self.root.after(140, self.next)

    def delete_sel(self):
        it = self.imgs[self.idx]
        if self.sel is None or self.sel >= len(it['boxes']):
            return
        del it['boxes'][self.sel]
        self.sel = None
        self.save()
        self.refresh_list()
        self.redraw()

    # ---------------- 存盘 ----------------
    def save(self, verbose=False):
        try:
            tmp = ANN + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as f:
                json.dump(self.ann, f, ensure_ascii=False, indent=1)
            os.replace(tmp, ANN)
            if verbose:
                print('已存盘', ANN)
        except Exception as e:
            print('存盘失败:', e)

    def on_close(self):
        self.save()
        self.root.destroy()


if __name__ == '__main__':
    root = tk.Tk()
    App(root)
    root.mainloop()
