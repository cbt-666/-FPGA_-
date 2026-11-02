#!/usr/bin/env python3
"""导航箭头 —— 全量标注 (309 帧全过一遍)

背景: 原标注只标了 193/309 帧, 其余 116 帧空着; 但现在模型以 0.7~0.97 的置信度
在这些"空帧"里检出了真箭头 —— 是当年漏标, 不是误检。要在这个视频上做到
100%, 必须把 309 帧全部过一遍。

本工具不改任何原文件:
    读  42_导航箭头/annotations.json      (人工标注, 作为初始值)
    读  tmp_ascii/nav_det_309.json        (模型提议框, 只是提示)
    写  46_导航箭头_全量/annotations_full.json

键盘:
    A            接受本帧全部模型提议框
    N            本帧无箭头 (清空并确认)
    Enter/空格   确认本帧并下一张
    1 / 2        给选中框(或整帧)标距离位数
    0            清除数字
    R / L / U    改选中框的类别
    Del          删除选中框
    P / N ...    上一张用 ','  下一张用 '.'  (N 已给"无箭头")
    F            适应窗口      Q 对准箭头      W 对准距离文字
    鼠标左键      点模型提议框=接受; 拖空白=画新框
    鼠标右键      删除框      滚轮=缩放     中键拖=平移

⚠️ 原标注里的 straight 其实是【掉头】(↩ 图标), 载入时已改名为 uturn。
"""
import os
import json
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk, ImageDraw

BASE = r'D:\ADAS_Datasets'
SRC_ANN = os.path.join(BASE, '42_导航箭头', 'annotations.json')
PROP = os.path.join(BASE, 'tmp_ascii', 'nav_det_309.json')
IMGDIR = os.path.join(BASE, '42_导航箭头', 'images')
ROOT = os.path.join(BASE, '46_导航箭头_全量')
ANN = os.path.join(ROOT, 'annotations_full.json')

CLASSES = ['right', 'left', 'uturn']
CN = {'right': '右转', 'left': '左转', 'uturn': '掉头'}
TK = {'right': '#ff40ff', 'left': '#ffc000', 'uturn': '#00b0ff'}
PL = {'right': (255, 64, 255), 'left': (255, 192, 0), 'uturn': (0, 176, 255)}
PC = (150, 150, 150)              # 模型提议框颜色
HANDLE = 4
MAG = 3.0
MAG_W, MAG_H = 320, 220


def iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    i = max(0, x2 - x1) * max(0, y2 - y1)
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - i
    return i / u if u > 0 else 0.0


def seed():
    """把人工标注 + 模型提议合并成待审队列"""
    os.makedirs(ROOT, exist_ok=True)
    src = json.load(open(SRC_ANN, encoding='utf-8'))['images']
    det = {r['file']: r for r in json.load(open(PROP, encoding='utf-8'))}
    out = []
    for r in src:
        boxes = []
        for b in r['boxes']:
            cls = 'uturn' if b['cls'] == 'straight' else b['cls']
            boxes.append({'cls': cls, 'box': list(b['box']),
                          'manual': True, 'digit': None})
        d = det.get(r['file'], {'dets': []})['dets']
        props = []
        for k, x in enumerate(d):
            cls = 'uturn' if x[0] == 'straight' else x[0]
            props.append({'cls': cls, 'box': [round(v, 1) for v in x[1:5]],
                          'conf': x[5], 'pid': k})
        # 已有标注能对上模型框的, 不算提议
        for b in boxes:
            hit = [p for p in props if iou(b['box'], p['box']) >= 0.5]
            for h in hit:
                props.remove(h)
        out.append({'file': r['file'], 'w': r['w'], 'h': r['h'],
                    'boxes': boxes, 'props': props, 'reviewed': False})
    ann = {'images': out}
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
            it.setdefault('reviewed', False)
            it.setdefault('props', [])
            for b in it['boxes']:
                b.setdefault('digit', None)

        self.idx = 0
        self.scale = 1.0
        self.ox = self.oy = 0.0
        self.pil = None
        self.tkimg = None
        self.magimg = None
        self.sel = None
        self.drag = None
        self.panning = None
        self.mag_pt = None

        root.title('导航箭头 · 全量标注 (309 帧)')
        root.geometry('1600x1000')

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

        right = ttk.Frame(root, width=380)
        right.pack(side='right', fill='y')
        right.pack_propagate(False)

        self.lbl_file = ttk.Label(right, text='', font=('Consolas', 10, 'bold'))
        self.lbl_file.pack(padx=8, pady=(10, 2), anchor='w')
        self.lbl_pos = ttk.Label(right, text='', font=('Consolas', 9))
        self.lbl_pos.pack(padx=8, anchor='w')
        self.lbl_prog = ttk.Label(right, text='', font=('Consolas', 9, 'bold'),
                                  foreground='#0a7')
        self.lbl_prog.pack(padx=8, anchor='w')

        ttk.Separator(right).pack(fill='x', pady=6)
        tk.Button(right, text='A  接受全部模型提议框', bg='#2d6a4f', fg='white',
                  font=('', 11, 'bold'), activebackground='#2d6a4f',
                  command=self.accept_all).pack(padx=8, fill='x', pady=2)
        tk.Button(right, text='N  本帧无箭头', bg='#6c757d', fg='white',
                  font=('', 11, 'bold'), activebackground='#6c757d',
                  command=self.mark_empty).pack(padx=8, fill='x', pady=2)
        tk.Button(right, text='Enter  确认并下一张', bg='#1d4ed8', fg='white',
                  font=('', 11, 'bold'), activebackground='#1d4ed8',
                  command=self.confirm_next).pack(padx=8, fill='x', pady=2)

        ttk.Separator(right).pack(fill='x', pady=6)
        ttk.Label(right, text='放大镜 (跟随鼠标 %.1fx)' % MAG,
                  font=('', 9, 'bold')).pack(padx=8, anchor='w')
        self.mag = tk.Canvas(right, width=MAG_W, height=MAG_H, bg='#000',
                             highlightthickness=1, highlightbackground='#888')
        self.mag.pack(padx=8, pady=4)

        nav = ttk.Frame(right)
        nav.pack(padx=8, pady=4, fill='x')
        ttk.Button(nav, text='◀ 上一张 (,)', command=self.prev).pack(side='left', fill='x', expand=True)
        ttk.Button(nav, text='下一张 (.) ▶', command=self.next).pack(side='left', fill='x', expand=True)

        ttk.Separator(right).pack(fill='x', pady=6)
        ttk.Label(right, text='距离位数 (选中框未选=整帧)',
                  font=('', 10, 'bold')).pack(padx=8, anchor='w')
        df = ttk.Frame(right)
        df.pack(padx=8, pady=4, fill='x')
        for d, txt, col in (('3', '3 位  ≥100米', '#c0392b'),
                            ('2', '2 位  <100米', '#d35400')):
            tk.Button(df, text=txt, bg=col, fg='white', font=('', 11, 'bold'),
                      activebackground=col, command=lambda dd=d: self.apply_digit(dd)
                      ).pack(fill='x', pady=2)
        ttk.Button(right, text='清除数字 (0)', command=lambda: self.apply_digit(None)
                   ).pack(padx=8, fill='x', pady=2)

        ttk.Separator(right).pack(fill='x', pady=6)
        ttk.Label(right, text='类别 (选中框后 R/L/U)', font=('', 9, 'bold')
                  ).pack(padx=8, anchor='w')
        cf = ttk.Frame(right)
        cf.pack(padx=8, pady=4, fill='x')
        for c in CLASSES:
            tk.Button(cf, text=CN[c], bg=TK[c], fg='white', activebackground=TK[c],
                      command=lambda cc=c: self.set_class(cc)).pack(fill='x', pady=1)

        ttk.Separator(right).pack(fill='x', pady=6)
        ttk.Label(right, text='本帧框 / 待接受提议', font=('', 9, 'bold')
                  ).pack(padx=8, anchor='w')
        self.lst = tk.Listbox(right, height=6, font=('Consolas', 10), exportselection=False)
        self.lst.pack(padx=8, pady=4, fill='x')
        self.lst.bind('<<ListboxSelect>>', self.on_list)

        vf = ttk.Frame(right)
        vf.pack(padx=8, pady=4, fill='x')
        ttk.Button(vf, text='适应 (F)', command=self.fit).pack(side='left', fill='x', expand=True)
        ttk.Button(vf, text='箭头 (Q)', command=self.goto_arrow).pack(side='left', fill='x', expand=True)
        ttk.Button(vf, text='距离 (W)', command=self.goto_dist).pack(side='left', fill='x', expand=True)

        self.lbl_stat = ttk.Label(right, text='', font=('Consolas', 9),
                                  foreground='#555', justify='left')
        self.lbl_stat.pack(padx=8, pady=8, anchor='w')

        for d in ('2', '3'):
            root.bind(d, lambda e, dd=d: self.apply_digit(dd))
        root.bind('0', lambda e: self.apply_digit(None))
        root.bind('r', lambda e: self.set_class('right'))
        root.bind('l', lambda e: self.set_class('left'))
        root.bind('u', lambda e: self.set_class('uturn'))
        root.bind('a', lambda e: self.accept_all())
        root.bind('n', lambda e: self.mark_empty())
        root.bind('<Return>', lambda e: self.confirm_next())
        root.bind('<space>', lambda e: self.next())
        root.bind(',', lambda e: self.prev())
        root.bind('.', lambda e: self.next())
        root.bind('<Delete>', lambda e: self.delete_sel())
        root.bind('<BackSpace>', lambda e: self.delete_sel())
        root.bind('f', lambda e: self.fit())
        root.bind('q', lambda e: self.goto_arrow())
        root.bind('w', lambda e: self.goto_dist())
        root.bind('s', lambda e: self.save(True))
        root.bind('<plus>', lambda e: self.zoom_at(1.25, None))
        root.bind('<equal>', lambda e: self.zoom_at(1.25, None))
        root.bind('<minus>', lambda e: self.zoom_at(0.8, None))
        root.protocol('WM_DELETE_WINDOW', self.on_close)

        self.jump_first_pending()

    # ---------------- 队列 ----------------
    def pending(self):
        return [i for i, it in enumerate(self.imgs) if not it['reviewed']]

    def jump_first_pending(self):
        p = self.pending()
        if p:
            self.load(p[0])
        else:
            self.load(0)

    def next(self):
        p = [i for i in self.pending() if i > self.idx]
        self.load(p[0] if p else min(self.idx + 1, len(self.imgs) - 1))

    def prev(self):
        self.load(max(0, self.idx - 1))

    def confirm_next(self):
        self.imgs[self.idx]['reviewed'] = True
        self.save()
        self.refresh_stat()
        p = [i for i in self.pending() if i > self.idx]
        if p:
            self.load(p[0])
        else:
            left = self.pending()
            if left:
                messagebox.showinfo('回到起点', '后面没有待审帧了, 跳到第一张未审: %s'
                                    % self.imgs[left[0]]['file'])
                self.load(left[0])
            else:
                messagebox.showinfo('完成', '全部 %d 帧已审完。' % len(self.imgs))

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
        flag = '' if it['reviewed'] else '   ★待审'
        self.lbl_file.config(text='[%d/%d] %s%s'
                             % (self.idx + 1, len(self.imgs), it['file'], flag))
        self.fit()
        self.refresh_list()
        self.refresh_stat()

    def fit(self):
        cw, ch = self.canvas.winfo_width(), self.canvas.winfo_height()
        if cw < 50 or ch < 50:
            cw, ch = 1200, 960
        if not self.pil:
            return
        w, h = self.pil.size
        self.scale = min(cw / w, ch / h) * 0.98
        self.ox = (cw - w * self.scale) / 2
        self.oy = (ch - h * self.scale) / 2
        self.redraw()

    def focus(self, ix, iy, scale):
        cw, ch = self.canvas.winfo_width(), self.canvas.winfo_height()
        if cw < 50 or ch < 50:
            cw, ch = 1200, 960
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
            p = self.imgs[self.idx]['props']
            if p:
                x1, y1, x2, y2 = p[0]['box']
                self.focus((x1 + x2) / 2, (y1 + y2) / 2, 1.6)
            else:
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
        for p in it['props']:
            x1, y1, x2, y2 = p['box']
            bx = [x1 * self.scale, y1 * self.scale, x2 * self.scale, y2 * self.scale]
            d.rectangle(bx, outline=PC, width=1)
            d.text((bx[0] + 2, max(0, bx[1] - 11)),
                   'P %s %.2f' % (CN[p['cls']], p['conf']), fill=PC)
        for k, b in enumerate(it['boxes']):
            x1, y1, x2, y2 = b['box']
            bx = [x1 * self.scale, y1 * self.scale, x2 * self.scale, y2 * self.scale]
            col = PL[b['cls']]
            d.rectangle(bx, outline=col, width=3 if k == self.sel else 2)
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
        self.lbl_pos.config(text='缩放 %.2fx  框 %d  提议 %d  已补 %d'
                            % (self.scale, len(it['boxes']), len(it['props']),
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
            self.lst.insert('end', '✓ %d %s%s' % (k, CN[b['cls']], b.get('digit') or '?'))
        for p in it['props']:
            self.lst.insert('end', 'P 提议 %s %.2f' % (CN[p['cls']], p['conf']))
        if self.sel is not None and self.sel < len(it['boxes']):
            self.lst.selection_clear(0, 'end')
            self.lst.selection_set(self.sel)

    def refresh_stat(self):
        n = len(self.imgs)
        done = sum(1 for it in self.imgs if it['reviewed'])
        boxes = [b for it in self.imgs for b in it['boxes']]
        digs = sum(1 for b in boxes if b.get('digit'))
        self.lbl_prog.config(text='已审 %d/%d   (剩 %d)' % (done, n, n - done))
        self.lbl_stat.config(text='总框 %d  已补位数 %d\n3位 %d  |  2位 %d  |  未补 %d'
                             % (len(boxes), digs,
                                sum(1 for b in boxes if b.get('digit') == '3'),
                                sum(1 for b in boxes if b.get('digit') == '2'),
                                len(boxes) - digs))

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

    def hit_prop(self, ix, iy):
        it = self.imgs[self.idx]
        for p in it['props']:
            x1, y1, x2, y2 = p['box']
            if x1 <= ix <= x2 and y1 <= iy <= y2:
                return p
        return None

    def accept(self, p):
        it = self.imgs[self.idx]
        it['boxes'].append({'cls': p['cls'], 'box': list(p['box']),
                            'manual': False, 'digit': None, 'conf': p['conf']})
        it['props'].remove(p)
        self.sel = len(it['boxes']) - 1
        self.save()
        self.refresh_list()
        self.redraw()

    def accept_all(self):
        it = self.imgs[self.idx]
        if not it['props']:
            return
        for p in list(it['props']):
            it['boxes'].append({'cls': p['cls'], 'box': list(p['box']),
                                'manual': False, 'digit': None, 'conf': p['conf']})
        it['props'].clear()
        self.sel = len(it['boxes']) - 1 if it['boxes'] else None
        self.save()
        self.refresh_list()
        self.redraw()

    def mark_empty(self):
        it = self.imgs[self.idx]
        it['boxes'] = []
        it['props'] = []
        it['reviewed'] = True
        self.sel = None
        self.save()
        self.refresh_list()
        self.redraw()
        self.refresh_stat()
        self.confirm_next()

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
            self.redraw()
            return
        p = self.hit_prop(ix, iy)
        if p is not None:
            self.accept(p)
            return
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
        cw, ch = self.canvas.winfo_width(), self.canvas.winfo_height()
        if cw < 50 or ch < 50:
            cw, ch = 1200, 960
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
            k = int(s[0])
            if k < len(self.imgs[self.idx]['boxes']):
                self.sel = k
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
        targets = ([self.sel] if self.sel is not None and self.sel < len(it['boxes'])
                   else range(len(it['boxes'])))
        for k in targets:
            it['boxes'][k]['digit'] = d
        self.save()
        self.refresh_list()
        self.redraw()
        self.refresh_stat()

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
            os.makedirs(ROOT, exist_ok=True)
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
