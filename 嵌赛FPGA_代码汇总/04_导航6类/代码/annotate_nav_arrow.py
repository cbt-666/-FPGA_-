#!/usr/bin/env python3
"""导航箭头(转向标) 检测标注工具 —— 3 类: 直行/左转/右转

数据: D:\\ADAS_Datasets\\42_导航箭头\\images  (官方 导航.mp4 抽帧, 每0.3s一帧)
标注: 屏幕左上"转向标"那个箭头图标(如 886米 左边的箭头)
      框住箭头本身即可; 没箭头(360视图帧)按 X 跳过

键盘:
    1/2/3      画完框后指定类别: 1=直行  2=左转  3=右转
               (也可以先框, 再点右侧按钮改类)
    Del/Back   删除选中框
    N / P      下一张 / 上一张
    Z          缩放到适应窗口
    Q          快速跳到"导航卡"区域(放大1.3x) —— 箭头就在那
    X          本图无箭头(跳过)
    +/-        放大 / 缩小
    S          手动存盘 (每次改动都自动存)
    鼠标左键拖   画新框
    鼠标右键点框 删除该框
    鼠标中键拖   平移
    鼠标滚轮     以光标为中心缩放
    鼠标移动     右上"放大镜"实时看细节
"""
import os
import sys
import json
import glob
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk, ImageDraw

ROOT = sys.argv[1] if len(sys.argv) > 1 else r'D:\ADAS_Datasets\42_导航箭头'
ANN = os.path.join(ROOT, 'annotations.json')
IMGDIR = os.path.join(ROOT, 'images')

CLASSES = ['straight', 'left', 'right']
CN = {'straight': '直行', 'left': '左转', 'right': '右转'}
TK = {'straight': '#00b0ff', 'left': '#ffc000', 'right': '#ff40ff'}
PL = {'straight': (0, 176, 255), 'left': (255, 192, 0), 'right': (255, 64, 255)}
HANDLE = 4          # 命中判定的容差(像素, 图像坐标)
MAG = 3.0           # 放大镜倍数
MAG_W, MAG_H = 300, 200


def build_index():
    fs = sorted(glob.glob(os.path.join(IMGDIR, '*.jpg')))
    if not fs:
        messagebox.showerror('没找到图', IMGDIR)
        sys.exit(1)
    return {'images': [{'file': os.path.basename(p), 'w': 1920, 'h': 1080,
                        'boxes': [], 'done': False} for p in fs]}


class App:
    def __init__(self, root):
        self.root = root
        if os.path.exists(ANN):
            self.ann = json.load(open(ANN, encoding='utf-8'))
        else:
            self.ann = build_index()
        self.imgs = self.ann['images']
        for it in self.imgs:                      # 容错: 未知类别 -> straight
            for b in it.get('boxes', []):
                if b.get('cls') not in CLASSES:
                    b['cls'] = CLASSES[0]

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

        root.title('导航箭头 标注工具 —— 直行/左转/右转')
        root.geometry('1560x980')

        # ---- 左侧画布 ----
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

        # ---- 右侧面板 ----
        right = ttk.Frame(root, width=340)
        right.pack(side='right', fill='y')
        right.pack_propagate(False)

        self.lbl_file = ttk.Label(right, text='', font=('Consolas', 10, 'bold'))
        self.lbl_file.pack(padx=8, pady=(10, 2), anchor='w')
        self.lbl_flag = ttk.Label(right, text='', font=('', 9, 'bold'))
        self.lbl_flag.pack(padx=8, anchor='w')
        self.lbl_pos = ttk.Label(right, text='', font=('Consolas', 9))
        self.lbl_pos.pack(padx=8, anchor='w')

        # 放大镜
        ttk.Separator(right).pack(fill='x', pady=6)
        ttk.Label(right, text='放大镜 (鼠标移动跟随, %.1fx)' % MAG,
                  font=('', 9, 'bold')).pack(padx=8, anchor='w')
        self.mag = tk.Canvas(right, width=MAG_W, height=MAG_H, bg='#000',
                             highlightthickness=1, highlightbackground='#888')
        self.mag.pack(padx=8, pady=4)

        nav = ttk.Frame(right)
        nav.pack(padx=8, pady=6, fill='x')
        ttk.Button(nav, text='◀ 上一张 (P)', command=self.prev).pack(side='left', fill='x', expand=True)
        ttk.Button(nav, text='下一张 (N) ▶', command=self.next).pack(side='left', fill='x', expand=True)

        ttk.Separator(right).pack(fill='x', pady=4)
        ttk.Label(right, text='框列表  (点选, Del 删除)', font=('', 9, 'bold')).pack(padx=8, anchor='w')
        self.lst = tk.Listbox(right, height=8, font=('Consolas', 10),
                              selectmode='browse', exportselection=False)
        self.lst.pack(padx=8, pady=4, fill='both', expand=True)
        self.lst.bind('<<ListboxSelect>>', self.on_list)

        ttk.Separator(right).pack(fill='x', pady=4)
        ttk.Label(right, text='类别  (选中框后按 1/2/3)', font=('', 9, 'bold')).pack(padx=8, anchor='w')
        cf = ttk.Frame(right)
        cf.pack(padx=8, pady=4, fill='x')
        for i, c in enumerate(CLASSES):
            tk.Button(cf, text='%d  %s' % (i + 1, CN[c]), bg=TK[c], fg='white',
                      activebackground=TK[c], relief='raised',
                      command=lambda cc=c: self.set_class(cc)).pack(fill='x', pady=1)

        ttk.Separator(right).pack(fill='x', pady=4)
        ttk.Button(right, text='删除选中框 (Del)', command=self.delete_sel).pack(padx=8, fill='x', pady=2)
        ttk.Button(right, text='本图无箭头 → 下一张 (X)', command=self.mark_empty).pack(padx=8, fill='x', pady=2)
        ttk.Button(right, text='跳到导航卡 (Q)', command=self.goto_card).pack(padx=8, fill='x', pady=2)
        ttk.Button(right, text='缩放到适应 (Z)', command=self.fit).pack(padx=8, fill='x', pady=2)

        self.lbl_stat = ttk.Label(right, text='', font=('', 9), foreground='#666',
                                  wraplength=320, justify='left')
        self.lbl_stat.pack(padx=8, pady=8, anchor='w')

        for i, c in enumerate(CLASSES):
            root.bind(str(i + 1), lambda e, cc=c: self.set_class(cc))
        root.bind('<Delete>', lambda e: self.delete_sel())
        root.bind('<BackSpace>', lambda e: self.delete_sel())
        root.bind('n', lambda e: self.next())
        root.bind('p', lambda e: self.prev())
        root.bind('z', lambda e: self.fit())
        root.bind('q', lambda e: self.goto_card())
        root.bind('x', lambda e: self.mark_empty())
        root.bind('s', lambda e: self.save(True))
        root.bind('<plus>', lambda e: self.zoom_at(1.25, None))
        root.bind('<equal>', lambda e: self.zoom_at(1.25, None))
        root.bind('<minus>', lambda e: self.zoom_at(0.8, None))
        root.protocol('WM_DELETE_WINDOW', self.on_close)

        self.load(0)

    # ---------------- 坐标变换 ----------------
    def to_canvas(self, x, y):
        return x * self.scale + self.ox, y * self.scale + self.oy

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
        self.sel = None
        self.drag = None
        self.lbl_file.config(text='[%d/%d]  %s' % (self.idx + 1, len(self.imgs), it['file']))
        if it.get('done'):
            self.lbl_flag.config(text='✔ 已标', foreground='#080')
        else:
            self.lbl_flag.config(text='— 待标', foreground='#888')
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

    def goto_card(self):
        """把视图跳到导航屏左上(转向标所在), 1.3x"""
        cw = self.canvas.winfo_width()
        ch = self.canvas.winfo_height()
        if cw < 50 or ch < 50:
            cw, ch = 1150, 900
        self.scale = 1.3
        # 目标图像坐标(导航卡左上: 转向标所在)居中
        cx, cy = 205, 660
        self.ox = cw / 2 - cx * self.scale
        self.oy = ch / 2 - cy * self.scale
        self.redraw()

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
            tag = CN[b['cls']]
            d.rectangle([bx[0], max(0, bx[1] - 12), bx[0] + 14 * len(tag) + 4, bx[1]], fill=col)
            d.text((bx[0] + 2, max(0, bx[1] - 12)), tag, fill=(0, 0, 0))
        if self.drag:
            x1, y1, x2, y2 = self.drag
            d.rectangle([x1 * self.scale, y1 * self.scale,
                         x2 * self.scale, y2 * self.scale],
                        outline=(255, 255, 0), width=2)

        self.tkimg = ImageTk.PhotoImage(disp)
        self.canvas.delete('all')
        self.canvas.create_image(self.ox, self.oy, anchor='nw', image=self.tkimg)
        self.lbl_pos.config(text='缩放 %.2fx   框数 %d' % (self.scale, len(it['boxes'])))
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
        # 画类别框(换算到放大镜局部坐标)
        d = ImageDraw.Draw(crop)
        for b in self.imgs[self.idx]['boxes']:
            bx1, by1, bx2, by2 = b['box']
            d.rectangle([(bx1 - x1) * MAG, (by1 - y1) * MAG,
                         (bx2 - x1) * MAG, (by2 - y1) * MAG],
                        outline=PL[b['cls']], width=2)
        if self.drag:
            dx1, dy1, dx2, dy2 = self.drag
            d.rectangle([(dx1 - x1) * MAG, (dy1 - y1) * MAG,
                         (dx2 - x1) * MAG, (dy2 - y1) * MAG],
                        outline=(255, 255, 0), width=2)
        # 十字准星
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
            x1, y1, x2, y2 = b['box']
            self.lst.insert('end', '%2d  %-8s %4.0f,%4.0f %4.0f,%4.0f'
                            % (k, CN[b['cls']], x1, y1, x2, y2))
        if self.sel is not None and self.sel < self.lst.size():
            self.lst.selection_clear(0, 'end')
            self.lst.selection_set(self.sel)
        n_img = len(self.imgs)
        n_done = sum(1 for i in self.imgs if i.get('done'))
        n_box = sum(len(i['boxes']) for i in self.imgs)
        cnt = {c: 0 for c in CLASSES}
        for i in self.imgs:
            for b in i['boxes']:
                cnt[b['cls']] = cnt.get(b['cls'], 0) + 1
        self.lbl_stat.config(
            text='已处理 %d/%d 张    总框 %d\n直行 %d | 左转 %d | 右转 %d'
                 % (n_done, n_img, n_box, cnt['straight'], cnt['left'], cnt['right']))

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
        # 新框默认继承"上一个框的类别", 没有就 straight —— 少按一次键
        last = it['boxes'][-1]['cls'] if it['boxes'] else CLASSES[0]
        it['boxes'].append({'cls': last, 'box': [round(v, 1) for v in (x1, y1, x2, y2)],
                            'manual': True})
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
        if self.sel is None or self.sel >= len(self.imgs[self.idx]['boxes']):
            return
        self.imgs[self.idx]['boxes'][self.sel]['cls'] = c
        self.save()
        self.refresh_list()
        self.redraw()

    def delete_sel(self):
        if self.sel is None or self.sel >= len(self.imgs[self.idx]['boxes']):
            return
        del self.imgs[self.idx]['boxes'][self.sel]
        self.sel = None
        self.save()
        self.refresh_list()
        self.redraw()

    def mark_empty(self):
        it = self.imgs[self.idx]
        it['boxes'] = []
        it['done'] = True
        self.sel = None
        self.save()
        self.refresh_list()
        self.next()

    def next(self):
        self.imgs[self.idx]['done'] = True
        self.save()
        if self.idx < len(self.imgs) - 1:
            self.load(self.idx + 1)

    def prev(self):
        if self.idx > 0:
            self.load(self.idx - 1)

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
