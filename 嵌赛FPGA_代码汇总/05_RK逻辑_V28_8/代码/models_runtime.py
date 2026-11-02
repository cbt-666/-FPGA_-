"""RK3568 RKNN model adapters for the FPGA YRG stream (720x1280).

Every model has its own input contract. Getting one wrong fails loudly (dtype)
or silently (clipped range -> garbage), so each adapter states what it feeds:

  scene    (merged5gray) float32 NHWC 0..255, 1024x1024 letterbox, 3 ch identical
                         output [1,9,21504] = 4 box + 5 class
  nav      (nav6_720)    float32 NHWC 0..255, DIRECT 1280x720, brightness grey
                         output box[1,4,18920] + cls[1,6,18920] (already sigmoid)
                         6 classes = direction x digit-count
  housing  (灯箱)          float32 NHWC 0..255, DIRECT 1280x768 stretch, RGB
                           output box[1,4,20160] DIRECT cx,cy,w,h in pixels
                           + confidence[1,1,20160]
  lane v2  (G640)        float32 NCHW 0..255, 640x360 grey
                           1 x 1 x H x W input, [1,3,H,W] logits
  cop      (cls_v2yrg3)  uint8 NHWC 0..255 RGB, DIRECT 96x96 stretch
                           output [1,3] logits -> 类1 / 类2 / other
                           FPGA 已经判完红绿，输入走单通道 YRG 重建，
                           **不重做** fpga_rg_gray()

⚠️ 2026-10-31：二值通道整个删除。FPGA 只发 MODE 1，全部模型都吃 YRG 灰度。
"""
from pathlib import Path
import logging
import time
import numpy as np
from vision_common import yrg_to_rgb, fpga_brightness_from_yrg, letterbox, decode_yolo
import lane_v2
import cop_traffic_signature

LOG = logging.getLogger('models')
SCENE_CLASSES = ('person', 'crosswalk', 'lane', 'traffic_light', 'nav')
# 2026-10-29 用户反馈「斑马线识别很不灵敏」。板上日志里 crosswalk 的分数
# 几乎全压在 0.20~0.22（v17_5: 0.20/0.21；v17_2: 0.22/0.41），而阈值正是
# 0.20 —— 一直在门槛线上擦。降到 0.12 让它进得来。
# 代价可控：斑马线只驱动一个"黄色边缘警告"位，不会让车停或动，
# 所以这一类比 person/traffic_light 更适合放宽。
SCENE_CONF = {'person': .40, 'crosswalk': .12, 'lane': .15, 'traffic_light': .25, 'nav': .70}
HOUSING_CONF = .25
HOUSING_N = 20160
# 车道只有灰度一种输入形态。A768（二值）连同它需要的 MODE 2 采集一起删除。
LANE_VARIANT = 'gray640'

# nav6: direction x digit count in ONE detection head, replacing the old
# 3-class arrow model plus the OpenCV template digit reader.
#   0 right_3  1 right_2  2 left_3  3 left_2  4 uturn_3  5 uturn_2
NAV_CLASSES = ('right_3', 'right_2', 'left_3', 'left_2', 'uturn_3', 'uturn_2')
NAV_N = 18920
NAV_CONF = .25
NAV_IOU = .5
# Screen-space gates from the model README, stated in the 1920x1080 coordinates
# the annotations used; the board frame is 1280x720, so every bound is scaled
# by 2/3. Without these the detector eats sky/buildings outside the windshield.
NAV_ROI = (0, 720, 470, 920)          # x1, x2, y1, y2 in 1920x1080
NAV_SIZE = (90, 270, 45, 225)         # w_min, w_max, h_min, h_max in 1920x1080
NAV_SCALE = 2 / 3

class RKNNHandle:
    def __init__(self, path, core_mask='auto'):
        if not Path(path).is_file():
            raise FileNotFoundError(f'Model missing: {path}')
        from rknnlite.api import RKNNLite
        self.rk = RKNNLite()
        try:
            ret = self.rk.load_rknn(str(Path(path).resolve()))
            if ret != 0: raise RuntimeError(f'load_rknn ret={ret}: {path}')
            if core_mask == 'auto':
                ret = self.rk.init_runtime()
            else:
                key={'0':'NPU_CORE_0','all':'NPU_CORE_ALL'}.get(core_mask)
                if key is None or not hasattr(RKNNLite,key):
                    raise RuntimeError(f'Unsupported core_mask {core_mask}')
                ret = self.rk.init_runtime(core_mask=getattr(RKNNLite,key))
            if ret != 0: raise RuntimeError(f'init_runtime ret={ret}: {path}')
        except Exception:
            self.rk.release();raise
        self.first=True
        self.last_infer_ms=0.
        LOG.info('RKNN model loaded: %s', path)

    def run(self, tensor, **kwargs):
        started=time.perf_counter()
        outs=self.rk.inference(inputs=[tensor], **kwargs)
        self.last_infer_ms=round((time.perf_counter()-started)*1000,2)
        if outs is None: raise RuntimeError('rknn.inference returned None')
        if self.first:
            LOG.info('RKNN input %s %s; outputs %s',tensor.shape,tensor.dtype,
                     [(np.asarray(v).shape,str(np.asarray(v).dtype)) for v in outs])
            self.first=False
        return outs
    def close(self): self.rk.release()

class LazyDetectors:
    def __init__(self, scene, nav, housing, lane, core_mask='auto', cop=None):
        self.paths={'scene':Path(scene),'nav':Path(nav),'housing':Path(housing),
                    'lane':Path(lane)}
        # 交警三分类模型是**可选**的：不给路径时 `cop_classify` 返回 None，
        # 交警手势分支自然永远不会触发。这样没有该模型的旧部署也照常启动。
        if cop is not None:
            self.paths['cop']=Path(cop)
        self.loaded={};self.core_mask=core_mask
        self.lane_variant=LANE_VARIANT
        self._cached_yrg=None;self._cached_gray=None
        self.last_model_stages={}
        try:
            self.get('scene')
        except BaseException:
            # A half-built detector must not leak NPU contexts. The supervisor
            # restarts this program on failure, so without this every restart
            # would strand one context until the driver refuses to allocate.
            self.close()
            raise

    def get(self,name):
        if name not in self.loaded:
            self.loaded[name]=RKNNHandle(self.paths[name],self.core_mask)
        return self.loaded[name]

    def _brightness(self,yrg):
        # ⚠️ 2026-10-28 板上日志：不要按对象身份缓存。
        # 依赖 `yrg is self._cached_yrg` 就等于依赖 CPython 复用同一块内存地址：
        # 上一帧的数组一旦被回收，下一帧极可能拿到同一个 id()，命中缓存后
        # 把**上一帧的灰度**当成这一帧喂给模型。板上第 2 次运行从第 84 轮起
        # 场景模型永久检不到任何东西，形态完全吻合——图是好的（帧号在推进、
        # 灰度级数 194），模型却在拿旧图推理。
        # 每轮一次 77/150/29 的整帧运算只要几毫秒，不值得用正确性去换。
        self._cached_yrg = None
        return fpga_brightness_from_yrg(yrg)

    def _record(self,name,started):
        h=self.get(name)
        self.last_model_stages[name]={'total_ms':round((time.perf_counter()-started)*1000,2),
                                      'rknn_inference_ms':h.last_infer_ms}

    def scene(self,yrg):
        started=time.perf_counter()
        gray=self._brightness(yrg)
        g,meta=letterbox(gray,(1024,1024))
        tensor=np.ascontiguousarray(np.repeat(g[:,:,None],3,axis=2)[None].astype(np.float32))
        # FP16 build: ONE output [1,9,21504] (box+cls concatenated).
        # Raw input is 0..255 float32; /255 is baked into the ONNX graph.
        outs=self.get('scene').run(tensor,data_format=['nhwc'],data_type='float32')
        dets=decode_yolo(outs,21504,SCENE_CLASSES,meta,yrg.shape,SCENE_CONF,
                         iou=.45,merge_crosswalk=True)
        self._record('scene',started)
        return dets

    def navigation(self, yrg):
        """nav6 六类：一次推理同时给出转向方向和剩余距离位数。

        输入契约（导航_方向与位数/README_导航6类.md）：
          * float32 NHWC 0..255，直接 1280x720，亮度灰度复制 3 通道
          * **绝不用红绿保留变换**——地图底图是蓝的，G>B 判据会把整屏打黑
          * graph 内部 mean=0 / std=255
          * 输出 box[1,4,18920] + cls[1,6,18920]，类分数已过 sigmoid，
            框是网络像素坐标，不乘 stride、不做 DFL、无 anchor
        NMS 必须跨类别：否则同一支箭头会在 right_3 和 right_2 各留一个框。
        """
        started = time.perf_counter()
        gray = self._brightness(yrg)
        # 720 model: DIRECT 1280x720, not 736 and not square letterbox.
        img = np.ascontiguousarray(
            np.repeat(gray[:, :, None], 3, axis=2)[None].astype(np.float32))
        outs = self.get('nav').run(img, data_format=['nhwc'], data_type='float32')
        dets = decode_yolo(outs, NAV_N, NAV_CLASSES, (1.0, 0, 0), yrg.shape,
                           {s: NAV_CONF for s in NAV_CLASSES}, iou=NAV_IOU,
                           agnostic=True)
        kept, rejected = [], {'roi': 0, 'size': 0}
        for box in dets:
            x1, y1, x2, y2 = box['xyxy']
            if not nav_roi_ok((x1 + x2) / 2, (y1 + y2) / 2):
                rejected['roi'] += 1
                continue
            if not nav_size_ok(x2 - x1, y2 - y1):
                rejected['size'] += 1
                continue
            kept.append(box)
        self._record('nav', started)
        self.last_model_stages['nav'].update({'kept': len(kept),
                                              'rejected': rejected})
        return kept

    def housing(self, yrg):
        """灯箱检测。返回原图坐标的 xyxy 框列表，按分数降序。

        输入契约（灯箱包 README，已与 .pt 逐元素对拍：置信图相关 1.0000）：
          * float32 NHWC 0..255，RGB（三通道有颜色，顺序不能反）
          * 1280x768 直接拉伸，不 letterbox、不补边
          * 输出 box[1,4,20160] 是【直接的 cx,cy,w,h 像素】—— 不乘 stride、
            不做 DFL、不加 anchor。按 ltrb 解码会得到完全错误的框。
          * 置信度[1,1,20160] 已过 sigmoid，NMS 只在这里做一次。
        """
        started = time.perf_counter()
        import cv2
        rgb = yrg_to_rgb(yrg)
        img = cv2.resize(rgb, (1280, 768), interpolation=cv2.INTER_LINEAR)
        # ONNX 要 0..1；RKNN 用 std=255 内部做同样的除法，这里喂 0..255。
        inp = np.ascontiguousarray(img[None].astype(np.float32))
        outs = self.get('housing').run(inp, data_format=['nhwc'], data_type='float32')
        if len(outs) != 2:
            raise ValueError(f'housing 需要双输出，得到 {len(outs)}')
        box = self._pick(out_n(outs[0]), (4, HOUSING_N))
        sig = self._pick(out_n(outs[1]), (1, HOUSING_N)).reshape(-1)
        idx = np.flatnonzero(sig >= HOUSING_CONF)
        dets = []
        for i in idx:
            cx, cy, w, h = box[:, i]
            x1, y1, x2, y2 = cx - w/2, cy - h/2, cx + w/2, cy + h/2
            # 缩放回原图：768 高是 NPU 对 32 倍数的要求，原图 720。
            dets.append([float(x1), float(y1) * 720/768,
                         float(x2), float(y2) * 720/768, float(sig[i])])
        dets = _nms_list(dets, .45)
        self._record('housing', started)
        return dets

    @staticmethod
    def _pick(a, shape):
        """输出张量统一成 [c, n]。"""
        a = np.asarray(a, np.float32)
        if a.ndim == 3:
            a = a[0]
        if a.shape == shape:
            return a
        if a.shape[::-1] == shape:
            return a.T
        raise ValueError(f'housing 输出形状异常 {a.shape}，期望 {shape}')

    def cop_classify(self, yrg):
        """交警三分类（cls_v2yrg3）：返回 (3,) 原始 logits，或 None（模型未装）。

        ⚠️ 与包里 `infer_cls.py` 的唯一差别：**不做 `fpga_rg_gray()`**。
        那个函数是给本地 BGR 视频用的，它自己再判一遍红绿；板上 FPGA 已经
        判完了，RK 端只有单通道 YRG，重做等于用 RK 的阈值覆盖 FPGA 的阈值。
        这里只做 `yrg_single_to_bgr()` + 96x96 拉伸。
        """
        if 'cop' not in self.paths:
            return None
        started = time.perf_counter()
        x = cop_traffic_signature.to_tensor(yrg)
        # 与交付包 infer_cls.py 的调法逐字一致：NHWC uint8，不传 data_type
        # （归一化 mean/std=127.5 已烘进 graph，运行时自己把 uint8 转 float）。
        outs = self.get('cop').run(x, data_format='nhwc')
        out = outs[0] if isinstance(outs, (list, tuple)) else outs
        logits = np.asarray(out, np.float32).reshape(-1)
        self._record('cop', started)
        return logits

    def lane_mask(self, yrg):
        """车道 v2 掩码：0 背景 / 1 白虚线 / 2 实线。

        ⚠️⚠️ 2026-10-31 用户硬性要求：「RK 端坚决不要做二值化」。
        输入是 YRG 帧，先按 FPGA 亮度公式取灰度，再**只做缩放**喂 G640。
        这里没有任何阈值运算；车道模型也不再需要 FPGA 的二值图。
        """
        started = time.perf_counter()
        t_pre = time.perf_counter()
        gray = self._brightness(yrg)
        inp = lane_v2.prepare(gray, LANE_VARIANT)
        t_inf = time.perf_counter()
        outs = self.get('lane').run(inp, data_format=['nchw'], data_type='float32')
        out = outs[0] if isinstance(outs, (list, tuple)) else outs
        mask = lane_v2.mask_from_logits(out)
        t_dec = time.perf_counter()
        self._record('lane', started)
        self.last_model_stages['lane'].update({
            'preprocess_ms': round((t_inf - t_pre) * 1000, 2),
            'decode_ms': round((t_dec - t_inf) * 1000, 2),
            'variant': LANE_VARIANT,
            'source': 'grey',
            'mask_shape': list(mask.shape),
        })
        return mask

    def close(self):
        for name,h in list(self.loaded.items()):
            try:
                h.close()
            except Exception:
                LOG.exception('释放 RKNN 模型 %s 失败', name)
        self.loaded.clear()


def out_n(o):
    return np.asarray(o)


def nav_roi_ok(cx, cy):
    """Navigation screen gate, in 1280x720 board coordinates.

    README states `0<=cx<=720 and 470<=cy<=920` for 1920x1080; both scale by
    2/3 on the board frame, so 0..480 horizontally and 313.3..613.3 vertically.
    """
    x1, x2, y1, y2 = (v * NAV_SCALE for v in NAV_ROI)
    return x1 <= cx <= x2 and y1 <= cy <= y2


def nav_size_ok(w, h):
    """Card / 3D-arrow size gate, in 1280x720 board coordinates."""
    w_lo, w_hi, h_lo, h_hi = (v * NAV_SCALE for v in NAV_SIZE)
    return w_lo <= w <= w_hi and h_lo <= h <= h_hi


def _nms_list(dets, iou_thr):
    dets = sorted(dets, key=lambda d: -d[4])
    keep = []
    while dets:
        a = dets.pop(0)
        keep.append(a)
        rest = []
        for b in dets:
            ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
            iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
            inter = ix * iy
            ua = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
            if (inter/ua if ua > 0 else 0.0) < iou_thr:
                rest.append(b)
        dets = rest
    return keep
