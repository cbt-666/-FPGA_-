"""FPGA YRG recovery, model-specific letterbox, and decoding helpers."""
import numpy as np


def yrg_to_rgb(yrg):
    import cv2
    if yrg.dtype != np.uint8 or yrg.ndim != 2 or yrg.shape != (720, 1280):
        raise ValueError(f'YRG expected (720,1280) uint8, got {yrg.shape}/{yrg.dtype}')
    rgb = cv2.cvtColor(yrg, cv2.COLOR_GRAY2RGB)
    rgb[yrg == 254] = (255, 0, 0)
    rgb[yrg == 255] = (0, 255, 0)
    return rgb


def fpga_brightness_from_yrg(yrg):
    """Recovered RGB -> FPGA fixed-point luminance, for scene/nav models.

    YRG gray 0..253 stays the same. Red 254 -> 77*255>>8 = 76;
    green 255 -> 150*255>>8 = 149.
    """
    gray = yrg.copy()
    gray[yrg == 254] = (77 * 255) >> 8
    gray[yrg == 255] = (150 * 255) >> 8
    return gray


def letterbox(img, wh, value=114):
    import cv2
    W, H = wh
    h, w = img.shape[:2]
    r = min(W / w, H / h)
    nw, nh = max(1, int(round(w*r))), max(1, int(round(h*r)))
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_AREA if r < 1 else cv2.INTER_LINEAR)
    left = (W - nw) // 2
    top = (H - nh) // 2
    if img.ndim == 2:
        out = np.full((H, W), value, np.uint8)
    else:
        out = np.full((H, W, img.shape[2]), value, np.uint8)
    out[top:top+nh, left:left+nw] = resized
    return out, (r, left, top)


def channel_matrix(obj, c, n):
    a = np.asarray(obj)
    if a.shape == (1, c, n): return a[0].astype(np.float32, copy=False)
    if a.shape == (1, n, c): return a[0].T.astype(np.float32, copy=False)
    if a.shape == (c, n): return a.astype(np.float32, copy=False)
    if a.shape == (n, c): return a.T.astype(np.float32, copy=False)
    raise ValueError(f'Expected output ({c},{n}) or transpose, got {a.shape}')


def xywh_to_xyxy(xywh, geo, source_shape):
    b = np.asarray(xywh, dtype=np.float32)
    x = np.empty_like(b)
    x[:, 0] = b[:, 0] - b[:, 2]/2
    x[:, 1] = b[:, 1] - b[:, 3]/2
    x[:, 2] = b[:, 0] + b[:, 2]/2
    x[:, 3] = b[:, 1] + b[:, 3]/2
    r, dx, dy = geo
    x[:, [0, 2]] = (x[:, [0, 2]] - dx) / r
    x[:, [1, 3]] = (x[:, [1, 3]] - dy) / r
    h,w = source_shape[:2]
    x[:, [0,2]] = np.clip(x[:, [0,2]],0,w)
    x[:, [1,3]] = np.clip(x[:, [1,3]],0,h)
    return x


def nms(boxes, scores, threshold=.45):
    if not len(boxes): return []
    x1,y1,x2,y2 = boxes.T
    areas = np.maximum(0,x2-x1) * np.maximum(0,y2-y1)
    order = np.argsort(scores)[::-1]
    keep = []
    while len(order):
        i = int(order[0]);keep.append(i)
        rest = order[1:]
        if not len(rest):break
        xx1 = np.maximum(x1[i],x1[rest]);yy1=np.maximum(y1[i],y1[rest])
        xx2 = np.minimum(x2[i],x2[rest]);yy2=np.minimum(y2[i],y2[rest])
        inter = np.maximum(0,xx2-xx1)*np.maximum(0,yy2-yy1)
        iou = inter/(areas[i]+areas[rest]-inter+1e-9)
        order=rest[iou<=threshold]
    return keep


def decode_yolo(single_or_split, n, class_names, geo, source_shape, confs,
                iou=.45, merge_crosswalk=False, agnostic=False):
    """Decode YOLO outputs already in network pixel xywh / class probabilities.

    single: 1x(4+C)xN; split: [1x4xN,1xCxN]; per-class NMS.
    agnostic=True suppresses across ALL classes at once. nav6 needs it: the same
    arrow is frequently scored as both right_3 and right_2, and per-class NMS
    leaves one box per class, which then reads as two different directions.
    """
    c = len(class_names)
    # RKNN returns list of outputs. FP16 builds give ONE tensor, INT8 gave two.
    if isinstance(single_or_split,(tuple,list)) and len(single_or_split)==1:
        single_or_split=single_or_split[0]
    if isinstance(single_or_split,(tuple,list)):
        if len(single_or_split)!=2:raise ValueError('expected one or two outputs')
        try:
            boxes=channel_matrix(single_or_split[0],4,n)
            score=channel_matrix(single_or_split[1],c,n)
        except ValueError:
            boxes=channel_matrix(single_or_split[1],4,n)
            score=channel_matrix(single_or_split[0],c,n)
    else:
        o = channel_matrix(single_or_split,4+c,n)
        boxes,score = o[:4],o[4:]
    if not (np.isfinite(boxes).all() and np.isfinite(score).all()):
        raise ValueError('model output contains NaN/Inf')
    score_max=score.max(axis=0);lab=score.argmax(axis=0)
    threshold=np.array([confs[nm] for nm in class_names],np.float32)
    chosen=np.flatnonzero(score_max>=threshold[lab])
    if not len(chosen): return []
    xyxy=xywh_to_xyxy(boxes[:,chosen].T,geo,source_shape)
    class_id=lab[chosen];values=score_max[chosen]
    if agnostic:
        result=[]
        for i in nms(xyxy,values,iou):
            a=xyxy[i]
            if a[2]-a[0]<2 or a[3]-a[1]<2:continue
            k=int(class_id[i])
            result.append({'class_name':class_names[k],'class_id':k,
                           'score':float(values[i]),
                           'xyxy': [float(v) for v in a]})
        return sorted(result,key=lambda x:-x['score'])
    result=[]
    for k,name in enumerate(class_names):
        ids=np.flatnonzero(class_id==k)
        if not len(ids):continue
        b=xyxy[ids];s=values[ids]
        if merge_crosswalk and name=='crosswalk':
            b,s=_merge_nearby(b,s,source_shape)
        for i in nms(b,s,iou):
            a=b[i]
            if a[2]-a[0]<2 or a[3]-a[1]<2:continue
            result.append({'class_name':name,'class_id':k,'score':float(s[i]),
                           'xyxy': [float(v) for v in a]})
    return sorted(result,key=lambda x:-x['score'])


def _merge_nearby(boxes,scores,shape,gap_x=.10,gap_y=.10):
    """Crosswalk ONLY. Merge at original image resolution, before class NMS."""
    if len(boxes)<=1:return boxes,scores
    h,w=shape[:2]
    boxes=boxes.copy();scores=scores.copy()
    while True:
        groups=[];used=set();merged=False
        for i in range(len(boxes)):
            if i in used:continue
            a=boxes[i].copy();v=scores[i]
            used.add(i)
            for j in range(i+1,len(boxes)):
                if j in used:continue
                b=boxes[j]
                x_gap=max(a[0],b[0])-min(a[2],b[2])
                y_gap=max(a[1],b[1])-min(a[3],b[3])
                if x_gap<gap_x*w and y_gap<gap_y*h:
                    a=np.array([min(a[0],b[0]),min(a[1],b[1]),max(a[2],b[2]),max(a[3],b[3])])
                    v=max(v,scores[j]);used.add(j);merged=True
            groups.append((a,v))
        boxes=np.array([x for x,_ in groups]);scores=np.array([v for _,v in groups])
        if not merged: return boxes,scores
