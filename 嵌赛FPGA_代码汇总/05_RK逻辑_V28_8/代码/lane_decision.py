"""V10: 按角度选目标线，再把空间相关的近角度候选纳入同一目标线组。

输入 mask: H×W, uint8, 0=背景, 1=虚线, 2=实线。
目标线组的虚线占比 = 全组标签1像素总数 / (标签1+标签2像素总数)；仅单段时用于分类。
目标线组有两段及以上断续线段时必判 dash；只有一段时严格 >0.03 判 dash。

兼容 V9 的 decode(mask) 输出字段；增加 target_group_count、target_members。
仅使用分割标签；不更改 FPGA UDP/RKNN 的输入、输出和安全策略。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class _Piece:
    x: np.ndarray
    y: np.ndarray
    angle: float
    slope: float
    xbar: float
    ybar: float
    y_min: float
    y_max: float
    dash: int
    solid: int


def _regress(x: np.ndarray, y: np.ndarray):
    xm, ym = float(np.mean(x)), float(np.mean(y))
    yy = y.astype(np.float64) - ym
    var_y = float(yy @ yy)
    if var_y <= 1.0:
        return None
    slope = float((yy @ (x.astype(np.float64) - xm)) / var_y)
    angle = math.degrees(math.atan2(1., -slope))
    return xm, ym, slope, angle


def _extract_pieces(mask, roi_top, roi_bottom, min_piece_pixels, min_piece_height, min_elongation):
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError('角度车道线检测需要 python3-opencv (cv2)') from exc
    binary = (mask[roi_top:roi_bottom] != 0).astype(np.uint8)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
    out = []
    for i in range(1, n):
        x0, y0, w, h, area = map(int, stats[i])
        if area < min_piece_pixels or h < min_piece_height:
            continue
        yy, xx = np.nonzero(labels[y0:y0+h, x0:x0+w] == i)
        if len(xx) < min_piece_pixels:
            continue
        x = (xx+x0).astype(np.float64)
        y = (yy+y0+roi_top).astype(np.float64)
        covariance = np.cov(np.stack([x, y]), bias=True)
        eig = np.linalg.eigvalsh(covariance)
        if eig[1] < 4 or eig[1]/max(eig[0],1.) < min_elongation:
            continue
        fit = _regress(x, y)
        if fit is None:
            continue
        xm,ym,slope,angle = fit
        if not (8. < angle < 172.):
            continue
        labels_piece = mask[y.astype(np.intp),x.astype(np.intp)]
        out.append(_Piece(x,y,angle,slope,xm,ym,float(np.min(y)),float(np.max(y)),
                          int(np.count_nonzero(labels_piece==1)),
                          int(np.count_nonzero(labels_piece==2))))
    return out


def _joinable(a,b,angle_tolerance,lateral_tolerance,max_gap):
    if abs(a.angle-b.angle) > angle_tolerance:
        return False
    gap = max(0.,a.y_min-b.y_max,b.y_min-a.y_max)
    if gap > max_gap:
        return False
    at_y = (a.ybar+b.ybar)/2
    xa = a.xbar + a.slope*(at_y-a.ybar)
    xb = b.xbar + b.slope*(at_y-b.ybar)
    return abs(xa-xb)<=lateral_tolerance


def _group(pieces,angle_tolerance,lateral_tolerance,max_gap):
    parent=list(range(len(pieces)))
    def root(i):
        while parent[i]!=i:
            parent[i]=parent[parent[i]]
            i=parent[i]
        return i
    for i in range(len(pieces)):
        for j in range(i+1,len(pieces)):
            if _joinable(pieces[i],pieces[j],angle_tolerance,lateral_tolerance,max_gap):
                parent[root(j)]=root(i)
    groups={}
    for i in range(len(pieces)):
        groups.setdefault(root(i),[]).append(pieces[i])
    return list(groups.values())


def _candidate_from_members(members,reference_y,min_line_pixels,min_height,dash_threshold):
    xx=np.concatenate([p.x for p in members])
    yy=np.concatenate([p.y for p in members])
    fit=_regress(xx,yy)
    if fit is None or len(xx)<min_line_pixels or np.ptp(yy)<min_height:
        return None
    xm,ym,slope,angle=fit
    if not (8.<angle<172.):
        return None
    dash=sum(p.dash for p in members)
    solid=sum(p.solid for p in members)
    total=dash+solid
    if not total:
        return None
    ratio=dash/total
    # 一次几何关联后的多个独立连通线段，优先判为虚线。
    # 注意：这一数量是独立片段数，不是模型类别数。
    return {'angle_deg':float(angle),'x_at_reference_y':float(xm+slope*(reference_y-ym)),
            'reference_y':reference_y,'dash_pixels':int(dash),'solid_pixels':int(solid),
            'dash_ratio':ratio,'class':'dash' if len(members)>1 or ratio>dash_threshold else 'solid',
            'fragment_count':len(members),'line_pixels':int(total),
            'y_range':[float(np.min(yy)),float(np.max(yy))]}


def _same_target(anchor,c,angle_tol,lateral_tol,max_gap,overlap_lateral_tol):
    """角度相近 + 在相同延长线走廊；重叠很多的平行线必须更加贴近。

    以 anchor 为中心测试，避免 A-B、B-C 的传递合并把相邻车道串成一个组。
    """
    if abs(anchor['angle_deg']-c['angle_deg'])>angle_tol:
        return False
    lateral=abs(anchor['x_at_reference_y']-c['x_at_reference_y'])
    if lateral>lateral_tol:
        return False
    a0,a1=anchor['y_range']
    b0,b1=c['y_range']
    gap=max(0.,a0-b1,b0-a1)
    if gap>max_gap:
        return False
    overlap=max(0.,min(a1,b1)-max(a0,b0))
    shorter=max(1.,min(a1-a0,b1-b0))
    if overlap/shorter>0.35 and lateral>overlap_lateral_tol:
        return False
    return True


def _select_group(side_candidates,dash_threshold,secondary_angle_tolerance,
                  secondary_lateral_tolerance,secondary_max_gap,
                  parallel_overlap_lateral_tolerance,w):
    if not side_candidates:
        return None
    anchor=min(side_candidates,key=lambda c:(abs(c['angle_deg']-90.),
                 abs(c['x_at_reference_y']-w/2),-c['line_pixels']))
    selected=[anchor]
    for c in side_candidates:
        if c is anchor:
            continue
        if _same_target(anchor,c,secondary_angle_tolerance,secondary_lateral_tolerance,
                        secondary_max_gap,parallel_overlap_lateral_tolerance):
            selected.append(c)
    dash=sum(c['dash_pixels'] for c in selected)
    solid=sum(c['solid_pixels'] for c in selected)
    total=dash+solid
    # 像素数权重；标签比例以原始像素整数计数，绝不平均各条线比例。
    angle=sum(c['angle_deg']*c['line_pixels'] for c in selected)/total
    xref=sum(c['x_at_reference_y']*c['line_pixels'] for c in selected)/total
    ratio=dash/total
    fragment_count=sum(c['fragment_count'] for c in selected)
    # 先判断同一条目标车道的独立线段个数；多个线段直接是虚线。
    # 只有唯一线段才依据标签1 / (标签1+标签2) > 3% 判定。
    kind = 'dash' if fragment_count > 1 or ratio > dash_threshold else 'solid'
    decision_basis = 'multiple_aligned_fragments' if fragment_count > 1 else 'pixel_ratio'
    return {'angle_deg':round(angle,3),'anchor_angle_deg':round(anchor['angle_deg'],3),
            'x_at_reference_y':round(xref,2),'reference_y':anchor['reference_y'],
            'dash_pixels':dash,'solid_pixels':solid,'dash_ratio':round(ratio,6),
            'class':kind, 'decision_basis':decision_basis,
            'fragment_count':fragment_count, 'target_line_count':fragment_count,
            'target_group_count':len(selected),'line_pixels':total,
            'y_range':[min(c['y_range'][0] for c in selected),
                       max(c['y_range'][1] for c in selected)],
            'target_members':[{'angle_deg':round(c['angle_deg'],3),
                               'x_at_reference_y':round(c['x_at_reference_y'],2),
                               'dash_pixels':c['dash_pixels'],'solid_pixels':c['solid_pixels'],
                               'dash_ratio':round(c['dash_ratio'],6),
                               'fragment_count':c['fragment_count'],
                               'y_range':[round(x,1) for x in c['y_range']]}
                              for c in selected]}


def decode(mask,dash_threshold=0.03,roi=(.45,.95),
           min_piece_pixels=14,min_piece_height=10,min_elongation=2.8,
           angle_tolerance=12.,lateral_tolerance=22.,max_join_gap_frac=.30,
           min_line_pixels=42,min_line_height_frac=.055,
           vertical_deadzone_deg=.35,
           secondary_angle_tolerance=3.,secondary_lateral_tolerance=42.,
           secondary_max_gap_frac=.45,
           parallel_overlap_lateral_tolerance=16.,**unused_legacy_kwargs):
    """选最接近90°的左右主候选，再汇总各自空间相关的多条近角度线段。"""
    if not isinstance(mask,np.ndarray) or mask.ndim!=2:
        raise ValueError('mask必须为二维 numpy 数组')
    if not (0<=dash_threshold<=1):
        raise ValueError('dash_threshold必须在0..1')
    if np.any((mask!=0)&(mask!=1)&(mask!=2)):
        raise ValueError('mask只能包含0,1,2')
    if not (0<=roi[0]<roi[1]<=1):
        raise ValueError('非法roi')
    h,w=mask.shape
    top,bottom=int(h*roi[0]),int(h*roi[1])
    if bottom<=top:
        raise ValueError('roi过小')
    for k,v in [('secondary_angle_tolerance',secondary_angle_tolerance),
                ('secondary_lateral_tolerance',secondary_lateral_tolerance),
                ('secondary_max_gap_frac',secondary_max_gap_frac),
                ('parallel_overlap_lateral_tolerance',parallel_overlap_lateral_tolerance)]:
        if not (math.isfinite(v) and v>=0):
            raise ValueError(k+'必须是非负有限值')
    pieces=_extract_pieces(mask,top,bottom,min_piece_pixels,min_piece_height,min_elongation)
    groups=_group(pieces,angle_tolerance,lateral_tolerance,h*max_join_gap_frac)
    reference_y=int(h*.80)
    candidates=[]
    for members in groups:
        c=_candidate_from_members(members,reference_y,min_line_pixels,
                                  h*min_line_height_frac,dash_threshold)
        if c is not None:
            candidates.append(c)
    left_candidates,right_candidates=[],[]
    for c in candidates:
        angle,xref=c['angle_deg'],c['x_at_reference_y']
        if angle<90.-vertical_deadzone_deg:
            left_candidates.append(c)
        elif angle>90.+vertical_deadzone_deg:
            right_candidates.append(c)
        elif xref<w/2:
            left_candidates.append(c)
        elif xref>w/2:
            right_candidates.append(c)
    choose=lambda cs:_select_group(cs,dash_threshold,secondary_angle_tolerance,
                     secondary_lateral_tolerance,h*secondary_max_gap_frac,
                     parallel_overlap_lateral_tolerance,w)
    left,right=choose(left_candidates),choose(right_candidates)
    return {
        'left':left['class'] if left else 'none',
        'right':right['class'] if right else 'none',
        'left_x':int(round(left['x_at_reference_y'])) if left else None,
        'right_x':int(round(right['x_at_reference_y'])) if right else None,
        'left_ev':float(left['line_pixels']) if left else 0.,
        'right_ev':float(right['line_pixels']) if right else 0.,
        'can_change_left':bool(left and left['class']=='dash'),
        'can_change_right':bool(right and right['class']=='dash'),
        'left_angle':left['angle_deg'] if left else None,
        'right_angle':right['angle_deg'] if right else None,
        'left_dash_ratio':left['dash_ratio'] if left else None,
        'right_dash_ratio':right['dash_ratio'] if right else None,
        'left_fragments':left['fragment_count'] if left else 0,
        'right_fragments':right['fragment_count'] if right else 0,
        'left_target_count':left['target_group_count'] if left else 0,
        'right_target_count':right['target_group_count'] if right else 0,
        'selected_lines':{'left':left,'right':right},
        'candidate_count':{'left':len(left_candidates),'right':len(right_candidates)},
        'detected_lines':sorted(candidates,key=lambda c:c['x_at_reference_y']),
        'decision_rule':'left<90 right>90; aggregate spatially aligned fragments; fragment_count>1 => dash, otherwise dash_ratio>0.03',
    }
