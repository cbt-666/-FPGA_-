#!/usr/bin/env python3
"""解压已下载的 Kaggle 数据集，并统计内容与标注格式"""
import zipfile, os, glob, collections, sys

JOBS = [
    (r'D:\ADAS_Datasets\04_交通锥路障_cone\_raw_obstacle.zip',
     r'D:\ADAS_Datasets\04_交通锥路障_cone\ObstacleDetection'),
    (r'D:\ADAS_Datasets\02_行人_pedestrian\_raw_widerperson.zip',
     r'D:\ADAS_Datasets\02_行人_pedestrian\WiderPerson'),
    (r'D:\ADAS_Datasets\02_行人_pedestrian\_raw_lane_pedestrian.zip',
     r'D:\ADAS_Datasets\02_行人_pedestrian\LanePedestrian'),
    (r'D:\ADAS_Datasets\04_交通锥路障_cone\_raw_cones5k.zip',
     r'D:\ADAS_Datasets\04_交通锥路障_cone\TrafficCones5k'),
]

for src, dst in JOBS:
    if not os.path.exists(src):
        print('跳过(不存在):', os.path.basename(src)); continue
    if os.path.exists(dst) and any(os.scandir(dst)):
        print('已解压:', os.path.basename(dst)); continue
    print('\n解压 %s ...' % os.path.basename(src))
    try:
        z = zipfile.ZipFile(src)
        z.extractall(dst)
        ns = z.namelist()
        ext = collections.Counter(n.rsplit('.', 1)[-1].lower()
                                  for n in ns if '.' in n)
        print('  条目 %d  类型 %s' % (len(ns), dict(ext.most_common(8))))
        # 找标注文件
        for kw, label in [('yaml', 'data.yaml'), ('json', 'JSON'),
                          ('txt', 'YOLO txt'), ('xml', 'VOC xml')]:
            hits = [n for n in ns if n.lower().endswith('.' + kw)]
            if hits:
                print('  %-10s %d 个，例: %s' % (label, len(hits), hits[0][:70]))
        # 顶层目录
        tops = collections.Counter(n.split('/')[0] for n in ns)
        print('  顶层:', dict(tops.most_common(6)))
    except Exception as e:
        print('  解压失败:', type(e).__name__, e)

print('\n=== 完成 ===')
