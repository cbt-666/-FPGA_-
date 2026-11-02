#!/usr/bin/env python3
# coding: utf-8
"""Create five synthetic 0/1/2 lane masks for offline software validation only."""
from pathlib import Path
import argparse
import cv2
import numpy as np


def put(m, xref, angle, y0, y1, label=1, thickness=3):
    slope=-1/np.tan(np.deg2rad(angle))
    p=lambda y:(int(round(xref+slope*(y-576))),int(y))
    cv2.line(m,p(y0),p(y1),int(label),int(thickness))


def main():
    p=argparse.ArgumentParser(description='生成5张合成的标签掩码(非RKNN模型输出)')
    p.add_argument('--output-dir',type=Path,default=Path('demo_masks'))
    args=p.parse_args()
    args.output_dir.mkdir(parents=True,exist_ok=True)
    for i in range(5):
        m=np.zeros((720,1280),np.uint8)
        put(m,550+i*3,85,340,425,2)
        put(m,582+i*3,84,510,650,1)
        put(m,780-i*2,102,340,650,2)
        np.save(args.output_dir/f'mask_{i+1:02d}.npy',m,allow_pickle=False)
    print('已生成5张合成mask，非NPU实测数据:',args.output_dir.resolve())


if __name__=='__main__':main()
