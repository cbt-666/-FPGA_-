#!/usr/bin/env python3
"""Run on RK3568: check that the five RKNN models load and infer.

交警三分类（cls_v2yrg3）是可选件：模型文件不在时该项标 skipped，
不据此把整个自检判失败。

2026-10-31：全工程只请求 MODE 1 灰度。车道模型固定 G640，自检不再有
--gray-only / --lane-variant 分支；若有测试输入落在 {0,253}，只是说明
自检图退化了，与本工程是否"在二值化"无关。"""
import json,sys,time,traceback
import numpy as np
from main import parser,build_models

def main():
 # start_v12.sh 只传 --lane-model（固定 G640），其余参数原样透传。
 args=parser().parse_args(['--require-all-models',*sys.argv[1:]])
 detectors=build_models(args)
 gray=np.zeros((720,1280),np.uint8)
 gray[:,::80]=200          # a few stripes so every branch sees non-trivial input
 results={}
 try:
  for name,fn in [('scene',lambda:detectors.scene(gray)),
                  ('navigation',lambda:detectors.navigation(gray)),
                  ('housing',lambda:detectors.housing(gray)),
                  ('lane',lambda:detectors.lane_mask(gray)),
                  ('cop',lambda:detectors.cop_classify(gray))]:
   start=time.perf_counter()
   try:
    r=fn()
    # 交警三分类是可选件：模型文件不在时 cop_classify 返回 None，
    # 这不是故障（交警分支会自动保持未触发），标 skipped 而不是失败。
    if name=='cop' and 'cop' not in detectors.paths:
     results[name]={'ok':True,'skipped':'交警模型未安装，交警分支不会触发'}
     continue
    results[name]={'ok':True,'wall_ms':round((time.perf_counter()-start)*1000,2),
                   'rknn_ms':detectors.get('nav' if name=='navigation' else name).last_infer_ms,
                   'result_len':len(r) if isinstance(r,list) else None,
                   'result_shape':list(r.shape) if isinstance(r,np.ndarray) else None}
   except Exception as ex:
    results[name]={'ok':False,'error':repr(ex),'traceback':traceback.format_exc()}
  if 'cop' in detectors.paths:
   import cop_traffic_signature as cts
   results['cop_contract']={'ok':True,
                            'value':{'img':cts.IMG_SIZE,
                                     'conf_min_by_kind':dict(cts.CONF_MIN_BY_KIND),
                                     'lock_rounds_by_kind':dict(cts.LOCK_ROUNDS_BY_KIND),
                                     # V28.5：类1 左转=绿(1)、类2 直行=红(0)
                                     'colour_by_kind':dict(cts.KIND_TO_COLOUR),
                                     'classes':list(cts.CLASS_TO_KIND.values())}}
  # nav6 fused the arrow detector and the template digit reader into one head,
  # so there is no separate OCR asset to verify any more.
  import models_runtime
  results['nav_classes']={'ok':True,'value':list(models_runtime.NAV_CLASSES)}
  results['lane_variant']={'ok':True,'value':detectors.lane_variant}
 finally:detectors.close()
 print(json.dumps(results,ensure_ascii=False,indent=2))
 if not all(v['ok'] for v in results.values()):raise SystemExit(1)

if __name__=='__main__':main()
