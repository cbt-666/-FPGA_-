"""RK3568 V12 mocked full-decision and model contract tests. No board required."""
import unittest,sys,struct
from pathlib import Path
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from protocol import Display,RED,GREEN,BLUE,YELLOW,GRAY,BINARY,encode_packet,decode_command
from main import parser,main_entry,request_one_frame
from strategy_engine import decide_pair
from lane_stability import LaneSolidStreaks
import traffic_rules
from udp_pair import FrameAssembler,is_binary
from vision_common import decode_yolo
import models_runtime

G=np.zeros((720,1280),dtype=np.uint8)
B=np.zeros((720,1280),dtype=np.uint8)

# Housing boxes are [x1,y1,x2,y2,score]; traffic_rules.judge counts YRG 254
# (red) / 255 (green) pixels inside the box, so `hood` paints them into G.
#
# V27：灯箱里的灯珠只占**长边的中间一格**（板端抓包实测），所以这里也按
# 那个形态涂 —— 整框涂满会被"长边三等分颜色集中度"当成数字/噪声滤掉。
def hood(colour,x=400,y=150,w=80,h=40,score=.9):
 value={'red':254,'green':255}.get(colour)
 if value is not None:
  if w>=h:
   x0=x+w/3.0; x1=x+2*w/3.0; y0,y1=y,y+h
  else:
   y0=y+h/3.0; y1=y+2*h/3.0; x0,x1=x,x+w
  G[int(max(y0,0)):int(min(y1,720)),
    int(max(x0,0)):int(min(x1,1280))]=value
 return [float(x),float(y),float(x+w),float(y+h),score]

def b(cls,x=120,y=580,score=.9):
 return {'class_name':cls,'class_id':0,'score':score,'xyxy':[x,y,x+45,y+60]}

class Fake:
 def __init__(self,sc=(),nav=(),traffic=(),mask=None):
  self.sc=list(sc);self.nav=list(nav);self.traffic=list(traffic)
  self.mask=mask if mask is not None else B
  self.calls=[]
 def scene(self,im):self.calls.append('scene');return self.sc
 def navigation(self,im):self.calls.append('nav');return self.nav
 def housing(self,im):self.calls.append('housing');return self.traffic
 def lane_mask(self,im):self.calls.append('lane');return self.mask

class TestPriority(unittest.TestCase):
 def setUp(self):self.s=LaneSolidStreaks(3)
 def do(self,f):return decide_pair(G,f,lane_streaks=self.s,lane_backend='angle')
 def test_nav_blue_left_above_red_traffic(self):
  f=Fake([b('nav'),b('traffic_light'),b('lane')],[b('left_2')],[hood('red')])
  d,r=self.do(f);self.assertEqual(d.left,BLUE);self.assertIsNone(d.straight);self.assertIsNone(d.right)
  self.assertEqual(r['branch'],'navigation_blue');self.assertEqual(f.calls,['scene','nav'])
 def test_nav_blue_right(self):
  d,r=self.do(Fake([b('nav')],[b('right_2')]))
  self.assertEqual(d.right,BLUE);self.assertIsNone(d.left)
 def test_nav_3digits_shows_blue_straight_never_traffic(self):
  # A NAV frame must never fall through to the traffic branch: the screen is
  # showing a route, so a stale junction light must not win.
  f=Fake([b('nav'),b('traffic_light')],[b('left_3')],[hood('green',x=600)])
  d,r=self.do(f);self.assertEqual(r['branch'],'navigation_blue');self.assertEqual(d.straight,BLUE)
  self.assertEqual(f.calls,['scene','nav'])
 def test_nav_no_detection_defaults_straight(self):
  # nav6 answers both questions in one head, so a total miss has no OCR to
  # wait on: the branch still owes the operator a blue arrow.
  f=Fake([b('nav')],[])
  d,r=self.do(f);self.assertEqual(r['branch'],'navigation_blue');self.assertEqual(d.straight,BLUE)
 def test_two_digit_turn_shows_the_turn(self):
  f=Fake([b('nav')],[b('left_2')])
  d,r=self.do(f);self.assertEqual(d.left,BLUE);self.assertEqual(r['branch'],'navigation_blue')
 def test_uturn_has_no_enable_bit_falls_back_to_straight(self):
  f=Fake([b('nav')],[b('uturn_2')])
  d,r=self.do(f);self.assertEqual(r['branch'],'navigation_blue');self.assertEqual(d.straight,BLUE)
 def test_nav_scene_always_lights_an_arrow(self):
  d,r=self.do(Fake([b('nav')],nav=[]))
  self.assertEqual(r['branch'],'navigation_blue');self.assertEqual(d.straight,BLUE)
 def test_no_targets_clear_previous(self):
  d,r=decide_pair(G,Fake(),previous_display=Display(RED,GREEN,BLUE,True),lane_streaks=self.s)
  self.assertEqual(d,Display());self.assertEqual(r['branch'],'none')
 def test_crosswalk_warning_only(self):
  d,r=self.do(Fake([b('crosswalk')]));self.assertTrue(d.edge_warning);self.assertIsNone(d.left)
 def test_person_lower_center_warning_only(self):
  d,_=self.do(Fake([b('person',x=550,y=550)]));self.assertTrue(d.edge_warning)
 def test_person_upper_no_warning(self):
  d,_=self.do(Fake([b('person',x=550,y=10)]));self.assertFalse(d.edge_warning)
 def test_traffic_single_red_lights_left_and_straight(self):
  # 2026-10-28 用户规则：一个框时颜色同时代表左转和直行，右转默认绿。
  # 旧版给 left=None，屏幕上左边永远不亮，用户看到的就是"左边不显示"。
  G[:]=0
  d,r=self.do(Fake([b('traffic_light')],traffic=[hood('red')]))
  self.assertEqual((d.left,d.straight,d.right),(RED,RED,GREEN))
 def test_traffic_three_by_x(self):
  # Three housings, left to right = left turn / straight / right turn.
  G[:]=0
  d,r=self.do(Fake([b('traffic_light')],
                   traffic=[hood('red',x=400,w=60),hood('green',x=700,w=60),
                            hood('red',x=1000,w=60)]))
  self.assertEqual((d.left,d.straight,d.right),(RED,GREEN,RED))
 def test_traffic_two_housings_left_and_straight(self):
  # Two housings, left to right = left turn / straight. Right turn has no lamp.
  G[:]=0
  d,r=self.do(Fake([b('traffic_light')],
                   traffic=[hood('red',x=450,w=70),hood('green',x=760,w=70)]))
  self.assertEqual((d.left,d.straight,d.right),(RED,GREEN,GREEN))
 def test_traffic_zero_lights_clears(self):
  # 2026-10-29：无框 -> 走 traffic 分支直接清零（用户规则："无框那就不输出，
  # 同时清零上一次箭头"）。旧版返回 traffic_hold 沿用上一轮灯色。
  d,r=self.do(Fake([b('traffic_light')]))
  self.assertEqual(r['branch'],'traffic')
  # 红绿灯模式必须永远三个箭头：无框时红灯兜底，不是空屏。
  self.assertEqual((d.left,d.straight,d.right),(RED,RED,RED))
 def test_traffic_scattered_boxes_are_all_used(self):
  # Boxes 900 px apart used to be reduced to the single most confident one.
  # Box count is what the mapping reads, so all three are kept in x order.
  G[:]=0
  f=Fake([b('traffic_light')],
         traffic=[hood('red',x=200,w=60,score=.9),hood('red',x=320,w=60,score=.85),
                  hood('green',x=1150,w=60,score=.6)])
  d,r=self.do(f)
  self.assertEqual(r['traffic']['layout'],'left_straight_right')
  self.assertEqual((d.left,d.straight,d.right),(RED,RED,GREEN))
 def test_traffic_off_is_stop_not_green(self):
  # 2026-10-31 V26 用户拍板：没亮的框不占方向位。
  # 旧规则是"off -> 左右直按红灯处理"，但那会让一个没亮的灯箱凭空禁止直行
  # （板端 seq7 实测：真值只有红左转，被读成"左转红+直行红"）。
  # 现在 off 框被 traffic_boxes 以 'unlit' 拒掉 -> 无框 -> 走 NoBoxHold 兜底。
  G[:]=0
  d,r=self.do(Fake([b('traffic_light')],traffic=[hood('off')]))
  # 无框兜底仍是三红（红绿灯永远三个箭头），但不能再是 1 框布局的
  # "左红+直红+右绿" —— 那等于拿一个没亮的框当中央灯去解释方向。
  self.assertEqual((d.left,d.straight,d.right),(RED,RED,RED))
  self.assertNotEqual(r['traffic']['layout'],'single_all_three')
 def test_lane_three_solids_then_red(self):
  f=Fake([b('lane')]);disp=Display()
  with patch('strategy_engine.select_lanes',return_value={'left':{'kind':'solid'},'right':{'kind':'none'}}):
   for k in range(3):
    disp,r=decide_pair(G,f,previous_display=disp,lane_streaks=self.s,
                       lane_backend='angle')
    self.assertEqual(disp.left,RED if k==2 else None)
  self.assertEqual(self.s.left,3)
 def test_lane_dash_is_green_and_resets(self):
  f=Fake([b('lane')]);self.s.left=2
  # 好版语义（V17_1 恢复）：虚线立即变黄，只有实线需要连续确认。
  with patch('strategy_engine.select_lanes',return_value={'left':{'kind':'dash'},'right':{'kind':'none'}}):
   d,r=self.do(f)
  self.assertEqual(d.left,YELLOW);self.assertEqual(self.s.left,0)
 def test_lane_missing_straight_hidden(self):
  f=Fake([b('lane')]);d,r=self.do(f)
  self.assertEqual(d,Display());self.assertEqual(r['branch'],'lane_hold')
 def test_warning_with_valid_nav(self):
  d,r=self.do(Fake([b('nav'),b('crosswalk')],[b('right_2')]))
  self.assertEqual(d.right,BLUE);self.assertTrue(d.edge_warning)
 def test_no_nav_scene_does_not_touch_nav_detector(self):
  # nav6 carries no state between rounds, so there is nothing to reset.
  f=Fake([b('crosswalk')]);self.do(f);self.assertNotIn('nav',f.calls)
 def test_scene_validation_on_main_entry(self):
  with self.assertRaises(ValueError):main_entry(np.zeros((1,1),np.uint8),Fake())
 def test_full_packet_next_gray(self):
  d,r=main_entry(G,Fake([b('nav')],[b('left_2')]))
  dd,mode=decode_command(encode_packet(d,GRAY));self.assertEqual(dd.left,BLUE);self.assertEqual(mode,GRAY)

class TestWireAndArgs(unittest.TestCase):
 def test_16bit_blue(self):self.assertEqual(encode_packet(Display(left=BLUE)),b'\x21\x04')
 def test_16bit_rejects_binary_mode(self):
  # 2026-10-31：RK 端只发 MODE 1。BINARY 必须被协议层直接拒绝。
  with self.assertRaises(ValueError):encode_packet(Display(),BINARY)
 def test_rk3568_model_defaults(self):
  args=parser().parse_args([])
  for name in ('scene_model','nav_model','traffic_model','lane_model'):
   s=str(getattr(args,name));self.assertIn('3568',s);self.assertIn('fp16',s)
  # 2026-10-29 V20：实线确认 1 -> 2（V18 取 1 后用户反馈"车道跳动频繁"）。
  # 不要改成 3：3 帧在车道/红绿灯交替时凑不满，箭头会消失。
  self.assertEqual(args.solid_confirm_frames,2)
 def test_models_exist(self):
  from main import model_paths
  d=model_paths(parser().parse_args(['--require-all-models']))
  # 2026-11-01 V28：四件套 + 交警三分类（cop 是可选件，但本工程随包发布）。
  self.assertEqual(len(d),5)
  self.assertIsNotNone(d['cop'],'交警三分类模型必须随包存在')
 def test_udp_rows(self):
  f=FrameAssembler();completed=None
  for row in range(720):
   completed=f.add(struct.pack('>IHH',10,row,720)+bytes([row%255])*1280)
  self.assertEqual(completed.frame_id,10)
 def test_binary_check(self):
  # 2026-10-31：允许 ≤0.1% 杂散像素，免得一个坏点把整帧判废（2s 超时 ×2 → 整轮作废）。
  self.assertTrue(is_binary(B))
  one_bad=B.copy();one_bad[2,2]=255
  self.assertTrue(is_binary(one_bad), '单个杂散像素不该丢掉整帧')
  many_bad=B.copy();many_bad[:8,:]=255            # 8行×1280 ≈ 1.1%
  self.assertFalse(is_binary(many_bad), '真正串了模式必须拒绝')

class FakeHandle:
 calls=[]
 def __init__(self,path,core_mask='auto'):
  self.path=str(path);self.last_infer_ms=1;self.core_mask=core_mask
 def run(self,t,**kw):
  self.calls.append((Path(self.path).name,t.shape,t.dtype,kw,float(t.max())))
  if 'merged5' in self.path:return [np.zeros((1,4,21504),np.float32),np.zeros((1,5,21504),np.float32)]
  if 'nav6' in self.path:return [np.zeros((1,4,18920),np.float32),np.zeros((1,6,18920),np.float32)]
  if 'housing' in self.path:return [np.zeros((1,4,20160),np.float32),np.zeros((1,1,20160),np.float32)]
  if 'G640' in self.path:
   h,w=360,640
   x=np.zeros((1,3,h,w),np.float32);x[0,2,:,:]=1;return [x]
  raise RuntimeError('unknown model')
 def close(self):pass

class TestInputOutputContracts(unittest.TestCase):
 def setUp(self):
  FakeHandle.calls=[]
  pp=patch.object(models_runtime,'RKNNHandle',FakeHandle);pp.start();self.addCleanup(pp.stop)
  self.m=models_runtime.LazyDetectors('scene.rknn','nav.rknn','housing.rknn','lane.rknn')
  # lazy model selection requires recognizable file names for FakeHandle
  self.m.paths={k:Path(v) for k,v in {'scene':'merged5gray_rk3568_fp16.rknn','nav':'nav6_720_rk3568_fp16.rknn','housing':'housing_3568_fp16.rknn','lane':'G640_rk3568_fp16.rknn'}.items()}
  self.m.loaded={}
 def test_scene_fp16_single_1024_float(self):
  out=self.m.scene(G);self.assertEqual(out,[])
  name,shape,dtype,kw,mm=FakeHandle.calls[-1]
  self.assertEqual(shape,(1,1024,1024,3));self.assertEqual(dtype,np.float32)
  self.assertEqual(kw['data_format'],['nhwc'])
 def test_nav_720_float32_18920(self):
  out=self.m.navigation(G);self.assertEqual(out,[])
  self.assertEqual(FakeHandle.calls[-1][1],(1,720,1280,3))
  self.assertEqual(FakeHandle.calls[-1][2],np.float32)
 def test_housing_768x1280_float32_direct_xywh(self):
  self.assertEqual(self.m.housing(G),[])
  shape=self.call_shape('housing')
  self.assertEqual(shape,(1,768,1280,3))
  self.assertEqual(FakeHandle.calls[-1][2],np.float32)
 def test_housing_box_head_is_not_stride_scaled(self):
  # The head is a direct cx,cy,w,h in network pixels. A 40x20 box must come
  # back as a 40x20 box (only the 768->720 height ratio is undone).
  class One(FakeHandle):
   def run(self,t,**kw):
    self.calls.append((Path(self.path).name,t.shape,t.dtype,kw,float(t.max())))
    b=np.zeros((1,4,20160),np.float32);b[0,:,0]=(100,200,40,20)
    c=np.zeros((1,1,20160),np.float32);c[0,0,0]=0.9
    return [b,c]
  with patch.object(models_runtime,'RKNNHandle',One):
   m=models_runtime.LazyDetectors('s.rknn','n.rknn','housing.rknn','lane.rknn')
   dets=m.housing(G)
  self.assertEqual(len(dets),1)
  x1,y1,x2,y2,s=dets[0]
  self.assertAlmostEqual(x2-x1,40,places=3)
  self.assertAlmostEqual(y2-y1,20*720/768,places=3)
 def test_lane_gray640_contract(self):
  mask=self.m.lane_mask(G)
  self.assertEqual(mask.shape,(360,640));self.assertTrue(np.all(mask==2))
  self.assertEqual(self.call_shape('lane'),(1,1,360,640))
  self.assertEqual(FakeHandle.calls[-1][3]['data_format'],['nchw'])
 def call_shape(self,name):
  for c in reversed(FakeHandle.calls):
   if name in c[0] or {'scene':'merged5','nav':'nav6','housing':'housing',
                       'lane':'G640'}[name] in c[0]:
    return c[1]
  raise AssertionError(f'no call for {name}')
 def test_scene_single_output_accepted(self):
  # FP16 build returns ONE tensor [1,9,21504]; it must decode, not raise.
  self.assertEqual(self.m.scene(G),[])
 def test_fpga_yrg_to_nav_gray_preserves_luminance(self):
  g=G.copy();g[5,5]=255;self.m.navigation(g)
  self.assertEqual(FakeHandle.calls[-1][-1],149)

if __name__=='__main__':unittest.main()
