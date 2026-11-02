"""Final RK3568 navigation, angle rollback, red-light and UDP integration checks."""
import sys,unittest,socket,threading,time,struct,hashlib
from pathlib import Path
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from main import parser,main_entry,capture_one_frame
from protocol import Display, RED, GREEN, BLUE, YELLOW, GRAY, BINARY, decode_command
from strategy_engine import decide_pair,eligible_navigation
from lane_stability import LaneSolidStreaks
from udp_pair import UDPLink
from test_v12_logic import Fake,b,G,B,hood
ROOT=Path(__file__).resolve().parents[1]

class FullStrategyTests(unittest.TestCase):
 def setUp(self):self.count=LaneSolidStreaks(3)
 def check(self,f):return decide_pair(G,f,lane_streaks=self.count,lane_backend='angle')
 def test_nav_valid_left_even_if_traffic_red(self):
  d,r=self.check(Fake([b('nav'),b('traffic_light')],[b('left_2')],[b('red')]))
  self.assertEqual(r['branch'],'navigation_blue');self.assertEqual((d.left,d.straight,d.right),(BLUE,None,None))
 def test_nav_three_digits_shows_blue_straight_not_traffic(self):
  d,r=self.check(Fake([b('nav'),b('traffic_light')],[b('right_3')],[b('red')]))
  self.assertEqual(r['branch'],'navigation_blue');self.assertEqual(d.straight,BLUE)
 def test_nav_two_digit_right_turn(self):
  d,r=self.check(Fake([b('nav')],[b('right_2')]))
  self.assertEqual(d.right,BLUE);self.assertIsNone(d.left)
 def test_uturn_card_falls_back_to_blue_straight(self):
  d,r=self.check(Fake([b('nav')],[b('uturn_2')]))
  self.assertEqual(r['branch'],'navigation_blue');self.assertEqual(d.straight,BLUE)
 def test_top_scoring_detection_decides_direction(self):
  # nav6 answers direction and distance in one label, so conflicting labels are
  # a decoding problem, not something to reconcile: the best box wins.
  f=Fake([b('nav')],[b('right_3',score=.95),b('left_2',score=.8)])
  d,r=self.check(f);self.assertEqual(d.straight,BLUE);self.assertIsNone(d.left)
  self.assertEqual(len(r['navigation']['readings']),1)
 def test_nav_no_detection_holds_previous_blue(self):
  from protocol import Display
  d,r=decide_pair(G,Fake([b('nav')],[]),previous_display=Display(right=BLUE),
                  lane_streaks=self.count,lane_backend='angle')
  self.assertEqual(d.right,BLUE);self.assertEqual(r['branch'],'navigation_hold')
 def test_four_aligned_lights_use_first_three(self):
  # Four lamp boxes in one row: left / straight / right are the first three,
  # anything further right is ignored rather than treated as unknown.
  G[:]=0
  d,r=self.check(Fake([b('traffic_light')],
                      traffic=[hood('red',x=i*90,w=60) for i in range(4)]))
  self.assertEqual((d.left,d.straight,d.right),(RED,RED,RED))
  self.assertEqual(r['traffic']['layout'],'left_straight_right')
 def test_two_lights_left_and_straight(self):
  # Two boxes -> left box is the left-turn light, right box the straight one.
  # Right turn has no lamp, so it is allowed by default.
  G[:]=0
  d,_=self.check(Fake([b('traffic_light')],
                      traffic=[hood('red',x=400,w=70),hood('green',x=700,w=70)]))
  self.assertEqual((d.left,d.straight,d.right),(RED,GREEN,GREEN))
 def test_straight_hidden_on_lane(self):
  with patch('strategy_engine.select_lanes',return_value={'left':{'kind':'solid'},'right':{'kind':'none'}}):
   d,_=self.check(Fake([b('lane')]))
  self.assertIsNone(d.straight)
 def test_three_solids_set_red_and_fourth_stays_red(self):
  with patch('strategy_engine.select_lanes',return_value={'left':{'kind':'solid'},'right':{'kind':'none'}}):
   for k in range(4):
    d,r=self.check(Fake([b('lane')]))
    self.assertEqual(d.left,RED if k>=2 else None)
  self.assertEqual(r['algorithm'],'v6_v7_angle')
 def test_dash_turns_green_after_confirmation(self):
  # 好版语义（V17_1 恢复）：虚线一帧即黄，不参与实线的确认计数。
  with patch('strategy_engine.select_lanes',return_value={'left':{'kind':'dash'},'right':{'kind':'none'}}):
   d,_=self.check(Fake([b('lane')]))
  self.assertEqual(d.left,YELLOW);self.assertIsNone(d.right)
 def test_intermittent_solid_count_resets(self):
  vals=iter(['solid','solid','dash','solid','solid','solid'])
  with patch('strategy_engine.select_lanes',side_effect=lambda *a,**kw:{'left':{'kind':next(vals)},'right':{'kind':'none'}}):
   outputs=[]
   for k in range(6):outputs.append(self.check(Fake([b('lane')]))[0].left)
  # 好版语义：虚线立即清零并变黄，实线重新从 1 数起，第六轮才凑到 3。
  self.assertEqual(outputs,[None,None,YELLOW,None,None,RED])
 def test_nav_breaks_solid_confirmation(self):
  with patch('strategy_engine.select_lanes',return_value={'left':{'kind':'solid'},'right':{'kind':'none'}}):
   self.check(Fake([b('lane')]));self.check(Fake([b('lane')]))
   self.check(Fake([b('nav')],[b('right_2')]))
   self.assertEqual(self.count.left,0)
   self.assertEqual(self.check(Fake([b('lane')]))[0].left,None)
 def test_no_target_clear_and_warning_independent(self):
  d,r=decide_pair(G,Fake([b('crosswalk')]),previous_display=Display(RED,GREEN,BLUE),lane_streaks=self.count)
  self.assertEqual(d,Display(edge_warning=True))
 def test_model_files_are_native_3568(self):
  paths=list((ROOT/'models').glob('*.rknn'))
  # 2026-11-01 V28：并入交警三分类 cls_v2yrg3，五个模型全部 FP16/rk3568。
  self.assertEqual(len(paths),5);self.assertTrue(all('3568' in p.name and 'fp16' in p.name for p in paths))
  self.assertFalse((ROOT/'models'/'A768_rk3568_fp16.rknn').exists(), 'A768 二值车道模型必须已删除')
 def test_v6_angle_files_byte_identical(self):
  old=ROOT.parent/'rebuild_3568'/'v6'
  if not old.is_dir():self.skipTest('Development copy not packaged')
  for name in ('lane_geometry.py','lane_decision.py'):
   self.assertEqual((ROOT/name).read_bytes(),(old/name).read_bytes())
 def test_no_fast_burst_or_rowscan_params(self):
  opts=parser().parse_args([])
  self.assertFalse(hasattr(opts,'lane_burst'))
  self.assertFalse(hasattr(opts,'lane_scan_step'))
 def test_two_image_input_contract(self):
  d,r=main_entry(G,Fake([b('nav')],[b('right_2')]))
  self.assertEqual(d.right,BLUE)

class NavModelFileTests(unittest.TestCase):
 def test_nav6_model_is_the_trained_six_class_build(self):
  m=ROOT/'models'/'nav6_720_rk3568_fp16.rknn'
  self.assertTrue(m.is_file())
  self.assertEqual(m.stat().st_size,8331907)
 def test_old_three_class_nav_model_is_gone(self):
  self.assertFalse((ROOT/'models'/'nav_arrow_720_rk3568_fp16.rknn').exists())

class TransportTest(unittest.TestCase):
 def test_one_real_udp_request_720_rows(self):
  service=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
  service.bind(('127.0.0.1',0));service.settimeout(4)
  received=[];exceptions=[]
  def serve():
   try:
    for seq in (1,):
     raw,peer=service.recvfrom(32)
     dd,mode=decode_command(raw)
     received.append((raw,mode))
     assert mode==GRAY,(seq,mode)
     row=bytes([14])*1280
     for y in range(720):
      service.sendto(struct.pack('>IHH',100+seq,y,720)+row,peer)
      if y%8==0:time.sleep(.0006)
   except Exception as e:exceptions.append(e)
  thread=threading.Thread(target=serve,daemon=True)
  thread.start()
  link=UDPLink('127.0.0.1',0,'127.0.0.1',service.getsockname()[1],
                '127.0.0.1',service.getsockname()[1]);link.configure_receiver(idle_timeout=1.,mode0_grace_ms=35.)
  try:
   a=parser().parse_args(['--pair-timeout','3','--max-retries','2','--retry-delay','0'])
   frame=capture_one_frame(link,Display(),a,cycle=1)
   self.assertEqual(frame.frame_id,101)
   self.assertEqual(int(frame.pixels[0,0]),14)
   self.assertEqual([x[1] for x in received],[GRAY])
   self.assertEqual(link.pair_timing_ms['pair_ms']>0,True)
  finally:link.close();service.close();thread.join(timeout=5)
  self.assertFalse(exceptions,str(exceptions))

if __name__=='__main__':unittest.main()
