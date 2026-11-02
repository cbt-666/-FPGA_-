import unittest
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
import cv2
from lane_decision import decode, _select_group


def line(xref, angle, y0,y1,dash,solid):
    return {'angle_deg':angle,'x_at_reference_y':xref,'reference_y':576,
            'y_range':[y0,y1],'dash_pixels':dash,'solid_pixels':solid,
            'line_pixels':dash+solid,'dash_ratio':dash/(dash+solid), 'fragment_count':1}


class TargetTest(unittest.TestCase):
    def choose(self,cands):
        return _select_group(cands,.03,3,42,720*.45,16,1280)

    def test_85_84_aggregate_not_pick_one(self):
        r=self.choose([line(560,85,360,450,20,80),line(589,84,520,640,70,30)])
        self.assertEqual(r['target_group_count'],2)
        self.assertEqual(r['dash_pixels'],90)
        self.assertEqual(r['solid_pixels'],110)
        self.assertEqual(r['dash_ratio'],.45)
        self.assertEqual(r['class'],'dash')
        self.assertEqual(r['anchor_angle_deg'],85)

    def test_nonadjacent_parallel_not_merged(self):
        r=self.choose([line(560,85,360,450,0,100),line(710,84,520,640,100,0)])
        self.assertEqual(r['target_group_count'],1)
        self.assertEqual(r['class'],'solid')

    def test_overlapping_parallel_not_merged(self):
        r=self.choose([line(560,85,350,650,0,100),line(590,84,360,640,100,0)])
        self.assertEqual(r['target_group_count'],1)

    def test_exactly_3_percent_single_piece_solid(self):
        r=self.choose([line(560,85,360,450,3,97)])
        self.assertEqual(r['class'],'solid')
        self.assertEqual(r['target_line_count'],1)
        self.assertEqual(r['decision_basis'],'pixel_ratio')

    def test_just_above_3_percent_dash(self):
        # 严格大于3%，相同线的两个候选统一按总像素判断。
        r=self.choose([line(560,85,360,450,4,96),line(589,84,520,640,4,96)])
        self.assertEqual(r['target_group_count'],2)
        self.assertEqual(r['dash_ratio'],.04)
        self.assertEqual(r['class'],'dash')

    def test_15_percent_is_dash_at_default_threshold(self):
        # 此案例在旧版20%会判实线；新版3%应判虚线。
        from lane_geometry import select_lanes
        m=np.zeros((720,1280),np.uint8)
        cv2.line(m,(490,350),(490,650),2,3)
        cv2.line(m,(490,350),(490,395),1,3)
        r=select_lanes(m)
        self.assertEqual(r['left']['kind'],'dash',r)
        self.assertGreater(r['left']['dash_ratio'],.03)
        self.assertLess(r['left']['dash_ratio'],.20)
        self.assertEqual(r['right']['kind'],'none')

    def test_can_override_threshold_for_comparison(self):
        from lane_geometry import select_lanes
        m=np.zeros((720,1280),np.uint8)
        cv2.line(m,(490,350),(490,650),2,3)
        cv2.line(m,(490,350),(490,395),1,3)
        self.assertEqual(select_lanes(m)['left']['kind'],'dash')
        self.assertEqual(select_lanes(m,ratio_threshold=.20)['left']['kind'],'solid')

    def test_5_percent_single_piece_new_vs_old_7_percent(self):
        # 仅单段才走比例，5% 大于3%而不大于7%。
        pieces=[line(560,85,360,450,5,95)]
        new=self.choose(pieces)
        old=_select_group(pieces,.07,3,42,720*.45,16,1280)
        self.assertEqual(new['target_group_count'],1)
        self.assertEqual(new['dash_ratio'],.05)
        self.assertEqual(new['class'],'dash')
        self.assertEqual(old['class'],'solid')

    def test_3_percent_strict_and_4_percent_single_piece(self):
        a=self.choose([line(560,85,360,450,3,97)])
        b=self.choose([line(560,85,360,450,4,96)])
        self.assertEqual(a['dash_pixels'],3)
        self.assertEqual(a['solid_pixels'],97)
        self.assertEqual(a['class'],'solid')
        self.assertEqual(b['class'],'dash')

    def test_adapter_and_decode_defaults_are_three_percent(self):
        import inspect
        from lane_geometry import select_lanes
        self.assertEqual(inspect.signature(decode).parameters['dash_threshold'].default,.03)
        self.assertEqual(inspect.signature(select_lanes).parameters['ratio_threshold'].default,.03)

    def test_angle_out_of_tolerance_not_merged(self):
        r=self.choose([line(560,88,360,450,0,100),line(570,84,520,640,100,0)])
        self.assertEqual(r['target_group_count'],1)

    def test_too_distant_y_not_merged(self):
        r=self.choose([line(560,85,0,50,0,100),line(568,84,500,600,100,0)])
        self.assertEqual(r['target_group_count'],1)

    def test_integration_left_right(self):
        # Left 85deg solid anchor, 84deg dash at displaced position; right 102deg solid.
        m=np.zeros((720,1280),np.uint8)
        def draw(xref,angle,y0,y1,cls):
            slope=-1/np.tan(np.deg2rad(angle))
            p=lambda y:(int(round(xref+slope*(y-576))),y)
            cv2.line(m,p(y0),p(y1),int(cls),3)
        draw(550,85,340,425,2)
        draw(582,84,510,650,1)
        draw(780,102,340,650,2)
        res=decode(m)
        self.assertEqual(res['left'],'dash',res)
        self.assertEqual(res['right'],'solid',res)
        self.assertEqual(res['left_target_count'],2,res)
        self.assertLess(res['selected_lines']['left']['anchor_angle_deg'],90)
        self.assertGreater(res['selected_lines']['right']['anchor_angle_deg'],90)

    def test_empty(self):
        r=decode(np.zeros((720,1280),np.uint8))
        self.assertEqual((r['left'],r['right']),('none','none'))

    def test_invalid_labels(self):
        with self.assertRaises(ValueError):decode(np.array([[3]],np.uint8))

    def test_vertical_deadzone_by_side(self):
        m=np.zeros((720,1280),np.uint8)
        cv2.line(m,(490,345),(490,670),1,3)
        cv2.line(m,(830,345),(830,670),2,3)
        r=decode(m)
        self.assertEqual((r['left'],r['right']),('dash','solid'))


if __name__=='__main__':unittest.main()
