"""端到端模拟：把 run_live 真跑起来，逐个覆盖 nav / traffic / lane / 空 四条分支。

2026-10-29 上板事故是 `run_live → main_entry` 的参数名不匹配，整场 458 轮
每轮 TypeError。这个脚本不 mock 掉那一层，让真实的调用链跑起来，
只要参数名、类型、返回值对不上就会立刻炸。
"""
import sys
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np
import main as m
from protocol import Display, GREEN, RED

GRAY = np.full((720, 1280), 200, np.uint8)


def cop_logits(cls, strength=20.0):
    """模拟 cls_v2yrg3 的输出：0=类1(路口->左转) 1=类2(路段->直行) 2=other。"""
    v = np.full(3, -strength / 2, np.float32)
    v[cls] = strength / 2
    return v


class FakeFrame:
    def __init__(self, fid, pixels=None):
        self.pixels = GRAY if pixels is None else pixels
        self.frame_id = fid


class _Sock:
    def getsockopt(self, *a):
        return 0

    def close(self):
        pass


class FakeLink:
    instances = []
    pixels = None          # 由 run_with 设置：模拟这一轮 FPGA 回的 YRG 帧

    def __init__(self, *a, **k):
        self.pair_timing_ms = {'gray_ms': 10.0, 'pair_ms': 10.0}
        self.last_report = {'gray_unique_values': 194}
        self.last_frame_thresholded = False
        self.latest_frame_id = 0
        self.requested = []
        self.sock = _Sock()
        FakeLink.instances.append(self)

    def request_frame(self, display, timeout=2., on_request=None):
        """模拟 FPGA：只有 MODE 1（灰度），回真正的多级灰度。

        2026-10-31：二值通道已删除，这里只发一条 0x?4xx 指令。如果哪天
        run_live 又去请求二值，这个假 link 会因为签名对不上直接 TypeError。
        """
        self.requested.append('gray')
        self.latest_frame_id += 1
        if on_request:
            on_request(bytes([0, 4]), 0)
        self.last_report = {'gray_unique_values': 194}
        return FakeFrame(self.latest_frame_id, FakeLink.pixels)

    def configure_receiver(self, *a, **k):
        pass

    def close(self):
        pass


def make_det(scenes, nav=(), housing=(), lane_dash=False, cop=None):
    # cop 传 list 时按轮取用，用于"先 armed 再切 other"这类时序用例；
    # 取完之后返回 None，等价于模型掉线。单值则每轮都返回同一个 logits。
    seq = list(cop) if isinstance(cop, (list, tuple)) else None

    class D:
        last_model_stages = {}

        def scene(self, g):
            return [dict(b) for b in scenes]

        def navigation(self, g):
            return list(nav)

        def housing(self, g):
            return list(housing)

        def lane_mask(self, g):
            if not lane_dash:
                return np.zeros((432, 768), np.uint8)
            mask = np.zeros((432, 768), np.uint8)
            mask[150:380, 150:162] = 1
            return mask

        def cop_classify(self, g):
            """V28：交警三分类。None = 模型没装（等价于关闭指纹）。"""
            if seq is not None:
                if not seq:
                    return None
                v = seq.pop(0)
                return None if v is None else np.asarray(v, np.float32)
            return None if cop is None else np.asarray(cop, np.float32)

        def close(self):
            pass

    return D()


def box(cls, xyxy=(100, 500, 200, 600), score=.9):
    return {'class_name': cls, 'class_id': 0, 'score': score,
            'xyxy': list(xyxy)}


def cop_lane_field():
    """V26：A 段形态的车道框 —— 6 列 × 2 行，横跨 0.12~0.88、纵跨 0.34~0.64。

    报告 v2 / 3.4 节：A 段横向铺展 p50 = 0.689、纵向 0.262；普通马路
    横向只有 0.286。V25 的用例是 12 个**同一位置**的框（铺展 0），
    新判据会正确地判成"不是 A"，所以这里必须造出真实的铺展。
    """
    xs = (155, 345, 535, 725, 920, 1125)
    ys = (245, 460)
    return [box('lane', xyxy=(x, y, x + 40, y + 60))
            for x in xs for y in ys]


def cop_light_field(n=6):
    """V26：B 段形态 —— 6 个信号灯框（旧判据只看"有没有顶部真灯"）。"""
    return [box('traffic_light', xyxy=(200 + 150 * i, 50, 300 + 150 * i, 120),
                score=.84) for i in range(n)]


def run_with(scenes, nav=(), housing=(), lane_dash=False, rounds=2, extra=(),
             pixels=None, model=None):
    """model = 这一轮 cop_classify 返回的 logits；None = 模型没装。"""
    args = m.parser().parse_args(['--max-cycles', str(rounds), *extra])
    det = make_det(scenes, nav, housing, lane_dash, model)
    counter = {'n': 0}
    sent = []

    def fake_pair(*a, **k):
        counter['n'] += 1
        return FakeFrame(counter['n'], pixels), None

    def fake_save(d):
        sent.append(d)

    FakeLink.pixels = pixels if pixels is not None else GRAY
    with patch.object(m, 'UDPLink', FakeLink), \
         patch.object(m, 'build_models', lambda a: det), \
         patch.object(m, 'read_display', lambda: Display()), \
         patch.object(m, 'save_display', fake_save):
        m.run_live(args)
    run_with.sent = sent
    # 真实轮数以"每轮 save_display 调了几次"为准；fake_pair 从来不会被
    # run_live 调用（它直接用 link.request_frame），按它计数永远是 0 轮。
    return len(sent)


cases = [
    ('导航', dict(scenes=[box('nav')], nav=[{'class_name': 'right_2',
                                             'class_id': 1, 'score': .95,
                                             'xyxy': [120, 440, 250, 480]}])),
    ('红绿灯', dict(scenes=[box('traffic_light')],
                    housing=[[400, 150, 480, 190, .9]])),
    ('车道', dict(scenes=[box('lane')], lane_dash=True)),
    ('空场景', dict(scenes=[])),
    ('双标签', dict(scenes=[box('lane'), box('traffic_light')],
                    housing=[[400, 150, 480, 190, .9]])),
    # 交警手势：判据完全来自分类模型，场景框不再参与。
    # --cop-lock-rounds 1 让第 1 轮就 armed，三轮里必然出现交警显示。
    ('交警模型类1(路口→左转绿)', dict(scenes=[], rounds=3,
                                   extra=('--cop-lock-rounds', '1',
                                          '--scene-every', '1'),
                                   model=cop_logits(0),
                                   cop='left')),
    ('交警模型类2(路段→直行红)', dict(scenes=[], rounds=3,
                                   extra=('--cop-lock-rounds', '1',
                                          '--scene-every', '1'),
                                   model=cop_logits(1),
                                   cop='straight')),
    # 反向用例 1：模型说 other -> 永远不能进交警模式，
    # 哪怕场景框完全是 A 段铺满视野的车道线。
    ('模型other不进交警', dict(scenes=cop_lane_field(), rounds=4,
                            extra=('--cop-lock-rounds', '1',
                                   '--scene-every', '1'),
                            model=cop_logits(2),
                            cop=None)),
    # 反向用例 2：模型没装（cop_classify 返回 None）-> 同样不进。
    ('无交警模型不进交警', dict(scenes=cop_lane_field(), rounds=4,
                             extra=('--cop-lock-rounds', '1',
                                    '--scene-every', '1'),
                             model=None,
                             cop=None)),
    # 时序用例：先 armed，再连续 other，必须撤防回普通模式。
    # 用真实 run_live 链路验证"退出交警模式"这一段（单测只测到状态机）。
    ('交警armed后切other要撤防', dict(
        scenes=[box('lane')], rounds=9,
        extra=('--cop-lock-rounds', '1', '--cop-release-rounds', '2',
               '--scene-every', '1'),
        # 前 3 轮：类1 持续 -> 第 1 轮 armed；
        # 后 6 轮：other -> 连续 2 轮即撤防，之后不能还是单箭头
        model=[cop_logits(0)] * 3 + [cop_logits(2)] * 6,
        cop=('left_then_release', 4))),
    # V26 灯箱过滤：倒计时数字框不能占方向位。
    # 两个真灯箱 + 一个同高但更矮的数字框 -> 必须只认 2 框布局（左+直），
    # 而不是被数字挤成 3 框（左/直/右），更不能把数字当灯塔。
    ('灯箱：数字框不占方向位', dict(
        scenes=[box('traffic_light')],
        housing=[[200, 120, 280, 230, .9],      # h=110 真灯（左）
                 [700, 120, 780, 230, .9],      # h=110 真灯（直）
                 [430, 150, 462, 214, .5]])),   # h=64 数字框
    # 反向用例 3：普通路口（顶部一只真灯 + 4 条窄车道线）**不能** armed。
    # V25 就是在这类画面上误进入交警模式（板上实测 34.8%）。
    ('普通路口不应进交警', dict(scenes=([box('traffic_light',
                                          xyxy=(600, 27, 647, 118), score=.814)]
                                     + [box('lane', xyxy=(500 + 20 * i, 560,
                                                          540 + 20 * i, 700))
                                        for i in range(4)]),
                                  rounds=4,
                                  extra=('--cop-lock-rounds', '1',
                                         '--scene-every', '1'),
                                  model=cop_logits(2),
                                  cop=None)),
    # V28.5：类2（路段/直行）的单箭头必须是**红**，且仍然只有一个箭头。
    ('交警类2 直行红箭头', dict(scenes=[box('lane')], rounds=4,
                             extra=('--cop-lock-rounds', '1',
                                    '--scene-every', '1'),
                             model=[cop_logits(1)] * 4,
                             cop='straight')),
]


def assert_single_cop_arrow(direction):
    """交警模式必须**只有一个箭头**：FPGA 靠这个判定播报语音。

    V28.5 起按指纹分色：左转绿、直行红。
    """
    want = ((GREEN, None, None) if direction == 'left'
            else (None, RED, None) if direction == 'straight'
            else (None, None, GREEN))
    got = [(d.left, d.straight, d.right) for d in run_with.sent]
    if want not in got:
        raise AssertionError(f'交警分支没有产出期望的单 {direction} 箭头 {want}：{got}')
    # 反向也要成立：不能出现两个方向都有箭头（FPGA 会当成普通模式）。
    for l, s, r in got:
        filled = sum(x is not None for x in (l, s, r))
        if filled > 1:
            raise AssertionError(f'交警模式下出现了 {filled} 个箭头：{(l, s, r)}')


_ABSENT = object()


def _is_cop_arrow(triple):
    """单个箭头且只可能来自 cop：任意单个绿，或**直行位**的单个红。

    车道分支会合法地只亮一侧实线红（左/右），那不算交警模式。
    """
    l, s, r = triple
    if sum(x is not None for x in triple) != 1:
        return False
    return GREEN in triple or s == RED


def assert_no_cop_arrow():
    """开放世界反例：普通路口绝不能 armed 成交警模式。

    FPGA 看到"只有一个箭头"就直接播报交警语音，所以误触发就是演示事故。

    ⚠️ V28.2：只把"单个**绿**箭头"当作交警模式。正常 lane 分支的左右两侧
    可以独立为 None，合法地只发一个黄色/红色箭头；旧断言把这种正常单箭头
    也当成误进交警模式，会误伤新用例，也证明不了单箭头来自 cop 分支。
    ⚠️ V28.5：类2（直行）改成红箭头后，判据补一条"直行位的单个红"。
    """
    for d in run_with.sent:
        colours = (d.left, d.straight, d.right)
        if _is_cop_arrow(colours):
            raise AssertionError(
                f'普通路口误进入交警模式：单个交警箭头 {colours}')


def assert_cop_arrow_then_released(direction, tail_clean):
    """先出现交警单箭头，之后必须真的退出，且退出后不能再跳回去。

    `tail_clean` = 末尾应当已是普通模式的轮数（喂 other 的轮数减去撤防窗口）。
    """
    got = [(d.left, d.straight, d.right) for d in run_with.sent]
    if not got:
        raise AssertionError('一轮都没跑')
    # 只把"单个交警箭头"算作交警模式；正常 lane 分支可以合法地只有一个
    # 黄/红箭头（左右两侧独立），不能把它当成交警模式还没撤防。
    single = [_is_cop_arrow(t) for t in got]
    want = (GREEN, None, None) if direction == 'left' else (None, RED, None)
    if want not in got:
        raise AssertionError(f'从未进入交警模式，期望单 {direction} 箭头 {want}：{got}')
    if any(single[-tail_clean:]):
        raise AssertionError(
            f'切到 other 之后末尾 {tail_clean} 轮里仍有单箭头：{got[-tail_clean:]}')
    first_clear = next(i for i, s in enumerate(single) if not s)
    if any(single[first_clear:]):
        raise AssertionError(f'撤防（第 {first_clear + 1} 轮）之后又跳回单箭头：{got}')


failed = []
for name, kw in cases:
    cop = kw.pop('cop', _ABSENT)
    try:
        n = run_with(**kw)
        if cop is _ABSENT:
            pass
        elif cop is None:
            # 反向用例：任何一轮都不能出现"只有一个箭头"的交警显示。
            assert_no_cop_arrow()
            print('    已确认没有进入交警模式')
        elif isinstance(cop, tuple) and cop[0] == 'left_then_release':
            assert_cop_arrow_then_released('left', tail_clean=cop[1])
            print('    已确认：进入过交警模式，并在 other 后撤防')
        else:
            assert_single_cop_arrow(cop)
            print(f'    交警显示已确认：只有一个 {cop} 箭头'
                  f'（{"绿" if cop == "left" else "红"}）')
        print(f'  {name}: {n} 轮 通过')
    except Exception as exc:
        failed.append((name, type(exc).__name__, str(exc)))
        print(f'  {name}: 失败 {type(exc).__name__}: {exc}')

if failed:
    print('\n有分支跑不通：')
    for x in failed:
        print('  ', x)
    sys.exit(1)
print('\n所有分支都跑通了真实 run_live 调用链')
