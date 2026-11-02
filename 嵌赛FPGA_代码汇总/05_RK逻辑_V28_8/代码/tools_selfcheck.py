"""启动前自检：每个模块都能 import，且真实调用链的参数名对得上。

直接用工程里的 python3 跑（板上是 python3，本机是 Windows 的 Python）。
有问题就非零退出，让 run.sh 停在启动之前。
"""
import importlib
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

CORE = ['protocol', 'udp_pair', 'models_runtime', 'strategy_engine', 'lane_v2',
        'lane_stability', 'traffic_rules', 'resource_probe', 'net_recovery',
        'mem_guard', 'restart_runtime', 'hang_watchdog', 'traffic_boxes',
        'cop_traffic_signature']
# live_watchdog 只用 fcntl，是 Linux 专属；本机自检时跳过。
if sys.platform.startswith('linux'):
    CORE.append('live_watchdog')

failed = []

for name in CORE:
    try:
        importlib.import_module(name)
    except Exception as exc:
        failed.append(f'{name}: {type(exc).__name__}: {exc}')

# 关键接口的参数名（上板踩过的那个组合）
try:
    import inspect
    import main
    import strategy_engine
    for fn, mod, expected in (
            (main.main_entry, 'main', ('last_good', 'last_good_hold',
                                       'last_good_traffic_hold', 'scene_retries',
                                       'empty_streak', 'frame_is_thresholded')),
            (strategy_engine.decide_pair, 'strategy_engine',
             ('last_good', 'last_good_hold', 'last_good_traffic_hold',
              'scene_retries', 'empty_streak', 'lane_band_top'))):
        params = set(inspect.signature(fn).parameters)
        missing = [p for p in expected if p not in params]
        if missing:
            failed.append(f'{mod}.{fn.__name__} 缺参数: {", ".join(missing)}')
    # argparse 里声明的开关必须真的存在，否则 run.sh 传进去会直接退出
    args = main.parser().parse_args([])
    for attr in ('last_good_hold', 'last_good_traffic_hold', 'scene_retries',
                 'scene_retry_window', 'mem_trim_every', 'mem_restart_every',
                 'lane_band_top', 'lane_band_bot'):
        if not hasattr(args, attr):
            failed.append(f'命令行缺少 --{attr.replace("_", "-")}')
    # 灰度契约：这两条只要有任何一个反弹，就是二值通道又回来了
    for fn, mod, forbidden in ((main.main_entry, 'main', ('binary',)),
                               (strategy_engine.decide_pair, 'strategy_engine',
                                ('binary', 'lane_variant', 'gray_is_binary'))):
        present = [p for p in forbidden
                   if p in inspect.signature(fn).parameters]
        if present:
            failed.append(f'{mod}.{fn.__name__} 又出现二值参数: '
                          f'{", ".join(present)}（用户要求「不再请求二值化」）')
    # 车道模型只认灰度。run.sh 里若还写着 A768，模型文件根本不存在。
    args2 = main.parser().parse_args([])
    if 'A768' in str(getattr(args2, 'lane_model', '')):
        failed.append('main.py 默认车道模型仍是 A768 —— 它已被删除，车道只做灰度')
except Exception as exc:
    failed.append(f'接口自检异常: {type(exc).__name__}: {exc}')

# ⚠️ 2026-10-31：二值通道整个删除，守门断言反过来写。
# 以前守的是"A768 必须真能拿到二值帧"（v18 板上整段失明）。现在用户要求
# 「不再请求二值化」，所以守的是**运行时不得出现任何二值请求**。
# 断言只扫运行期文件，不扫 tests/ —— 测试里出现旧名字是历史证据，不是回归。
RUNTIME_PY = ('main.py', 'udp_pair.py', 'strategy_engine.py', 'models_runtime.py',
              'lane_v2.py', 'protocol.py', 'model_selfcheck.py',
              'fpga_pair_preflight.py', 'simulate_fpga.py', 'live_watchdog.py',
              'cop_traffic_signature.py')
FORBIDDEN_SNIPPETS = (
    ('BINARY)', '请求 MODE 2 的指令编码'),
    ("'binary'", '二值模式字符串'),
    ('bin768', '二值车道变体'),
    ('prepare_binary', 'RK 端二值预处理'),
    ('prearm(', '轮末预置下一轮模式'),
    ('capture_sequential_pair', '双请求采集'),
    ('gray_is_binary', '二值帧标志沿用旧名'),
)
try:
    for name in RUNTIME_PY:
        path = ROOT / name
        if not path.is_file():
            failed.append(f'运行期文件缺失: {name}')
            continue
        src = path.read_text(encoding='utf-8')
        for snippet, why in FORBIDDEN_SNIPPETS:
            if snippet in src:
                failed.append(f'{name} 仍含 {snippet!r}（{why}）—— '
                              '二值通道必须彻底删除')
    _main = (ROOT / 'main.py').read_text(encoding='utf-8')
    if 'capture_one_frame(' not in _main:
        failed.append('主循环没有调用 capture_one_frame —— 单帧采集路径不存在')
    if 'IMAGE_MODE' not in _main:
        failed.append('main.py 没有用 protocol.IMAGE_MODE —— 指令的模式位没被钉死')
    _lane = (ROOT / 'lane_v2.py').read_text(encoding='utf-8')
    if 'np.where' in _lane:
        failed.append('lane_v2.py 里还有 np.where —— 疑似又出现了阈值类运算')
    if '>' in _lane and '190' in _lane:
        failed.append('lane_v2.py 里还有 190 —— 那是旧的 th190 二值阈值')
except Exception as exc:
    failed.append(f'二值通道自检异常: {type(exc).__name__}: {exc}')

if failed:
    print('自检失败：')
    for f in failed:
        print('  -', f)
    sys.exit(1)
print('自检通过：模块可导入，关键接口参数一致，运行期无二值请求')
