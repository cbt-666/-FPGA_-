#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RK3568 FPGA multi-model controller.

Each round: send ONE 2-byte (LE) display command, receive ONE complete
MODE 1 (YRG grey) frame, then run scene -> navigation / traffic / lane ->
update the display.

## 只有灰度（2026-10-31）

用户要求：「把所有和二值化有关的东西都删掉、车道换成灰度模型、不再请求
二值化、RK 端坚决不要做二值化」。

所以指令的 bit10~11 恒为 01（MODE 1），全工程没有第二条采集路径：
  * 不存在模式锁存、模式翻转、双请求白收（错误汇集第 7、11 条不再适用）；
  * 车道模型是 G640（灰度 640×360），与场景/导航/灯箱共用同一张 YRG 帧；
  * RK 端没有任何阈值运算，`lane_v2.prepare` 只有一次缩放。

唯一的完整性检查：若回来的帧取值塌缩成 {0,253}，说明 FPGA 的模式 1 还没把
`median_data`（中值滤波后的真灰度）接出来 —— 那是**上游给错了图**，不是
RK 端二值化的产物。这时灯箱本轮不判色，避免数不到 254/255 凭空造全向红灯。

On a timeout only the grey request is retried, retaining the last display.
UDP headers carry no mode/transaction ID; see the protocol notes.
"""
import argparse
import json
import logging
import sys
from restart_runtime import (ProgramRestartRequested, read_display, save_display,
                             restart_streak, mark_restart, reset_restart_streak,
                             reexec_program)
from hang_watchdog import HangWatchdog, default_on_stall
import resource_probe
import net_recovery
import mem_guard
import time
from pathlib import Path
import numpy as np
from protocol import IMAGE_MODE, encode_packet, encode_command, Display, GREEN, RED
from udp_pair import UDPLink
from models_runtime import LazyDetectors
from strategy_engine import (decide_pair, SceneEmptyHold, SceneThrottle,
                             WarningHold, NavigationHold, BranchSwitch,
                             TrafficHoldLimit, NavDirectionHold, LastGood,
                             EmptyStreak, NoBoxHold, TrafficEvidenceGuard,
                             LayoutHold, branch_family)
from lane_stability import LaneSolidStreaks
from cop_traffic_signature import TrafficSignature
import cop_traffic_signature

LOG = logging.getLogger('rk3568')
ROOT = Path(__file__).resolve().parent


def _ensure_file(path, label):
    if not Path(path).is_file():
        raise FileNotFoundError(f'{label} 模型不存在: {path}\n参见 README_先看.md 的模型文件说明')


def main_entry(gray: np.ndarray, detectors,
               person_y=.55, person_x=.25, previous_display=None, lane_streaks=None,
               lane_dash_ratio=.05, empty_scene_hold=None, lane_backend='v2',
               lane_roi_top=.44, lane_roi_bottom=.97, lane_row_step=3,
               lane_band_top=0.10, lane_band_bot=0.90,
               lane_chain_min_rows=3, lane_dash_frac=.5, scene_throttle=None,
               warning_hold=None, nav_hold=None, branch_switch=None,
               traffic_hold_limit=None, nav_direction_hold=None,
               frame_is_thresholded=False, last_good=None, last_good_hold=0,
               last_good_traffic_hold=None, scene_retries=0,
               empty_streak=None, no_box_hold=None, traffic_guard=None,
               layout_hold=None, cop_signature=None):
    """公开单轮算法入口: YRG uint8 -> Display + JSON result.

    `frame_is_thresholded` 表示这一帧的取值集合塌缩成 {0,253}（FPGA 的模式 1
    还没接出 `median_data`，回来的其实是阈值图）。这不是 RK 端二值化 ——
    RK 端一个阈值运算都不允许有。它只用来让红绿灯本轮不判色、
    落到 traffic_hold，避免凭空造出全向红灯。
    """
    if gray.dtype != np.uint8 or gray.shape != (720, 1280):
        raise ValueError('gray must be (720,1280) uint8, encoded as FPGA YRG')
    t0=time.perf_counter()
    disp,result=decide_pair(gray,detectors,person_y=person_y,person_x=person_x,
                            previous_display=previous_display, lane_streaks=lane_streaks,
                            lane_dash_ratio=lane_dash_ratio, empty_scene_hold=empty_scene_hold,
                            lane_backend=lane_backend, lane_roi_top=lane_roi_top,
                            lane_roi_bottom=lane_roi_bottom, lane_row_step=lane_row_step,
                            lane_band_top=lane_band_top, lane_band_bot=lane_band_bot,
                            lane_chain_min_rows=lane_chain_min_rows,
                            lane_dash_frac=lane_dash_frac, scene_throttle=scene_throttle,
                            warning_hold=warning_hold, nav_hold=nav_hold,
                            branch_switch=branch_switch,
                            traffic_hold_limit=traffic_hold_limit,
                            nav_direction_hold=nav_direction_hold,
                            frame_is_thresholded=frame_is_thresholded,
                            last_good=last_good, last_good_hold=last_good_hold,
                            last_good_traffic_hold=last_good_traffic_hold,
                            scene_retries=scene_retries,
                            empty_streak=empty_streak,
                            no_box_hold=no_box_hold,
                            traffic_guard=traffic_guard,
                            layout_hold=layout_hold, cop_signature=cop_signature)
    result['elapsed_ms']=round((time.perf_counter()-t0)*1000,2)
    hex_code = encode_command(disp, IMAGE_MODE)
    result['command_hex']=f'0x{hex_code:04X}'
    result['command_bytes']=encode_packet(disp, IMAGE_MODE).hex(' ')
    result['requested_frame'] = 'gray'
    return disp,result


def model_paths(args):
    d={'scene':args.scene_model,'navigation':args.nav_model,
       'housing':args.traffic_model,'lane':args.lane_model}
    _ensure_file(d['scene'],'五类场景')
    # 交警手势模型是可选件：文件不在就让交警分支保持"永不触发"，不要因此拒绝启动。
    # ⚠️ 注意：run.sh 的 check_models.py 按 models_manifest.json 检查全部模型，
    # 所以走 run.sh 时 cop 模型仍必须存在（演示包完整性）；只有直接启动
    # main.py / start_v12.sh 才允许缺省（缺省 = 交警分支永不触发，其余分支照常）。
    cop = getattr(args,'cop_model',None)
    d['cop'] = cop if (cop is not None and Path(cop).is_file()) else None
    # The other three are lazily initialized. Check now only if user asks.
    if args.require_all_models:
        for k,v in d.items():
            if k=='cop' and v is None:
                continue
            _ensure_file(v,k)
    return d


def build_models(args):
    files=model_paths(args)
    return LazyDetectors(files['scene'],files['navigation'],files['housing'],
                         files['lane'],args.core_mask,files['cop'])


def _capture(path,frame,frame_id):
    path.mkdir(exist_ok=True,parents=True)
    np.save(path/'gray_yrg.npy',frame,allow_pickle=False)
    (path/'frame_ids.json').write_text(json.dumps(frame_id),encoding='utf-8')


def _diag_frame(root, cycle, frame, diag, result):
    """前 N 轮把帧本体 + 取值谱 + 判定一起落盘，用于定位"模型到底看到了什么"。

    2026-10-31 用户要「先跑一段日志，我不确定」——与其猜 FPGA 给的是
    真灰度还是阈值图，不如把原图存下来自己看。只存前 N 轮，别写爆存储。
    """
    try:
        import cv2
        d = Path(root) / f'round_{cycle:05d}'
        d.mkdir(parents=True, exist_ok=True)
        px = frame.pixels
        np.save(d / 'frame.npy', px, allow_pickle=False)
        cv2.imwrite(str(d / 'frame.png'), px)
        info = {
            'round': cycle,
            'frame_id': int(frame.frame_id),
            'diag': {k: v for k, v in (diag or {}).items()
                     if k.startswith('frame_') or k in
                     ('gray_unique_values', 'looks_thresholded', 'elapsed_ms')},
            'branch': result.get('branch'),
            'scene_targets': result.get('scene_targets'),
            'lane_source': result.get('lane_source'),
            'lane': result.get('lane'),
            'lane_updates': result.get('lane_updates'),
            'traffic': (result.get('traffic') or {}).get('layout'),
            'traffic_colours': (result.get('traffic') or {}).get('colours'),
            'traffic_guard': (result.get('traffic_guard') or {}).get('state'),
            'display': result.get('display'),
            'display_reason': result.get('display_reason'),
        }
        (d / 'info.json').write_text(json.dumps(info, ensure_ascii=False, indent=2),
                                     encoding='utf-8')
    except Exception as exc:
        LOG.warning('第%d轮 帧诊断导出失败（不影响主流程）：%s', cycle, exc)


def _dump_lane_jump(root,cycle,gray,result,ids):
    """Persist the exact frame behind a lane arrow change.

    The recorded logs carry only the post-hoc line statistics, which is enough
    to show THAT a lane reading jumped but not what the pixels looked like.
    When the board reports a wrong lane arrow on site, one of these dumps is
    the difference between a guess and a reproducible case.
    """
    try:
        d=root/f'round{cycle:05d}'
        d.mkdir(exist_ok=True,parents=True)
        np.save(d/'gray_yrg.npy',gray,allow_pickle=False)
        payload={'cycle':cycle,'frames':ids,
                 'display':result.get('display'),
                 'previous_display':result.get('previous_display'),
                 'lane':result.get('lane'),
                 'display_reason':result.get('display_reason'),
                 'weak_evidence_change':result.get('lane_weak_evidence_change')}
        (d/'decision.json').write_text(json.dumps(payload,ensure_ascii=False),
                                       encoding='utf-8')
        LOG.warning('车道箭头发生变化，已保存该轮原始双图到 %s（%s -> %s）',
                    d,payload['previous_display'],payload['display'])
    except Exception:
        LOG.exception('保存车道跳变帧失败（不影响主循环）')


def capture_one_frame(link, display, args, cycle, purpose=''):
    """向 FPGA 请求**灰度帧**（MODE 1），收满一帧。失败只重发这一条。

    2026-10-31 用户要求「不再请求二值化」。全工程只剩这一种采集：
    指令的 bit10~11 恒为 01，不存在第二种模式，也就不存在模式串台、
    锁存延迟、双请求白收这一类问题（错误汇集第 7、11 条）。
    """
    attempt = 0
    t_start = time.perf_counter()
    while True:
        attempt += 1
        try:
            def report(command, drained):
                LOG.info('第%d轮 灰度图(YRG)%s 第%d次请求: 16bit=0x%04X UDP=[%s] 清理旧UDP包=%d',
                         cycle, purpose, attempt,
                         int.from_bytes(command,'little'),
                         command.hex(' '), drained)
            frame = link.request_frame(display, timeout=args.pair_timeout,
                                       on_request=report)
            elapsed = round((time.perf_counter() - t_start) * 1000, 2)
            link.pair_timing_ms = {'gray_ms': elapsed, 'pair_ms': elapsed,
                                   'attempts': attempt}
            diag=getattr(link,'last_report',{})
            levels=diag.get('gray_unique_values')
            note=(f'  灰度级数={levels}'
                  + (' ⚠️看起来是阈值图(FPGA 模式1 未接出中值灰度)'
                     if diag.get('looks_thresholded') else ''))
            LOG.info('第%d轮 灰度图接收完整: frame_id=%d 行数=720 RX=%.1fms, drain=%.1fms%s',
                     cycle, frame.frame_id, diag.get('elapsed_ms',0),
                     getattr(link,'last_drain_ms',0), note)
            link.last_gray_frame_id = int(frame.frame_id)
            link.last_frame_thresholded = bool(diag.get('looks_thresholded'))
            return frame
        except (TimeoutError, ValueError) as exc:
            LOG.warning('第%d轮 灰度图未接收完整；重发（尝试%d）: %s',
                        cycle, attempt, exc)
            if args.max_retries and attempt >= args.max_retries:
                raise RuntimeError(f'灰度图连续接收失败达到上限：{attempt}') from exc
            if args.retry_delay:
                time.sleep(args.retry_delay)


# 旧名保留：单元测试与外部脚本仍在 import `request_one_frame`。
request_one_frame = capture_one_frame


def _drop_restored_cop_arrow(display):
    """重启后不要把上一次交警模式的单箭头当成普通显示恢复。

    交警模式只发一个箭头；lane 分支不会发绿、也不会发**直行**箭头、
    traffic 分支强制三箭头、navigation 只发蓝，所以下面两种"恰好一个箭头"
    只可能来自 cop：
      * 单个绿箭头            -> 类 1（左转）
      * 直行位置的单个红箭头  -> 类 2（直行，V28.5 起改红）
    重启后 BranchSwitch.last 还是 None，首轮不会算切换，空场景 hold 会把这个
    箭头原样沿用，FPGA 可能继续按交警模式播报语音。V28.2 起直接丢掉。

    ⚠️ **只有"直行位置"的红才能丢**：车道分支左/右两侧独立，合法地只亮
    一个红箭头（实线），那一条必须照常恢复，不能一起清掉。
    """
    colours = (display.left, display.straight, display.right)
    if sum(c is not None for c in colours) != 1:
        return display
    if GREEN in colours or colours[1] == RED:
        LOG.warning('恢复的显示是单个%s箭头（上次可能处于交警模式）：'
                    '不沿用，等模型重新 armed',
                    '绿' if GREEN in colours else '直行红')
        return Display()
    return display


def run_live(args):
    models = None
    link = None
    try:
        models = build_models(args)
        link = UDPLink(args.bind, args.port, args.source_ip, args.source_port,
                       args.signal_ip, args.signal_port)
    except Exception:
        if link is not None:
            link.close()
        if models is not None:
            models.close()
        raise
    LOG.info('UDP bind=%s:%s source=%s:%s target=%s:%s SO_RCVBUF=%d',
             args.bind, args.port, args.source_ip, args.source_port,
             args.signal_ip, args.signal_port,
             link.sock.getsockopt(__import__('socket').SOL_SOCKET,
                                  __import__('socket').SO_RCVBUF))
    if hasattr(link,'configure_receiver'):
        link.configure_receiver(drain_budget_ms=getattr(args,'rx_drain_ms',15),
                                drain_max_packets=getattr(args,'rx_drain_max',4096),
                                idle_timeout=getattr(args,'rx_idle_timeout',.8),
                                bad_header_limit=getattr(args,'rx_bad_header_limit',64),
                                mode0_grace_ms=getattr(args,'rx_mode0_grace_ms',35.0))
    LOG.info('低延迟接收参数：整体超时=%.2fs，UDP空闲超时=%.2fs，连续坏包头快速失败=%d，'
             '清包预算=%.1fms',args.pair_timeout,getattr(args,'rx_idle_timeout',.8),
             getattr(args,'rx_bad_header_limit',64),getattr(args,'rx_drain_ms',15))
    # Display retained on receive / inference failure; no stale-timeout arrows.
    # V28.2：交警模式的单绿箭头不跨重启沿用（见 _drop_restored_cop_arrow）。
    display = _drop_restored_cop_arrow(read_display())
    lane_streaks = LaneSolidStreaks(args.solid_confirm_frames)
    empty_scene_hold = SceneEmptyHold(args.empty_scene_confirm_frames)
    scene_throttle = SceneThrottle(args.scene_every, args.scene_max_hold,
                                   args.scene_nav_max_hold,
                                   args.scene_traffic_max_hold)
    warning_hold = WarningHold(args.warning_confirm_frames,
                               args.warning_max_hold)
    nav_hold = NavigationHold(args.nav_hold_frames)
    branch_switch = BranchSwitch()
    # 2026-10-29：按颜色计时的"绿灯过期"已停用。它和"红绿灯模式永远三个箭头"
    # 直接冲突（绿灯 6 轮先到期、红灯 15 轮后到期 → 箭头一个一个消失）。
    # 沿用多久改由 NoBoxHold 统一回答：连续 N 轮无框 → 全红。
    # 传 None 表示不启用；保留该类是为了老日志/对照还能读。
    traffic_hold_limit = None
    nav_direction_hold = NavDirectionHold(args.nav_turn_confirm_frames,
                                          args.nav_turn_confident)
    last_good = LastGood() if args.last_good_hold else None
    empty_streak = EmptyStreak(args.scene_retry_window)
    no_box_hold = NoBoxHold(args.no_box_hold_frames)
    # 2026-10-29 高速幻影红绿灯：场景模型说"有红绿灯"只是提示，灯箱模型
    # 给出灯箱大小的框才算实证（见 strategy_engine.TrafficEvidenceGuard）。
    traffic_guard = TrafficEvidenceGuard(args.traffic_release_frames)
    # 框数抖动：框数决定方向语义，跳一轮就整体错位（红绿灯"乱跳"的根因）。
    layout_hold = LayoutHold(args.traffic_layout_confirm_frames)
    # 交警手势状态机。窗口 12 轮是"够稳又不粘"的折中——
    # 按 30 fps 算的 45 帧平滑映射到板端（一轮 ≈ 0.7 s）就是 31 s，
    # 一旦误触发会粘住半分钟；手势切换时又要等半分钟才退出。
    # 12 轮 ≈ 8 s，多数票需要 7/12 才能维持，误触发不会长期粘在屏幕上。
    cop_signature = (TrafficSignature(args.cop_window, args.cop_lock_rounds,
                                      args.cop_release_rounds)
                     if args.cop_fingerprint else None)
    if cop_signature is not None:
        # 用 model_paths(args) 而不是 models.paths：启动日志只依赖 args，
        # 不依赖 detectors 的内部属性（测试替身没有 paths，会在这里崩）。
        cop_ready = model_paths(args)['cop'] is not None
        LOG.info('交警手势=开启：分类模型 %s（%s）；窗口 %d 轮 / 连续确认 左转 %d 轮、直行 %d 轮 / '
                 '%d 轮退出；左转=**绿**箭头 直行=**红**箭头'
                 '（各自只有一个箭头）；置信度门槛 左转 %.3f / 直行 %.3f',
                 args.cop_model, '已加载' if cop_ready else '**文件不存在，交警分支不会触发**',
                 args.cop_window,
                 cop_signature.lock_by_kind.get('1'), cop_signature.lock_by_kind.get('2'),
                 args.cop_release_rounds,
                 cop_signature.conf_by_kind.get('1'), cop_signature.conf_by_kind.get('2'))
    LOG.info('车道判定=%s，dash阈值=%.2f，ROI=%.2f~%.2f 步长=%d；五分类每 %d 轮一次(最多沿用 %d 轮)；'
             '实线确认 %d 轮；斑马线/行人报警确认 %d 轮(最多沿用 %d 轮)；导航蓝箭头最长保持 %d 轮',
             args.lane_backend, args.lane_dash_ratio, args.lane_roi_top,
             args.lane_roi_bottom, args.lane_row_step, args.scene_every,
             args.scene_max_hold, args.solid_confirm_frames,
             args.warning_confirm_frames, args.warning_max_hold,
             args.nav_hold_frames)
    LOG.info('图像模式=仅灰度(MODE 1，从不请求二值)；车道模型=G640(灰度)；'
             '红灯=灯箱检框+数框判向')
    LOG.info('程序启动：连续程序重启次数=%d，恢复箭头显示=%s',
             restart_streak(), display.as_dict())
    count = 0
    consecutive_pair_failures = 0
    infer_fail_streak = 0
    health_prev = {}
    # The board log ended with 5-16 s rounds and a 9.3 s receive, with no OOM
    # and no restart -- a host-level stall. Sample the host periodically so the
    # next freeze leaves evidence instead of just a gap in the timestamps.
    health_every = getattr(args, 'health_every', 20)
    trim_every = max(0, int(getattr(args, 'mem_trim_every', 50) or 0))
    restart_every = max(0, int(getattr(args, 'mem_restart_every', 0) or 0))
    # The NPU call has no timeout; without this a hung inference freezes the
    # board with the last arrows still lit and no log line explaining why.
    watchdog = None
    if args.hang_timeout > 0:
        watchdog = HangWatchdog(
            args.hang_timeout,
            on_stall=lambda: default_on_stall(args.hang_timeout, watchdog.where)
        ).start()
    try:
        while True:
            if args.max_cycles and count >= args.max_cycles:
                break
            cycle_start=time.perf_counter()
            if watchdog is not None:
                watchdog.beat('等待采集')
            # 一轮只发**一条**图像指令，且永远是 MODE 1 灰度（2026-10-31）。
            #
            # 二值请求通道整个删除（用户要求「不再请求二值化」），所以：
            #   * 不存在模式翻转，也不存在"锁存延迟"（错误汇集第 11 条不再适用）；
            #   * 灯箱帧永远有 254/255，红绿灯不会因数不到颜色而全红；
            #   * 车道模型（G640）与其它模型吃同一张 YRG 帧，一次采集全部分支共用。
            try:
                gray = capture_one_frame(link, display, args, count+1)
                consecutive_pair_failures = 0
                # A fully acquired frame confirms the new receiver is healthy.
                reset_restart_streak()
            except RuntimeError as exc:
                lane_streaks.reset()
                empty_scene_hold.empty = 0  # Broken capture is not a consecutive no-target scene.
                consecutive_pair_failures += 1
                LOG.error('第%d轮采图失败，保持原显示，重发灰度请求'
                          '（连续失败%d/%d）：%s',
                          count+1, consecutive_pair_failures, args.max_pair_failures, exc)
                if (args.max_pair_failures and
                        consecutive_pair_failures >= args.max_pair_failures):
                    # A lost interface/route is not fixed by restarting Python:
                    # execv re-runs main.py and never re-enters start_v12.sh, so
                    # setup_fpga_net.sh does not run again. rk3568_v17_orig.log
                    # burned six restarts this way, 61 ENETUNREACH in a row.
                    if net_recovery.is_network_fault(exc):
                        LOG.critical('连续 %d 次采图失败，且判定为网络层故障（网卡/路由丢失）。'
                                     '重启 Python 无法恢复，改为原地退避重试。',
                                     consecutive_pair_failures)
                        attempted, ok, detail = net_recovery.try_reconfigure(
                            ROOT / 'setup_fpga_net.sh')
                        if not attempted:
                            LOG.critical('未尝试重配网卡：%s。请人工检查 '
                                         '`ip -br addr` / `ip route` / 网线，'
                                         '恢复后自动继续。', detail)
                        elif ok:
                            LOG.critical('已重新执行 setup_fpga_net.sh 并通过：%s', detail)
                        else:
                            LOG.critical('重新执行 setup_fpga_net.sh 失败：%s', detail)
                        consecutive_pair_failures = 0
                        time.sleep(max(args.recovery_delay, 2.0))
                        continue
                    LOG.critical('连续 %d 次整轮采图失败，接收端无法恢复。',consecutive_pair_failures)
                    action=getattr(args,'failure_action','resync')
                    if action=='restart':
                        current=restart_streak()
                        if args.restart_dry_run:
                            LOG.critical('程序重启演练：本应重新执行 main.py；不会重启系统，实际不重启程序')
                        elif args.restart_max_streak == 0 or current < args.restart_max_streak:
                            mark_restart()
                            LOG.critical('即将重新启动 Python 主程序（连续第%d次，上限%s）；不重启 Linux。'
                                         'UDP 与 RKNN 将先释放，再重新初始化。',
                                         current+1, str(args.restart_max_streak) if args.restart_max_streak else '不限')
                            # Leaving run_live invokes its finally first.
                            raise ProgramRestartRequested('采图连续失败，重新运行主程序') from exc
                        else:
                            LOG.error('程序连续重启次数达到%d次上限；'
                                      '改为原进程继续重同步，不退出、不反复重启。'
                                      '完整收到下一组图像后重置重启次数。',
                                      args.restart_max_streak)
                    else:
                        LOG.warning('恢复动作=resync：重新开始请求灰度图')
                    # Stay running when re-exec is disallowed or dry-run.
                    consecutive_pair_failures=0
                if args.recovery_delay:
                    time.sleep(args.recovery_delay)
                continue
            ids = {'gray_frame_id': gray.frame_id}
            try:
                models.last_model_stages={}  # do not report stale stage timings from previous cycles
                if watchdog is not None:
                    watchdog.beat('模型推理+决策')
                # 这一帧的取值集合塌缩成 {0,253} = FPGA 的模式 1 还没把
                # median_data（真灰度）接出来，回来的其实是阈值图。
                # 这不是 RK 端二值化的产物（RK 端一个阈值都没有），
                # 只用来让灯箱本轮不判色，否则数不到 254/255 会凭空造红灯。
                frame_is_thresholded = bool(
                    getattr(link, 'last_frame_thresholded', False))
                display, next_result = main_entry(gray.pixels,
                                                   models,
                                                   args.person_y, args.person_x,
                                                   previous_display=display,
                                                   lane_streaks=lane_streaks,
                                                   lane_dash_ratio=args.lane_dash_ratio,
                                                   empty_scene_hold=empty_scene_hold,
                                                   lane_backend=args.lane_backend,
                                                   lane_roi_top=args.lane_roi_top,
                                                   lane_roi_bottom=args.lane_roi_bottom,
                                                   lane_row_step=args.lane_row_step,
                                                   lane_band_top=args.lane_band_top,
                                                   lane_band_bot=args.lane_band_bot,
                                                   lane_chain_min_rows=args.lane_chain_min_rows,
                                                   lane_dash_frac=args.lane_dash_frac,
                                                   scene_throttle=scene_throttle,
                                                   warning_hold=warning_hold,
                                                   nav_hold=nav_hold,
                                                   branch_switch=branch_switch,
                                                   traffic_hold_limit=traffic_hold_limit,
                                                   nav_direction_hold=nav_direction_hold,
                                                   frame_is_thresholded=frame_is_thresholded,
                                                   last_good=last_good,
                                                   last_good_hold=args.last_good_hold,
                                                   last_good_traffic_hold=(
                                                       args.last_good_traffic_hold
                                                       or args.last_good_hold),
                                                   scene_retries=args.scene_retries,
                                                   empty_streak=empty_streak,
                                                   no_box_hold=no_box_hold,
                                                   traffic_guard=traffic_guard,
                                                   layout_hold=layout_hold,
                                                   cop_signature=cop_signature)
                # Preserve left/right/straight state across Python re-exec.
                save_display(display)
                next_result['requested_frame'] = 'gray'
                next_result['scene_targets'] = sorted(
                    next_result.get('scene_targets') or [])
                if next_result.get('display_changed'):
                    LOG.info('显示状态更新: branch=%s reason=%s from=%s to=%s',
                             next_result.get('branch'), next_result.get('display_reason'),
                             next_result.get('previous_display'), next_result.get('display'))
                # 这一轮完整跑通了，之前的连续失败清零。
                infer_fail_streak = 0
                next_result['frames'] = ids
                if (args.dump_lane_jump and next_result.get('display_changed')
                        and 'lane' in str(next_result.get('branch',''))):
                    _dump_lane_jump(args.dump_lane_jump, count+1,
                                    gray.pixels, next_result, ids)
                next_result['model_stage_ms']=dict(getattr(models,'last_model_stages',{}))
                # 帧诊断：前 N 轮把原图和判定一起存下来。
                diag_n = getattr(args, 'diag_frames', 0) or 0
                if diag_n and (count + 1) <= diag_n:
                    _diag_frame(getattr(args, 'diag_dir', None) or
                                (ROOT / 'diag_frames'),
                                count + 1, gray,
                                getattr(link, 'last_report', {}), next_result)
                next_result['cycle_wall_ms']=round((time.perf_counter()-cycle_start)*1000,2)
                # 一行贯通的时间线：收图 → 场景 → 分支模型 → 决策 → 发包。
                # 用户要求像调试器那样能直接从日志看出每一段花了多久。
                # 绝对值都在上面的 JSON 里，这里再打一行人眼可读的，
                # 免得每次要拿 grep 拼。
                _stage = []
                _rx = getattr(link,'pair_timing_ms',{})
                _stage.append('收图 %.0f' % (_rx.get('pair_ms') or 0))
                _t = next_result.get('timing_ms') or {}
                _stage.append('场景 %.0f%s' % (_t.get('scene_model_ms') or 0,
                                              '(缓存)' if _t.get('scene_cached') else ''))
                if _t.get('scene_retries'):
                    _stage.append('场景重试x%d' % _t['scene_retries'])
                for _k, _n in (('navigation_model_and_digits_ms','导航'),
                               ('traffic_model_ms','灯箱'),
                               ('lane_model_ms','车道')):
                    if _t.get(_k):
                        _stage.append('%s %.0f' % (_n, _t[_k]))
                if _t.get('traffic_rules_ms'):
                    _stage.append('判灯 %.0f' % _t['traffic_rules_ms'])
                if _t.get('lane_geometry_ms'):
                    _stage.append('车道几何 %.0f' % _t['lane_geometry_ms'])
                _stage.append('决策 %.0f' % (next_result.get('elapsed_ms') or 0))
                LOG.info('第%d轮 耗时: %s | 整轮 %.0fms | 分支=%s',
                         count+1, ' → '.join(_stage),
                         next_result['cycle_wall_ms'], next_result.get('branch'))
                if health_every and (count+1) % health_every == 0:
                    try:
                        cur = resource_probe.sample()
                        next_result['host'] = cur
                        for line in resource_probe.warn_lines(health_prev, cur):
                            LOG.warning('系统资源：%s（第%d轮）', line, count+1)
                        LOG.info('系统资源 第%d轮: RSS=%sMB 可用内存=%sMB 负载=%s 温度=%s°C',
                                 count+1, cur['rss_mb'], cur['mem_available_mb'],
                                 cur['load1'], cur['temp_c'])
                        health_prev = cur
                    except Exception:
                        LOG.debug('系统资源采样失败（不影响主循环）', exc_info=True)
                # 定期把空闲堆还给内核。板上 2 GB 内存，2026-10-28 跑到第 335 轮
                # 可用内存只剩 150 MB，被内核 OOM 杀掉（返回 137）。
                # 本轮只有这一张灰度图，且已经跑完，不再被 next_result 引用。
                # 2026-10-31：以前这里还要等二值图用完，现在只有一张。
                if trim_every and (count+1) % trim_every == 0:
                    try:
                        before, after, called = mem_guard.trim()
                        next_result['mem_trim'] = {'rss_before_mb': before,
                                                   'rss_after_mb': after,
                                                   'malloc_trim': called}
                        LOG.info('内存回收 第%d轮: RSS %s→%sMB%s', count+1, before, after,
                                 '' if called else '（本机 libc 无 malloc_trim，只做了 gc）')
                    except Exception:
                        LOG.debug('内存回收失败（不影响主循环）', exc_info=True)
                # malloc_trim 只回收「本进程 glibc 堆里的空闲页」。2026-10-28
                # 上板日志证明这不是全部：trim 之后 RSS 确实被打回 234MB，
                # 但那个进程之外的占用（1969-可用-RSS）从 870MB 一路涨到
                # 1241MB，也就是 NPU 运行时 / 内核侧每 100 轮漏 ~64MB。
                # 那部分只能在进程退出时释放，所以给一个周期性的进程内重启：
                # os.execv 重跑 main.py（不重启 Linux），NPU 上下文随之重建。
                if restart_every and (count+1) % restart_every == 0:
                    host = next_result.get('host') or {}
                    avail = host.get('mem_available_mb')
                    rss = host.get('rss_mb')
                    LOG.critical('定期重启：已跑 %d 轮（RSS=%sMB 可用=%sMB），'
                                 '重建 NPU 上下文释放进程外泄漏；不重启 Linux',
                                 count+1, rss, avail)
                    raise ProgramRestartRequested('到达 --mem-restart-every 周期')
                next_result['request_sequence']=['gray']
                next_result['receive_timing_ms']=getattr(link,'pair_timing_ms',{})
                pair_ms=next_result['receive_timing_ms'].get('pair_ms',0)
                if pair_ms>800 or next_result['elapsed_ms']>450:
                    LOG.warning('慢周期: 收图%.1fms，模型/决策%.1fms；'
                                '按字段判断瓶颈而不是仅看总响应',pair_ms,next_result['elapsed_ms'])
                print(json.dumps(next_result, ensure_ascii=False), flush=True)
                if args.save_every and (count+1) % args.save_every == 0:
                    _capture(args.capture_dir, gray.pixels, ids)
                count += 1
            except ProgramRestartRequested:
                # 定期重启走这里：本轮已经不打印了，直接往上抛给 main_entry，
                # 由它 os.execv 重跑 main.py（不重启 Linux）。
                raise
            except Exception as exc:
                lane_streaks.reset()  # Failed inference cannot count as consecutive evidence.
                empty_scene_hold.empty = 0
                infer_fail_streak += 1
                LOG.exception('模型推理/分支出错，保留上一显示、继续下一轮；不冒用旧结果'
                              '（连续第 %d 次）', infer_fail_streak)
                # ⚠️ 2026-10-29 上板：run_live 给 main_entry 传了一个签名里没有的
                # 参数，458 轮**每轮**都抛 TypeError，全被这里吞成一行日志，
                # 屏幕从头黑到尾而进程看起来一切正常。连续失败到上限就不再
                # 假装没事——交给外层按程序重启处理，让故障暴露出来。
                if infer_fail_streak >= args.max_infer_failures > 0:
                    raise ProgramRestartRequested(
                        f'模型/决策连续失败 {infer_fail_streak} 轮，'
                        '不是偶发单帧问题，重新初始化全部组件') from exc
                count += 1
                if args.stop_on_infer_error:
                    raise
    except KeyboardInterrupt:
        LOG.info('用户退出；不主动发送新的显示指令')
    finally:
        if watchdog is not None:
            watchdog.stop()
        try:
            if link is not None:
                link.close()
        except Exception:
            LOG.exception('UDP socket 关闭失败（程序重启时继续清理后续资源）')
        try:
            if models is not None:
                models.close()
        except Exception:
            LOG.exception('RKNN release 失败；接下来程序级 execv 将回收进程资源')


def run_offline(args):
    gray=np.load(args.gray_npy,allow_pickle=False)
    models=build_models(args)
    try:
        disp,result=main_entry(gray,models,args.person_y,args.person_x,
                               lane_dash_ratio=args.lane_dash_ratio)
        result['note']='offline: no FPGA command transmitted'
        print(json.dumps(result,indent=2,ensure_ascii=False))
    finally:models.close()


def parser():
    p=argparse.ArgumentParser(description='RK3568 FPGA 4 models (grey only) / 16bit overlay')
    p.add_argument('--gray-npy',type=Path,help='离线模式：720x1280 YRG npy')
    p.add_argument('--capture-dir',type=Path,default=Path('captures'))
    p.add_argument('--scene-model',type=Path,default=ROOT/'models/merged5gray_rk3568_fp16.rknn')
    p.add_argument('--nav-model',type=Path,default=ROOT/'models/nav6_720_rk3568_fp16.rknn',
                   help='导航 6 类模型（方向×距离位数合一，单次推理）')
    p.add_argument('--traffic-model',type=Path,default=ROOT/'models/housing_3568_fp16.rknn',
                   help='红绿灯灯箱检测模型（单类，只出框）')
    p.add_argument('--lane-model',type=Path,default=ROOT/'models/G640_rk3568_fp16.rknn',
                   help='车道 v2 灰度模型（G640，640x360）')
    p.add_argument('--cop-model',type=Path,
                   default=ROOT/'models/cls_v2yrg3_rk3568_fp16.rknn',
                   help='交警手势分类模型（96x96，左转/直行/other）。不给或文件不存在时'
                        '交警分支不会触发，其余分支照常')
    p.add_argument('--require-all-models',action='store_true',help='启动前检查五个模型均存在（交警手势模型为可选件，不做要求）')
    p.add_argument('--core-mask',choices=('auto','0','all'),default='auto')
    p.add_argument('--bind',default='0.0.0.0')
    p.add_argument('--port',type=int,default=1234)
    p.add_argument('--source-ip',default='192.168.1.10')
    p.add_argument('--source-port',type=int,default=1234)
    p.add_argument('--signal-ip',default='192.168.1.10')
    p.add_argument('--signal-port',type=int,default=1234)
    p.add_argument('--pair-timeout',type=float,default=2.0,
                   help='每张单独请求图像的接收超时时间（秒）')
    p.add_argument('--retry-delay',type=float,default=.05)
    p.add_argument('--max-retries',type=int,default=2,
                   help='每张图像最大请求次数，默认2；连续多轮失败按程序重启策略处理')
    p.add_argument('--max-cycles',type=int,default=0,help='0持续循环')
    p.add_argument('--max-pair-failures',type=int,default=5,
                   help='连续整轮采图失败达到此次数后执行恢复动作，默认5；0表示不触发动作')
    p.add_argument('--failure-action',choices=('restart','resync'),default='restart',
                   help='连续失败时默认重新执行本 Python 程序（不重启 Linux）；resync只重新请求双图')
    p.add_argument('--restart-max-streak',type=int,default=0,
                   help='连续 Python 重执行上限，默认0=不限次数；可设置3恢复旧上限，完整收到双图后清零')
    p.add_argument('--restart-delay',type=float,default=0.5,
                   help='程序级重新执行之前的等待秒数（默认0.5秒）')
    p.add_argument('--restart-dry-run',action='store_true',
                   help='仅记录触发程序重启，不真正重新执行程序')
    p.add_argument('--rx-idle-timeout',type=float,default=.8,
                   help='接收端连续没有来自FPGA的UDP包时快速重试，默认0.8秒')
    p.add_argument('--rx-mode0-grace-ms',type=float,default=35.0,
                   help='切换模式时允许前一模式包短暂残留的时间，默认35毫秒')
    p.add_argument('--rx-bad-header-limit',type=int,default=64,
                   help='无任何有效行时，达到该坏包头数量立即重同步（默认64）')
    p.add_argument('--rx-drain-ms',type=float,default=60.,
                   help='每次发送采图请求前清理已排队UDP的时间预算，默认60毫秒。'
                        '2026-10-31：原来是 15 ms——而缓冲有 9 帧（4096 包 ≈ 5.7 帧），'
                        '15 ms 一轮只能清 1.5 帧左右，队列头永远陈旧。'
                        '60 ms 相对单轮 680 ms 可忽略，但足够把 4 MB 缓冲清空')
    p.add_argument('--rx-drain-max',type=int,default=4096,
                   help='每次清理旧包最大数，默认4096（≈5.7 帧），且受时间预算限制')
    p.add_argument('--recovery-delay',type=float,default=.15,
                   help='整轮采图失败后切回灰度请求前的等待秒数，默认1')
    p.add_argument('--lane-dash-ratio',type=float,default=.05,
                   help='扫描算法：段内虚线像素占比高于此值判虚线，默认5%%（模型对虚线弱，阈值要低）')
    p.add_argument('--lane-backend',choices=('v2','angle'),default='v2',
                   help='v2=新车道模型+几何解码(默认)；angle=旧连通域+角度算法(仅作对照)')
    p.add_argument('--lane-roi-top',type=float,default=.44,
                   help='扫描区顶部，相对画面高度，默认0.44')
    p.add_argument('--lane-roi-bottom',type=float,default=.97,
                   help='扫描区底部，相对画面高度，默认0.97')
    p.add_argument('--lane-row-step',type=int,default=3,
                   help='每隔几行扫描一次，默认3')
    p.add_argument('--lane-band-top',type=float,default=.10,
                   help='v2 几何解码参考带的上界，相对画面高度，默认0.10（原来是0.20，'
                        '只看到近场）；用户要求「可以判别全屏，没必要只取下面一半」')
    p.add_argument('--lane-band-bot',type=float,default=.90,
                   help='v2 几何解码参考带的下界，相对画面高度，默认0.90')
    p.add_argument('--lane-chain-min-rows',type=int,default=3,
                   help='一条车道线至少需要多少行观测才算有效，默认3')
    p.add_argument('--lane-dash-frac',type=float,default=.5,
                   help='一条线中判为虚线的行占比达到此值 -> 整条线判虚线，默认0.5')
    p.add_argument('--scene-every',type=int,default=1,
                   help='每几轮跑一次五分类；中间轮次沿用上次结果(降频省时)。'
                        'V28.8：2 → 1（回滚 V28.7）。板端实测 scene 缓存复用会让'
                        '某一帧被旧场景标签路由到错误的模型分支（红绿灯画面按没有'
                        '灯箱的帧去数灯），表现为三个箭头判断全错。'
                        '设1表示每轮都跑（整轮 +537ms）')
    p.add_argument('--scene-max-hold',type=int,default=30,
                   help='scene 结果最多沿用多少轮，超过后强制重跑一次，默认30')
    p.add_argument('--scene-traffic-max-hold',type=int,default=0,
                   help='红绿灯场景结果最多沿用几轮，默认0（每轮都真跑 scene）。'
                        'V28.8：1 → 0，与 --scene-every 1 一起杜绝缓存错路由')
    p.add_argument('--scene-nav-max-hold',type=int,default=0,
                   help='导航场景结果最多沿用几轮，默认0（每轮都真跑 scene）。'
                        'V28.8：1 → 0，与 --scene-every 1 一致；'
                        '导航模型一旦空返回则下一轮强制重跑 scene')
    p.add_argument('--empty-scene-confirm-frames',type=int,default=2,
                   help='三类场景(nav/traffic/lane)均未检出，连续几轮才清空箭头；默认2抑制单帧漏检闪烁，设1立即清空')
    p.add_argument('--solid-confirm-frames',type=int,default=2,
                   help='实线需要连续多少轮才判红，默认 2。板上实测：左右两侧的假实线里'
                        '大比例是孤立单帧（左侧连续段 1,1,2；右侧 1,4,1,14,1,2），'
                        '真实实线至少连续 2 帧。虚线不受影响（仍一帧立即变黄）。'
                        '⚠️ 不要改回 3：3 帧在车道/红绿灯交替时凑不满，箭头会消失')
    p.add_argument('--warning-confirm-frames',type=int,default=1,
                   help='斑马线/行人报警需要连续多少轮命中才置位，默认1（V26：'
                        '用户要求"斑马线灵敏度调高"；V25 及以前是2）。'
                        '误报由几何判据（cy≥0.45 且面积≤25%%）+ --warning-max-hold'
                        '兜住，不再靠"多等一轮"')
    p.add_argument('--warning-max-hold',type=int,default=4,
                   help='报警置位后、证据消失时最多再沿用几轮，默认4。'
                        '板上 V25 日志实测锁存段 49/30/30/23 轮——一次误检'
                        '能亮一两秒。设1表示证据一消失立即撤')
    p.add_argument('--nav-hold-frames',type=int,default=3,
                   help='导航蓝箭头在五分类连续漏检时最多保持多少轮；默认3。'
                        '原来给到 10 是为了压黑屏（108 轮里 52 轮黑），但板上实测'
                        '按住段中位 1.8 秒、最长 6.2 秒，代价太大。'
                        '导航模型一空返回就强制重跑 scene，所以 3 轮足够撑过闪断')
    p.add_argument('--traffic-hold-frames',type=int,default=15,
                   help='红绿灯解不出灯时，红灯最多沿用多少轮后熄灯；默认15。'
                        '实测曾连续 56.9 秒显示绿灯而其中仅 12 轮真确认。设0关闭时限')
    p.add_argument('--traffic-green-hold-frames',type=int,default=6,
                   help='绿灯沿用的上限帧数，默认6（比红灯短：过期的绿灯是危险方向）')
    p.add_argument('--nav-turn-confirm-frames',type=int,default=2,
                   help='导航方向族（左/右/直）切换需要连续多少轮同向低分证据才改，默认2；'
                        '高分（>=--nav-turn-confident）一帧即改。设1关闭方向迟滞')
    p.add_argument('--nav-turn-confident',type=float,default=0.85,
                   help='导航方向切换的单帧置信度门槛，默认0.85')
    p.add_argument('--person-y',type=float,default=.55,
                   help='中下部行人框中心y的最小画面比例(临时阈值)')
    p.add_argument('--person-x',type=float,default=.25,
                   help='前方区域左右排除比例(临时阈值)')
    p.add_argument('--save-every',type=int,default=0)
    p.add_argument('--dump-lane-jump',type=Path,default=None,
                   help='车道箭头颜色一变化就把该轮灰度图和判定保存到该目录；'
                        '用于现场抓「偶尔跳错」的原始素材，默认关闭')
    p.add_argument('--hang-timeout',type=float,default=60.0,
                   help='主循环多少秒没有心跳就判定卡死（导出线程栈后退出，交由 live_watchdog 重启）；'
                        '0 关闭。正常一轮最慢约 5 秒，默认 60 秒留足余量')
    p.add_argument('--health-every',type=int,default=20,
                   help='每多少轮记录一次 RSS/可用内存/负载/SoC 温度，默认20；设0关闭。'
                        '用于定位整机卡死：板上日志曾出现收图 9.3 秒、推理 5.6 秒而无 OOM 记录')
    p.add_argument('--last-good-hold',type=int,default=10,
                   help='屏幕上没有箭头时，回退到本分支族最近一次有颜色的显示，'
                        '最多回退这么多轮，默认 10；0=关闭。'
                        '红绿灯另有更短的上限（过期绿灯是危险方向），见 --last-good-traffic-hold')
    p.add_argument('--last-good-traffic-hold',type=int,default=4,
                   help='红绿灯族回退到旧灯色的最长轮数，默认 4（比车道短很多：'
                        '过期的绿灯会让人以为还能走）。0 表示跟 --last-good-hold 一样')
    p.add_argument('--traffic-layout-confirm-frames',type=int,default=2,
                   help='红绿灯框数连续几轮一致才改方向语义，默认2。框数是方向语义的'
                        '输入（1框=左直同色 / 2框=左+直 / 3框=左直右），抖动一轮就会'
                        '让左/直/右整体错位；2026-10-29 用户反馈「红绿灯会乱跳」即此。'
                        '设1关闭（框数一变立刻采纳）')
    p.add_argument('--traffic-release-frames',type=int,default=12,
                   help='红绿灯分支在"见过真灯箱"之后，要连续多少轮都拿不到灯箱大小的框'
                        '才判定路口已过、把画面让给车道分支，默认12。'
                        '2026-10-29 高速幻影红绿灯：高速上没有真灯，所以这个值只影响'
                        '过路口后的额外轮数（实测真路口灯箱漏检最长9轮）；设0表示'
                        '一拿到框就立即放开（会被单帧漏检抖掉）')
    p.add_argument('--no-box-hold-frames',type=int,default=3,
                   help='红绿灯连续几轮没检出框才清零箭头，默认 3；'
                        '0 表示永远沿用上一轮灯色。2026-10-29 用户定：单轮漏检不该灭灯，'
                        '一直挂着消失的灯也不行。'
                        '⚠️ 设 1 等于"一帧没框就清零"：二值帧上灯箱必然检不到框，'
                        '每两轮就会闪一次全红（2026-10-31 复核发现旧默认值就是 1，'
                        '与这段帮助文字自相矛盾）')
    p.add_argument('--scene-retry-window',type=int,default=3,
                   help='只在「场景刚刚变空」的头几轮才重问，默认 3；设 0 表示一直重问。'
                        '真·空白画面每轮多跑一次 480ms 会让单轮从 700ms 涨到 1200ms')
    p.add_argument('--scene-retries',type=int,default=1,
                   help='场景模型一次什么也没检到时立刻重问几次，默认 1（最多问 2 次）；'
                        '0 关闭。空答案不缓存，且不会把空答案写进节流缓存')
    p.add_argument('--mem-trim-every',type=int,default=50,
                   help='每多少轮做一次 gc.collect()+malloc_trim(0)，把本进程空闲页还给内核，默认50；设0关闭')
    p.add_argument('--mem-restart-every',type=int,default=250,
                   help='每多少轮做一次进程内重启（os.execv，不重启 Linux），重建 NPU 上下文，'
                        '默认250；设0关闭。2026-10-28 上板：进程外占用每100轮漏约64MB，'
                        'trim 收不回来，只能靠进程退出释放')
    p.add_argument('--max-infer-failures',type=int,default=20,
                   help='模型/决策连续失败多少轮就判定为"不是偶发"，走程序重启，默认20；'
                        '0 关闭。2026-10-29 上板：一个参数名笔误让 458 轮全部 TypeError，'
                        '这些错误原本只被吞成日志，屏幕上什么都看不出来')
    p.add_argument('--stop-on-infer-error',action='store_true')
    p.add_argument('--diag-frames',type=int,default=0,
                   help='前 N 轮把原始帧 + 取值谱 + 判定一起导出到 --diag-dir，默认 0 关闭。'
                        '用于确认"模型到底看到了什么图"：真灰度几十~几百级，'
                        '阈值图十几级，纯二值只有 {0,253}。排查完记得关掉，别写爆存储')
    p.add_argument('--diag-dir',type=Path,default=None,
                   help='帧诊断导出目录，默认 ./diag_frames')
    p.add_argument('--no-cop-fingerprint', dest='cop_fingerprint',
                   action='store_false', default=True,
                   help='关闭交警手势分支（默认开启）。开启后识别到有效手势会**只发一个'
                        '箭头**，FPGA 据此播报交警语音；关闭则完全走原有逻辑。'
                        '判据是手势分类模型的输出（左转/直行/other）')
    p.add_argument('--cop-window',type=int,default=12,
                   help='交警手势多数票窗口（轮），默认 12 ≈ 板上 8 s')
    p.add_argument('--cop-lock-rounds',type=int,default=None,
                   help='连续多少轮同一手势才进入交警模式。不传时按类别取默认值。'
                        '置信度门槛：左转 0.95 / 直行 0.90。传值则两类都用这个值')
    p.add_argument('--cop-release-rounds',type=int,default=1,
                   help='连续多少轮模型给出 other / 低于置信度门槛就退出交警模式，默认 1'
                        '（V28.7：3 → 1，用户要求"一旦没识别到交警就退出"）')
    p.add_argument('--log-level',choices=('DEBUG','INFO','WARNING','ERROR'),default='INFO')
    return p


def default_argcheck(args):
    """启动期的配置自相矛盾一律在这里报错，不要留到板上静默失明。"""
    if not 0 <= args.person_y <= 1 or not 0 <= args.person_x < .5:
        raise SystemExit('--person-y / --person-x 超范围')
    # 2026-10-31：二值请求通道已整个删除，这里不再有"车道变体 vs 采集模式"
    # 的矛盾可查。车道固定 G640、采集固定 MODE 1，唯一的组合。
    # 若哪天又冒出二值开关，tools_selfcheck 的"运行时不出现二值请求"
    # 断言会先把它拦在启动之前。


def main():
    args=parser().parse_args()
    logging.basicConfig(level=getattr(logging,args.log_level),
                        format='%(asctime)s %(levelname)s %(message)s')
    default_argcheck(args)
    if (args.pair_timeout<=0 or args.rx_idle_timeout<=0 or args.rx_drain_ms<=0 or
        args.rx_bad_header_limit<1 or args.rx_drain_max<1 or
        args.rx_mode0_grace_ms<0 or args.hang_timeout<0 or
        args.max_retries<1 or args.max_pair_failures<0 or
        args.restart_max_streak<0 or args.restart_delay<0 or
        args.solid_confirm_frames<1 or args.empty_scene_confirm_frames<1 or
        args.warning_confirm_frames<1 or args.nav_hold_frames<0 or
        args.traffic_hold_frames<0 or args.traffic_green_hold_frames<0 or
        args.nav_turn_confirm_frames<1 or not 0<args.nav_turn_confident<=1 or
        args.scene_nav_max_hold<0 or args.scene_traffic_max_hold<0 or args.mem_trim_every<0 or
        args.mem_restart_every<0 or args.last_good_hold<0 or
        args.last_good_traffic_hold<0 or args.scene_retries<0 or
        args.scene_retry_window<0 or args.no_box_hold_frames<0 or args.traffic_release_frames<0 or args.traffic_layout_confirm_frames<1 or args.max_infer_failures<0 or
        not 0<=args.lane_band_top<args.lane_band_bot<=1):
        raise SystemExit('接收/恢复参数无效：请检查超时、重试和重启保护参数')
    if args.gray_npy:
        run_offline(args)
    else:
        while True:
            try:
                run_live(args)
                break
            except ProgramRestartRequested as exc:
                # run_live has already closed the UDP sockets and released NPU handles.
                # Network/FPGA may be offline for minutes. Keep restarting, but do
                # not busy-spin or repeatedly initialize the NPU without delay.
                consecutive = max(1, restart_streak())
                delay = min(15.0, max(args.restart_delay, 0.5) * (2 ** min(consecutive-1, 5)))
                LOG.critical('%s；%.2f秒后从 main.py 第一行重新开始（失败次数=%d，不重启Linux）',
                             exc, delay, consecutive)
                time.sleep(delay)
                try:
                    reexec_program(Path(__file__).resolve(), sys.argv[1:])
                except OSError:
                    # If Python itself cannot exec (e.g. filesystem temporarily busy),
                    # still do not exit: reconstruct all sockets/models in this process.
                    LOG.exception('Python execv 失败；改为当前进程重新初始化全部组件，不退出。')
                    continue
                else:
                    # os.execv never returns in real execution. For mocked tests, stop here.
                    return

if __name__=='__main__':main()
