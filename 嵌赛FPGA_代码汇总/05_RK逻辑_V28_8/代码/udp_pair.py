"""Raw UDP row reconstruction and one-frame-per-request collector.

2026-10-31：全工程只请求 MODE 1（YRG 灰度）。二值模式（MODE 2）的采集通道
已整个删除。图像 UDP 包头 >IHH 没有模式位、没有请求 ID，这在只有一种模式的
时候不再是问题 —— 也就不可能再从数据里分不清"这是哪个模式的在途帧"。
每个请求用新的组帧器；不完整的响应在重发时丢弃。
"""
import socket
import struct
import time
from dataclasses import dataclass
import numpy as np

W, H, ROW_BYTES = 1280, 720, 1288
HEADER = struct.Struct('>IHH')

@dataclass
class ImageFrame:
    frame_id: int
    pixels: np.ndarray


class FrameAssembler:
    def __init__(self, max_partial=4):
        self.pending = {}
        self.max_partial = max_partial
        self.invalid = 0
        self.duplicates = 0

    def add(self, packet: bytes):
        if len(packet) != ROW_BYTES:
            self.invalid += 1
            return None
        fid, row, height = HEADER.unpack_from(packet)
        if height != H or row >= H:
            self.invalid += 1
            return None
        p = self.pending.get(fid)
        if p is None:
            if len(self.pending) >= self.max_partial:
                del self.pending[next(iter(self.pending))]
            p = [np.empty((H, W), np.uint8), np.zeros(H, np.bool_), 0]
            self.pending[fid] = p
        data, seen, count = p
        if seen[row]:
            self.duplicates += 1
            return None
        data[row] = np.frombuffer(packet, dtype=np.uint8, count=W, offset=8)
        seen[row] = True
        p[2] = count + 1
        if p[2] == H:
            del self.pending[fid]
            return ImageFrame(fid, data)
        return None


def is_binary(image):
    """FPGA 二值帧：像素只能是 0（黑）或 253（白）。

    ⚠️ 2026-10-31：RK 端**不再请求**这种帧，本函数只用于识别"上游给错了图"
    （见 looks_thresholded），以及离线工具读旧 npy。工程里没有任何路径
    会因为它的判定去丢弃或转换一帧。
    """
    if image.dtype != np.uint8:
        return False
    return float(np.mean((image == 0) | (image == 253))) >= 0.999


def looks_thresholded(image):
    """Diagnostic only: TRUE when a MODE 1 frame carries only {0, 253}.

    MODE 1 是 YRG：灰度 0..253 合法，254/255 标红/绿。真 8-bit 灰度帧的
    取值集合永远跨很多级，所以"整帧只有 {0,253} 这两个值"说明 FPGA 的模式 1
    位流还没把 `median_data`（中值滤波后的真灰度）接出来，发的是阈值图。

    这不是 RK 端二值化的产物 —— RK 端一个阈值运算都没有。它只用来让灯箱
    本轮不判色（错误汇集第 7 条：阈值图里数不到 254/255 → 全向红）。

    全黑帧（只有 0）**不算**可疑：夜间无灯场景合法。要求两个值都出现。
    永远不要用这个结论去丢帧，它只是日志里的提示。
    """
    if image.dtype != np.uint8:
        return False
    values = np.unique(image)
    return values.size == 2 and bool(np.isin(values, (0, 253)).all())


# 旧名保留：旧版测试与外部脚本仍在 import `looks_binary`。
looks_binary = looks_thresholded


class FastProtocolMismatch(ValueError):
    """Received an unequivocal, sustained wrong wire format; retry early."""


class UDPLink:
    """MODE 1 (grey) UDP collector with low-overhead drain and early fault detection.

    2026-10-31：只请求 MODE 1，没有第二种模式可切换。响应头 >IHH 依旧没有
    模式位/请求 ID，但在单模式下这不再构成歧义。永远不拼接不完整的帧。
    """

    def __init__(self, bind='0.0.0.0', port=1234, source_ip='192.168.1.10',
                 source_port=1234, signal_ip='192.168.1.10', signal_port=1234,
                 rcvbuf=4*1024*1024):
        """rcvbuf 默认 4 MB（内核翻倍成 8 MB，约 9 帧）。

        2026-10-31：原来 16 MB（实际 32 MB，约 36 帧）。一帧 927 360 B，
        FPGA 按 53 fps 推流 ≈ 47 MB/s，单轮 680 ms 就推 32 MB —— 缓冲越大，
        队列头越陈旧，模式切换后要等整个队列翻过去才收到新模式。实测
        `清理旧UDP包` 在 0~1200 之间乱跳，正是这个原因。

        取 4 MB（≈9 帧，约 0.17 s 陈旧）：仍能吸收一轮推理期间的积压，
        同时让"排空"真的排得空 —— 排空上限 4096 包 ≈ 5.7 帧，缓冲若比这还深，
        排空永远追不上（旧配置 36 帧 vs 5.7 帧就是这个局面）。
        """
        import logging
        self.log=logging.getLogger('udp_pair')
        self.source_ip=source_ip
        self.source_port=source_port
        self.target=(signal_ip,signal_port)
        self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,rcvbuf)
            self.sock.bind((bind,port))
        except Exception:
            self.sock.close()
            raise
        self.sock.settimeout(.1)
        self.recv_buf=bytearray(2048)
        self.pixel_view=np.frombuffer(self.recv_buf,dtype=np.uint8,count=W,offset=8)
        self.last_report={}
        self.last_frame_id=None
        self.last_mode=None
        # 请求的是 MODE 1，但收到的帧取值塌缩成 {0,253}：FPGA 的模式 1 位流
        # 还在发阈值图（median_data 没接出来）。红绿灯在那种图上读不到
        # 254/255，必须让它本轮不判灯。这不是 RK 端二值化。
        self.last_frame_thresholded=False
        self.last_drain_hit_limit=False
        self.drain_budget_ms=15.0
        self.drain_max_packets=4096
        self.idle_timeout=.8
        self.bad_header_limit=64
        self.bad_length_limit=64
        self.mode0_grace_ms=35.0
        # 最近一次成功组帧的帧号，供"帧号屏障"跳过在途旧帧使用。
        self.latest_frame_id = None

    def configure_receiver(self, *, drain_budget_ms=None, drain_max_packets=None,
                           idle_timeout=None, bad_header_limit=None,
                           bad_length_limit=None, mode0_grace_ms=None):
        for key,val in [('drain_budget_ms',drain_budget_ms),
                        ('drain_max_packets',drain_max_packets),
                        ('idle_timeout',idle_timeout),
                        ('bad_header_limit',bad_header_limit),
                        ('bad_length_limit',bad_length_limit)]:
            if val is not None:
                if float(val)<=0:raise ValueError(key+' must be positive')
                setattr(self,key,val)
        if mode0_grace_ms is not None:
            if float(mode0_grace_ms)<0:raise ValueError("mode0_grace_ms must be >=0")
            self.mode0_grace_ms=float(mode0_grace_ms)

    def drain_pending(self,max_packets=None):
        """Bound old-packet flushing by BOTH packet count and a wall-time budget.

        FPGA streaming may never pause; a flush-until-empty loop can therefore
        starve new requests. Flushing does not establish an FPGA mode barrier.
        """
        if max_packets is None:max_packets=self.drain_max_packets
        start=time.perf_counter()
        count=0
        self.sock.setblocking(False)
        try:
            while count<max_packets and (time.perf_counter()-start)*1000<self.drain_budget_ms:
                try:
                    self.sock.recvfrom_into(self.recv_buf)
                    count+=1
                except (BlockingIOError,InterruptedError):
                    break
                except OSError as exc:
                    # A drain is pure housekeeping: the NIC going down, running
                    # out of socket buffer, or a route disappearing must not
                    # abort the run. Report it as zero packets drained and let
                    # the real request surface the fault through send().
                    self.last_drain_error=f'{type(exc).__name__}: {exc}'
                    break
        finally:
            self.sock.settimeout(.1)
        elapsed=(time.perf_counter()-start)*1000
        self.last_drain_hit_limit=(count>=max_packets or elapsed>=self.drain_budget_ms)
        self.last_drain_ms=round(elapsed,2)
        return count

    def send(self,command: bytes):
        if len(command)!=2:raise ValueError('new FPGA commands must be EXACTLY 2 bytes')
        try:
            return self.sock.sendto(command,self.target)
        except OSError as exc:
            # A transient link drop (NO-CARRIER / route gone) must not kill the
            # process. main.py's capture-failure path keeps the last display and
            # resyncs; bubbling OSError used to escape that handler and force a
            # full program restart, which also wiped the solid-confirm streaks
            # and left one side's arrow hidden or stale.
            raise TimeoutError(f'发送采图请求失败（网络不可达？）: {exc}') from exc

    def collect_frame(self,timeout=2.,expected_mode=None):
        """Return the first complete image. Detect persistent mismatched headers early.

        `timeout` bounds the whole request. `idle_timeout` bounds silence since
        request/last matching UDP packet. A continuous stream of invalid packets
        is terminated early once the bad-header threshold is reached with no
        valid mode1/2 rows. Missing rows are NEVER invented or borrowed.

        Mode is verified by CONTENT, not by frame id: the FPGA's frame_id is a
        single monotonic counter that keeps advancing across mode switches, so
        "a low frame id" does not mean "the previous mode". A MODE 2 request
        only accepts an all-{0,253} frame, and a MODE 1 request tags a frame
        that *looks* binary so the caller can skip colour judgments.
        """
        from collections import OrderedDict
        if expected_mode not in (1,2):raise ValueError('expected_mode must be 1 or 2')
        if timeout<=0:raise ValueError('timeout must be positive')
        pending=OrderedDict()
        diag={'udp_received':0,'source_matched':0,'wrong_source_ip':0,
              'wrong_source_port':0,'wrong_packet_length':0,'bad_header':0,
              'valid_rows':0,'duplicate_rows':0,'new_frame_ids':0,
              'partial_evicted':0,'old_frame_id':0,
              'mode0_header_like':0,'bad_header_examples':[],
              'early_abort_reason':None}
        start=time.monotonic()
        end=start+timeout
        last_rx=start

        def finalize(reason=None):
            diag['elapsed_ms']=round((time.monotonic()-start)*1000,2)
            diag['incomplete_frames']=len(pending)
            diag['max_rows_in_partial']=max((p[2] for p in pending.values()),default=0)
            if reason:diag['early_abort_reason']=reason
            self.last_report=diag

        while True:
            now=time.monotonic()
            # A finite RGB565 burst can end before the grace interval expires.
            # Do not spend the entire idle timeout waiting for more wrong packets.
            if (diag['valid_rows']==0 and diag['bad_header']>=self.bad_header_limit
                    and (now-start)*1000>=self.mode0_grace_ms):
                kind=('mode0_header_burst' if diag['mode0_header_like']>=self.bad_header_limit//2
                      else 'bad_header_burst')
                finalize(kind)
                raise FastProtocolMismatch(f'{kind} mode={expected_mode}; stats={diag}')
            if now>=end:
                finalize('total_timeout')
                raise TimeoutError(f'single-frame timeout={timeout}s mode={expected_mode}; stats={diag}')
            if now-last_rx>=self.idle_timeout:
                finalize('rx_idle_timeout')
                raise TimeoutError(f'UDP quiet for {self.idle_timeout}s mode={expected_mode}; stats={diag}')
            self.sock.settimeout(min(.12,max(.001,end-now),max(.001,self.idle_timeout-(now-last_rx))))
            try:
                n,addr=self.sock.recvfrom_into(self.recv_buf)
            except socket.timeout:
                continue
            except OSError as exc:
                # Errno 101/113 (network unreachable / no route) arrive here
                # when the cable is pulled or the FPGA power-cycles. They used
                # to escape this method as OSError, which main.py does not
                # catch, so the whole process died -- three of the five
                # restarts in rk3568_v15_7.log. Surface them on the same
                # TimeoutError path the retry/restart logic already handles.
                diag['rx_oserror']=diag.get('rx_oserror',0)+1
                diag['rx_oserror_last']=f'{type(exc).__name__}: {exc}'
                finalize('rx_oserror')
                raise TimeoutError(f'接收时网卡/路由故障 mode={expected_mode}: '
                                   f'{exc}; stats={diag}') from exc
            diag['udp_received']+=1
            if self.source_ip and addr[0]!=self.source_ip:
                diag['wrong_source_ip']+=1
                continue
            if self.source_port is not None and addr[1]!=self.source_port:
                diag['wrong_source_port']+=1
                continue
            last_rx=time.monotonic()
            diag['source_matched']+=1
            if n!=ROW_BYTES:
                diag['wrong_packet_length']+=1
                if diag['wrong_packet_length']>=self.bad_length_limit and diag['valid_rows']==0:
                    finalize('bad_packet_length_burst')
                    raise FastProtocolMismatch(f'wrong UDP payload lengths repeated mode={expected_mode}; stats={diag}')
                continue
            fid,row,height=HEADER.unpack_from(self.recv_buf)
            if height!=H or row>=H:
                diag['bad_header']+=1
                # The FPGA streams a 54 fps RGB565 frame when no image-mode
                # command has been latched, so its header reads 0x0000 in bytes
                # 0..5 and 0x00 in byte 6. Interleaving that stream with the
                # requested picture is what turns one frame_id into two random
                # frames; measured 32 289 packets in 2 s that never assembled.
                # Naming it explicitly in the report saves the operator from
                # hunting a phantom cabling fault.
                if (self.recv_buf[0:2]==b'\x00\x00' and
                        self.recv_buf[6]==0 and self.recv_buf[7] in (0,1)):
                    diag['mode0_header_like']+=1
                if len(diag['bad_header_examples'])<4:
                    diag['bad_header_examples'].append({
                        'hex8':bytes(self.recv_buf[:8]).hex(' '),
                        'as_mode12_frame_id':int(fid),
                        'as_mode12_line':int(row),
                        'as_mode12_height':int(height)})
                # Don't wait 5s to reject an entire burst of mode0 / malformed packets.
                if diag['bad_header']>=self.bad_header_limit and diag['valid_rows']==0:
                    mode0_like=diag['mode0_header_like']>=self.bad_header_limit//2
                    # A mode switch is asynchronous. Old RGB565 rows in the socket
                    # during the first few ms are not proof the new mode failed.
                    # Give FPGA a bounded grace period; reject persistent mismatch.
                    grace_remaining=(time.monotonic()-start)*1000 < self.mode0_grace_ms
                    if not (mode0_like and grace_remaining):
                        kind='mode0_header_burst' if mode0_like else 'bad_header_burst'
                        finalize(kind)
                        raise FastProtocolMismatch(f'{kind} mode={expected_mode}; stats={diag}')
                continue
            if self.last_mode==expected_mode and self.last_frame_id is not None:
                delta=(fid-self.last_frame_id)&0xFFFFFFFF
                if not 0<delta<0x80000000:
                    diag['old_frame_id']+=1
                    continue
            diag['valid_rows']+=1
            acc=pending.get(fid)
            if acc is None:
                if len(pending)>=4:
                    pending.popitem(last=False)
                    diag['partial_evicted']+=1
                acc=[np.empty((H,W),np.uint8),np.zeros(H,np.bool_),0]
                pending[fid]=acc
                diag['new_frame_ids']+=1
            if acc[1][row]:
                diag['duplicate_rows']+=1
                continue
            acc[0][row,:]=self.pixel_view
            acc[1][row]=True
            acc[2]+=1
            if acc[2]==H:
                data=acc[0]
                del pending[fid]
                # ── 帧诊断 ─────────────────────────────────────────────
                # 2026-10-31：用户要"先看喂给模型的到底是什么图"。
                # 这里把每帧的取值谱打出来，一眼就能分辨
                #   * 真灰度：几十~几百个级数
                #   * 阈值图：十几个级数（FPGA 抖动后仍保留少量中间值）
                #   * 纯二值：只有 {0,253}
                uniq=np.unique(data)
                diag['frame_unique_count']=int(uniq.size)
                diag['frame_unique_head']=[int(x) for x in uniq[:8]]
                diag['frame_min']=int(uniq[0]); diag['frame_max']=int(uniq[-1])
                diag['frame_white_pct']=round(float((data > 128).mean()) * 100, 2)
                diag['frame_mean']=round(float(data.mean()), 2)
                # Diagnostic ONLY -- never reject a frame. 全工程只请求 MODE 1，
                # 这里唯一要回答的是"回来的到底是不是真灰度"。FPGA 的模式 1
                # 若还没把 median_data 接出来，回来的就是阈值图（取值塌缩成
                # 极少几级），红绿灯因此数不到 254/255。这不是 RK 端二值化。
                diag['gray_unique_values']=int(uniq.size)
                if looks_thresholded(data):
                    diag['looks_thresholded']=diag.get('looks_thresholded',0)+1
                    if len(diag.get('looks_thresholded_examples',[]))<3:
                        diag.setdefault('looks_thresholded_examples',[]).append({
                            'frame_id':int(fid),
                            'values':[int(x) for x in uniq[:8]],
                            'unique_values_count':len(uniq)})
                self.last_mode=expected_mode
                self.last_frame_id=fid
                self.latest_frame_id=fid
                diag['complete_frames']=1
                finalize()
                return ImageFrame(fid,data)

    def request_frame(self,display,timeout=2.,on_request=None):
        """发一条**灰度**指令（MODE 1），收一帧。

        2026-10-31：二值请求通道已整个删除（用户要求「不再请求二值化」）。
        全工程只有 MODE 1，所以这里不再有 image_mode 参数 —— 也就不可能
        误传模式、翻转模式、或收到另一个模式的在途帧。
        """
        from protocol import encode_packet, IMAGE_MODE
        dropped=self.drain_pending()
        command=encode_packet(display,IMAGE_MODE)
        self.send(command)
        if on_request:on_request(command,dropped)
        try:
            return self.collect_frame(timeout,expected_mode=IMAGE_MODE)
        except (TimeoutError,FastProtocolMismatch) as exc:
            raise type(exc)(f'{exc}; drained_before_request={dropped}; '
                            f'drain_ms={self.last_drain_ms}; '
                            f'drain_limit_hit={self.last_drain_hit_limit}') from exc

    def close(self):
        self.sock.close()
