#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Python 3.8-compatible, dependency-free FPGA UDP probe.
Does not require RKNN, and purposely DOES NOT filter incoming source before reporting.
"""
import argparse
import collections
import ipaddress
import json
import socket
import struct
import subprocess
import time


def _run(*args):
    p=subprocess.run(args,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    return p.stdout.strip() or p.stderr.strip()


def probe(fpga_ip, fpga_port, local_ip, listen_port, wait, mode, count, expected_source_port, quiet=False):
    state={"local_ip":local_ip,"listen_port":listen_port,"fpga_ip":fpga_ip,
           "fpga_port":fpga_port,"expected_source_port":expected_source_port,"mode":mode,"received":0,"expected_rows":0,
           "sources":{},"packet_sizes":{},"first_packets":[],
           "neighbor_before":_run('ip','neigh','show',fpga_ip),"route":_run('ip','-4','route','get',fpga_ip)}
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,8*1024*1024)
        sock.bind((local_ip,listen_port))
        sock.settimeout(0.2)
        if not quiet:
            print('监听 %s:%d；FPGA %s:%d；发送模式%d 请求' %
                  (local_ip,listen_port,fpga_ip,fpga_port,mode),flush=True)
        for attempt in range(count):
            # Mode bits are 10..11, no arrows enabled.
            cmd=struct.pack('<H',mode<<10)
            sock.sendto(cmd,(fpga_ip,fpga_port))
            if not quiet:print('发送 #%d: %s' %(attempt+1,cmd.hex(' ')),flush=True)
            end=time.monotonic()+wait
            while time.monotonic()<end:
                try:data,addr=sock.recvfrom(4096)
                except socket.timeout:continue
                state['received']+=1
                if addr[0]==fpga_ip and addr[1]==expected_source_port and len(data)==1288:
                    state['expected_rows']+=1
                k='%s:%d'%addr
                state['sources'][k]=state['sources'].get(k,0)+1
                n=len(data)
                state['packet_sizes'][str(n)]=state['packet_sizes'].get(str(n),0)+1
                if len(state['first_packets'])<3:
                    state['first_packets'].append({'source':k,'size':n,'hex16':data[:16].hex(' ')})
        state['neighbor_after']=_run('ip','neigh','show',fpga_ip)
    except OSError as e:
        state['socket_error']='%s: %s'%(type(e).__name__,str(e))
    finally:sock.close()
    if not quiet:
        print('===== FPGA UDP 诊断结果 =====')
        print(json.dumps(state,ensure_ascii=False,indent=2),flush=True)
        print('====== 诊断结论 ======',flush=True)
        if state.get('socket_error'):
            print('套接字错误，请先检查本机 IPv4/端口占用。',flush=True)
        elif state['received']==0:
            print('0 个 UDP 回包：与 RKNN 模型无关。检查邻居表是否 FAILED/INCOMPLETE、FPGA 的 IP/MAC/目的地址/UDP 端口及是否响应模式1请求。',flush=True)
            print('可用 sudo tcpdump -ni eth0 -e "arp or udp" 对照实际二层包（仅在已安装 tcpdump 时）。',flush=True)
        elif state['expected_rows']>0:
            print('收到来源与格式都符合预期的 1288 字节图像行包。',flush=True)
        else:print('收到 UDP，但是来源 IP/端口或 1288 字节格式不符；请检查 FPGA 源端口、图像模式及协议。',flush=True)
    return 0 if state['expected_rows']>0 else (4 if state['received']==0 else 5)


def main():
    p=argparse.ArgumentParser(description='不依赖 RKNN 的 FPGA UDP 原始回包测试')
    p.add_argument('--fpga-ip',default='192.168.1.10')
    p.add_argument('--fpga-port',type=int,default=1234)
    p.add_argument('--expected-source-port',type=int,default=None)
    p.add_argument('--local-ip',default='192.168.1.20')
    p.add_argument('--listen-port',type=int,default=1234)
    p.add_argument('--wait',type=float,default=2.0)
    p.add_argument('--mode',type=int,choices=(1,2),default=1)
    p.add_argument('--count',type=int,default=2)
    p.add_argument('--quiet',action='store_true')
    a=p.parse_args()
    for val in (a.fpga_ip,a.local_ip):ipaddress.IPv4Address(val)
    if a.wait<=0 or a.count<1: p.error('wait/count must be positive')
    return probe(a.fpga_ip,a.fpga_port,a.local_ip,a.listen_port,a.wait,a.mode,a.count,
                 a.expected_source_port if a.expected_source_port is not None else a.fpga_port,a.quiet)

if __name__=='__main__':
    raise SystemExit(main())
