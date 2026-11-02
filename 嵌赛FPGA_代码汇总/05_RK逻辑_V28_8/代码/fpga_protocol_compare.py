#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Raw protocol comparison for FPGA firmware versions. Never loads RKNN."""
import argparse
import collections
import json
import socket
import time
import struct
import subprocess

def run(local_ip,local_port,fpga_ip,fpga_port,seconds=2.0):
    cases=[('passive',None),('legacy_one_byte_00',b'\x00'),
           ('v12_gray_0004',b'\x00\x04'),('v12_binary_0008',b'\x00\x08')]
    out={}
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,16*1024*1024)
        sock.bind((local_ip,local_port))
        sock.settimeout(.1)
        for name,cmd in cases:
            # Allow old buffered traffic to empty before each test.
            sock.setblocking(False)
            for _ in range(4096):
                try:sock.recvfrom(4096)
                except (BlockingIOError,InterruptedError):break
            sock.settimeout(.1)
            if cmd is not None:
                sock.sendto(cmd,(fpga_ip,fpga_port))
            print('测试 {} 发送 {}'.format(name,cmd.hex(' ') if cmd is not None else '不发送'),flush=True)
            summary={'received':0,'sources':{},'sizes':{},'header_matches':0,
                     'pixel_sample_values':[],'first_8_bytes':[]}
            end=time.monotonic()+seconds
            samples=set()
            while time.monotonic()<end:
                try:data,addr=sock.recvfrom(4096)
                except socket.timeout:continue
                summary['received']+=1
                k='{}:{}'.format(addr[0],addr[1]); summary['sources'][k]=summary['sources'].get(k,0)+1
                sz=str(len(data));summary['sizes'][sz]=summary['sizes'].get(sz,0)+1
                if len(summary['first_8_bytes'])<3:summary['first_8_bytes'].append(data[:8].hex(' '))
                if len(data)==1288:
                    fid,row,height=struct.unpack_from('>IHH',data)
                    if 0<=row<720 and height==720:summary['header_matches']+=1
                    if len(samples)<16:samples.update(data[8: min(len(data),72)])
            summary['pixel_sample_values']=sorted(samples)[:16]
            out[name]=summary
            print(json.dumps({name:summary},ensure_ascii=False),flush=True)
    finally:sock.close()
    try:
        p=subprocess.run(['ip','neigh','show',fpga_ip],stdout=subprocess.PIPE,stderr=subprocess.PIPE,universal_newlines=True)
        out['arp_neighbor']=p.stdout.strip() or p.stderr.strip()
    except Exception:out['arp_neighbor']='ip 命令不可用'
    print('===== 所有诊断信息 =====',flush=True)
    print(json.dumps(out,indent=2,ensure_ascii=False),flush=True)
    print('提示：文档单字节 00/1..9 是显示控制；V12 双字节 00 04/00 08 是采图请求。'
          '若仅支持旧单字节指令，应更新 FPGA 固件协议，不能只靠修改 RK3568 Python 实现 V12 的双模式请求。',flush=True)
    return out

if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--local-ip',default='0.0.0.0')
    p.add_argument('--local-port',type=int,default=1234)
    p.add_argument('--fpga-ip',default='192.168.1.10')
    p.add_argument('--fpga-port',type=int,default=1234)
    p.add_argument('--seconds',type=float,default=2.0)
    a=p.parse_args()
    run(a.local_ip,a.local_port,a.fpga_ip,a.fpga_port,a.seconds)
