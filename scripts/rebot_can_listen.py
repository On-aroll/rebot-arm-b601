#!/usr/bin/env python3
"""Receive raw CAN frames only. No motor SDK, send calls, enable or zero writes."""
import argparse
import collections
import datetime
import json
import socket
import struct
import subprocess
import time
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]

def interface_info(interface):
    result=subprocess.run(['ip','-details','-statistics','-json','link','show','dev',interface],capture_output=True,text=True,check=True)
    return json.loads(result.stdout)[0]

def require_listen_only(info):
    data=info.get('linkinfo',{}).get('info_data',{})
    if info.get('link_type')!='can':raise ValueError('所选接口不是 CAN 接口')
    if 'UP' not in info.get('flags',[]):raise ValueError('CAN 接口尚未启动；需要管理员先配置为 listen-only 并启动')
    # Supported modes do not mean enabled modes. Require the active flag.
    modes=data.get('ctrlmode',[])
    if not isinstance(modes,list) or 'LISTEN-ONLY' not in modes:
        raise ValueError('未确认启用 LISTEN-ONLY 模式，拒绝打开监听')
    if data.get('state') in ('STOPPED','BUS-OFF'):raise ValueError('CAN 接口处于停止或 bus-off 状态')

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface',default='can0')
    parser.add_argument('--seconds',type=float,default=8.)
    parser.add_argument('--include-errors',action='store_true',help='Also receive local CAN driver error notifications; still receive only')
    args=parser.parse_args()
    if not .1<=args.seconds<=30:parser.error('--seconds must be between 0.1 and 30')
    report=dict(time_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),interface=args.interface,mode='passive_listen_only',transmit_calls=0,feedback_verified=False)
    try:
        info=interface_info(args.interface);report['interface_before']=info;require_listen_only(info)
        histogram=collections.Counter();samples=[];count=0;error_count=0
        with socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW) as bus:
            if args.include_errors:
                bus.setsockopt(socket.SOL_CAN_RAW,2,struct.pack('=I',0x1FFFFFFF))  # CAN_RAW_ERR_FILTER
            bus.bind((args.interface,));end=time.monotonic()+args.seconds
            while time.monotonic()<end:
                bus.settimeout(min(.2,max(.001,end-time.monotonic())))
                try:frame=bus.recv(16)
                except socket.timeout:continue
                if len(frame)!=16:continue
                raw_id,length,payload=struct.unpack('=IB3x8s',frame)
                extended=bool(raw_id&0x80000000);is_error=bool(raw_id&0x20000000);can_id=raw_id&(0x1FFFFFFF if extended or is_error else 0x7FF)
                if is_error:error_count+=1
                count+=1;key=('error:' if is_error else 'frame:')+f'{can_id:08X}'
                if len(histogram)<4096 or key in histogram:histogram[key]+=1
                if len(samples)<200:samples.append(dict(id=key,extended=extended,remote=bool(raw_id&0x40000000),error=bool(raw_id&0x20000000),data=payload[:min(length,8)].hex()))
        report.update(frames_received=count,data_frames_received=count-error_count,error_frames_received=error_count,can_id_counts=dict(histogram),samples=samples,interface_after=interface_info(args.interface))
        report['result']='observed_raw_traffic' if count>error_count else ('driver_error_notifications_only' if error_count else 'no_passive_traffic')
        report['note']='仅记录原始报文，尚未验证电机身份、轴映射或编码器反馈。无报文不代表故障，电机可能需读取请求后才回复。'
    except (ValueError,OSError,subprocess.CalledProcessError,json.JSONDecodeError) as e:
        report.update(result='blocked',error=str(e))
    output=ROOT/'outputs/can_passive_probe.json';output.parent.mkdir(exist_ok=True);output.write_text(json.dumps(report,ensure_ascii=False,indent=2))
    summary={k:v for k,v in report.items() if k not in ('interface_before','interface_after','samples','can_id_counts')}
    print(json.dumps(summary,ensure_ascii=False,indent=2))
    return 1 if report['result']=='blocked' else 0

if __name__=='__main__':raise SystemExit(main())
