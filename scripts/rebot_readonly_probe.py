#!/usr/bin/env python3
"""Explicit read-only diagnostic; never enable, move, set mode or write parameters.

Does not run at service startup. Refuses other CAN subscribers, closes only the
bus and handles. Reporting known model mapping is not physical model identity.
"""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time
from types import SimpleNamespace
from rebot_mit_hardware import SdkTransport

def probe(output, rounds=3):
    bus=SdkTransport(); report=dict(source='explicit-sdk-readonly-probe',started_at=time.time(),
        hardware_motion_commands=0,model_source='existing official ID/model configuration, not model readback',
        serial_identity_verified=False,rounds=[],errors=[],physical_acceptance=False)
    try:
        bus.open_readonly()
        report['reply_ids']={str(i):bus.motors[i].robstride_ping_host_id(0xFD,300) for i in range(1,7)}
        if any(ids[0]!=int(i) for i,ids in report['reply_ids'].items()): raise ValueError('响应 ID 不一致')
        for _ in range(rounds):
            started=time.monotonic()
            samples=bus.read(range(1,7),SimpleNamespace(io_timeout=.2,feedback_age=2.))
            report['rounds'].append(dict(time_unix_s=time.time(),duration_s=time.monotonic()-started,
                samples={str(i):asdict(s) for i,s in samples.items()}))
    except Exception as error: report['errors'].append(str(error))
    finally:
        try: bus.close()
        except Exception as error: report['errors'].append('close: '+str(error))
        report['ended_at']=time.time(); report['ok']=not report['errors'] and len(report['rounds'])==rounds
        output=Path(output); output.parent.mkdir(parents=True,exist_ok=True)
        output.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    return report

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(); result=probe(args.output)
    print(json.dumps(result,ensure_ascii=False,indent=2)); raise SystemExit(0 if result['ok'] else 1)
