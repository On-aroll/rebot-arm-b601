#!/usr/bin/env python3
"""Explicit offline software-ledger recovery. Never imports or opens hardware."""
import argparse
import json
import os
from pathlib import Path
import shutil
import tempfile
import time
from rebot_config import ConfigStore, digest, validate_state


def restore(root, source):
    store=ConfigStore(root)
    with store.locked():
        source=Path(source).resolve()
        if source.parent != (store.root/'history').resolve():
            raise ValueError('只能恢复本配置目录的不可变历史记录；外部备份请在界面导入草稿')
        data=json.loads(source.read_text())
        if set(data) != {'state','checksum'} or digest(data['state']) != data['checksum']:
            raise ValueError('历史校验失败，保留现场')
        validate_state(data['state'])
        head=store.root/'machine_profile.json'
        backup=store.root/f'pre-recovery-{time.time_ns()}.json'
        if head.exists(): shutil.copy2(head,backup); os.chmod(backup,0o600)
        fd,name=tempfile.mkstemp(prefix='.restore-',dir=store.root)
        try:
            with os.fdopen(fd,'wb') as out:
                out.write(source.read_bytes()); out.flush(); os.fsync(out.fileno())
            os.replace(name,head)
            fd=os.open(store.root,os.O_RDONLY)
            try: os.fsync(fd)
            finally: os.close(fd)
        finally: Path(name).unlink(missing_ok=True)
        return data['state']['revision']

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('history_file',type=Path)
    p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1]/'outputs/private_config')
    p.add_argument('--confirm-software-only',action='store_true',required=True)
    a=p.parse_args()
    print('仅软件账本已恢复为历史版本',restore(a.root,a.history_file),'；必须重启本地服务以清空审批会话；实机仍 LOCKED')
