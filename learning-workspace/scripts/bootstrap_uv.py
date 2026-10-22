#!/usr/bin/env python3
"""Install one verified uv binary without touching system Python packages."""
import hashlib
import io
import os
from pathlib import Path
import tempfile
import urllib.request
import zipfile

ROOT = Path(__file__).resolve().parents[1]
URL = (
    "https://files.pythonhosted.org/packages/14/0b/"
    "6602a613533602e7a083e908c4375f9b0e78a94ba2e21c29b21c41a0a611/"
    "uv-0.13.1-py3-none-manylinux_2_17_x86_64.manylinux2014_x86_64.whl"
)
SHA256 = "696dcf6d807d85da978f4db8472da81dccc1254b18b52cbd1367c4d923c421e0"


def main():
    target = ROOT / ".tools/uv"
    if target.is_file():
        return
    print("下载固定版本 uv 0.13.1（校验 SHA256）…", flush=True)
    with urllib.request.urlopen(URL, timeout=180) as response:
        content = response.read()
    if hashlib.sha256(content).hexdigest() != SHA256:
        raise RuntimeError("uv 下载校验失败，请重新运行安装。")
    with zipfile.ZipFile(io.BytesIO(content)) as archive:
        binary = archive.read("uv-0.13.1.data/scripts/uv")
    target.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=target.parent, delete=False) as stream:
        stream.write(binary)
        temporary = Path(stream.name)
    temporary.chmod(0o755)
    os.replace(temporary, target)


if __name__ == "__main__":
    main()
