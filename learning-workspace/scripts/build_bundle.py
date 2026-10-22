#!/usr/bin/env python3
"""Package only distributable files; exclude user configuration, data, and environments."""
import hashlib
import gzip
from pathlib import Path
import re
import shutil
import tarfile

ROOT = Path(__file__).resolve().parents[1]
FILES = ["VERSION", "LICENSE", "CHANGELOG.md", "README.md", "install.sh", "robotctl", "config.example.json", "requirements.in",
         "requirements-cpu.lock", "requirements-cuda.lock", ".gitignore", "THIRD_PARTY_NOTICES.md",
         "VERIFICATION.md", "scripts/bootstrap_uv.py", "scripts/robotctl.py", "scripts/self_test.py",
         "scripts/build_bundle.py", "tests/test_workflow.py", "vendor/ORIGIN.json",
         "vendor/LICENSE.lerobot", "vendor/lerobot-0.4.4-py3-none-any.whl",
         "docs/decisions/0001-linux-deployment.md", "docs/decisions/0002-github-releases.md",
         "docs/SUPPORT.md", "docs/RELEASING.md", "scripts/system_deps.sh", "install-release.py",
         "tests/test_release.py", "deployment/rebotarm/README.md"]


def asset_name(version):
    if not re.fullmatch(r"[0-9]+\.[0-9]+\.[0-9]+", version):
        raise ValueError("VERSION 必须是数字形式的 major.minor.patch。")
    return f"seeed-lerobot-{version}-linux-x86_64.tar.gz"


def main():
    version = (ROOT / "VERSION").read_text().strip()
    output = ROOT / "dist" / asset_name(version)
    output.parent.mkdir(exist_ok=True)
    # Stable metadata makes repeated builds from the same files byte-identical.
    with output.open("wb") as raw, gzip.GzipFile(filename="", fileobj=raw, mode="wb", mtime=0) as compressed:
        with tarfile.open(fileobj=compressed, mode="w") as archive:
            for name in sorted(FILES):
                source = ROOT / name
                info = archive.gettarinfo(source, arcname="seeed-lerobot-linux/" + name)
                info.uid = info.gid = info.mtime = 0
                info.uname = info.gname = ""
                info.mode = 0o755 if name in ("install.sh", "robotctl") else 0o644
                with source.open("rb") as stream:
                    archive.addfile(info, stream)
    digest = hashlib.sha256(output.read_bytes()).hexdigest()
    output.with_suffix(output.suffix + ".sha256").write_text(f"{digest}  {output.name}\n")
    shutil.copyfile(ROOT / "install-release.py", output.parent / "install-release.py")
    installer_hash = hashlib.sha256((output.parent / "install-release.py").read_bytes()).hexdigest()
    (output.parent / "SHA256SUMS").write_text(
        f"{digest}  {output.name}\n{installer_hash}  install-release.py\n")
    print(f"{output} ({output.stat().st_size / 1024 / 1024:.2f} MiB)")


if __name__ == "__main__":
    main()
