#!/usr/bin/env python3
"""Download and verify a GitHub Release, then install it as the current user."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shlex
import shutil
import subprocess
import sys
import tarfile
import tempfile
import urllib.request


def version_number(tag):
    if not re.fullmatch(r"v[0-9]+\.[0-9]+\.[0-9]+", tag):
        raise ValueError("版本必须形如 v0.1.0。")
    return tag[1:]


def download(url, target, api=False):
    if not url.startswith("https://"):
        raise ValueError("下载地址必须使用 HTTPS。")
    headers = {"User-Agent": "seeed-lerobot-installer", "Accept": "application/vnd.github+json"}
    if api:
        headers["Accept"] = "application/octet-stream"
    # Only send credentials to GitHub's API. Redirected binary asset requests do not inherit them.
    token = os.environ.get("GH_TOKEN") or os.environ.get("GITHUB_TOKEN")
    if token and url.startswith("https://api.github.com/"):
        headers["Authorization"] = f"Bearer {token}"
    request = urllib.request.Request(url, headers=headers)

    class Redirect(urllib.request.HTTPRedirectHandler):
        def redirect_request(self, request, fp, code, msg, headers, newurl):
            redirected = super().redirect_request(request, fp, code, msg, headers, newurl)
            if redirected:
                redirected.remove_header("Authorization")
            return redirected

    with urllib.request.build_opener(Redirect).open(request, timeout=180) as response, target.open("wb") as stream:
        shutil.copyfileobj(response, stream)


def unpack_verified(archive_path, checksums_path, destination, tag):
    expected = None
    for line in checksums_path.read_text().splitlines():
        parts = line.split()
        if len(parts) == 2 and parts[1] == archive_path.name:
            expected = parts[0]
    if not expected or not re.fullmatch(r"[a-f0-9]{64}", expected):
        raise ValueError("SHA256SUMS 缺少此安装包的有效校验值。")
    if hashlib.sha256(archive_path.read_bytes()).hexdigest() != expected:
        raise ValueError("SHA256 校验失败；未执行安装。")
    with tarfile.open(archive_path, "r:gz") as archive:
        members = archive.getmembers()
        for member in members:
            path = PurePosixPath(member.name)
            if (path.is_absolute() or ".." in path.parts or not path.parts or
                    path.parts[0] != "seeed-lerobot-linux" or not member.isfile() or
                    len(path.parts) < 2 or path.parts[1] in
                    {"config.json", "data", "logs", ".state", ".venv", ".tools"}):
                raise ValueError(f"安装包包含不允许的路径或文件类型：{member.name}")
        # Paths and file types have been validated; no tar links or device nodes are permitted.
        if hasattr(tarfile, "data_filter"):
            archive.extractall(destination, members=members, filter="data")
        else:
            archive.extractall(destination, members=members)
    project = destination / "seeed-lerobot-linux"
    if (project / "VERSION").read_text().strip() != version_number(tag):
        raise ValueError("安装包 VERSION 与 Release 标签不一致。")
    if not (project / "install.sh").is_file():
        raise ValueError("安装包缺少 install.sh。")
    return project


def install(args):
    if not re.fullmatch(r"[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+", args.repo):
        raise ValueError("--repo 格式应为 owner/repository。")
    if sys.platform != "linux" or os.geteuid() == 0:
        raise ValueError("请在 Linux 上使用普通用户运行；系统依赖由 sudo 安装。")
    if args.version != "latest":
        version_number(args.version)
    prefix = args.prefix.expanduser().resolve()
    prefix.mkdir(parents=True, exist_ok=True)
    with (prefix / ".install.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        marker = prefix / ".repository"
        if marker.exists() and marker.read_text().strip() != args.repo:
            raise ValueError("此目录属于其他仓库；请使用不同的 --prefix。")
        if not marker.exists() and any(p.name != ".install.lock" for p in prefix.iterdir()):
            raise ValueError("目标目录非空且不是此安装器管理的目录；请选择新的 --prefix。")
        marker.write_text(args.repo + "\n")
        with tempfile.TemporaryDirectory(prefix="seeed-release-") as temporary:
            temporary = Path(temporary)
            endpoint = "latest" if args.version == "latest" else "tags/" + args.version
            metadata_path = temporary / "release.json"
            download(f"https://api.github.com/repos/{args.repo}/releases/{endpoint}", metadata_path)
            metadata = json.loads(metadata_path.read_text())
            tag = metadata["tag_name"]
            version = version_number(tag)
            name = f"seeed-lerobot-{version}-linux-x86_64.tar.gz"
            assets = {asset["name"]: asset["url"] for asset in metadata["assets"]}
            for asset in (name, "SHA256SUMS"):
                if asset not in assets:
                    raise ValueError(f"Release 缺少 {asset}；请选择带部署附件的已发布版本。")
                download(assets[asset], temporary / asset, api=True)
            project = unpack_verified(temporary / name, temporary / "SHA256SUMS", temporary / "unpack", tag)
            target = prefix / "releases" / tag
            target.parent.mkdir(exist_ok=True)
            if not target.exists():
                shutil.move(str(project), target)
            elif not (target / ".release-owner").is_file():
                raise ValueError("版本目录已存在且来源不明；请选择新的安装目录。")
            (target / ".release-owner").write_text(args.repo + "\n")
        shared = prefix / "shared"
        shared.mkdir(exist_ok=True)
        for item in ("data", "logs", ".state", "config.json"):
            source = shared / item
            if item != "config.json":
                source.mkdir(exist_ok=True)
            link = target / item
            if not link.is_symlink():
                if link.exists():
                    raise ValueError(f"不覆盖现有文件：{link}")
                link.symlink_to(source, target_is_directory=item != "config.json")
        command = ["bash", str(target / "install.sh"), "--device", args.device]
        if args.skip_system:
            command.append("--skip-system")
        subprocess.run(command, check=True)
        pending = prefix / ".current-next"
        pending.unlink(missing_ok=True)
        pending.symlink_to(target, target_is_directory=True)
        os.replace(pending, prefix / "current")
        print(f"\nRelease {tag} 安装并通过自检。启动菜单：\n{shlex.quote(str(prefix / 'current/robotctl'))}")
        print(f"配置、数据与模型：{shared}\n旧版本保留在：{prefix / 'releases'}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", required=True, help="GitHub owner/repository")
    parser.add_argument("--version", default="latest", help="latest 或 v0.1.0")
    parser.add_argument("--prefix", type=Path, default=Path.home() / ".local/share/seeed-lerobot")
    parser.add_argument("--device", choices=("cpu", "cuda"), default="cpu")
    parser.add_argument("--skip-system", action="store_true")
    args = parser.parse_args()
    try:
        install(args)
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"安装未完成：{error}\n修复后可重复运行。已有版本和 shared/ 数据保留。\n")


if __name__ == "__main__":
    main()
