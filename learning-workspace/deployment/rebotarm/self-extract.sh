#!/usr/bin/env bash
# Binary payload follows the marker at the end of this header.
set -Eeuo pipefail
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  cat <<'USAGE'
ReBot Arm B601-RS 1.0.0 · Ubuntu / Kubuntu 24.04 x86_64 · ROS 2 Jazzy
用法：bash rebotarm-b601-rs-1.0.0-ubuntu24.04-x86_64.run [选项]
  无选项                 安装系统依赖、构建、自检并打开工作台
  --dry-run              校验并查看安装计划
  --no-launch            安装完成后不启动网页
  --skip-system          使用管理员已经预装的系统包
  --prefix /绝对路径     自定义安装目录（不含空格）
  --extract /空目录路径  仅解包，不安装
需联网下载 ROS / apt 和 Python 依赖；请以普通用户运行。
USAGE
  exit 0
fi
command -v python3 >/dev/null || { echo '需要系统 Python 3.12。' >&2; exit 1; }
payload_line="$(awk '/^__REBOTARM_PAYLOAD_BELOW__$/{print NR+1;exit}' "$0")"
tmp_dir="$(mktemp -d -t rebotarm-install.XXXXXXXX)"
trap 'rm -rf -- "$tmp_dir"' EXIT
tail -n +"${payload_line}" "$0" >"${tmp_dir}/payload.tar.gz"
printf '%s  %s\n' '@PAYLOAD_SHA@' "${tmp_dir}/payload.tar.gz" | sha256sum --check --status || {
  echo '安装包损坏：SHA256 校验失败，请重新拷贝。' >&2; exit 1;
}
extract_to="${tmp_dir}/source"
if [[ "${1:-}" == --extract ]]; then
  [[ $# == 2 && ! -e "$2" ]] || { echo '--extract 需要一个尚不存在的目标目录。' >&2; exit 2; }
  extract_to="$2"
fi
python3 - "${tmp_dir}/payload.tar.gz" "${extract_to}" <<'PY'
import pathlib, sys, tarfile
destination = pathlib.Path(sys.argv[2])
destination.mkdir(parents=True, exist_ok=False)
with tarfile.open(sys.argv[1]) as archive:
    archive.extractall(destination, filter='data')
PY
if [[ "${1:-}" == --extract ]]; then
  printf '已校验并解包到：%s\n' "${extract_to}"
  exit 0
fi
/usr/bin/python3 "${extract_to}/deployment/install.py" "$@"
exit 0
__REBOTARM_PAYLOAD_BELOW__
