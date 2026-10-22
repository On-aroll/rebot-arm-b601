#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
DEVICE=cpu
SKIP_SYSTEM=0
DRY_RUN=0
while (($#)); do
    case "$1" in
        --device)
            [[ $# -ge 2 ]] || { echo '缺少 --device 参数'; exit 2; }
            DEVICE="$2"; shift 2 ;;
        --skip-system) SKIP_SYSTEM=1; shift ;;
        --dry-run) DRY_RUN=1; shift ;;
        -h|--help)
            echo '用法：bash install.sh [--device cpu|cuda] [--skip-system] [--dry-run]'
            echo '默认 CPU。cuda 使用 CUDA 12.8 的 PyTorch，需要 NVIDIA 显卡及兼容驱动。'
            echo '--skip-system：由管理员预装系统依赖后使用；不安装系统包或修改用户组。'
            exit 0 ;;
        *) echo "未知参数：$1"; exit 2 ;;
    esac
done
[[ "$DEVICE" == cpu || "$DEVICE" == cuda ]] || { echo '--device 仅支持 cpu / cuda'; exit 2; }
source "$ROOT/scripts/system_deps.sh"
check_binary_platform
# shellcheck disable=SC1091
source /etc/os-release
platform_plan "$ID" "${ID_LIKE:-}" "${VERSION_ID:-0}"
if [[ "$SYSTEM_MANAGER" == manual ]] && ((!SKIP_SYSTEM)); then
    echo '此系统需要由管理员预装依赖，再运行 --skip-system；参见 docs/SUPPORT.md。'; exit 1
fi
if ((DRY_RUN)); then
    echo "安装计划：${PRETTY_NAME:-$ID} / x86_64 / $DEVICE / $SYSTEM_MANAGER"
    echo "目录：$ROOT/.venv；Python 3.12.12；依赖：requirements-$DEVICE.lock"
    echo "系统包安装：$((1-SKIP_SYSTEM))；现有 config.json 与 data/ 将保留。"
    echo "系统包：${SYSTEM_PACKAGES[*]}；串口用户组：$SERIAL_GROUP"
    echo '最后执行软件诊断和离线 ACT 训练/推理自检。'
    exit 0
fi
[[ "$EUID" -ne 0 ]] || { echo '请使用普通用户运行此脚本，安装系统包时会调用 sudo。'; exit 1; }
mkdir -p logs .state
LOG="$ROOT/logs/install-$(date +%Y%m%d-%H%M%S).log"
exec > >(tee -a "$LOG") 2>&1
trap 'status=$?; echo "安装失败（退出码 $status），日志：$LOG。解决后可重复运行安装。"; exit "$status"' ERR
export UV_SYSTEM_CERTS=true
export UV_HTTP_TIMEOUT=180
export UV_PYTHON_INSTALL_DIR="$ROOT/.tools/python"
export UV_LINK_MODE=copy
if ((!SKIP_SYSTEM)); then
    install_system_dependencies
fi
command -v python3 >/dev/null || { echo '需要系统 python3。'; exit 1; }
python3 scripts/bootstrap_uv.py
UV="$ROOT/.tools/uv"
if [[ ! -x .venv/bin/python ]]; then
    "$UV" venv --python 3.12.12 .venv
fi
TORCH_INDEX=https://download.pytorch.org/whl/cpu
[[ "$DEVICE" != cuda ]] || TORCH_INDEX=https://download.pytorch.org/whl/cu128
"$UV" pip sync --python .venv/bin/python --require-hashes "requirements-$DEVICE.lock" \
    --extra-index-url "$TORCH_INDEX" --index-strategy unsafe-best-match
"$UV" pip check --python .venv/bin/python
if [[ ! -f config.json ]]; then
    .venv/bin/python - "$DEVICE" <<'PY'
import json, sys
from pathlib import Path
config = json.loads(Path('config.example.json').read_text())
config['training']['device'] = sys.argv[1]
Path('config.json').write_text(json.dumps(config, ensure_ascii=False, indent=2) + '\n')
PY
fi
# Preserve user configuration on reinstall. The selected backend is recorded separately.
printf '%s\n' "$DEVICE" > .state/device
./robotctl doctor --software-only
./robotctl self-test --device "$DEVICE"
printf '\n安装完成。\n启动菜单：%q/robotctl\n首次连接：./robotctl configure\n日志：%s\n' "$ROOT" "$LOG"
