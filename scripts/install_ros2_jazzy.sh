#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# ROS 2 Jazzy 安装脚本 — Ubuntu 24.04 (noble) / amd64
# 生成日期：2026-10-06   生成依据：本机实测（见脚本末尾"核查记录"）
#
# 用法：
#   sudo bash scripts/install_ros2_jazzy.sh            # 默认 desktop（含 RViz）
#   sudo bash scripts/install_ros2_jazzy.sh ros-base   # 只要最小运行时
#
# 默认取 desktop 的理由（2026-10-06 用户指定）：reBot 的官方 ROS 2 集成包含
# rebotarm_moveit_config 与 rebotarm_bringup/rviz，需要 RViz 才能跑起来。
#
# 说明：本脚本需要 root。它会改动系统状态（新增 apt 源、安装系统包）。
# ---------------------------------------------------------------------------
set -euo pipefail

VARIANT="${1:-desktop}"
KEYRING="/usr/share/keyrings/ros-archive-keyring.gpg"
EXPECTED_FPR="C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654"
SRC_LIST="/etc/apt/sources.list.d/ros2.list"

log() { printf '\n[==] %s\n' "$*"; }
die() { printf '\n[XX] %s\n' "$*" >&2; exit 1; }

# ---------------------------------------------------------------------------
# 0. 前置检查
# ---------------------------------------------------------------------------
log "前置检查"

[ "$(id -u)" -eq 0 ] || die "请以 root 运行：sudo bash $0"

# shellcheck disable=SC1091
. /etc/os-release
[ "${VERSION_CODENAME:-}" = "noble" ] \
  || die "仅支持 Ubuntu 24.04 (noble)，当前：${VERSION_CODENAME:-未知}"

ARCH="$(dpkg --print-architecture)"
[ "$ARCH" = "amd64" ] || echo "[!!] 非 amd64（当前 $ARCH），ROS 2 二进制包支持可能不全"

case "$VARIANT" in
  ros-base) NEED_GB=4 ;;
  desktop)  NEED_GB=8 ;;
  *) die "未知变体 '$VARIANT'，可选：ros-base | desktop" ;;
esac

AVAIL_GB="$(df -Pk / | awk 'NR==2{print int($4/1048576)}')"
[ "$AVAIL_GB" -ge "$NEED_GB" ] \
  || die "磁盘不足：需要约 ${NEED_GB} GiB，当前可用 ${AVAIL_GB} GiB"
echo "  系统: ${PRETTY_NAME} / ${ARCH} / 可用磁盘 ${AVAIL_GB} GiB / 安装变体 ros-jazzy-${VARIANT}"

MEM_AVAIL_MB="$(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo)"
[ "$MEM_AVAIL_MB" -lt 2048 ] \
  && echo "  [!!] 当前可用内存仅 ${MEM_AVAIL_MB} MiB，安装可继续，但编译/运行 RViz 前建议先释放内存"
echo "  可用内存: ${MEM_AVAIL_MB} MiB"

# ---------------------------------------------------------------------------
# 0b. 修复上一轮遗留的 apt 源
#     若 /etc/apt/sources.list.d/ros2.list 已存在但密钥是 ASCII 装甲格式，
#     之后每一次 apt-get update 都会因 NO_PUBKEY 失败；而本脚本带 set -e，
#     会在"修密钥"那一步之前就被这个失败打断（2026-10-06 实测踩到）。
#     所以先把遗留密钥修好，再做任何 apt 操作。
# ---------------------------------------------------------------------------
if [ -f "$SRC_LIST" ] && [ -f "$KEYRING" ] \
   && head -c 40 "$KEYRING" | grep -q 'BEGIN PGP PUBLIC KEY BLOCK'; then
  log "发现上一轮遗留的 ASCII 装甲密钥，先转为二进制"
  gpg --batch --yes --dearmor -o "$KEYRING.bin" "$KEYRING" \
    && mv -f "$KEYRING.bin" "$KEYRING" \
    && chmod 0644 "$KEYRING" \
    && echo "  已修复" \
    || die "修复失败，请手动执行：
      sudo gpg --batch --yes --dearmor -o ${KEYRING}.bin ${KEYRING}
      sudo mv -f ${KEYRING}.bin ${KEYRING}
      sudo chmod 644 ${KEYRING}"
fi

# ---------------------------------------------------------------------------
# 1. locale 必须是 UTF-8（ROS 2 硬性要求，否则 ros2 CLI 与日志会异常）
# ---------------------------------------------------------------------------
log "检查 locale"
if ! locale 2>/dev/null | grep -qE 'LANG=.*(UTF-8|utf8)'; then
  echo "  当前非 UTF-8，正在修正（en_US.UTF-8）"
  apt-get update -qq || echo "  [!!] apt update 有告警（可能是第三方源），继续"
  apt-get install -y -qq locales
  locale-gen en_US.UTF-8
  update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
  export LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
  echo "  已设置为 en_US.UTF-8（重新登录后全局生效）"
else
  echo "  已是 UTF-8：$(locale 2>/dev/null | grep -m1 '^LANG=')"
fi

# ---------------------------------------------------------------------------
# 2. 依赖
# ---------------------------------------------------------------------------
log "安装依赖 curl / ca-certificates / gnupg"
# 不能让第三方源的问题中断安装：这里失败过一次（上一轮遗留的坏 ROS 源），
# 脚本会在修复密钥之前就被 set -e 打断。
apt-get update -qq || echo "  [!!] apt update 有告警（可能是第三方源），继续"
apt-get install -y -qq curl ca-certificates gnupg

# ---------------------------------------------------------------------------
# 3. 取 ROS 公钥
#    注意：packages.ros.org/ros.key 提供的是【已过期副本】（2025-06-01 到期），
#    直接照抄官方教程会导致 apt 报 EXPKEYSIG。
#    本脚本改从 keyserver 取同指纹的更新副本（现有效期至 2030-06-01），
#    并以指纹作为最终信任锚点 —— 无论走哪条通道，指纹不符即中止。
# ---------------------------------------------------------------------------
log "获取 ROS 公钥"
mkdir -p /usr/share/keyrings

fetch_key() {
  if curl -fsSL --max-time 45 \
       'https://keyserver.ubuntu.com/pks/lookup?op=get&search=0xF42ED6FBAB17C654' \
       -o "$KEYRING" 2>/dev/null; then
    return 0
  fi
  # 回退：本机走 MITM 代理，部分域名证书校验不过。
  # 此处仅用于取回公钥，随后以指纹校验兜底。
  echo "  [!!] TLS 校验失败，改用不校验通道取回密钥（随后强制校验指纹）"
  curl -fsSLk --max-time 45 \
    'https://keyserver.ubuntu.com/pks/lookup?op=get&search=0xF42ED6FBAB17C654' \
    -o "$KEYRING"
}

fetch_key || die "无法取回 ROS 公钥"

# keyserver.ubuntu.com returns ASCII-ARMORED text. apt reads a file named .gpg
# as a BINARY keyring, so leaving it armored makes apt fail with NO_PUBKEY even
# though `gpg --show-keys` happily verifies the fingerprint (gpg accepts both
# formats -- that is exactly how this slipped through on 2026-10-06).
if head -c 40 "$KEYRING" | grep -q 'BEGIN PGP PUBLIC KEY BLOCK'; then
  echo "  密钥为 ASCII 装甲格式，转为二进制 keyring（否则 apt 报 NO_PUBKEY）"
  gpg --batch --yes --dearmor -o "$KEYRING.bin" "$KEYRING" \
    || die "dearmor 失败"
  mv -f "$KEYRING.bin" "$KEYRING"
fi

FPR="$(gpg --show-keys --with-colons "$KEYRING" 2>/dev/null \
        | awk -F: '/^fpr:/{print $10; exit}')"
[ "$FPR" = "$EXPECTED_FPR" ] \
  || die "密钥指纹不符！期望 $EXPECTED_FPR，实得 ${FPR:-（空）}。已中止，未改动 apt 源。"
echo "  指纹校验通过: $FPR"

EXP="$(gpg --show-keys --with-colons "$KEYRING" 2>/dev/null \
        | awk -F: '/^pub:/{print $7; exit}')"
NOW="$(date -u +%s)"
if [ -n "$EXP" ] && [ "$EXP" -lt "$NOW" ]; then
  die "密钥已过期（$(date -u -d "@$EXP" +%Y-%m-%d)），请更换获取渠道"
fi
echo "  有效期至: ${EXP:-（永不过期）}"

chmod 0644 "$KEYRING"

# ---------------------------------------------------------------------------
# 4. apt 源
#    用 http:// 而非 https:// 的理由：ROS 2 官方文档本身就是 http://，
#    且本机 MITM 代理会让 packages.ros.org 的 https 证书校验失败。
#    包的完整性由 Release 文件的 GPG 签名保证，不依赖传输层 TLS。
# ---------------------------------------------------------------------------
log "写入 apt 源"
cat > "$SRC_LIST" <<EOF
deb [arch=${ARCH} signed-by=${KEYRING}] http://packages.ros.org/ros2/ubuntu ${VERSION_CODENAME} main
EOF
chmod 0644 "$SRC_LIST"
echo "  $SRC_LIST:"
sed 's/^/    /' "$SRC_LIST"

# 补齐 apt 目录（本机曾缺失 /var/lib/apt/lists，会导致 update 失败）
mkdir -p /var/lib/apt/lists/partial /var/cache/apt/archives/partial

# ---------------------------------------------------------------------------
# 5. 安装
# ---------------------------------------------------------------------------
log "apt update（新增 ROS 源）"
apt-get update

log "安装 ros-jazzy-${VARIANT} + ros-dev-tools"
DEBIAN_FRONTEND=noninteractive apt-get install -y \
  "ros-jazzy-${VARIANT}" \
  ros-dev-tools \
  python3-rosdep \
  python3-colcon-common-extensions

# ---------------------------------------------------------------------------
# 6. 验证
# ---------------------------------------------------------------------------
log "验证安装"
# NOTE: `ros2 --version` does NOT exist -- the CLI rejects it with
# "unrecognized arguments: --version" (hit on 2026-10-06, made the script exit
# non-zero after a perfectly good install). Use --help plus real checks.
# shellcheck disable=SC1091
set +u
. /opt/ros/jazzy/setup.bash
set -u

echo "  安装前缀: /opt/ros/jazzy"
echo "  ROS_DISTRO: ${ROS_DISTRO:-（未设置）}"

[ -x /opt/ros/jazzy/bin/ros2 ] || die "未找到 /opt/ros/jazzy/bin/ros2，安装可能不完整"
ros2 --help > /dev/null 2>&1 || die "ros2 CLI 无法执行"
echo "  ros2 CLI: 可用"

PKG_COUNT="$(ros2 pkg list 2>/dev/null | wc -l)"
echo "  可见包数量: ${PKG_COUNT}"
if [ "${PKG_COUNT}" -lt 50 ]; then
  echo "  [!!] 包数量偏少，请手动确认：source /opt/ros/jazzy/setup.bash && ros2 pkg list | head"
fi

# 演示节点自检（能跑通说明 runtime 正常）
if timeout 60 ros2 topic list > /dev/null 2>&1; then
  echo "  ros2 topic list: 通过"
else
  echo "  [!!] ros2 topic list 未通过，检查 ~/.ros 权限与 DDS 配置"
fi

# ---------------------------------------------------------------------------
# 7. 提示
# ---------------------------------------------------------------------------
cat <<'NOTE'

[==] 完成。后续请注意：

  1. 每次使用 ROS 2 前先 source（或写进 ~/.bashrc）：
       source /opt/ros/jazzy/setup.bash

  2. 【重要】不要与 rebot-sim 环境同时 source。
     ROS 2 Jazzy 用系统 Python 3.12；rebot-sim 是独立的 Python 3.10.22 venv。
     两者混用会互相污染。按规划（§4 第 6 条）二者通过进程/接口通信，
     不要塞进同一个 shell。

  3. reBot 的 ROS 2 运行时模型不在当前工作区。
     官方 RS URDF 的 README 写明：
       "ROS 2 运行时模型仍位于 rebotarm_ros2_RS/src/rebotarm_bringup/description/"
     该仓库尚未纳入 configs/asset-manifest.json，需要另行固定 commit 后拉取。

  4. rosdep 初始化（构建 reBot ROS 2 包时需要）：
       sudo rosdep init && rosdep update

NOTE

# ---------------------------------------------------------------------------
# 核查记录（2026-10-06，本机实测）
# ---------------------------------------------------------------------------
# - 系统：Ubuntu 24.04.4 LTS (noble)，内核 7.0.0-34-generic，x86_64
# - ROS 2 源可达：packages.ros.org/ros2/ubuntu/dists/noble/Release
#   （Origin: ROS / Suite: noble / Date: 2026-09-23 / Components: main）
# - 源签名密钥：keyid F42ED6FBAB17C654，指纹 C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
# - 坑：packages.ros.org/ros.key 是过期副本（2025-06-01 到期）→ 改用 keyserver 副本
#   （同指纹，有效期至 2030-06-01）
# - 网络：HTTPS_PROXY=127.0.0.1:35553 为 MITM 代理，packages.ros.org 的 https
#   证书校验不过 → 源改用官方同样采用的 http://
# - 本机无 CUDA（RTX 3050 走 nouveau，未装官方驱动）→ 不影响 ROS 2 本体，
#   但 GPU 加速的感知/学习节点不可用
