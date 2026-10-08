#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# RobStride 电机 ID 扫描 —— 包装官方 motorbridge-cli scan
#
# 为什么用官方 CLI 而不是 scripts/rebot_can_ping.py：
#   官方 scan 会依次尝试多个 host_id（0xFD / 0xFF / 0xFE / 0x00 / 0xAA），
#   而自建脚本只试了 0xFD 一个。若电机期望的 host_id 不是 0xFD，
#   ping 永远不会被响应 —— 这是 2026-10-07 排查中发现的覆盖缺口。
#
# 安全：官方 scan 只做 ping 与读参数，**不使能、不发运动指令、不改电机参数**。
#       但它会向总线发送帧，属于显式操作。
#
# 用法:
#   bash scripts/rebot_can_scan.sh                    # 默认 RobStride rs-06，ID 1..16
#   bash scripts/rebot_can_scan.sh rs-00              # 按 rs-00 扫描
#   bash scripts/rebot_can_scan.sh rs-06 1 32         # 指定型号与 ID 范围
#   VENDOR=damiao bash scripts/rebot_can_scan.sh 4340P   # 按达妙协议扫描
#   VENDOR=all bash scripts/rebot_can_scan.sh         # 依次试四种协议
#
# ⚠️ 型号归属（2026-10-07 发现）：
#   RS  = RobStride 电机，传输 CAN，URDF 末端帧 gripper_end
#   DM  = 达妙电机，官方配置走串口桥 /dev/ttyACM0，URDF 末端帧 end_link
#   实物到底哪一版必须以电机标识为准。中转板印有"达妙科技"字样，
#   故优先用 VENDOR=damiao 试。
# ---------------------------------------------------------------------------
set -uo pipefail

export HOME="${HOME:-/home/bonvoyage}"

CLI="$HOME/.local/share/robotics/envs/rebot-sim/bin/motorbridge-cli"
CHANNEL="${CHANNEL:-can0}"
VENDOR="${VENDOR:-robstride}"
MODEL="${1:-}"
START="${2:-1}"
END="${3:-16}"

# 未指定型号时按 vendor 取一个合理默认
if [ -z "$MODEL" ]; then
  case "$VENDOR" in
    robstride) MODEL="rs-06" ;;
    damiao)    MODEL="4340P" ;;
    *)         MODEL="4340" ;;
  esac
fi

[ -x "$CLI" ] || { echo "[XX] 找不到官方 CLI: $CLI" >&2; exit 1; }

# 前置检查：接口必须 UP，且不能是 LISTEN-ONLY（否则发不出去）
if ! ip link show "$CHANNEL" >/dev/null 2>&1; then
  echo "[XX] 接口 $CHANNEL 不存在。适配器插好了吗？" >&2
  exit 1
fi
if ! ip -brief link show "$CHANNEL" 2>/dev/null | grep -q "UP"; then
  cat >&2 <<EOF
[XX] 接口 $CHANNEL 处于 DOWN，无法发送。请先启动（需要终端密码）：
      sudo ip link set $CHANNEL down
      sudo ip link set $CHANNEL type can bitrate 1000000 listen-only off
      sudo ip link set $CHANNEL up
EOF
  exit 1
fi
# ⚠️ 必须同时检查 loopback 与 listen-only。
# 两个标志都只有显式指定才会改变 —— 只写 bitrate 会两个都保留。
# 残留的 loopback 会让帧根本不驱动外部总线，扫描结果无效（2026-10-07 实测踩到）。
for flag in LISTEN-ONLY LOOPBACK; do
  if ip -details link show "$CHANNEL" 2>/dev/null | head -3 | grep -q "$flag"; then
    cat >&2 <<EOF
[XX] 接口 $CHANNEL 仍处于 $flag，发不出去（或帧不会上总线）。
     注意：只写 bitrate 不会清除 ctrlmode 标志，必须显式关掉：
      sudo ip link set $CHANNEL down
      sudo ip link set $CHANNEL type can bitrate 1000000 loopback off listen-only off
      sudo ip link set $CHANNEL up
EOF
    exit 1
  fi
done

echo "扫描 $CHANNEL  协议=$VENDOR  型号=$MODEL  ID 范围=$START..$END"
[ "$VENDOR" = "robstride" ] && echo "host_id 候选：0xFD 0xFF 0xFE 0x00 0xAA"
[ "$VENDOR" = "damiao" ] && echo "达妙反馈 ID 约定：feedback_id = 0x10 + motor_id"
echo "（只做查询与读寄存器，不使能、不运动、不改参数）"
echo

if [ "$VENDOR" = "robstride" ]; then
  exec "$CLI" scan \
    --vendor robstride --channel "$CHANNEL" --model "$MODEL" \
    --start-id "$START" --end-id "$END" \
    --feedback-ids 0xFD,0xFF,0xFE,0x00,0xAA --timeout-ms 80
else
  exec "$CLI" scan \
    --vendor "$VENDOR" --channel "$CHANNEL" --model "$MODEL" \
    --start-id "$START" --end-id "$END" \
    --feedback-base 0x10 --timeout-ms 80
fi
