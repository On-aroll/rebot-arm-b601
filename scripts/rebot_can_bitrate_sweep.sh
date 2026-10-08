#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# CAN 波特率扫描 —— 逐个速率试，看哪个能扫到电机
#
# 为什么需要：
#   官方 RS 参考配置是 1 Mbps，但那是**另一台 DevArm** 上的实测值
#   （见 external/reBotArm_control_py/docs/gravity_calibration_rs_2026-07-17.md），
#   从未与本台实物核对。若本台电机被配成 500 k / 250 k 等，
#   在 1 Mbps 下永远不会应答 —— 表现就是"扫描零响应"。
#
# 本脚本需要 sudo（要改接口配置），因此整体以 root 运行：
#
#   sudo bash scripts/rebot_can_bitrate_sweep.sh
#
# 安全：只做 ping 与读寄存器，**不使能、不发运动指令、不改电机参数**。
#       但会向总线发送查询帧。
# ---------------------------------------------------------------------------
set -uo pipefail

CHANNEL="${CHANNEL:-can0}"
VENDOR="${VENDOR:-robstride}"
MODEL="${MODEL:-}"
START="${START:-1}"
END="${END:-16}"
RATES="${RATES:-1000000 800000 500000 250000 125000}"

if [ "$(id -u)" -ne 0 ]; then
  echo "[XX] 本脚本需要 sudo（要改接口配置）：" >&2
  echo "     sudo bash \"$(cd "$(dirname "$0")" && pwd)/$(basename "$0")\"" >&2
  exit 1
fi

# ⚠️ 不能用 $HOME 定位：sudo 会把 HOME 设成 /root，
# 于是 $HOME/.local/... 变成 /root/.local/... 而报"找不到"（2026-10-07 实测踩到）。
# 改为按候选列表逐个探测，并允许用 MOTORBRIDGE_CLI 显式覆盖。
CLI=""
for cand in \
  "${MOTORBRIDGE_CLI:-}" \
  "/home/bonvoyage/.local/share/robotics/envs/rebot-sim/bin/motorbridge-cli" \
  "/home/bonvoyage/.local/bin/motorbridge-cli" \
  "$(command -v motorbridge-cli 2>/dev/null || true)"
do
  [ -n "$cand" ] && [ -x "$cand" ] && { CLI="$cand"; break; }
done
[ -n "$CLI" ] || {
  echo "[XX] 找不到 motorbridge-cli。请指定：MOTORBRIDGE_CLI=/path/to/motorbridge-cli sudo -E bash $0" >&2
  exit 1
}
echo "使用 CLI: $CLI"

if [ -z "$MODEL" ]; then
  case "$VENDOR" in
    robstride) MODEL="rs-06" ;;
    damiao)    MODEL="4340P" ;;
    *)         MODEL="4340" ;;
  esac
fi

echo "=========================================================="
echo "波特率扫描  channel=$CHANNEL  协议=$VENDOR  型号=$MODEL  ID $START..$END"
echo "待试速率: $RATES"
echo "=========================================================="
echo

found_any=0
for rate in $RATES; do
  echo "----------------------------------------------------------"
  echo ">>> 波特率 $rate"
  echo "----------------------------------------------------------"
  ip link set "$CHANNEL" down 2>/dev/null
  # ⚠️ 必须同时显式关掉 loopback 与 listen-only。
  # ctrlmode 标志只有显式指定才会改变 —— 只写 bitrate 两个标志都会保留。
  # 2026-10-07 实测踩到：残留的 loopback 让整轮扫描的帧根本没上总线，
  # 五个波特率全零，白跑一轮。
  if ! ip link set "$CHANNEL" type can bitrate "$rate" loopback off listen-only off 2>&1; then
    echo "    [!!] 该速率不被适配器接受，跳过"
    continue
  fi
  ip link set "$CHANNEL" up || { echo "    [!!] 启动失败，跳过"; continue; }

  # 自检：确认真的处于普通模式，否则本轮结果无效
  cm=$(ip -details -json link show dev "$CHANNEL" 2>/dev/null \
    | /usr/bin/python3 -c "import json,sys;print(json.load(sys.stdin)[0].get('linkinfo',{}).get('info_data',{}).get('ctrlmode') or '')" 2>/dev/null)
  case "$cm" in
    *LOOPBACK*|*LISTEN-ONLY*)
      echo "    [XX] 本轮 ctrlmode 仍含 $cm —— 帧发不到总线，结果无效，跳过"
      continue ;;
  esac

  # 取扫描前的收发计数，用于判断总线上有没有任何帧（注意：本适配器
  # 不上报总线错误，但 rx 计数仍可能有效，作为辅助信息记录）
  before_rx=$(ip -details -statistics -json link show dev "$CHANNEL" 2>/dev/null \
    | /usr/bin/python3 -c "import json,sys;print(json.load(sys.stdin)[0].get('stats64',{}).get('rx',{}).get('packets',0))" 2>/dev/null || echo 0)

  out=$("$CLI" scan --vendor "$VENDOR" --channel "$CHANNEL" --model "$MODEL" \
        --start-id "$START" --end-id "$END" \
        --feedback-base 0x10 --timeout-ms 80 2>&1)
  echo "$out" | grep -E "^\[hit\]|scan done" || echo "$out" | tail -2

  hits=$(echo "$out" | grep -c "^\[hit\]" || true)
  after_rx=$(ip -details -statistics -json link show dev "$CHANNEL" 2>/dev/null \
    | /usr/bin/python3 -c "import json,sys;print(json.load(sys.stdin)[0].get('stats64',{}).get('rx',{}).get('packets',0))" 2>/dev/null || echo 0)

  echo "    → 命中 $hits 个；本轮接收帧数 $before_rx → $after_rx"
  if [ "$hits" -gt 0 ]; then
    found_any=1
    echo "    ★ 波特率 $rate 有响应！"
  fi
  echo
done

echo "=========================================================="
if [ "$found_any" -eq 1 ]; then
  echo "结论：至少一个波特率扫到了电机 —— 以命中的那个速率为准。"
else
  echo "结论：所有速率均零响应。"
  echo
  echo "下一步建议（按信息量）："
  echo "  1) 确认机械臂确实通电，且电机处于可通信状态（看指示灯）"
  echo "  2) 核对 CAN_H / CAN_L 是否接反 —— 接反时终端电阻仍是 60Ω，"
  echo "     万用表量不出问题，但通信完全不通"
  echo "  3) 验证适配器是否真的在驱动总线：在持续发送期间量 CAN_H/CAN_L"
  echo "     对地电压，正常应能看到 CAN_H 升高、CAN_L 降低"
  echo "  4) 向 Seeed 确认本台设备的出厂波特率与电机 ID 分配"
fi
echo "=========================================================="

# 恢复成只监听（观察用）。同样要显式关掉 loopback。
ip link set "$CHANNEL" down 2>/dev/null
ip link set "$CHANNEL" type can bitrate 1000000 loopback off listen-only on 2>/dev/null
ip link set "$CHANNEL" up 2>/dev/null
echo
echo "已恢复：$CHANNEL 1 Mbps / LISTEN-ONLY（loopback 已关）"
