#!/usr/bin/env bash
# 重启（或首次启动）reBot 仿真服务，并把进程号写进 outputs/dashboard.pid。
#
# 为什么要脚本：改完 scripts/*.py 必须重启服务才生效，而手工重启有三件容易出错的事
# —— 凭旧 PID 杀错进程、忘了 MUJOCO_GL/PYOPENGL_PLATFORM 导致渲染起不来、
# 以及起完不知道成没成。这里三件都兜住：杀之前核对命令行，env 写死，起完轮询端口。
#
# 用法:
#   scripts/restart_sim.sh              # 重启 8765 主服务
#   scripts/restart_sim.sh --port 8766  # 另起一个验证实例（不动主服务）
#   scripts/restart_sim.sh --host 0.0.0.0   # 绑所有网卡，供同一局域网内别人打开
#   scripts/restart_sim.sh --status     # 只看状态，不做任何改动
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PORT=8765
HOST=127.0.0.1
STATUS_ONLY=0

while [ $# -gt 0 ]; do
  case "$1" in
    --port) PORT="$2"; shift 2 ;;
    --host) HOST="$2"; shift 2 ;;
    --status) STATUS_ONLY=1; shift ;;
    *) echo "未知参数：$1" >&2; exit 2 ;;
  esac
done

PIDFILE="$ROOT/outputs/dashboard.pid"
LOGFILE="$ROOT/outputs/dashboard.log"
PYTHON="/home/bonvoyage/.local/share/robotics/envs/rebot-sim/bin/python"
[ -x "$PYTHON" ] || PYTHON="$HOME/.local/share/robotics/envs/rebot-sim/bin/python"

listening_pid() {
  # 端口没在监听时返回空；调用方用 [ -z ] 判断。
  ss -ltnp 2>/dev/null | grep ":$PORT " | grep -oE "pid=[0-9]+" | head -1 | cut -d= -f2
}

CURRENT="$(listening_pid || true)"

if [ "$STATUS_ONLY" = 1 ]; then
  if [ -n "$CURRENT" ]; then
    echo "端口 $PORT 正在监听，PID $CURRENT"
    ps -p "$CURRENT" -o pid=,etime=,cmd=
  else
    echo "端口 $PORT 没有服务在监听"
  fi
  exit 0
fi

if [ -n "$CURRENT" ]; then
  CMD="$(ps -p "$CURRENT" -o cmd= 2>/dev/null || true)"
  # 只结束确实是本项目仿真服务的进程；别的程序占着这个端口就交给人处理。
  case "$CMD" in
    *rebot_sim.py*serve*) echo "停止现有服务 PID $CURRENT（$CMD）"; kill "$CURRENT" ;;
    *) echo "端口 $PORT 被别的进程占用，不动它：$CMD" >&2; exit 1 ;;
  esac
  for _ in $(seq 1 40); do
    ps -p "$CURRENT" >/dev/null 2>&1 || break
    sleep 0.5
  done
  if ps -p "$CURRENT" >/dev/null 2>&1; then
    echo "PID $CURRENT 5 秒内没退出，检查是否有动作程序正在执行" >&2
    exit 1
  fi
  echo "已停止"
else
  echo "端口 $PORT 当前无服务"
fi

mkdir -p "$ROOT/outputs"
cd "$ROOT"
MUJOCO_GL=egl PYOPENGL_PLATFORM=egl setsid nohup "$PYTHON" scripts/rebot_sim.py serve --port "$PORT" --host "$HOST" \
  >> "$LOGFILE" 2>&1 < /dev/null &

for _ in $(seq 1 120); do
  NEW="$(listening_pid || true)"
  [ -n "$NEW" ] && break
  sleep 0.5
done

if [ -z "${NEW:-}" ]; then
  echo "60 秒内端口 $PORT 没有起来，日志尾部：" >&2
  tail -20 "$LOGFILE" >&2
  exit 1
fi

echo "$NEW" > "$PIDFILE"
echo "仿真服务已启动：http://127.0.0.1:$PORT  （PID $NEW，已写入 $PIDFILE）"
echo "控制台：http://127.0.0.1:$PORT/studio/console"
if [ "$HOST" != "127.0.0.1" ]; then
  echo "局域网共享已开启（绑定 $HOST），同一网络的其他人用下面地址打开："
  ip -4 addr show scope global 2>/dev/null | grep -oE 'inet [0-9.]+' | awk '{print "  http://"$2":'"$PORT"'/studio/console"}'
  echo "⚠️ 无鉴权：同网段任何人都能下发仿真指令；/machine-state 会暴露实机读数。"
fi
