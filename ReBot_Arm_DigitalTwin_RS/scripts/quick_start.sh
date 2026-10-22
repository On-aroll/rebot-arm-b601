#!/usr/bin/env bash
set -Eeuo pipefail

# One-click entry point for the safe RS simulation and local web console.
# It never starts the physical /rebotarm controller.
ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." >/dev/null 2>&1 && pwd)"
ENV_SCRIPT="${ROOT_DIR}/scripts/rs_env.sh"
RUN_DIR="${XDG_RUNTIME_DIR:-/tmp}/rebotarm-rs-${UID}"
PID_FILE="${RUN_DIR}/managed.pids"
LOCK_FILE="${RUN_DIR}/quick-start.lock"
LOG_DIR="${ROOT_DIR}/outputs/quick-start"
WEB_DIR="${ROOT_DIR}/reBotArm_simulator-RS"
MODE="all"
OPEN_BROWSER=1

usage() {
  cat <<'EOF'
Usage: ./scripts/quick_start.sh [--simulation-only | --web-only] [--no-browser]

Default: starts safe RS simulation, rosbridge, and the local web UI.
--web-only: starts only the web UI and opens the project startup page.
The physical controller is never started by this command.
EOF
}

while (($#)); do
  case "$1" in
    --simulation-only) MODE="simulation" ;;
    --web-only) MODE="web" ;;
    --no-browser) OPEN_BROWSER=0 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

mkdir -p "${RUN_DIR}" "${LOG_DIR}"

# Serialize desktop, CLI, and web-button launches.  This also makes a second
# click wait for the first launch instead of racing it into duplicate nodes.
exec 9>"${LOCK_FILE}"
flock -x 9

pid_is_owned() {
  local pid="$1" name="$2" cmd cwd
  [[ "${pid}" =~ ^[0-9]+$ ]] || return 1
  kill -0 "${pid}" 2>/dev/null || return 1
  cmd="$(ps -p "${pid}" -o args= 2>/dev/null || true)"
  case "${name}" in
    simulation)
      [[ "${cmd}" == *"${ROOT_DIR}/scripts/start_rs_sim.sh"* || "${cmd}" == *"fake_rs_bringup.launch.py"* ]]
      ;;
    web)
      cwd="$(readlink -f "/proc/${pid}/cwd" 2>/dev/null || true)"
      [[ "${cwd}" == "${WEB_DIR}" && "${cmd}" == *"server.js"* ]]
      ;;
    *) return 1 ;;
  esac
}

prune_pid_file() {
  local tmp="${PID_FILE}.tmp.$$" pid name
  : >"${tmp}"
  if [[ -f "${PID_FILE}" ]]; then
    while read -r pid name; do
      [[ -n "${pid}" && -n "${name}" ]] || continue
      if pid_is_owned "${pid}" "${name}"; then
        if ! grep -Fqx "${pid} ${name}" "${tmp}"; then
          printf '%s %s\n' "${pid}" "${name}" >>"${tmp}"
        fi
      fi
    done <"${PID_FILE}"
  fi
  mv -f "${tmp}" "${PID_FILE}"
}

prune_pid_file

append_pid() {
  local pid="$1" name="$2"
  if ! grep -Fqx "${pid} ${name}" "${PID_FILE}" 2>/dev/null; then
    printf '%s %s\n' "${pid}" "${name}" >>"${PID_FILE}"
  fi
}

port_open() {
  local port="$1"
  if command -v ss >/dev/null 2>&1; then
    ss -ltn "sport = :${port}" 2>/dev/null | grep -q LISTEN
  else
    (exec 3<>"/dev/tcp/127.0.0.1/${port}") >/dev/null 2>&1
  fi
}

wait_for_port() {
  local port="$1" label="$2" attempts=40
  until port_open "${port}"; do
    ((attempts--)) || { echo "Timed out waiting for ${label} on port ${port}." >&2; return 1; }
    sleep 0.25
  done
}

start_simulation() {
  source "${ENV_SCRIPT}"
  # ROS graph discovery can briefly miss a live node. The launcher lock is
  # authoritative and also prevents truncating a running simulator's log.
  if ! flock -n /tmp/rebotarm_rs_sim.lock -c true; then
    echo "RS simulation is already running (launcher lock held)."
    return
  fi
  if ros2 node list 2>/dev/null | grep -qx '/fake_rebotarm_rs_driver'; then
    echo "RS simulation is already running."
    return
  fi

  local log_file="${LOG_DIR}/simulation.log"
  # The web console owns the 3D view; a separate native window is optional.
  export REBOTARM_MUJOCO_VIEWER="${REBOTARM_MUJOCO_VIEWER:-false}"
  if [[ "${REBOTARM_MUJOCO_VIEWER}" == false ]]; then
    export MUJOCO_GL="${MUJOCO_GL:-egl}"
  fi
  setsid "${ROOT_DIR}/scripts/start_rs_sim.sh" >"${log_file}" 2>&1 < /dev/null 9>&- &
  local pid=$!
  append_pid "${pid}" simulation
  echo "Starting RS simulation (pid=${pid}); log: ${log_file}"
  for _ in {1..40}; do
    if ros2 node list 2>/dev/null | grep -qx '/fake_rebotarm_rs_driver'; then
      echo "RS simulation is ready."
      return
    fi
    sleep 0.25
  done
  echo "Simulation is still starting; inspect ${log_file} if needed." >&2
}

start_web() {
  if port_open 3002; then
    echo "Web simulator is already running at http://127.0.0.1:3002"
    return
  fi

  local node_bin=""
  if command -v node >/dev/null 2>&1; then
    node_bin="$(command -v node)"
  elif [[ -x /usr/lib/chatgpt/resources/cua_node/bin/node ]]; then
    node_bin=/usr/lib/chatgpt/resources/cua_node/bin/node
  else
    echo "Node.js is required to start the web simulator." >&2
    return 1
  fi

  local log_file="${LOG_DIR}/web.log"
  # The simulator has a fixed local endpoint.  Do not inherit a custom PORT
  # used by a parent development server into this child process.
  (cd "${WEB_DIR}" && PORT=3002 exec setsid "${node_bin}" server.js) 9>&- >"${log_file}" 2>&1 < /dev/null &
  local pid=$!
  append_pid "${pid}" web
  wait_for_port 3002 "web simulator"
  echo "Web simulator is ready at http://127.0.0.1:3002"
}

if [[ "${MODE}" != web ]]; then start_simulation; fi
if [[ "${MODE}" != simulation ]]; then
  start_web
  if ((OPEN_BROWSER)) && command -v xdg-open >/dev/null 2>&1; then
    xdg-open http://127.0.0.1:3002/startup.html >/dev/null 2>&1 || true
  fi
fi

echo "Project startup page: http://127.0.0.1:3002/startup.html"
