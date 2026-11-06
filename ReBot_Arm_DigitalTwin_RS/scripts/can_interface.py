#!/usr/bin/python3
"""Identify the PEAK USB adapter before using the audited can0 executor.

No CAN socket or motor command is created here. Configuration is guarded by
prepare_can.sh's existing can0 ownership lock and kernel receiver checks.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

IP = "/usr/sbin/ip"
NET = Path("/sys/class/net")
USB_SETTLE_SECONDS = 6


class InterfaceChanged(RuntimeError):
    """The kernel removed or replaced an interface during discovery."""


def require_interface(name, ifindex):
    try:
        current = int((NET / name / "ifindex").read_text())
    except FileNotFoundError:
        current = None
    if current != ifindex:
        raise InterfaceChanged(f"{name} 在 USB 枚举期间已改变，请等待适配器重新识别。")


def inventory():
    links = json.loads(subprocess.check_output(
        [IP, "-j", "-details", "link", "show", "type", "can"], text=True, timeout=2))
    drivers = {}
    for item in links:
        name = item["ifname"]
        require_interface(name, item["ifindex"])
        try:
            drivers[name] = (NET / name / "device/driver").resolve(strict=True).name
        except FileNotFoundError:
            drivers[name] = None
        require_interface(name, item["ifindex"])
    return links, drivers


def describe(links, drivers):
    usb = [item for item in links if drivers.get(item["ifname"]) == "peak_usb"]
    result = dict(ready=False, present=bool(usb), interface=None, up=False, bitrate=0, state="", usb_port=None)
    if len(usb) > 1:
        return {**result, "message": "检测到多个 PEAK USB-CAN，未自动选择或改名；请先保留本机械臂的适配器。"}
    if not usb:
        return {**result, "message": "未在已识别的 USB 设备中找到机械臂的 PEAK USB-CAN。"}
    selected = usb[0]
    name = selected["ifname"]
    usb_port = selected.get("parentdev", "").split(":", 1)[0] or None
    port_label = f" · USB {usb_port}" if usb_port else ""
    data = selected.get("linkinfo", {}).get("info_data", {})
    up = "UP" in selected.get("flags", [])
    bitrate = data.get("bittiming", {}).get("bitrate", 0)
    state = data.get("state", "")
    result.update(interface=name, up=up, bitrate=bitrate, state=state, usb_port=usb_port)
    if name != "can0":
        message = f"PEAK USB-CAN · {name}{port_label}。点击启用 CAN，安全检查后校正命名。"
        if any(item["ifname"] == "can0" for item in links):
            message += " 当前 can0 由板载或其他接口占用。"
        return {**result, "message": message}
    result.update(ready=up and bitrate == 1000000 and state in ("ERROR-ACTIVE", "ERROR-WARNING", "ERROR-PASSIVE"),
                  message=f"can0 · PEAK USB-CAN · {'UP' if up else 'DOWN'} · "
                          f"{str(bitrate / 1000000) + ' Mbps' if bitrate else '未配置速率'} · {state}{port_label}")
    return result


def discover_usb():
    deadline = time.monotonic() + USB_SETTLE_SECONDS
    while True:
        try:
            snapshot = inventory()
            if describe(*snapshot)["present"] or time.monotonic() >= deadline:
                return snapshot
        except InterfaceChanged:
            if time.monotonic() >= deadline:
                raise
        time.sleep(0.2)


def configuration_commands(links, drivers):
    status = describe(links, drivers)
    if status["ready"]:
        return []
    name = status["interface"]
    if not name:
        raise RuntimeError(status["message"])
    commands = []
    if status["state"] == "BUS-OFF":
        raise RuntimeError("USB-CAN 为 BUS-OFF，请检查接线与供电；未自动重置接口。")
    if status["up"]:
        raise RuntimeError(f"{name} 已启用但尚未就绪；未关闭或重新配置活动接口。")
    if name != "can0":
        by_name = {item["ifname"]: item for item in links}
        if "can0" in by_name:
            native = by_name["can0"]
            if drivers.get("can0") != "mttcan" or "can_native0" in by_name:
                raise RuntimeError("can0 / can_native0 名称冲突，未修改未知接口。")
            if "UP" in native.get("flags", []):
                if native.get("linkinfo", {}).get("info_data", {}).get("state") != "BUS-OFF":
                    raise RuntimeError("板载 can0 仍活动，禁止自动关闭或改名。")
                commands.append(["link", "set", "can0", "down"])
            commands.append(["link", "set", "can0", "name", "can_native0"])
        commands.append(["link", "set", name, "name", "can0"])
    return commands + [["link", "set", "can0", "type", "can", "bitrate", "1000000"],
                       ["link", "set", "can0", "up"]]


def require_no_receivers(lines, interfaces):
    for line in lines:
        fields = line.split()
        if len(fields) >= 5 and fields[0] in {*interfaces, "any"}:
            raise RuntimeError("CAN 存在接收者，未重新配置。请先在支撑条件下结束原控制会话。")


def prepare(apply=False):
    links, drivers = discover_usb()
    commands = configuration_commands(links, drivers)
    if not commands:
        print("PEAK / SocketCAN 已为 can0 · UP / 1 Mbps，复用当前配置。")
        return
    receivers = []
    for kind in ("all", "fil", "inv", "sff", "eff", "err"):
        filename = Path("/proc/net/can") / ("rcvlist_" + kind)
        if filename.exists():
            receivers.extend(filename.read_text().splitlines())
    require_no_receivers(receivers, {item["ifname"] for item in links})
    if not apply and os.geteuid() != 0:
        # One authentication for the whole checked transaction, no permanent rule.
        elevated = ["sudo", "-n"] if subprocess.run(
            ["sudo", "-n", "true"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0 else ["pkexec"]
        subprocess.run([*elevated, "/usr/bin/python3", str(Path(__file__).resolve()), "--apply"], check=True)
        return
    if os.geteuid() != 0:
        raise RuntimeError("CAN 配置需要本机管理员认证。")
    indexes = {item["ifname"]: item["ifindex"] for item in links}
    adapter_index = indexes[describe(links, drivers)["interface"]]
    for args in commands:
        name = args[2]
        require_interface(name, indexes[name])
        print("配置接口：" + " ".join(args), flush=True)
        subprocess.run([IP, *args], check=True)
        if args[3] == "name":
            indexes[args[4]] = indexes.pop(name)
    status = describe(*inventory())
    require_interface("can0", adapter_index)
    if not status["ready"]:
        raise RuntimeError(status["message"])
    print("USB-CAN 配置完成：" + status["message"])


def main():
    action = sys.argv[1] if len(sys.argv) == 2 else ""
    if action == "--status":
        try:
            status = describe(*inventory())
        except InterfaceChanged as error:
            status = {"ready": False, "present": False, "message": str(error)}
        print(json.dumps(status, ensure_ascii=False))
    elif action == "--require-ready":
        status = describe(*discover_usb())
        if not status["ready"]:
            raise RuntimeError("实机反馈未启动：" + status["message"])
        print(status["message"])
    elif action in ("--prepare", "--apply"):
        prepare(apply=action == "--apply")
    else:
        raise RuntimeError("usage: can_interface.py --status | --require-ready | --prepare | --apply")


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, OSError, subprocess.SubprocessError) as error:
        print(str(error), file=sys.stderr)
        sys.exit(1)
