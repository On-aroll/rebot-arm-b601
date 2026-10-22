#!/usr/bin/env python3
"""Local-only entry points for the Seeed B601-RS LeRobot workflow."""
import argparse
from datetime import datetime
import glob
import importlib
import importlib.metadata
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def stamp():
    return datetime.now().strftime("%Y%m%d-%H%M%S-%f")


def local_path(value):
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (ROOT / path).resolve()


def load_config(path):
    if not path.exists() and path == ROOT / "config.json":
        path = ROOT / "config.example.json"
    config = json.loads(path.read_text(encoding="utf-8"))
    for key in ("robot", "teleop", "cameras", "dataset", "training", "display_data"):
        if key not in config:
            raise ValueError(f"配置缺少 {key}")
    if config["robot"]["type"] != "seeed_b601_rs_follower":
        raise ValueError("此部署包针对 B601-RS，robot.type 应为 seeed_b601_rs_follower。")
    if config["robot"]["can_adapter"] != "socketcan":
        raise ValueError("此部署包使用 SocketCAN；请设置 robot.can_adapter=socketcan。")
    if not re.fullmatch(r"[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+", config["dataset"]["repo_id"]):
        raise ValueError("dataset.repo_id 格式应为 local/数据集名称。")
    if not config["cameras"]:
        raise ValueError("视觉 ACT 至少需要一个摄像头。")
    for name, camera in config["cameras"].items():
        if camera["type"] != "opencv":
            raise ValueError(f"{name}：此部署包支持普通 USB RGB 摄像头（opencv）。")
        for key in ("width", "height", "fps"):
            if type(camera[key]) is not int or camera[key] < 1:
                raise ValueError(f"摄像头 {name}.{key} 必须是正整数。")
    for section, keys in {
        "dataset": ("fps", "num_episodes", "episode_time_s", "reset_time_s"),
        "training": ("steps", "batch_size", "chunk_size"),
    }.items():
        for key in keys:
            if type(config[section][key]) is not int or config[section][key] < 1:
                raise ValueError(f"{section}.{key} 必须是正整数。")
    if config["training"]["device"] not in ("cpu", "cuda"):
        raise ValueError("training.device 必须是 cpu 或 cuda。")
    if type(config["training"]["num_workers"]) is not int or config["training"]["num_workers"] < 0:
        raise ValueError("training.num_workers 必须是非负整数。")
    if type(config["display_data"]) is not bool:
        raise ValueError("display_data 必须是 true 或 false。")
    return config


def options(prefix, values):
    return [f"--{prefix}.{key}={json.dumps(value, ensure_ascii=False) if not isinstance(value, str) else value}"
            for key, value in values.items()]


def build_command(config, args, run_id=None):
    run_id = run_id or stamp()
    command = args.command
    robot = options("robot", config["robot"])
    cameras = f"--robot.cameras={json.dumps(config['cameras'])}"
    teleop = options("teleop", config["teleop"])
    data = config["dataset"]
    dataset = [f"--dataset.{key}={data[key]}" for key in
               ("repo_id", "fps", "num_episodes", "episode_time_s", "reset_time_s")]
    dataset += [f"--dataset.root={local_path(data['root'])}",
                f"--dataset.single_task={data['task']}", "--dataset.push_to_hub=false",
                "--dataset.vcodec=h264"]
    display = f"--display_data={str(config['display_data']).lower()}"
    module = "lerobot.scripts.lerobot_"
    output = None
    if command == "cameras":
        return [sys.executable, "-m", module + "find_cameras", "opencv"], None
    if command == "calibrate":
        selected = robot if args.target == "follower" else teleop
        return [sys.executable, "-m", module + "calibrate", *selected], None
    if command == "teleop":
        return [sys.executable, "-m", module + "teleoperate", *robot, cameras, *teleop,
                f"--fps={data['fps']}", display], None
    if command == "record":
        return [sys.executable, "-m", module + "record", *robot, cameras, *teleop,
                *dataset, display, "--play_sounds=false", f"--resume={str(args.resume).lower()}"], None
    if command == "train":
        training = config["training"]
        device = args.device or training["device"]
        output = (ROOT / "data/outputs" / f"act-{run_id}").resolve()
        cmd = [sys.executable, "-m", module + "train", "--policy.type=act",
               f"--dataset.repo_id={data['repo_id']}",
               f"--dataset.root={local_path(data['root'])}", "--dataset.video_backend=pyav",
               f"--policy.device={device}", "--policy.push_to_hub=false", "--wandb.enable=false",
               f"--output_dir={output}", f"--job_name=act-{run_id}", "--eval_freq=0",
               f"--steps={args.steps or training['steps']}",
               f"--batch_size={args.batch_size or training['batch_size']}",
               f"--num_workers={training['num_workers']}",
               f"--policy.chunk_size={training['chunk_size']}",
               f"--policy.n_action_steps={training['chunk_size']}"]
        return cmd, output
    if command == "run":
        if args.model:
            model = local_path(args.model)
        else:
            pointer = ROOT / ".state/latest-model.txt"
            if not pointer.exists():
                raise ValueError("尚未训练模型，请先 train，或通过 --model 指定本地模型目录。")
            model = Path(pointer.read_text().strip())
        if not (model / "config.json").is_file() or not (model / "model.safetensors").is_file():
            raise ValueError(f"模型不完整：{model}，应包含 config.json 和 model.safetensors。")
        manifest = model / "deployment.json"
        if manifest.exists():
            saved = json.loads(manifest.read_text())
            if saved["cameras"] != config["cameras"] or saved["robot"] != config["robot"]:
                raise ValueError("当前机械臂/摄像头配置与训练时不同，请先核对模型目录中的 deployment.json。")
        evaluation = f"eval_rebot-{run_id}"
        dataset = [f"--dataset.repo_id=local/{evaluation}",
                   f"--dataset.root={ROOT / 'data/evaluations' / evaluation}",
                   f"--dataset.single_task={data['task']}", f"--dataset.fps={data['fps']}",
                   f"--dataset.episode_time_s={data['episode_time_s']}",
                   f"--dataset.reset_time_s={data['reset_time_s']}",
                   "--dataset.num_episodes=1", "--dataset.push_to_hub=false", "--dataset.vcodec=h264"]
        return [sys.executable, "-m", module + "record", *robot, cameras, *dataset,
                f"--policy.path={model}", f"--policy.device={args.device or config['training']['device']}",
                display, "--play_sounds=false"], None
    raise ValueError(f"未知命令 {command}")


def execute(cmd, label, dry_run=False, env=None):
    print("\n" + shlex.join(map(str, cmd)), flush=True)
    if dry_run:
        return
    (ROOT / "logs").mkdir(exist_ok=True)
    log_path = ROOT / "logs" / f"{label}-{stamp()}.log"
    print(f"日志：{log_path}", flush=True)
    with log_path.open("w", encoding="utf-8") as stream:
        with subprocess.Popen(cmd, cwd=ROOT, env=env, stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, text=True, bufsize=1) as process:
            try:
                for line in process.stdout:
                    print(line, end="", flush=True)
                    stream.write(line)
                    stream.flush()
                status = process.wait()
            except KeyboardInterrupt:
                # The terminal also delivers SIGINT to the child in the same process group.
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.terminate()
                raise
    if status:
        raise RuntimeError(f"{label} 失败（退出码 {status}），请查看 {log_path}")


def doctor(config, software_only=False):
    failures = []
    print(f"Python：{sys.version.split()[0]}；环境：{sys.prefix}")
    for package in ("lerobot", "torch", "torchvision", "av", "lerobot_robot_seeed_b601",
                    "lerobot_teleoperator_rebot_arm_102", "motorbridge"):
        try:
            importlib.import_module(package)
            print(f"[通过] {package} {importlib.metadata.version(package)}")
        except Exception as error:
            print(f"[失败] {package}: {error}")
            failures.append(package)
    try:
        from lerobot.robots.config import RobotConfig
        from lerobot.teleoperators.config import TeleoperatorConfig
        RobotConfig.get_choice_class(config["robot"]["type"])
        TeleoperatorConfig.get_choice_class(config["teleop"]["type"])
        print("[通过] B601-RS / reBot 102 插件注册")
    except Exception as error:
        print(f"[失败] 插件注册：{error}")
        failures.append("插件注册")
    if not shutil.which("ffmpeg"):
        print("[失败] 系统 ffmpeg 未安装")
        failures.append("ffmpeg")
    if "torch" not in failures:
        import torch
        cuda = torch.cuda.is_available()
        print(f"GPU：{'可用：' + torch.cuda.get_device_name(0) if cuda else '不可用，CPU 模式可以运行；训练会较慢'}")
    if software_only:
        if failures:
            raise RuntimeError("软件诊断未通过：" + ", ".join(failures))
        return
    print(f"桌面：{os.environ.get('XDG_CURRENT_DESKTOP', '未知')}；会话：{os.environ.get('XDG_SESSION_TYPE', '未知')}")
    print(f"桌面会话：{'已检测到' if os.environ.get('DISPLAY') else '未检测到 DISPLAY；录制快捷键需要桌面终端'}")
    print(f"串口：{', '.join(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')) or '未发现'}")
    print(f"视频设备：{', '.join(glob.glob('/dev/video*')) or '未发现'}")
    ports = [("主臂", config["teleop"]["port"])]
    for name, port in ports:
        if not Path(port).exists() or not os.access(port, os.R_OK | os.W_OK):
            print(f"[失败] {name} {port} 不存在或没有读写权限；检查 USB 连接与串口用户组（dialout / uucp）。")
            failures.append(name)
    can = Path("/sys/class/net") / config["robot"]["port"]
    if not can.exists():
        print(f"[失败] 未发现 {can.name}；检查 USB-CAN 连接和适配器驱动。")
        failures.append("CAN")
    else:
        print(f"CAN：{can.name}，运行 ./robotctl setup-can 配置 1 Mbit/s。")
    for name, camera in config["cameras"].items():
        device = Path(f"/dev/video{camera['index_or_path']}" if isinstance(camera['index_or_path'], int)
                      else str(camera['index_or_path']))
        if not device.exists() or not os.access(device, os.R_OK | os.W_OK):
            print(f"[失败] 摄像头 {name}: {device} 不存在或没有权限。")
            failures.append(name)
    if failures:
        raise RuntimeError("诊断发现待处理项：" + ", ".join(failures))


def configure(config, path):
    def ask(title, current):
        return input(f"{title} [{current}]：").strip() or current
    config["robot"]["port"] = ask("从臂 CAN 接口", config["robot"]["port"])
    config["teleop"]["port"] = ask("主臂串口", config["teleop"]["port"])
    camera = config["cameras"][next(iter(config["cameras"]))]
    index = ask("第一个摄像头编号或 /dev/video 路径", str(camera["index_or_path"]))
    camera["index_or_path"] = int(index) if str(index).isdigit() else index
    config["dataset"]["task"] = ask("任务描述", config["dataset"]["task"])
    config["training"]["device"] = ask("训练设备 cpu/cuda", config["training"]["device"])
    text = json.dumps(config, ensure_ascii=False, indent=2) + "\n"
    # Validate before replacing a working configuration.
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(text, encoding="utf-8")
    try:
        load_config(temporary)
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)
    print(f"已保存：{path}；其他参数可直接编辑此 JSON。")


def parser():
    cli = argparse.ArgumentParser(description="Seeed B601-RS Linux 部署工具")
    cli.add_argument("--config", type=Path, default=ROOT / "config.json")
    commands = cli.add_subparsers(dest="command")
    check = commands.add_parser("doctor", help="检查软件与设备")
    check.add_argument("--software-only", action="store_true")
    commands.add_parser("configure", help="交互填写设备配置")
    smoke = commands.add_parser("self-test", help="无需硬件的离线 ACT 训练和推理自检")
    smoke.add_argument("--device", choices=["cpu", "cuda"], default="cpu")
    for name in ("cameras", "setup-can", "calibrate", "teleop", "record", "train", "run"):
        child = commands.add_parser(name)
        child.add_argument("--dry-run", action="store_true", help="仅显示命令，不运行或连接设备")
        if name == "calibrate":
            child.add_argument("target", choices=["leader", "follower"])
        if name == "record":
            child.add_argument("--resume", action="store_true", help="继续向现有数据集追加示范")
        if name in ("train", "run"):
            child.add_argument("--device", choices=["cpu", "cuda"])
        if name == "train":
            child.add_argument("--steps", type=int)
            child.add_argument("--batch-size", type=int)
        if name == "run":
            child.add_argument("--model", help="本地 pretrained_model 目录；默认最近一次训练结果")
    return cli


def menu(cli, config_path):
    actions = ["configure", "doctor", "self-test", "cameras", "setup-can", "calibrate leader",
               "calibrate follower", "teleop", "record", "train", "run"]
    descriptions = ["填写设备配置", "环境与设备诊断", "无需硬件的软件自检", "查找摄像头", "配置 CAN 总线",
                    "校准主臂", "校准从臂", "遥操作", "采集示范数据", "训练 ACT", "运行训练好的模型"]
    while True:
        print("\nSeeed B601-RS：")
        for index, action in enumerate(actions, 1):
            print(f"{index:2}. {descriptions[index-1]} ({action})")
        selected = input("选择编号（0 退出）：").strip()
        if selected == "0":
            return
        if not selected.isdigit() or not 1 <= int(selected) <= len(actions):
            continue
        try:
            dispatch(cli.parse_args(["--config", str(config_path), *actions[int(selected)-1].split()]))
        except (ValueError, RuntimeError, OSError, KeyError) as error:
            print(f"错误：{error}")
        except KeyboardInterrupt:
            print("\n操作已停止。")


def dispatch(args):
    config = load_config(args.config)
    if args.command == "configure":
        return configure(config, args.config)
    if args.command == "doctor":
        return doctor(config, args.software_only)
    if args.command == "self-test":
        environment = dict(os.environ, HF_HUB_OFFLINE="1", HF_DATASETS_OFFLINE="1", OMP_NUM_THREADS="2")
        return execute([sys.executable, str(ROOT / "scripts/self_test.py"), "--device", args.device],
                       "self-test", env=environment)
    if args.command == "setup-can":
        interface = config["robot"]["port"]
        if not re.fullmatch(r"[A-Za-z0-9_.-]{1,15}", interface):
            raise ValueError("无效 CAN 接口名称。")
        if not args.dry_run and not (Path("/sys/class/net") / interface).exists():
            raise ValueError(f"未找到 {interface}，先检查 USB-CAN 连接和驱动。")
        for operation in (["down"], ["type", "can", "bitrate", "1000000"], ["up"]):
            execute(["sudo", "ip", "link", "set", interface, *operation], "setup-can", args.dry_run)
        return
    if args.command == "train":
        if args.steps is not None and args.steps < 1 or args.batch_size is not None and args.batch_size < 1:
            raise ValueError("steps 和 batch-size 必须大于零。")
        if not args.dry_run and not (local_path(config["dataset"]["root"]) / "meta/info.json").is_file():
            raise ValueError("尚无本地数据集，请先 record 或在配置中填写已有数据集目录。")
    if args.command == "record" and not args.dry_run:
        exists = local_path(config["dataset"]["root"]).exists()
        if exists != args.resume:
            raise ValueError("已有数据集请加 --resume；首次录制不要加 --resume。")
    cmd, output = build_command(config, args)
    if not args.dry_run and args.command in ("record", "teleop", "calibrate", "run"):
        print("即将连接真实机械臂，请确保设备已固定、周围没有障碍物；Ctrl+C 停止。", flush=True)
    if not args.dry_run and args.command in ("train", "run"):
        device = args.device or config["training"]["device"]
        if device == "cuda":
            import torch
            if not torch.cuda.is_available():
                raise ValueError("CUDA 不可用，请使用 --device cpu，或检查 NVIDIA 驱动并安装 CUDA 版依赖。")
    execute(cmd, args.command, args.dry_run)
    if output and not args.dry_run:
        model = output / "checkpoints/last/pretrained_model"
        if not (model / "model.safetensors").is_file():
            raise RuntimeError(f"训练未产生预期权重：{model}")
        (model / "deployment.json").write_text(json.dumps(config, ensure_ascii=False, indent=2))
        (ROOT / ".state").mkdir(exist_ok=True)
        (ROOT / ".state/latest-model.txt").write_text(str(model))
        print(f"模型：{model}\n运行：./robotctl run")


def main():
    cli = parser()
    args = cli.parse_args()
    try:
        if args.command is None:
            menu(cli, args.config)
        else:
            dispatch(args)
    except KeyboardInterrupt:
        print("\n操作已停止。", file=sys.stderr)
        return 130
    except (ValueError, RuntimeError, OSError, KeyError) as error:
        print(f"错误：{error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
