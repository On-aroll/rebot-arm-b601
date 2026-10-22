#!/usr/bin/env python3
"""Exercise real dataset/video, ACT training, checkpointing, and inference, without hardware."""
import argparse
import json
import os
from pathlib import Path
import tempfile

os.environ.setdefault("HF_HUB_OFFLINE", "1")
os.environ.setdefault("HF_DATASETS_OFFLINE", "1")
os.environ.setdefault("OMP_NUM_THREADS", "2")

import numpy as np
import torch

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors
from robotctl import ROOT, build_command, execute, load_config


def self_test(device):
    if device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA 不可用。请检查 NVIDIA 显卡、驱动和 CUDA 版 PyTorch；也可改用 CPU 安装。")
    torch.set_num_threads(2)
    directory = ROOT / ".state/self-test"
    directory.mkdir(parents=True, exist_ok=True)
    print("自检使用合成图像和关节数据，不连接机械臂，不验证抓取效果。", flush=True)
    with tempfile.TemporaryDirectory(prefix="act-", dir=directory) as temporary:
        work = Path(temporary)
        dataset_root = work / "dataset"
        names = ["shoulder_pan.pos", "shoulder_lift.pos", "elbow_flex.pos", "wrist_flex.pos",
                 "wrist_yaw.pos", "wrist_roll.pos", "gripper.pos"]
        features = {
            "observation.state": {"dtype": "float32", "shape": (7,), "names": names},
            "action": {"dtype": "float32", "shape": (7,), "names": names},
            "observation.images.front": {"dtype": "video", "shape": (64, 64, 3),
                                         "names": ["height", "width", "channels"]},
        }
        dataset = LeRobotDataset.create(
            repo_id="local/deployment_self_test", root=dataset_root, fps=10,
            features=features, robot_type="seeed_b601_rs_follower", vcodec="h264", video_backend="pyav",
        )
        generator = np.random.default_rng(42)
        for episode in range(2):
            for frame in range(12):
                state = np.sin(np.arange(7, dtype=np.float32) + frame / 10 + episode).astype(np.float32)
                dataset.add_frame({"observation.state": state, "action": state + 0.1,
                                   "observation.images.front": generator.integers(0, 256, (64, 64, 3), dtype=np.uint8),
                                   "task": "Synthetic deployment check"})
            dataset.save_episode(parallel_encoding=False)
        dataset.finalize()
        del dataset
        config = load_config(ROOT / "config.example.json")
        config["dataset"]["repo_id"] = "local/deployment_self_test"
        config["dataset"]["root"] = str(dataset_root)
        config["training"].update(device=device, chunk_size=4, num_workers=0)
        args = argparse.Namespace(command="train", device=device, steps=2, batch_size=2)
        command, _ = build_command(config, args, run_id="self-test")
        # The smoke run uses the same training entry point with a smaller transformer and no downloads.
        command = [arg for arg in command if not arg.startswith("--output_dir=")]
        output = work / "output"
        command += [f"--output_dir={output}", "--policy.pretrained_backbone_weights=null",
                    "--policy.dim_model=64", "--policy.n_heads=4", "--policy.dim_feedforward=128",
                    "--policy.n_encoder_layers=1", "--policy.n_decoder_layers=1",
                    "--policy.n_vae_encoder_layers=1", "--log_freq=1", "--save_freq=2"]
        execute(command, "self-test-training", env=os.environ.copy())
        model_root = output / "checkpoints/last/pretrained_model"
        policy = ACTPolicy.from_pretrained(model_root)
        policy.to(device).eval()
        before, after = make_pre_post_processors(policy.config, pretrained_path=str(model_root))
        dataset = LeRobotDataset("local/deployment_self_test", root=dataset_root, video_backend="pyav")
        sample = dataset[0]
        observation = {key: value for key, value in sample.items() if key.startswith("observation.")}
        with torch.inference_mode():
            action = after(policy.select_action(before(observation)))
        if tuple(action.shape) != (1, 7) or not torch.isfinite(action).all():
            raise RuntimeError(f"推理输出异常：{action}")
        report = {"passed": True, "device": device, "episodes": 2, "frames": 24,
                  "training_steps": 2, "action_shape": list(action.shape),
                  "hardware_tested": False, "grasp_success_tested": False}
        (directory / "last-result.json").write_text(json.dumps(report, indent=2) + "\n")
        print("\n自检通过：H.264 视频数据集 → ACT 训练 2 步 → 保存/加载权重 → 有限的 7 维动作输出。")
        print(f"报告：{directory / 'last-result.json'}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", choices=["cpu", "cuda"], default="cpu")
    self_test(parser.parse_args().device)
