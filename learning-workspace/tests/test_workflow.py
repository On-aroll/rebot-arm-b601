"""Check deployment boundaries without importing robot drivers or touching devices."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

SOURCE = Path(__file__).resolve().parents[1] / "scripts/robotctl.py"
SPEC = importlib.util.spec_from_file_location("robotctl", SOURCE)
ctl = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ctl)


class WorkflowTests(unittest.TestCase):
    def setUp(self):
        self.config = ctl.load_config(ctl.ROOT / "config.example.json")
        self.parser = ctl.parser()

    def test_record_and_training_share_the_local_dataset(self):
        record, _ = ctl.build_command(self.config, self.parser.parse_args(["record"]))
        train, _ = ctl.build_command(self.config, self.parser.parse_args(["train"]))
        root = next(arg for arg in record if arg.startswith("--dataset.root="))
        self.assertIn(root, train)
        self.assertIn("--dataset.push_to_hub=false", record)
        self.assertIn("--policy.push_to_hub=false", train)
        self.assertIn("--wandb.enable=false", train)
        self.assertIn("--dataset.video_backend=pyav", train)

    def test_dry_run_never_starts_a_process_or_writes_state(self):
        for arguments in (["record", "--dry-run"], ["calibrate", "follower", "--dry-run"],
                          ["train", "--dry-run"], ["setup-can", "--dry-run"]):
            with patch.object(ctl.subprocess, "Popen") as process, patch.object(Path, "mkdir") as mkdir:
                ctl.dispatch(self.parser.parse_args(arguments))
                process.assert_not_called()
                mkdir.assert_not_called()

    def test_mismatched_camera_config_prevents_model_execution(self):
        with tempfile.TemporaryDirectory() as temporary:
            model = Path(temporary)
            (model / "config.json").write_text("{}")
            (model / "model.safetensors").touch()
            saved = copy.deepcopy(self.config)
            saved["cameras"]["front"]["index_or_path"] = 2
            (model / "deployment.json").write_text(json.dumps(saved))
            args = self.parser.parse_args(["run", "--model", temporary, "--dry-run"])
            with self.assertRaisesRegex(ValueError, "配置与训练时不同"):
                ctl.build_command(self.config, args)

    def test_invalid_steps_fail_before_starting_training(self):
        args = self.parser.parse_args(["train", "--steps", "-1", "--dry-run"])
        with patch.object(ctl, "execute") as process, self.assertRaises(ValueError):
            ctl.dispatch(args)
        process.assert_not_called()

    def test_custom_missing_config_does_not_silently_use_default(self):
        with self.assertRaises(FileNotFoundError):
            ctl.load_config(Path("/definitely-missing-config.json"))


if __name__ == "__main__":
    unittest.main()
