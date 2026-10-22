"""Build a local inert SDK fixture; check relocatable offline tool delivery."""
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('qualification_bundle', ROOT / 'scripts/build_rebotarm_bundle.py')
builder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(builder)


class QualificationBundleTests(unittest.TestCase):
    def fixture(self, root):
        source, overlay, stage = (root / name for name in ('source', 'overlay', 'stage'))
        files = {
            'configs/csp-sdk-lock.json': '',
            'configs/private.json': 'not distributed',
            'scripts/rs_env.sh': 'REBOTARM_RS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"',
            'scripts/quick_start.sh': 'ENV_SCRIPT="${ROOT_DIR}/scripts/rs_env.sh"',
            'examples/autonomy/qualify.py': '# inert offline tool\n',
            'examples/autonomy/qualification/common.py': '# pure module\n',
            'examples/autonomy/fixtures/qualification/input.json': '{}',
            'reBotArm_simulator-RS/server.js': '// inert\n',
            'reBotArm_simulator-RS/.env': 'SECRET=hidden',
            'reBotArm_simulator-RS/local.key': 'private',
            'scripts/outputs/private.log': 'private',
            'rebotarm_ros2_RS/src/rebotarmcontroller/rebotarmcontroller/csp_twin_driver.py':
                'root = "/home/bonvoyage/Desktop/seeed_Robotics"',
            'rebotarm_ros2_RS/src/rebotarmcontroller/rebotarmcontroller/csp_runtime/rebot_seeed_transport.py':
                "OFFICIAL = Path(__file__).resolve().parents[4] / 'third_party/reBotArm_control_py_csp'",
            'rebotarm_ros2_RS/third_party/reBotArm_control_py/legacy.py': '# legacy\n',
            'rebotarm_ros2_RS/third_party/README.md': 'provenance',
            'docs/guide.md': 'guide',
            'docs/research/evidence.md': 'research',
            'docs/history/original.md': 'history',
            'docs/outputs/private.md': 'excluded',
            'requirements-rs-hardware.txt': '# pinned runtime\n',
            'requirements-vision-offline.txt': '# pinned vision\n',
        }
        for name, value in files.items():
            path = source / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(value)
        sdk = source / 'rebotarm_ros2_RS/third_party/reBotArm_control_py_csp'
        sdk.mkdir()
        (sdk / 'inert.py').write_text('# no device imports\n')
        subprocess.run(['git', 'init', '-q', str(sdk)], check=True)
        subprocess.run(['git', '-C', str(sdk), 'add', 'inert.py'], check=True)
        subprocess.run(['git', '-C', str(sdk), '-c', 'user.name=Fixture', '-c', 'user.email=fixture@example.invalid',
                        'commit', '-qm', 'fixture'], check=True)
        commit = subprocess.check_output(['git', '-C', str(sdk), 'rev-parse', 'HEAD'], text=True).strip()
        lock = dict(schema_version=1, commit=commit, path='rebotarm_ros2_RS/third_party/reBotArm_control_py_csp')
        (source / 'configs/csp-sdk-lock.json').write_text(json.dumps(lock))
        overlay.mkdir();stage.mkdir()
        for name in ('README.md', 'rebotarm-app', 'install_desktop_launcher.sh'):
            (overlay / name).write_text('# fixture\n')
        return source, overlay, stage, sdk, commit

    def test_offline_tools_docs_and_lock_are_delivered_and_hashed(self):
        with tempfile.TemporaryDirectory() as temp:
            source, overlay, stage, sdk, commit = self.fixture(Path(temp))
            with patch.object(builder, 'OVERLAY', overlay), patch.object(builder, 'SDK_COMMIT', commit):
                manifest = builder.stage_bundle(source, stage)
            for name in ('configs/csp-sdk-lock.json', 'requirements-vision-offline.txt', 'docs/research/evidence.md',
                         'docs/history/original.md', 'rebotarm_ros2_RS/third_party/README.md',
                         'examples/autonomy/qualify.py', 'examples/autonomy/qualification/common.py',
                         'examples/autonomy/fixtures/qualification/input.json'):
                self.assertEqual(manifest['files'][name], hashlib.sha256((stage / name).read_bytes()).hexdigest())
            for name in ('configs/private.json', 'reBotArm_simulator-RS/.env', 'reBotArm_simulator-RS/local.key',
                         'scripts/outputs/private.log', 'docs/outputs/private.md'):
                self.assertFalse((stage / name).exists(), name)
            bundled = stage / sdk.relative_to(source)
            self.assertEqual(subprocess.check_output(['git', '-C', str(bundled), 'rev-parse', 'HEAD'], text=True).strip(), commit)
            self.assertFalse(subprocess.check_output(['git', '-C', str(bundled), 'status', '--porcelain', '--untracked-files=all'], text=True).strip())
            self.assertFalse(subprocess.check_output(['git', '-C', str(bundled), 'remote'], text=True).strip())
            self.assertFalse((bundled / '.git/FETCH_HEAD').exists())

    def test_changed_lock_rejected_before_staging(self):
        with tempfile.TemporaryDirectory() as temp:
            source, overlay, stage, _, _ = self.fixture(Path(temp))
            with patch.object(builder, 'OVERLAY', overlay), patch.object(builder, 'SDK_COMMIT', '0' * 40):
                with self.assertRaisesRegex(RuntimeError, '清单'):
                    builder.stage_bundle(source, stage)
            self.assertEqual(list(stage.iterdir()), [])

    def test_untracked_sdk_file_rejected_before_staging(self):
        with tempfile.TemporaryDirectory() as temp:
            source, overlay, stage, sdk, commit = self.fixture(Path(temp))
            (sdk / 'unexpected.py').write_text('unknown')
            with patch.object(builder, 'OVERLAY', overlay), patch.object(builder, 'SDK_COMMIT', commit):
                with self.assertRaisesRegex(RuntimeError, '已审核提交'):
                    builder.stage_bundle(source, stage)
            self.assertEqual(list(stage.iterdir()), [])


if __name__ == '__main__':unittest.main()
