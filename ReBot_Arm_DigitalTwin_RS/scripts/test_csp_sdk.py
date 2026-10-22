"""Dependency checks use disposable local Git repos; no network or hardware."""
import ast
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

module_spec = importlib.util.spec_from_file_location('csp_sdk', Path(__file__).with_name('csp_sdk.py'))
sdk = importlib.util.module_from_spec(module_spec)
module_spec.loader.exec_module(sdk)


class LockTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.directory = self.root / 'rebotarm_ros2_RS/third_party/reBotArm_control_py_csp'
        self.directory.mkdir(parents=True)
        self.run_git('init', '-q')
        self.run_git('config', 'user.name', 'offline test')
        self.run_git('config', 'user.email', 'test@localhost')
        (self.directory / 'source.py').write_text('pinned = True\n')
        self.run_git('add', 'source.py')
        self.run_git('commit', '-qm', 'fixture')
        self.run_git('remote', 'add', 'origin', 'https://example.invalid/sdk.git')
        self.lock = dict(schema_version=1, path=str(self.directory.relative_to(self.root)),
                         url='https://example.invalid/sdk.git', commit=sdk.git(self.directory, 'rev-parse', 'HEAD'))
        (self.root / 'configs').mkdir()
        self.save_lock()

    def run_git(self, *args):
        subprocess.run(['git', '-C', str(self.directory), *args], check=True, capture_output=True)

    def save_lock(self):
        (self.root / 'configs/csp-sdk-lock.json').write_text(json.dumps(self.lock))

    def test_clean_checkout_is_accepted_without_changes(self):
        self.assertEqual(sdk.check(self.root), self.lock)
        self.assertEqual(sdk.prepare(self.root), self.lock)
        self.assertEqual(sdk.git(self.directory, 'status', '--porcelain'), '')

    def test_modified_untracked_or_wrong_commit_rejected_and_mirror_origin_allowed(self):
        source = self.directory / 'source.py'
        source.write_text('local edit\n')
        with self.assertRaisesRegex(ValueError, 'modified'):sdk.prepare(self.root)
        self.assertEqual(source.read_text(), 'local edit\n')
        self.run_git('checkout', '--', 'source.py')
        extra = self.directory / 'new.py';extra.write_text('unreviewed\n')
        with self.assertRaisesRegex(ValueError, 'untracked'):sdk.check(self.root)
        extra.unlink()
        self.lock['commit'] = '0' * 40;self.save_lock()
        with self.assertRaisesRegex(ValueError, 'commit'):sdk.check(self.root)
        self.lock['commit'] = sdk.git(self.directory, 'rev-parse', 'HEAD');self.save_lock()
        self.run_git('remote', 'set-url', 'origin', 'https://example.invalid/other.git')
        self.assertEqual(sdk.check(self.root), self.lock)

    def test_gitfile_metadata_is_accepted_and_unknown_nested_repo_rejected(self):
        metadata = self.root / 'sdk-metadata'
        (self.directory / '.git').rename(metadata)
        (self.directory / '.git').write_text(f'gitdir: {metadata}\n')
        self.assertEqual(sdk.check(self.root), self.lock)
        rogue = self.directory.parent / 'unregistered';rogue.mkdir();(rogue / '.git').mkdir()
        with self.assertRaisesRegex(ValueError, 'Unregistered'):sdk.check(self.root)

    def test_missing_metadata_cannot_fall_back_to_parent_git(self):
        (self.directory / '.git').rename(self.root / 'metadata')
        with self.assertRaisesRegex(ValueError, 'missing'):sdk.check(self.root)

    def test_lock_matches_hardware_loader_pin_without_importing_hardware(self):
        lock, _ = sdk.spec(sdk.ROOT)
        loader = sdk.ROOT / 'rebotarm_ros2_RS/src/rebotarmcontroller/rebotarmcontroller/csp_runtime/rebot_seeed_transport.py'
        tree = ast.parse(loader.read_text())
        pin = next(ast.literal_eval(n.value) for n in tree.body if isinstance(n, ast.Assign)
                   and any(isinstance(t, ast.Name) and t.id == 'COMMIT' for t in n.targets))
        self.assertEqual(lock['commit'], pin)


if __name__ == '__main__':unittest.main()
