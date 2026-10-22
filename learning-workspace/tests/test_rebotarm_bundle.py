"""Deployment integrity and retry invariants; never connect CAN."""
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import tarfile
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


installer = load_module('rebotarm_installer', ROOT / 'deployment/rebotarm/install.py')


class RebotarmBundleTests(unittest.TestCase):
    def make_wrapper(self, root):
        data = io.BytesIO()
        with tarfile.open(fileobj=data, mode='w:gz') as archive:
            for name, content in {'README.md': b'test', 'deployment/install.py': b'print("installed")\n'}.items():
                info = tarfile.TarInfo(name)
                info.size = len(content)
                archive.addfile(info, io.BytesIO(content))
        payload = data.getvalue()
        header = (ROOT / 'deployment/rebotarm/self-extract.sh').read_text()
        header = header.replace('@PAYLOAD_SHA@', hashlib.sha256(payload).hexdigest())
        wrapper = root / 'installer.run'
        wrapper.write_bytes(header.encode() + payload)
        return wrapper

    def test_self_extraction_at_a_different_path(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            wrapper = self.make_wrapper(root)
            destination = root / 'another computer'
            subprocess.run(['bash', str(wrapper), '--extract', str(destination)], check=True, capture_output=True)
            self.assertEqual((destination / 'README.md').read_text(), 'test')

    def test_corrupt_payload_is_rejected_before_writing_destination(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            wrapper = self.make_wrapper(root)
            with wrapper.open('ab') as handle:
                handle.write(b'corruption')
            destination = root / 'destination'
            result = subprocess.run(['bash', str(wrapper), '--extract', str(destination)], capture_output=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse(destination.exists())

    def test_existing_extract_destination_is_preserved(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            wrapper = self.make_wrapper(root)
            destination = root / 'destination'
            destination.mkdir()
            (destination / 'keep').write_text('keep')
            result = subprocess.run(['bash', str(wrapper), '--extract', str(destination)], capture_output=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual((destination / 'keep').read_text(), 'keep')

    def test_manifest_rejects_changed_source(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / 'source.py').write_text('old')
            (root / 'BUNDLE_MANIFEST.json').write_text(json.dumps({'files': {'source.py': '0' * 64}}))
            with self.assertRaisesRegex(RuntimeError, '安装文件校验失败'):
                installer.verify(root)

    def test_failed_build_keeps_current_configuration_and_reports(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source, prefix = root / 'source', root / 'prefix'
            (source / 'reBotArm_simulator-RS').mkdir(parents=True)
            (source / 'reBotArm_simulator-RS/.env.example').write_text('DEFAULT=1\n')
            (prefix / 'versions/old').mkdir(parents=True)
            (prefix / 'current').symlink_to(prefix / 'versions/old')
            shared = prefix / 'shared/outputs'
            shared.mkdir(parents=True)
            (shared / 'csp-session-important.json').write_text('preserve recovery')
            (prefix / 'shared/web.env').write_text('USER_KEY=keep\n')
            manifest = dict(version='1.0.0', source_id='a' * 64, files={})
            with patch.object(installer, 'SOURCE', source), \
                 patch.object(installer, 'verify', return_value=manifest), \
                 patch.object(installer, 'check_platform'), \
                 patch.object(installer.shutil, 'disk_usage', return_value=SimpleNamespace(free=20 * 1024**3)), \
                 patch.object(installer, 'run', side_effect=subprocess.CalledProcessError(1, 'build')), \
                 patch.object(installer.tarfile, 'open') as archive, \
                 patch.object(installer, 'desktop_entry') as desktop, \
                 patch('sys.argv', ['install.py', '--prefix', str(prefix), '--skip-system', '--no-launch']):
                tools = prefix / 'versions/1.0.0-aaaaaaaaaaaa/.tools/node'
                tools.mkdir(parents=True)
                # An interrupted installation can retry an existing target.
                (tools.parents[1] / 'reBotArm_simulator-RS').mkdir()
                with self.assertRaises(subprocess.CalledProcessError):
                    installer.main()
                desktop.assert_not_called()
            self.assertEqual((prefix / 'current').resolve(), prefix / 'versions/old')
            self.assertEqual((prefix / 'shared/web.env').read_text(), 'USER_KEY=keep\n')
            self.assertEqual((shared / 'csp-session-important.json').read_text(), 'preserve recovery')


if __name__ == '__main__':
    unittest.main()
