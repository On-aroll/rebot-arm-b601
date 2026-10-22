"""Exercise the release trust boundary and failed-upgrade data preservation."""
import argparse
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


release = module("release", ROOT / "install-release.py")
builder = module("builder", ROOT / "scripts/build_bundle.py")


class ReleaseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        builder.main()
        cls.version = (ROOT / "VERSION").read_text().strip()
        cls.tag = "v" + cls.version
        cls.archive = ROOT / "dist" / builder.asset_name(cls.version)

    def test_bundle_is_reproducible_and_excludes_user_state(self):
        before = self.archive.read_bytes()
        builder.main()
        self.assertEqual(before, self.archive.read_bytes())
        with tarfile.open(self.archive) as archive:
            names = {str(Path(item.name).relative_to("seeed-lerobot-linux")) for item in archive}
        self.assertEqual(names, set(builder.FILES))
        self.assertTrue({"config.json", "data", ".venv", ".tools", "logs"}.isdisjoint(names))

    def test_corrupt_archive_is_rejected_before_extraction(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            archive = root / self.archive.name
            archive.write_bytes(b"not the approved release")
            destination = root / "unpack"
            with self.assertRaisesRegex(ValueError, "SHA256"):
                release.unpack_verified(archive, ROOT / "dist/SHA256SUMS", destination, self.tag)
            self.assertFalse(destination.exists())

    def test_path_traversal_and_symlinks_are_rejected(self):
        for name, kind in (("seeed-lerobot-linux/../../outside", tarfile.REGTYPE),
                           ("seeed-lerobot-linux/link", tarfile.SYMTYPE),
                           ("seeed-lerobot-linux/config.json", tarfile.REGTYPE)):
            with self.subTest(name=name), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                path = root / "bad.tar.gz"
                with tarfile.open(path, "w:gz") as archive:
                    info = tarfile.TarInfo(name)
                    info.type = kind
                    info.linkname = "/tmp/outside"
                    info.size = 1 if kind == tarfile.REGTYPE else 0
                    archive.addfile(info, io.BytesIO(b"x"))
                sums = root / "SHA256SUMS"
                sums.write_text(hashlib.sha256(path.read_bytes()).hexdigest() + "  " + path.name)
                with self.assertRaisesRegex(ValueError, "不允许"):
                    release.unpack_verified(path, sums, root / "unpack", self.tag)
                self.assertFalse((root / "unpack").exists())

    def fake_download(self, url, target, api=False):
        if target.name == "release.json":
            target.write_text(json.dumps({"tag_name": self.tag, "assets": [
                {"name": self.archive.name, "url": "https://api.github.com/asset/archive"},
                {"name": "SHA256SUMS", "url": "https://api.github.com/asset/checksum"}]}))
        else:
            shutil.copyfile(ROOT / "dist" / target.name, target)

    def test_upgrade_failure_keeps_current_and_shared_data_then_can_retry(self):
        with tempfile.TemporaryDirectory() as temporary:
            prefix = Path(temporary) / "install with spaces"
            old = prefix / "releases/v0.0.0"
            old.mkdir(parents=True)
            (prefix / ".repository").write_text("example/robot\n")
            (prefix / "current").symlink_to(old)
            shared = prefix / "shared"
            (shared / "data").mkdir(parents=True)
            (shared / "config.json").write_text('{"keep": true}')
            (shared / "data/sentinel").write_text("keep dataset")
            args = argparse.Namespace(repo="example/robot", version=self.tag, prefix=prefix,
                                      device="cpu", skip_system=True)
            with patch.object(release, "download", side_effect=self.fake_download), \
                    patch.object(release.os, "geteuid", return_value=1000), \
                    patch.object(release.subprocess, "run", side_effect=subprocess.CalledProcessError(1, "install")):
                with self.assertRaises(subprocess.CalledProcessError):
                    release.install(args)
            self.assertEqual((prefix / "current").resolve(), old)
            self.assertEqual((shared / "config.json").read_text(), '{"keep": true}')
            self.assertEqual((shared / "data/sentinel").read_text(), "keep dataset")
            with patch.object(release, "download", side_effect=self.fake_download), \
                    patch.object(release.os, "geteuid", return_value=1000), \
                    patch.object(release.subprocess, "run"):
                release.install(args)
            self.assertEqual((prefix / "current").resolve(), prefix / "releases" / self.tag)
            self.assertEqual((prefix / "current/config.json").resolve(), shared / "config.json")

    def test_platform_family_plans(self):
        for distro, like, version, expected, group in (
            ("ubuntu", "debian", "24.04", "apt", "dialout"),
            ("debian", "", "13", "apt", "dialout"),
            ("linuxmint", "ubuntu debian", "22.1", "apt", "dialout"),
            ("fedora", "", "44", "dnf", "dialout"),
            ("manjaro", "arch", "", "pacman", "uucp"),
            ("opensuse-leap", "suse", "16", "manual", "dialout")):
            output = subprocess.check_output(["bash", "-c",
                'source "$1"; platform_plan "$2" "$3" "$4"; printf "%s %s" "$SYSTEM_MANAGER" "$SERIAL_GROUP"',
                "test", str(ROOT / "scripts/system_deps.sh"), distro, like, version], text=True)
            self.assertEqual(output, f"{expected} {group}")


if __name__ == "__main__":
    unittest.main()
