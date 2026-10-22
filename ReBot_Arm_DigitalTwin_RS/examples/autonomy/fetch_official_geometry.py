"""Download and hash-check only three pinned perception files; no SDK or execution."""
import argparse
import hashlib
from pathlib import Path
import urllib.request

from replay_official_vision import DEFAULT_UPSTREAM, EXPECTED, SHA


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=DEFAULT_UPSTREAM)
    args = parser.parse_args()
    for name, digest in EXPECTED.items():
        url = f'https://raw.githubusercontent.com/Seeed-Projects/reBot-DevArm-Grasp/{SHA}/utils/{name}'
        data = urllib.request.urlopen(url, timeout=25).read()
        if hashlib.sha256(data).hexdigest() != digest:
            parser.exit(1, f'源码哈希不匹配: {name}\n')
        path = args.output / 'utils' / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
        print(f'{name} {digest}')


if __name__ == '__main__':
    main()
