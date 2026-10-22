# Third-party sources

- Tutorial: https://wiki.seeedstudio.com/cn/rebot_arm_b601_rs_lerobot/
- Bundled LeRobot wheel: built without source changes from https://github.com/Seeed-Projects/lerobot/tree/fe078af31247a6144200abbfa3c2ce29d0062033 .
- Upstream LeRobot: https://github.com/huggingface/lerobot . Copyright 2024 The HuggingFace Inc. team and other contributors as recorded in individual source files. Licensed under Apache-2.0. The bundled wheel contains the original license; a copy is also provided as `vendor/LICENSE.lerobot`.
- Seeed robot and teleoperator plugins are downloaded from PyPI at the exact versions recorded in the requirements locks. Their package metadata and source files retain their respective notices.
- uv 0.13.1 is downloaded from its PyPI wheel and verified against the fixed SHA256 in `scripts/bootstrap_uv.py`. Project: https://github.com/astral-sh/uv . The uv distribution is licensed under MIT or Apache-2.0.
- PyTorch, torchvision, PyAV, FFmpeg and other dependencies retain their own licenses. This archive does not bundle their installed environments or pretrained weights. Runtime dependencies and hashes are listed in `requirements-cpu.lock` / `requirements-cuda.lock`.

The deployment wrapper is separate from the upstream projects. Source revision and wheel checksum are recorded in `vendor/ORIGIN.json`.
