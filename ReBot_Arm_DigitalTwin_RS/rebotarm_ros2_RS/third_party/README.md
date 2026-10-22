# Control SDK sources

The legacy SDK below is ordinary source tracked by the parent repository.
The newer physical CSP path uses a separate pinned checkout. They are distinct
dependencies; do not substitute one for the other.

| Directory | Upstream | Baseline revision |
|---|---|---|
| `reBotArm_control_py` | `Yang-Ci/reBotArm_control_py` | `5ba28acef46237eb6a7560658bbc43b06cf8a259` |
| `reBotArm_control_py_csp` | `Seeed-Projects/reBotArm_control_py` | `6415d43130d1e143c70dc106096a857ac5556f81` |
The SDK includes the RS Cartesian trajectory duration safety adjustment. The
MuJoCo model derived from `LAN-GER/reBot-B601-RS-for-mujoco_sim` revision
`1249cb6efdf393ba636056fc41df30dc6ba389aa` is integrated directly under
`rebotarm_ros2_RS/src/rebotarm_mujoco_rs/models`; its algorithm package is not used.
These integrated files are the single source of truth; no patch or override copy
is required.

The CSP directory is ignored by the parent repository. Recreate it from the
repository root with `bash scripts/setup_csp_sdk.sh`; inspect it read-only with
`bash scripts/setup_csp_sdk.sh --check`. The canonical URL, path and full commit
are in `configs/csp-sdk-lock.json`. Offline mirror origins are allowed when the
commit and clean source tree match. Existing changes are never overwritten.
Both standalone `.git` directories and submodule `.git` files are supported;
other nested repositories are rejected. Keep the CSP Git metadata: the physical
loader verifies its commit. Do not convert the live dependency during a session.

Update legacy vendored files through the parent repository. A normal clone alone
does not contain the CSP checkout; run its preparation step before doctor/build.
