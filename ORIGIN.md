# Source provenance

Parkour inference is additionally adapted from the local checkout of
`https://github.com/wangsy1999/unitree_mujoco`: `example/cpp/g1_parkour_policy.h`,
`example/parkour/g1.json`, and `exported/g1_parkour/`, at commit
`7c6642f89e25f72dd7de3cf7683f166bce943fef`.
The original BSD project license is retained. Network/config details and SHA256
values are recorded in [docs/PARKOUR.md](docs/PARKOUR.md).
No MuJoCo simulator, XML or mesh assets are copied into this repository.

Extracted from the working tree of `wangsy1999/unitree_mujoco`, based on commit `7c6642f89e25f72dd7de3cf7683f166bce943fef`.
The source includes subsequent local DreamWaQ hardware deployment changes.
This is an independent source tree; no submodule, symlink or parent repository is required.
The original repository LICENSE is retained. Unitree SDK2 and ONNX Runtime are external dependencies.

The web monitor was synchronized from the tested robot deployment on 2026-10-19.
It originates from `simulate_python/joint_monitor.py` and
`simulate_python/web/joint_monitor.html` in the same `unitree_mujoco` commit above,
with the onboard domain-0, dry-run telemetry and Chinese UI changes retained.
In this repository it lives in `web/joint_monitor.py` and
`web/static/joint_monitor.html`; only the static-resource path was adjusted.
`scripts/monitor.sh` replaces the robot-specific absolute-path launcher with a
relocatable source/install launcher. Python SDK and CycloneDDS are external dependencies.

Model: `models/policy.onnx`
SHA256: `4f23975849ef66d73a170ee3435a274a4c42dd31830f8b07161add84846a871d`

Configuration: `config/g1_locomotion.yaml`
Original SHA256: `09f7c133dc5e043aed7572d545e51f12dadf59cd11bcafa338e452c890db0c59`
Current SHA256: `98841ae0ffb5ae209a0b9fbb70119025b598a828d08e163130d87c277cd29c48`
The current configuration removes the unused `hardware.joint_effort` field after
removing the software estimated-PD-torque stop; policy parameters are unchanged.
