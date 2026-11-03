#!/usr/bin/env bash
set -euo pipefail
arm_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
command -v uv >/dev/null || { echo 'Install uv first: https://docs.astral.sh/uv/getting-started/installation/' >&2; exit 1; }
git -C "$arm_root/.." submodule update --init --recursive -- arm/vendor/i2rt
# Keep the CAN/MuJoCo Python 3.11 environment separate from RoboStack Python.
unset PYTHONPATH PYTHONHOME
uv sync --project "$arm_root/vendor/i2rt" --locked --no-dev --python 3.11.17
"$arm_root/vendor/i2rt/.venv/bin/python" -c 'import i2rt, mujoco, can, tyro; print("I2RT environment ready (no hardware opened).")'
