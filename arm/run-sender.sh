#!/usr/bin/env bash
set -euo pipefail
arm_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
sender_python="$arm_root/vendor/i2rt/.venv/bin/python"
if [[ ! -x "$sender_python" ]]; then
    echo 'Run pixi run --locked yam-setup first.' >&2
    exit 1
fi
unset PYTHONPATH PYTHONHOME
exec "$sender_python" -u "$arm_root/yam_state_sender.py" "$@"
