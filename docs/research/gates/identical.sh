#!/usr/bin/env bash
# Are the generated outputs byte-identical between two commits? Generates the rover, its cameras and every
# world in `git archive` trees of both (sim/data shared from this checkout) and diffs sim/models and
# sim/worlds; writes GATES_SCRATCH/results/identical.json. Usage: identical.sh <base commit> [<commit>]
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
repo="$(cd "$here/../../../.." && pwd)"
scratch="${GATES_SCRATCH:-$(python -c 'import tempfile; print(tempfile.gettempdir())')/rover_gates}"
base="$1"
head="${2:-HEAD}"
work="$scratch/identical"
rm -rf "$work"
mkdir -p "$work" "$scratch/results"
for name in base head; do
  commit="$base"
  [[ "$name" == head ]] && commit="$head"
  mkdir -p "$work/$name"
  git -C "$repo" archive "$commit" sim | tar -x -C "$work/$name"
  rm -rf "$work/$name/sim/data" "$work/$name/sim/models/rover" "$work/$name/sim/models/chase_camera" \
    "$work/$name/sim/models/eye_camera"
  ln -s "$repo/sim/data" "$work/$name/sim/data"
  (cd "$work/$name" && python sim/gen_model.py && python sim/gen_worlds.py) > "$work/$name.log" 2>&1
done
status=identical
diff -r --no-dereference "$work/base/sim/models" "$work/head/sim/models" > "$work/diff.txt" 2>&1 || status=different
diff -r --no-dereference "$work/base/sim/worlds" "$work/head/sim/worlds" >> "$work/diff.txt" 2>&1 || status=different
files=$(find "$work/head/sim/models" "$work/head/sim/worlds" -type f | wc -l | tr -d ' ')
mb=$(du -sm "$work/head/sim/models" | cut -f1)
cat > "$scratch/results/identical.json" <<EOF
{"base": "$(git -C "$repo" rev-parse --short "$base")", "head": "$(git -C "$repo" rev-parse --short "$head")",
 "result": "$status", "files_compared": $files, "models_mb": $mb, "differences": $(wc -l < "$work/diff.txt" | tr -d ' ')}
EOF
cat "$scratch/results/identical.json"
rm -rf "$work/base" "$work/head"
