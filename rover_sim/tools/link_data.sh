#!/usr/bin/env bash
# Link the large, git-ignored data files (DEMs, imagery, prototype textures)
# from the main checkout into a git worktree: sim/tools/link_data.sh [main checkout]
# The main checkout defaults to the first entry of `git worktree list`.
set -euo pipefail
here="$(git rev-parse --show-toplevel)"
main="${1:-$(git -C "$here" worktree list --porcelain | awk '/^worktree /{print $2; exit}')}"
[[ "$main" == "$here" ]] && { echo "link_data.sh: this is the main checkout; nothing to link"; exit 0; }
cd "$main"
git ls-files --others --ignored --exclude-standard -- sim/data | while read -r f; do
  if [[ ! -e "$here/$f" ]]; then
    mkdir -p "$here/$(dirname "$f")"
    ln -s "$main/$f" "$here/$f"
  fi
done
echo "link_data.sh: linked ignored data from $main"
