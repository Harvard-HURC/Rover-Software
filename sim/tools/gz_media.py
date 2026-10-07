#!/usr/bin/env python3
"""Patched gz-rendering media for the sim (design spec D12, section 7):
python sim/tools/gz_media.py [--build-dir DIR] (pixi run sim-media).

Copies the environment's share/gz/gz-rendering8 to <build>/gz-rendering-media,
applies sim/patches/gz-rendering8-ogre2-media.diff and writes the sky, the
haze and the mission sun of urc/lighting.py into it:
- Terra: roughness 1 for a layer without a roughness map (stock: 0, a
  mirror; the dark wavy "puddles" on the terrain were sky reflections, M);
- a procedural clear desert sky (zenith-to-horizon gradient, Mie glow, sun
  disk) instead of the stock cumulus cubemap: gz-sim 8 never applies an SDF
  <sky><cubemap_uri>;
- distance haze for Pbs and Terra: gz-sim 8 never applies an SDF <fog>.
It writes gzenv.MEDIA_COMPLETE last; gzenv sets GZ_RENDERING_RESOURCE_PATH
only then. Soft: when anything fails (an environment upgrade moved what the
diff patches, ...) it warns, leaves no media directory (also none from an
earlier run) and exits 0, so Gazebo runs with its stock media and sim-build
never fails. The GLSL sky was written for Linux and never run (only Metal
was tested).
"""
import argparse
import re
import shutil
import sys
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_DIR))
import gzenv  # noqa: E402
from urc import lighting, textures  # noqa: E402

PATCH = SIM_DIR / "patches" / "gz-rendering8-ogre2-media.diff"
HUNK = re.compile(r"^@@ -(\d+)(?:,(\d+))? \+(\d+)(?:,(\d+))? @@")


class PatchError(Exception):
    pass


def environment_media():
    """share/gz/gz-rendering8 of the Python environment this runs in (pixi's)."""
    return Path(sys.prefix) / "share" / "gz" / "gz-rendering8"


def parse(text):
    """A unified diff: [(path without its first component, [(old_start,
    old_lines, new_lines)])]."""
    lines = text.splitlines()
    files, i = [], 0
    while i < len(lines):
        if not lines[i].startswith("+++ "):
            i += 1
            continue
        path = lines[i][4:].split("\t")[0].strip().split("/", 1)[1]  # -p1
        hunks = []
        i += 1
        while i < len(lines) and lines[i].startswith("@@"):
            m = HUNK.match(lines[i])
            if not m:
                raise PatchError(f"bad hunk header {lines[i]!r}")
            old_count = int(m.group(2) or 1)
            new_count = int(m.group(4) or 1)
            old, new = [], []
            i += 1
            while len(old) < old_count or len(new) < new_count:
                if i >= len(lines):
                    raise PatchError(f"{path}: the diff ends inside a hunk")
                line = lines[i]
                tag, body = (line[:1], line[1:]) if line else (" ", "")
                if tag in " -":
                    old.append(body)
                if tag in " +":
                    new.append(body)
                if tag not in " -+\\":
                    raise PatchError(f"{path}: bad hunk line {line!r}")
                i += 1
            hunks.append((int(m.group(1)), old, new))
        files.append((path, hunks))
    if not files:
        raise PatchError("no files in the diff")
    return files


def apply(text, root):
    """Apply a unified diff to the tree at root, strictly: every hunk's old
    lines must be found as they are (at their line or elsewhere, once), else
    PatchError and nothing is written."""
    out = {}
    for path, hunks in parse(text):
        target = Path(root) / path
        if target.exists():
            lines = target.read_text().splitlines()
        elif all(not old for _, old, _ in hunks):
            lines = []  # a new file
        else:
            raise PatchError(f"{path}: missing")
        shift = 0
        for start, old, new in hunks:
            at = max(start - 1 + shift, 0) if old else len(lines)
            if lines[at:at + len(old)] != old:
                places = [k for k in range(len(lines) - len(old) + 1) if lines[k:k + len(old)] == old]
                if len(places) != 1:
                    raise PatchError(f"{path}: hunk at line {start} does not apply")
                at = places[0]
            lines[at:at + len(old)] = new
            shift += len(new) - len(old)
        out[target] = "\n".join(lines) + "\n"
    for target, body in out.items():
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(body)
    return sorted(out)


def tokens(sun=lighting.MISSION, sky=lighting.SKY):
    """The ${TOKEN} values the diff leaves in the shaders: the sun (towards
    it, world frame), sky and haze colours in linear light."""
    def vec(values):
        return ", ".join(f"{v:.4f}" for v in values)

    def colour(srgb):
        return vec(textures.srgb_to_linear(srgb))

    return {"SUN_DIR": vec(sun.toward), "ZENITH": colour(sky.zenith), "HORIZON": colour(sky.horizon),
            "HORIZON_EXPONENT": f"{sky.horizon_exponent:.2f}", "GROUND_TINT": vec(sky.ground_tint),
            "HAZE_RGB": colour(sky.haze), "HAZE_BETA": f"{sky.haze_beta:.3e}", "HAZE_MAX": f"{sky.haze_max:.3f}"}


def fill(paths, values):
    """Replace ${TOKEN}s in the patched files; PatchError if any is unknown."""
    for path in paths:
        text = path.read_text()
        for key, value in values.items():
            text = text.replace("${" + key + "}", value)
        left = re.findall(r"\$\{\w+\}", text)
        if left:
            raise PatchError(f"{path}: unknown tokens {sorted(set(left))}")
        path.write_text(text)


def build(source, build_dir):
    """Make <build_dir>/gz-rendering-media from the stock media at source;
    returns it. Raises on any failure, having removed what it wrote."""
    target = Path(build_dir) / gzenv.MEDIA_DIR
    staging = target.with_name(target.name + ".partial")
    shutil.rmtree(staging, ignore_errors=True)
    try:
        if not (Path(source) / "ogre2" / "media").is_dir():
            raise PatchError(f"{source} holds no ogre2/media")
        shutil.copytree(source, staging)
        fill(apply(PATCH.read_text(), staging), tokens())
        (staging / gzenv.MEDIA_COMPLETE).write_text(f"patched with {PATCH.name}\n")
        shutil.rmtree(target, ignore_errors=True)
        staging.rename(target)
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return target


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--build-dir", default=str(gzenv.BUILD_DIR), help="default sim/build")
    parser.add_argument("--source", default=str(environment_media()), help="stock media (default: the env's)")
    args = parser.parse_args()
    try:
        target = build(args.source, args.build_dir)
    except (PatchError, OSError) as e:
        shutil.rmtree(Path(args.build_dir) / gzenv.MEDIA_DIR, ignore_errors=True)
        print(f"gz_media: WARNING: no patched media ({e}); Gazebo uses its stock media "
              "(mirror-like terrain, cumulus sky, no haze)", file=sys.stderr)
        return 0
    print(f"gz_media: patched media in {target}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
