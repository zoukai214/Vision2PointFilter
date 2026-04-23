#!/usr/bin/env python3
import argparse
import pathlib
import subprocess
import sys


def run(cmd):
    print("[run]", " ".join(str(x) for x in cmd), flush=True)
    subprocess.run(cmd, check=True)


def main():
    parser = argparse.ArgumentParser(
        description="Build and run the deskew_clip_export tool."
    )
    parser.add_argument("clip_root", nargs="?", help="Clip root directory")
    parser.add_argument("output_dir", nargs="?", help="Optional output directory")
    parser.add_argument(
        "--build-only",
        action="store_true",
        help="Configure and build the project, then exit",
    )
    args = parser.parse_args()

    repo_root = pathlib.Path(__file__).resolve().parents[1]
    build_dir = repo_root / "build"
    bin_path = repo_root / "bin" / "deskew_clip_export"
    config_path = repo_root / "config" / "deskew_clip_export.yaml"

    run(
        [
            "cmake",
            "-S",
            str(repo_root),
            "-B",
            str(build_dir),
            "-DCMAKE_BUILD_TYPE=Release",
        ]
    )
    run(
        [
            "cmake",
            "--build",
            str(build_dir),
            "--target",
            "deskew_clip_export",
            "-j4",
        ]
    )

    if args.build_only:
        return 0

    if not args.clip_root:
        raise SystemExit("clip_root is required unless --build-only is used")

    clip_root = pathlib.Path(args.clip_root).resolve()
    output_dir = (
        pathlib.Path(args.output_dir).resolve()
        if args.output_dir
        else clip_root / "output"
    )

    run([str(bin_path), str(config_path), str(clip_root), str(output_dir)])
    return 0


if __name__ == "__main__":
    sys.exit(main())
