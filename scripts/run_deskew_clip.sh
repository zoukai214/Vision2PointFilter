#!/usr/bin/env bash
set -euo pipefail

if (( $# < 1 || $# > 2 )); then
  echo "usage: $0 <clip_root> [output_dir]" >&2
  exit 2
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN_PATH="${REPO_ROOT}/bin/deskew_clip_export"
CONFIG_PATH="${REPO_ROOT}/config/deskew_clip_export.yaml"

if [[ ! -x "${BIN_PATH}" ]]; then
  echo "[info] binary not found, building first" >&2
  "${REPO_ROOT}/scripts/build.sh"
fi

CLIP_ROOT="$1"
OUTPUT_DIR="${2:-${CLIP_ROOT}/output}"

"${BIN_PATH}" "${CONFIG_PATH}" "${CLIP_ROOT}" "${OUTPUT_DIR}"
