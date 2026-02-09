#!/usr/bin/env bash
ROOT_DIR="$(cd "$(dirname "$0")/.."; pwd)"
PYTHONPATH="$ROOT_DIR/src" python -m navisar.main
