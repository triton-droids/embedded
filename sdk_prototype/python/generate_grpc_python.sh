#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$ROOT/python/robot_sdk_demo"
PYTHON_BIN="${PYTHON:-python3}"

"$PYTHON_BIN" -m grpc_tools.protoc \
  -I "$ROOT/proto" \
  --python_out="$OUT" \
  --grpc_python_out="$OUT" \
  "$ROOT/proto/robot_sdk.proto"

"$PYTHON_BIN" - "$OUT" <<'PY'
from pathlib import Path
import sys

target = Path(sys.argv[1]) / "robot_sdk_pb2_grpc.py"
text = target.read_text()
text = text.replace("import robot_sdk_pb2 as robot__sdk__pb2", "from . import robot_sdk_pb2 as robot__sdk__pb2")
target.write_text(text)
PY
