#!/usr/bin/env bash
# ADAPT-X developer task runner (Linux / macOS).
#
# Usage: scripts/dev.sh <task>
#   install | run | test | test-unit | test-integration
#   lint | format | format-check | typecheck | check
#   docker-build | docker-up
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

PYTHON="$ROOT/.venv/bin/python"
if [ ! -x "$PYTHON" ]; then
  echo "No virtual environment found. Creating one at $ROOT/.venv" >&2
  python3 -m venv "$ROOT/.venv"
fi

case "${1:-}" in
  install)          "$PYTHON" -m pip install --upgrade pip && "$PYTHON" -m pip install -e ".[dev]" ;;
  run)              "$PYTHON" -m adaptx ;;
  test)             "$PYTHON" -m pytest ;;
  test-unit)        "$PYTHON" -m pytest tests/unit ;;
  test-integration) "$PYTHON" -m pytest tests/integration ;;
  lint)             "$PYTHON" -m ruff check . ;;
  format)           "$PYTHON" -m ruff format . ;;
  format-check)     "$PYTHON" -m ruff format --check . ;;
  typecheck)        "$PYTHON" -m mypy ;;
  check)
    "$PYTHON" -m ruff check .
    "$PYTHON" -m ruff format --check .
    "$PYTHON" -m mypy
    "$PYTHON" -m pytest
    ;;
  docker-build)     docker compose build ;;
  docker-up)        docker compose up ;;
  *)
    echo "Usage: scripts/dev.sh {install|run|test|test-unit|test-integration|lint|format|format-check|typecheck|check|docker-build|docker-up}" >&2
    exit 2
    ;;
esac
