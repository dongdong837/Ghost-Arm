#!/usr/bin/env bash
set -e
GHOST_PACKAGE_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$GHOST_PACKAGE_ROOT/dist"
# Explicit allowlist excludes credentials, logs, screenshots and runtime output.
tar --exclude='__pycache__' --exclude='*.pyc' --exclude='.env' --exclude='.env.*' \
  --exclude='*.key' --exclude='*.pem' \
  -czf "$GHOST_PACKAGE_ROOT/dist/ghost-arm-source.tar.gz" \
  --transform='s,^,ghost-arm/,' -C "$GHOST_PACKAGE_ROOT" \
  README.md .gitignore ghost.sh src assets config launch scripts tests docs
printf '源码包：%s\n' "$GHOST_PACKAGE_ROOT/dist/ghost-arm-source.tar.gz"
