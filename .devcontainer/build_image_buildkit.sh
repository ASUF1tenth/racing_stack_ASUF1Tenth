#!/bin/bash

# Find the repository root dynamically (parent of .devcontainer)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$REPO_ROOT"

# Force BuildKit to be used
export DOCKER_BUILDKIT=1

# Resolve actual user ID and group ID (even if run with sudo)
ACTUAL_USER="${SUDO_USER:-$USER}"
ACTUAL_UID="${SUDO_UID:-$(id -u)}"
ACTUAL_GID="${SUDO_GID:-$(id -g)}"

# Build the docker image for arm64
docker build --platform linux/arm64 \
    --build-arg USERNAME="$ACTUAL_USER" \
    --build-arg UID="$ACTUAL_UID" \
    --build-arg GID="$ACTUAL_GID" \
    -t nuc_forzaeth_racestack_ros2:jazzy \
    -f .devcontainer/Dockerfile .