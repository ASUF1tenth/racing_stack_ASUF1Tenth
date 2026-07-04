#!/bin/bash
# Script to reinstall VS Code as a native Debian package (.deb) instead of Snap.
# This avoids Snap sandboxing conflicts with Docker and GUI forwarding.

set -e

echo "=== Removing VS Code Snap ==="
if snap list | grep -q "^code "; then
    sudo snap remove code
    echo "Snap VS Code successfully removed."
else
    echo "VS Code Snap is not installed. Skipping removal."
fi

echo "=== Downloading Official VS Code Debian Package ==="
TEMP_DEB="$(mktemp /tmp/vscode_XXXXXX.deb)"
wget -O "$TEMP_DEB" "https://code.visualstudio.com/sha/download?build=stable&os=linux-deb-x64"

echo "=== Installing VS Code .deb ==="
sudo apt-get update
sudo apt-get install -y "$TEMP_DEB"

echo "=== Cleaning Up ==="
rm -f "$TEMP_DEB"

echo "=== VS Code (.deb) Reinstallation Complete ==="
