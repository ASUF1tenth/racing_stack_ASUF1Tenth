#!/bin/bash
# Script to reinstall Docker Engine via APT instead of Snap.
# This resolves AppArmor/sandbox limitations on X11 and Wayland socket bind mounts.

set -e

echo "=== Removing Snap Docker ==="
if snap list | grep -q "^docker "; then
    sudo snap remove --purge docker
    echo "Snap Docker successfully purged."
else
    echo "Snap Docker is not installed. Skipping removal."
fi

echo "=== Setting Up Docker APT Repository ==="
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg

sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

echo "=== Installing Official Docker Engine ==="
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

echo "=== Configuring User Permissions ==="
# Add current user to docker group
if ! getent group docker > /dev/null; then
    sudo groupadd docker
fi
sudo usermod -aG docker "$USER"

echo "=== Configuring VS Code Mount settings.json (for Wayland/Snap issues) ==="
SETTINGS_FILE="$HOME/.config/Code/User/settings.json"
if [ -f "$SETTINGS_FILE" ]; then
    if ! grep -q "dev.containers.mountWaylandSocket" "$SETTINGS_FILE"; then
        # Insert settings flag before the closing brace
        sed -i 's/\}/,\n    "dev.containers.mountWaylandSocket": false\n\}/' "$SETTINGS_FILE"
        echo "Disabled Wayland socket mounts in VS Code User settings."
    fi
else
    # Create new settings file if it doesn't exist
    mkdir -p "$(dirname "$SETTINGS_FILE")"
    cat <<EOF > "$SETTINGS_FILE"
{
    "dev.containers.mountWaylandSocket": false
}
EOF
    echo "Created VS Code User settings.json with Wayland socket mounts disabled."
fi

echo "=========================================================="
echo " Docker (APT) Reinstallation Complete!"
echo " IMPORTANT: You MUST log out and log back in (or reboot)"
echo " for group permissions to take effect!"
echo "=========================================================="
