# Guide: Using VS Code Devcontainers

If you are new to Docker or ROS 2, development inside a **Devcontainer (Development Container)** is the easiest and most seamless way to work with this stack. 

This guide explains what a Devcontainer is, how to install it, and how to use it for developing, building, and running GUI tools.

---

## 1. What is a VS Code Devcontainer?

A Devcontainer allows you to use a Docker container as a full-featured development environment. 
* Instead of running VS Code on your host machine and trying to compile ROS 2 code natively, VS Code runs its **editor engine directly inside the container**.
* Your project files are mounted into the container in real-time. Any file you edit in VS Code is instantly updated on your host machine.
* Any extensions you install (like C++ debugging or ROS tools) run inside the container, giving you autocompletion and linting for ROS 2 libraries automatically.

---

## 2. Prerequisites & Installation

To use Devcontainers, install the following on your host workstation:

1. **Docker**: Ensure Docker is installed and configured to run without `sudo` (see [Docker Post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/)).
2. **VS Code**: Download and install Visual Studio Code.
3. **Dev Containers Extension**: 
   * Open VS Code.
   * Press `Ctrl + Shift + X` to open the Extensions marketplace.
   * Search for **"Dev Containers"** (published by Microsoft) and click **Install**.

---

## 3. How to Open the Workspace in the Devcontainer

1. **Set up X11 permissions on the host** (needed for RViz2 or Matplotlib GUIs to work):
   Open a terminal on your host machine **inside the workspace project folder** (the `src/` directory) and run:
   ```bash
   source .devcontainer/xauth_setup.sh
   ```
2. **Build the Docker Image on the Host**:
   Before opening the container in VS Code for the first time, you must build the baseline Docker image. In the same terminal (inside the `src/` directory), run:
   ```bash
   env UID=$(id -u) GID=$(id -g) docker compose build nuc
   ```
   *(Note: Prepending `env UID=... GID=...` maps your host user ID and group ID inside the container, preventing file permission conflicts and compile-time user creation crashes).*

3. **Open the project folder**:
   In VS Code, open the folder `/home/mohany/Projects/f1tenth/highlevel/asuf1tenth/src`.
   > [!WARNING]
   > You **must** open the `src` folder directly in VS Code, **not** the parent directory `asuf1tenth`. The Dev Container configuration is designed to mount the opened directory exactly as the workspace source folder. Opening the parent directory will break file mounts, path resolution, and compilation.
4. **Reopen in Container**:
   * A pop-up should appear in the bottom-right corner asking to *"Reopen in Container"*. Click it.
   * If it doesn't appear, press `F1` (or `Ctrl + Shift + P`) to open the VS Code Command Palette, type **"Dev Containers: Reopen in Container"**, and press `Enter`.
5. **Wait for Build**:
   VS Code will start building the Docker image (if not already built) and launch the container. Once completed, you will see a green bar in the bottom-left corner showing: **`Dev Container: ForzaETH Race Stack Jazzy`**.

> [!IMPORTANT]
> **Troubleshooting: Snap Docker & Wayland Socket Error on Ubuntu**
> If your host is running a **Wayland** session and **Docker is installed via Snap**, you may get an error stating: *"An error occurred setting up the container"* because Snap Docker's security sandbox prevents mounting files from `/run/user/...`.
> 
> **To resolve this**:
> 1. Open your VS Code **User Settings** (Press `Ctrl + ,`).
> 2. Search for `Mount Wayland Socket` and **disable** it (this adds `"dev.containers.mountWaylandSocket": false` to your user `settings.json`).
> 3. Click **Retry** on the Devcontainer setup pop-up.
> *Note: Disabling this will NOT break GUI apps, as our container uses standard X11 socket forwarding (`/tmp/.X11-unix`) under XWayland, which Snap Docker is allowed to mount.*
> 
> **Automated Reinstallation Scripts**:
> We have provided utility scripts in your workspace to automate the reinstallation of VS Code and Docker to bypass snap conflicts:
> * **VS Code Reinstaller**: Run [.install_utils/reinstall_vscode_deb.sh](file:///home/mohany/Projects/f1tenth/highlevel/asuf1tenth/src/.install_utils/reinstall_vscode_deb.sh) on the host to swap Snap VS Code for the native `.deb` package.
> * **Docker Engine Reinstaller**: Run [.install_utils/reinstall_docker_apt.sh](file:///home/mohany/Projects/f1tenth/highlevel/asuf1tenth/src/.install_utils/reinstall_docker_apt.sh) on the host to swap Snap Docker for the official APT engine and disable Wayland socket mounts.

---

## 4. Building the Code (Workspace Compilation)

Once inside the container, you compile the code using native VS Code tasks:
1. Press `Ctrl + Shift + B` (or select **Terminal -> Run Build Task...**).
2. Select the build type (e.g., **`Release`**).
3. VS Code will run the compilation script in a terminal panel, automatically bypassing packages that should not be built (like `f110_gym`).

---

## 5. Running GUI Applications (RViz2 / Matplotlib)

GUI applications running from within a Docker container can sometimes be tricky to forward back to your screen. This section explains how to handle GUI forwarding for both local and remote (SSH) setups.

### Scenario A: Working Natively on a Local Desktop Session
If you are running VS Code directly on your development workstation (not over SSH):

1. **On the Host (before opening the container)**:
   Ensure you run the following in your host terminal to allow X11 connections:
   ```bash
   xhost +local:$USER
   source .devcontainer/xauth_setup.sh
   ```
2. **Inside the Devcontainer Terminal**:
   Export your host's display number (typically `:0`):
   ```bash
   export DISPLAY=:0
   ```
3. Run your ROS 2 nodes (e.g., RViz2). The windows will open directly on your screen.

### Scenario B: Working Remotely via SSH (e.g. Developer Laptop connecting to Car NUC)
If your code runs on the car NUC/Pi and you connect to it remotely via VS Code SSH:

1. **SSH with X Forwarding**:
   Open a terminal on your local laptop and connect to the remote machine using the `-X` flag:
   ```bash
   ssh -X <username>@<car_ip>
   ```
2. **On the Remote Host (in the SSH session)**:
   Initialize the Xauth file:
   ```bash
   cd /home/mohany/Projects/f1tenth/highlevel/asuf1tenth/src
   source .devcontainer/xauth_setup.sh
   ```
3. **Capture the Display Number**:
   Run the following inside the SSH session to get the virtual display port allocated by SSH:
   ```bash
   echo $DISPLAY
   # Output will look like: localhost:10.0
   ```
   *Keep this SSH connection running in the background.*
4. **Inside the Devcontainer Terminal**:
   In your VS Code window connected to the remote container, export the captured display value:
   ```bash
   export DISPLAY=localhost:10.0  # Use the exact value from step 3
   ```
5. Run your ROS 2 nodes. The graphics will be forwarded through the SSH tunnel and render on your local laptop screen!
