# Jetson Nano & ARM64 Docker Guide

This guide documents the problems, findings, workarounds, and recommendations for running the ForzaETH F1TENTH race stack on the NVIDIA Jetson Nano Developer Kit (running Ubuntu 18.04 / JetPack 4.6).

---

## 1. Summary of Build Troubleshooting

When setting up the ROS 2 Jazzy (Ubuntu 24.04) race stack on the Jetson Nano, we resolved several legacy host issues:

### A. Docker Compose Command Compatibility
*   **Problem:** `docker compose` is not recognized.
*   **Finding:** JetPack 4.6 is built on Ubuntu 18.04, which has Docker Compose V1 (`docker-compose` with a hyphen).
*   **Resolution:** Prepend `version: '3.3'` to the top of the [docker-compose.yaml](file:///home/mohany/Projects/f1tenth/highlevel/asuf1tenth/src/docker-compose.yaml) file to prevent syntax failures under Compose V1.

### B. Base Image Architecture Mismatch
*   **Problem:** Building failed with `exec format error` at the first instruction (`RUN userdel ubuntu`).
*   **Finding:** The original base image (`osrf/ros:jazzy-desktop`) is built only for `amd64` (x86_64) on Docker Hub. Docker fell back to pulling the x86 layers, which cannot run on the Jetson's `aarch64` CPU.
*   **Resolution:** Changed the base image in [.devcontainer/Dockerfile](file:///home/mohany/Projects/f1tenth/highlevel/asuf1tenth/src/.devcontainer/Dockerfile) to **`ros:jazzy-ros-base`** (which has official ARM64 support). The desktop components are installed natively during the build phase via:
    ```dockerfile
    RUN apt-get install -y ros-jazzy-desktop
    ```

---

## 2. GPU Acceleration Configuration (For Future Reference)

Currently, the container runs using **CPU Software Rendering (`llvmpipe`)**. This is the safest default because the host operating system's (Ubuntu 18.04) NVIDIA graphics drivers are binary-incompatible with the container's (Ubuntu 24.04) newer system libraries. Forcing driver mounting causes the container's local graphics to crash with `Error: couldn't find RGB GLX visual`.

### How to Enable GPU Acceleration in the Future
If the host operating system is upgraded to a newer version (like a community Ubuntu 20.04/22.04 image or JetPack 6.x on a newer Jetson board), you can enable hardware GPU acceleration by modifying [.docker_utils/main_dock.sh](file:///home/mohany/Projects/f1tenth/highlevel/asuf1tenth/src/.docker_utils/main_dock.sh):

1.  **Modify the script to dynamically detect the NVIDIA runtime and inject environment variables:**
    ```bash
    # Detect if NVIDIA container runtime is available on the host
    GPU_FLAGS=""
    if docker info 2>/dev/null | grep -q -E "Runtimes:.*nvidia"; then
        GPU_FLAGS="--runtime nvidia --env NVIDIA_VISIBLE_DEVICES=all --env NVIDIA_DRIVER_CAPABILITIES=all"
    fi
    ```

2.  **Pass these flags to the `docker run` command:**
    ```bash
    docker run --tty \
        --interactive \
        $GPU_FLAGS \
        --network=host \
        ...
    ```

---

## 3. Recommendation: Distributed Container Architecture

Since the core ForzaETH navigation stack (MPC, planners, state estimation) runs efficiently on the CPU, you do not need the GPU for driving. However, if you want to implement GPU-heavy compute nodes (like YOLO for cameras or TensorRT for deep learning inference), you should use a **Distributed Container Architecture** rather than attempting to port the main stack to older ROS versions.

```mermaid
graph TD
    subgraph Jetson Host (JetPack 4.6 / Ubuntu 18.04)
        A[DDS Shared Network / Loopback]
        
        subgraph Container 1: AI Perception (GPU)
            B[ROS 2 Foxy Container] -->|NVIDIA Runtime| G[Jetson GPU]
            B -->|Publishes: /camera/detected_objects| A
        end
        
        subgraph Container 2: Main Stack (CPU)
            C[ROS 2 Jazzy Container] -->|CPU Core| H[Sensors/LiDAR/Actuators]
            A -->|Subscribes| C
            C -->|Runs| D[MPC Tracker]
            C -->|Runs| E[Particle Filter]
        end
    end
```

### How It Works:
1.  **AI Perception Container (GPU-Enabled):** Run a container matching the host's version (e.g. Ubuntu 18.04 / ROS 2 Foxy base image) utilizing `--runtime nvidia`. This container runs your deep learning models natively using CUDA and TensorRT.
2.  **Navigation Stack Container (CPU-only):** Run your main ROS 2 Jazzy stack inside a CPU-only container. It handles planning, tracking, and LiDAR sensor processing without any driver conflicts.
3.  **Communication:** The nodes in the different containers publish and subscribe to topics natively over the network.

---

## 4. Caveats & Requirements for Success

To get this distributed container architecture working successfully, keep the following constraints in mind:

### A. DDS and Network Discovery
*   **Network Host Mode:** Both containers must be started with `--net=host` (as configured in `main_dock.sh`) so they share the host's loopback interface.
*   **Domain ID Matching:** Ensure both containers export the same `ROS_DOMAIN_ID` (e.g., `export ROS_DOMAIN_ID=6`).
*   **DDS Implementation:** Ensure both distros are using compatible DDS middleware. FastDDS version mismatches between ROS 2 distros can cause discovery failures. Forcing both containers to use **Eclipse CycloneDDS** is highly recommended:
    ```bash
    sudo apt-get install -y ros-<distro>-rmw-cyclonedds-cpp
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    ```

### B. Message Definition Schema Conflicts
*   **Standard Messages:** Standard messages like `sensor_msgs/Image`, `sensor_msgs/LaserScan`, or `geometry_msgs/PoseStamped` are highly stable and cross-distro compatible.
*   **Custom Messages:** If you are using custom message structures specific to the ForzaETH stack, Foxy will not know how to deserialize them. Keep your interfaces restricted to standard ROS 2 message definitions when bridging data between the GPU and CPU containers.

### C. Resource Constraints (RAM)
*   The Jetson Nano Developer Kit has only **4GB of shared RAM** for both the CPU and the GPU. 
*   Running two separate Docker containers, two ROS 2 middleware stacks, and a deep learning model will easily push the RAM to its limits.
*   **Mitigation:** 
    *   Disable unnecessary GUI services on the Jetson Nano host (`sudo systemctl set-default multi-user.target`).
    *   Create a Swap partition of at least 4GB to prevent Out-Of-Memory (OOM) kernel crashes.
