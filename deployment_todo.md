# Deployment & Hardware TODO

This file tracks the required tasks to deploy the baseline stack on the physical vehicle hardware.

## 1. Cross-Compilation for ARM64 (Raspberry Pi 4/5, Jetson Nano)
* **Goal**: Build and compile the ROS 2 Jazzy workspace on a powerful workstation, then deploy the compiled workspace directly onto target ARM64 devices without compiling on the edge.
- `[ ]` Setup Docker Buildx environment for multi-architecture builds.
- `[ ]` Configure cross-compilation container using QEMU to target ARM64 (`aarch64`).
- `[ ]` Test deploying the compiled `/install` folder directly to the Raspberry Pi 4.
- `[ ]` Document the swap partition configuration (min 4GB) for the Pi 4 if local compilation is ever required.

## 2. Vehicle Networking & Static IP configuration
* **Goal**: Configure a stable network interface on the vehicle to enable reliable SSH and multi-machine ROS 2 communication (e.g. running RViz2 on a laptop while the stack runs on the car).
- `[ ]` Assign a static IP address (e.g., `192.168.1.100`) to the vehicle computer.
- `[ ]` Configure `ROS_HOSTNAME` and `ROS_DOMAIN_ID` variables in the vehicle's startup bashrc.
- `[ ]` Setup network check scripts to verify low-latency connection (ping times < 2ms) between the laptop and the car.
