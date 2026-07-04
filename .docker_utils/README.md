# Docker Structure and Guidelines
**Note:** Currently only installation via docker has been tested and is supported.

## Clone Repo
Be aware to clone the ROS 2 branch!
```bash
git clone -b ros2-jazzy --recurse-submodules git@github.com:ForzaETH/race_stack.git 
cd race_stack
```

## Structure
The docker image is defined with the [Dockerfile](../.devcontainer/Dockerfile). A single unified image is built to support both simulation and physical hardware deployment.


**NOTE**: it is suggested to set a static IP for the robot with the ROS_HOSTNAME environment variable, so that the IP of the robot is always the same, as from [networking structure](../stack_master/checklists/networking.md).
Setting up a static ip at this moment will allow you to set extra computers (that can ping such IP) to listen to the ROS messags on the robot, enabling fast and quick development. 


## How to use (docker container)
**Note**: this following tutorial assumes you are using an x86 platform (e.g. Intel CPU). TODO: arm instructions

**Note 2**: docker is assumed to be installed and runnable without sudo (e.g. on Linux see [Post-Installation steps](https://docs.docker.com/engine/install/linux-postinstall/)).

**Step 1/5: Build the container**
Build the docker image with `docker compose` by prepending your user and group IDs inline (this avoids shell-specific read-only variable assignment errors):
```bash
env UID=$(id -u) GID=$(id -g) docker compose build nuc
```
*(Note: Prepending `env UID=$(id -u) GID=$(id -g)` is mandatory. If omitted, docker compose defaults to blank strings for UID and GID, which triggers syntax errors and user-creation crashes during the Dockerfile build process).*

**Step 2/5: create the folder structure for caching colcon builds**
Create a folder structure that resembles the following. Note that it is a folder up from the position of the `race_stack`.

```bash
<race_stack directory>/../
...
├── cache
│   └── jazzy
│       ├── build
│       ├── install
│       └── log
└ ...
```

It can be done with the following command:
```bash
cd <race_stack folder>
mkdir -p ../cache/jazzy/build ../cache/jazzy/install ../cache/jazzy/log
```

**Step 3/5: Set up the launch script**
The launch script [`main_dock.sh`](./main_dock.sh) is configured to dynamically resolve its path using `BASH_SOURCE`. No manual modification of `FORZAETH_DIR` is required.

**Step 4/5: Set up X forwarding and launch the container**
To get GUI applications (like RViz2 and Matplotlib) properly forwarded:

1. If your host is running a **Wayland** session (default for many modern Ubuntu installations), run the following on the host to allow the container connection to your desktop:
   ```bash
   xhost +local:$USER
   ```

2. Run the setup script to establish the container `.Xauthority` configuration:
   ```bash
   cd <race_stack folder>
   source .devcontainer/xauth_setup.sh
   ```

3. Launch the docker container in the same terminal:
   ```bash
   ./.docker_utils/main_dock.sh
   ```

You will now have access to a terminal inside the container. 
Here, run the postcreate command with the following line
```bash
cd ~/ws/src/race_stack
./.install_utils/post_create_command.sh
```
This will setup the final packages and configurations needed for the container to work correctly.

Once the setup is done, compile the workspace using one of these options:
* **Option A: VS Code Devcontainer Task (Recommended)**: Press `Ctrl + Shift + B` (or select `Terminal -> Run Build Task...`) and choose your build profile (e.g., `Release`). This utilizes the pre-configured script which automatically skips the `f110_gym` python package.
* **Option B: Manual Terminal Build**: If building inside the container terminal, run:
  ```bash
  cd ~/ws
  colcon build --symlink-install --packages-ignore f110_gym
  ```
  *(Note: We must pass `--packages-ignore f110_gym` because the simulator python package is already installed via pip in editable mode during the setup phase, and its custom layout is incompatible with colcon's `--symlink-install` path resolution).*

**Step 5/5: Open additional terminals, reopen a closed terminal**
You can now attach multiple terminals to the container with the secondary scritp:
```bash
# in a terminal outside of the container
cd <race_stack folder>
./.docker_utils/sec_dock.sh
```

Once the main docker container is closed, you can also reopen the same one with the attach script:
```bash
# in a terminal outside of the container
cd <race_stack folder>
./.docker_utils/main_attach_dock.sh
```


## How to use (VSCode devcontainer)

> [!NOTE]
> For a beginner-friendly setup guide explaining what VS Code Devcontainers are and how to install and use them, refer to our [VS Code Devcontainer Guide](../devcontainer_guide.md).

**Note**: this following tutorial assumes you are using an x86 platform (e.g. Intel CPU). TODO: arm instructions

**Note 2**: docker is assumed to be installed and runnable without sudo (e.g. on Linux see [Post-Installation steps](https://docs.docker.com/engine/install/linux-postinstall/)).

**Step 1/5: setup the X forwarding** 

In case you want to use the VSCode devcontainer in a remote machine, and you want graphical application to be forwarded, you need to setup the xauth file for the container. This can be done by running the following command:
```bash
cd <race_stack folder>
source .devcontainer/xauth_setup.sh
```

Make sure to do this in a remote terminal with X forwarding enabled, as described in the [GUI applications documentation](./README_GUI.md).

Then print out the `DISPLAY` variable number with the following command, in order to remember it for later use:
```bash
echo $DISPLAY
```
an example output could be 
```
localhost:10.0
```
**Note**: this container must then be left running.

**Step 2/5: create the folder structure for caching colcon builds**

If not present, further create a folder structure that resembles the following. Note that it is a folder up from the position of the `race_stack`.

```bash
<race_stack directory>/../
...
├── cache
│   └── jazzy
│       ├── build
│       ├── install
│       └── log
└ ...
```

It can be done with the following command:
```bash
cd <race_stack folder>
mkdir -p ../cache/jazzy/build ../cache/jazzy/install ../cache/jazzy/log
```

**Step 3/5: Build the container**

In a terminal connected to the remote machine you want to use, move to the location of the racestack, and build the docker container with the compose command (prepending user/group IDs is mandatory to prevent compilation crashes during user creation):
```bash
cd <race_stack_directory>
env UID=$(id -u) GID=$(id -g) docker compose build nuc
```

Change the `image` attribute in the devcontainer file correspondingly:
```json5
//<race_stack_directory>/.devcontainer/devcontainer.json
...
    "image": "nuc_forzaeth_racestack_ros2:jazzy",
...
```

**Step 4/5: Open the devcontainer**

**Note** this step must be done strictly after the completion of step 1, as otherwise the permission file mounted in the devcontainer might be wrong.

Open the devcontainer on the car, first by opening up VSCode, then connecting to the car with the remote connection button to the bottom left (Connect to Host...), then open the race stack folder, and reopen in the devcontainer.
Once in the devcontainer, open a terminal and export the `DISPLAY` variable number. For example:
```bash
export DISPLAY=localhost:10.0
```
Note: the full <name>:<port> couple is needed as from step 1.


You can now enjoy a terminal with GUI forwarding! If you need multiple GUI applications, make sure to export the `DISPLAY` variable in each terminal you want to use GUI applications in.

**Step 5/5: Open additional terminals** 
You can also attach multiple terminals to the container with the secondary script, from outside VSCode:
```bash
# in a terminal outside of the container
cd <race_stack folder>
./.docker_utils/sec_dock.sh
```
The name used inside the `sec_dock.sh` file must be the same as the one set in the `image` field of the `devcontainer.json` in step 3.

## How to use GUI applications with the container
To have more information on how to use GUI applications with remote containers, please refer to the [GUI applications documentation](./README_GUI.md).

---
[Go back to the main README](../README.md)

### To manually install dependencies (should not be necessary if you build in docker):
```bash
# general ubuntu packages dependencies
xargs sudo apt-get install -y < ~/ws/src/race_stack/.install_utils/linux_req/linux_req.txt

# ubuntu packages dependencies for sim
xargs sudo apt-get install -y < ~/ws/src/race_stack/.install_utils/linux_req/linux_req_sim.txt

# ubuntu packages dependencies for car
xargs sudo apt-get install -y < ~/ws/src/race_stack/.install_utils/linux_req/linux_req_car.txt

# python dependencies
pip install -r ~/ws/src/race_stack/.install_utils/python_req.txt --break-system-packages

# setup f1tenth_gym
source ~/ws/src/race_stack/.install_utils/f110_sim_setup.sh
```
