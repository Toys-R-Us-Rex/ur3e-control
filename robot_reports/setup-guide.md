# HES_Duckify — UR3e Setup Guide

## 1. Load the Docker image

```bash
docker load < iscoin-simulator-0.1.0.tar.gz
```

Verify it loaded:

```bash
docker images | grep iscoin
```

## 2. Fix Wayland/Hyprland display forwarding

The simulator uses Gazebo GUI which needs X11. On Hyprland (Wayland), you need XWayland forwarding.

Install xhost:

```bash
sudo pacman -S xorg-xhost
```

Allow Docker to access your display (run this every session, or add to your shell rc):

```bash
xhost +local:docker
```

## 3. Start the simulator

The `docker-compose.yml` is inside `ur3e-simulator/.docker/`, not at the repo root.

```bash
cd ~/Documents/HES_Duckify/ur3e-simulator/.docker
docker compose run --rm --name iscoin_simulator cpu
```

> Use `gpu` instead of `cpu` if you have an NVIDIA GPU with nvidia-container-toolkit.

## 4. Launch Gazebo inside the container

Once inside the container shell:

```bash
ros2 launch iscoin_simulation_gz iscoin_sim_control.launch.py
```

## 5. Send commands to the simulator

Open a second terminal on the host and enter the running container:

```bash
docker exec -it iscoin_simulator /bin/bash
```

Run the demo trajectory:

```bash
ros2 run iscoin_driver demo.py
```

Or a custom trajectory:

```bash
ros2 run iscoin_driver demo.py --ros-args -p traj:=<path-to>/custom_traj.json
```

## 6. Set up the Python control library

```bash
cd ~/Documents/HES_Duckify/ur3e-control
uv sync
source .venv/bin/activate
```

## 7. Open in PyCharm

1. File → Open → `~/Documents/HES_Duckify/ur3e-control`
2. Settings → Project → Python Interpreter → Add Interpreter → Existing → select `.venv/bin/python`
3. Create your scripts inside the project

## Quick reference — stop everything

- `CTRL+C` in the Gazebo terminal to stop the simulator
- `exit` to leave the container
- The `--rm` flag auto-removes the container when it stops
