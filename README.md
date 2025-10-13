

## 🦾 ROS2 Simulator — `my_bot`

For setup help, you can also refer to the **BU Mars Rover Club Discord** (see pinned “ROS2 simulator” thread).

---

### 🚀 Quick Start (after container is already set up)

Run these inside the Docker environment (usually accessible at [http://localhost:6080/](http://localhost:6080/)):

```bash
git pull
colcon build
source install/setup.bash
ros2 launch my_bot rsp.launch.py
```

---

### 🧱 First-Time Container Setup (if starting fresh or new teammate)

If you’re launching a brand-new Docker container, you’ll need to set up environment variables and ensure Gazebo models load correctly.

#### 1️⃣ Launch the container

```bash
docker run -it \
  --name ros2_dev_env \
  --network host \
  --volume ~/Desktop/my_bot:/home/ubuntu/Desktop/my_bot \
  ros2_dev_env:stable
```

> Adjust the volume path if your local `my_bot` folder lives somewhere else.

---

#### 2️⃣ Source ROS2 and workspace

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/my_bot/install/setup.bash
```

#### 3️⃣ Fix Gazebo model and world paths

```bash
export GZ_SIM_RESOURCE_PATH=/usr/share/gz-sim-7/worlds:/usr/share/gz-sim-7/models
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/ubuntu/Desktop/my_bot/install/my_bot/share/my_bot/worlds
```

You can make this automatic by adding those lines to your `~/.bashrc` inside the container.

---

### 🌍 Using the Custom World File

Gazebo images in Docker don’t include the default `sun` and `ground_plane` models.
To avoid missing-URI errors, we use a **self-contained `empty.sdf`** that defines its own light and ground plane.

✅ You already have this pushed to:

```
my_bot/install/my_bot/share/my_bot/worlds/empty.sdf
```

If you ever see errors like:

```
Error Code 13: Unable to find uri[model://sun]
Error Code 13: Unable to find uri[model://ground_plane]
```

then make sure that `empty.sdf` file is being referenced in your launch configuration.

---

### 🧹 Saving Your Container Progress

After installing or editing anything inside Docker, **commit** your container so you don’t lose your setup:

```bash
docker ps -a        # find container ID
docker commit <container_id> ros2_dev_env:stable
```

Next time, start directly with:

```bash
docker run -it --name ros2_dev_env --network host ros2_dev_env:stable
```

This ensures your workspace, packages, and environment variables persist.

---

### ✅ Debug Reference

| Issue                    | Cause                        | Fix                            |
| ------------------------ | ---------------------------- | ------------------------------ |
| `model://sun not found`  | Default models missing       | Use self-contained `empty.sdf` |
| `ign: command not found` | Ignition tools not installed | Install `ros-humble-ros-gz`    |
| Gazebo world loads black | No `<light>` defined         | Add `<light>` to `empty.sdf`   |
| Workspace not recognized | Not sourced                  | `source install/setup.bash`    |
| Container reset          | Changes not saved            | `docker commit` your container |

---

### 🧩 Optional: make environment auto-load

Edit `~/.bashrc` inside your container and append:

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/my_bot/install/setup.bash
export GZ_SIM_RESOURCE_PATH=/usr/share/gz-sim-7/worlds:/usr/share/gz-sim-7/models
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/ubuntu/Desktop/my_bot/install/my_bot/share/my_bot/worlds
```

Then run:

```bash
source ~/.bashrc
```

Now every new terminal inside Docker will be ready to launch immediately.

---
