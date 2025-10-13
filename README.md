

## 🦾 ROS2 Simulator — `my_bot`

For setup help, you can also refer to the **BU Mars Rover Club Discord** (see pinned “ROS2 simulator” thread).

---

### 🚀 Quick Start (after container is already set up)

Run these inside the Docker environment (usually accessible at [http://localhost:6080/](http://localhost:6080/)):

```bash
git pull
colcon build --symlink-install
source install/setup.bash
ros2 launch my_bot launch_sim.launch.py
```
then run the following in a seperate window
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
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



### 🐋 Docker Usage & Persistence Guide

Your ROS 2 simulator runs inside a Docker container to ensure everyone has the same dependencies and setup.
Here’s how to **work safely, save progress, and resume later** without creating unnecessary new images.

---

#### 💾 1️⃣ Saving Progress (When You’ve Made Changes)

If you’ve installed packages, edited environment variables, or modified files *inside* Docker (not just your mounted `my_bot` folder),
save those changes back into your base image so you can reopen the same setup later:

```bash
docker commit ros2_dev_env ros2_dev_env:stable
```

✅ This updates the existing `ros2_dev_env:stable` image **in place** — it does not duplicate another 10 GB image layer.

You can confirm your save worked with:

```bash
docker images
```

If the **“CREATED”** timestamp updates, your checkpoint is saved.

---

#### 🧱 2️⃣ Resuming Work Later

When you exit or stop your container, your setup is *not deleted* — it’s paused.
You can reopen it instantly with:

```bash
docker start -ai ros2_dev_env
```

If the container was removed or you need a clean slate, recreate it with:

```bash
docker run -it \
  --name ros2_dev_env \
  --network host \
  -v ~/Desktop/my_bot:/home/ubuntu/Desktop/my_bot \
  ros2_dev_env:stable
```

> The `-v` flag ensures your local project files stay synced on your host system (`~/Desktop/my_bot`).
> Anything you edit in that folder will persist even if you delete or rebuild the container.

---

#### 🧰 3️⃣ Common Workflow Example

Typical daily flow:

```bash
# On host (Mac or Linux)
docker start -ai ros2_dev_env

# Inside container
cd ~/Desktop/my_bot
git pull
colcon build
source install/setup.bash
ros2 launch my_bot rsp.launch.py
```

If you installed new dependencies (e.g., `apt install` inside Docker):

```bash
exit   # leave container
docker commit ros2_dev_env ros2_dev_env:stable
```

Then, next session, you can just run `docker start -ai ros2_dev_env` again — all changes preserved.

---

#### 🧼 4️⃣ Cleaning Up Space

To remove stopped containers or old dangling images safely:

```bash
docker container prune
docker image prune -a
```

> ⚠️ Make sure your latest `ros2_dev_env:stable` image appears in `docker images` before pruning.

---

#### ✅ Quick Reference

| Action           | Command                                                                                                                 | Notes                     |
| ---------------- | ----------------------------------------------------------------------------------------------------------------------- | ------------------------- |
| Launch container | `docker run -it --name ros2_dev_env --network host -v ~/Desktop/my_bot:/home/ubuntu/Desktop/my_bot ros2_dev_env:stable` | First time setup          |
| Resume container | `docker start -ai ros2_dev_env`                                                                                         | Continue existing session |
| Save changes     | `docker commit ros2_dev_env ros2_dev_env:stable`                                                                        | Save environment updates  |
| List images      | `docker images`                                                                                                         | Check save time           |
| Free space       | `docker system prune -a`                                                                                                | Clean up old layers       |


---

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
