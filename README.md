# SAMI ROS 2 Simulation Workspace (`sami_ws`)

This workspace allows you to simulate the **SAMI** robot in RViz and run joint animations using pre-recorded JSON behavior files.

---

## 0. Initial Setup (Fresh VM)

If you're using a clean VM with ROS 2 Jazzy, start by installing dependencies and cloning the workspace:

```bash
git clone https://github.com/jlruballos/sami_ws.git
sudo apt install ros-jazzy-joint-state-publisher-gui
sudo apt install ros-jazzy-xacro
```

Then:

```bash
cd ~/sami_ws
```

---

## 1. Build the Workspace

In your terminal:

```bash
colcon build
```

Then, source the workspace:

```bash
source install/setup.bash
```

---

## 2. Launch the SAMI Robot in RViz

To visualize the robot and automatically play a behavior animation:

```bash
ros2 launch sami_sim display.launch.py
```

You should see **SAMI** loaded in RViz with a joint control panel on the left.

---

## 3. Run the Behavior Playback

> **Important:** Before running the script, **close** the Joint State Publisher GUI to avoid conflicts.

Open a **new terminal**, then:

```bash
cd ~/sami_ws
source install/setup.bash
ros2 run sami_sim json_joint_publisher.py
```

This will play the behavior file specified by the `behavior` variable inside the script (default: `TEST.json`).

---

## 4. Using Custom Behavior Files

To change which behavior file is played:

1. Place your `.json` file in the `sami_sim/behaviors/` directory.
2. Open `json_joint_publisher.py`.
3. Modify this line near the top:

```python
behavior = 'YourFile.json'
```
4. Rebuild your workspace and re-run the script:

```bash
colcon build
source install/setup.bash
ros2 run sami_sim json_joint_publisher.py
```

---

## Summary

| Task                         | Command                                                    |
| ---------------------------- | ---------------------------------------------------------- |
| Initial setup (fresh VM)     | `git clone ... && sudo apt install ...`                    |
| Navigate to workspace        | `cd ~/sami_ws`                                             |
| Build workspace              | `colcon build`                                             |
| Source environment           | `source install/setup.bash`                                |
| Launch RViz + robot          | `ros2 launch sami_sim display.launch.py`                   |
| Run behavior playback        | `ros2 run sami_sim json_joint_publisher.py`                |
| Use custom behavior file     | Edit `behavior = 'YourFile.json'` in `json_joint_publisher.py` |

---
