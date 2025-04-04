# SAMI ROS 2 Simulation Workspace (`sami_ws`)

This workspace allows you to simulate the **SAMI** robot in RViz and run joint animations using pre-recorded JSON behavior files.

---

## 0. Navigate to the Workspace

Before building or launching anything, make sure you're in your ROS 2 workspace directory:

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

## 3. Run the Behavior Playback

> **Important:** Before running the script, **close** the Joint State Publisher GUI to avoid conflicts.

Open a **new terminal**, then:

```bash
cd ~/sami_ws
```

```bash
source install/setup.bash
```

Run the script directly:

```bash
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
behavior = 'TEST.json'
```
4. Follow the steps from ## 1. Build the Workspace

---

## Summary

| Task                      | Command                                                    |
| ------------------------- | ---------------------------------------------------------- |
| Navigate to workspace     | `cd ~/sami_ws`                                             |
| Build workspace           | `colcon build`                                             |
| Source environment        | `source install/setup.bash`                                |
| Launch RViz + animation   | `ros2 launch sami_sim display.launch.py`                   |
| Run behavior manually     | `ros2 run sami_sim json_joint_publisher.py`                |
| Use custom behavior file  | Edit `behavior = 'YourFile.json'` in `json_joint_publisher.py` |

---
