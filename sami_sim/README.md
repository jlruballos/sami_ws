# sami ROS 2 Simulation Workspace (`sami_ws`)

This workspace allows you to simulate the sami robot in RViz and run joint animations using pre-recorded JSON motion files.

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

## 2. Launch the sami Robot in RViz

To visualize the robot and manually control the joints using a GUI:

```bash
ros2 launch sami_sim display.launch.py
```

You should see sami loaded in RViz with a joint control panel on the left.

---

## 3. Run the JSON Motion Playback

>**Important:** Before running the motion script, **close** the Joint State Publisher GUI to avoid conflicts.

Open a **new terminal**, then:

```bash
source install/setup.bash
```

Now run the animation script:

```bash
ros2 run sami_sim json_joint_publisher.py
```

If successful, sami will perform a wave motion in RViz.

---

## 4. Using Custom Motion Files

The `json_joint_publisher.py` script reads from a motion file defined in the code.

To use your own JSON animation file, modify this line in the script:

```python
animation_file = '/home/your_username/sami_ws/sami_sim/motion/right_wave.json'  # ← Replace with your actual file path
```

---

## Summary

| Task                  | Command                                      |
| --------------------- | -------------------------------------------- |
| Navigate to workspace | `cd ~/sami_ws`                              |
| Build workspace       | `colcon build`                               |
| Source environment    | `source install/setup.bash`                  |
| Launch RViz with GUI  | `ros2 launch sami_sim display.launch.py`    |
| Run animation script  | `ros2 run sami_sim json_joint_publisher.py` |

---