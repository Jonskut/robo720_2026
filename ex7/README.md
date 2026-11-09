# Exercise 7: Force and Compliance Control

To get the latest changes in the repo to your local directory, navigate to the course directory in your terminal, run `git pull`, and resolve all possible merge conflicts (you may save your own edits, but in the future it is possible that the templates will not work with your code). If you want to save your own code separately and just get the fresh template, navigate to your home directory and run:

```bash
git clone https://github.com/tau-alma/robo720_2026.git
```

Once inside container again, navigate to `ros2_ws` and build and source the packages:

```bash
cd ~/ros2_ws
colcon build --parallel-workers $(($(nproc)/2)) --executor sequential
source install/setup.bash
```

## Launch setups

To launch impedance controller, run:

```bash
ros2 launch ex7 impedance_controller.launch.py
```

To launch admittance controller, run:

```bash
ros2 launch ex7 admittance_controller.launch.py
```

To launch direct force controller, run:

```bash
ros2 launch ex7 direct_force_controller.launch.py
```

## Switching parameters during run-time

### Fake sensor measurements

Both impedance and admittance controller launch files also launch `fake_wrench_publisher` node which publishes dummy force-torque measurements. Only admittance controller subscribes to the fake measurements by default. To subscribe to real sensor measurements, run in a separate terminal:

```bash
ros2 param set /<node_name> use_fake_wrench false
```

To subscribe to fake measurements, run:

```bash
ros2 param set /<node_name> use_fake_wrench true
```

Node name is either `impedance_controller` or `admittance_controller`.

### Sensor measurements as feedback

Each controller has ROS parameter `use_force_measurement`. Utilize it to see difference in the controllers when using force-torque measurement as feedback vs. plain motion control.

To switch the parameter, run:

```bash
ros2 param set /<node_name> use_force_measurement <bool>
```

Node name is either `impedance_controller`, `admittance_controller` or `direct_force_controller` and bool is either `true` or `false`.

```bash
source install/setup.bash
ros2 run goal_publishers goal_pose_publisher.py
```
