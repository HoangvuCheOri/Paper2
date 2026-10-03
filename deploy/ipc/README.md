# AMR IPC deployment

The IPC workspace contains source and local build products only. Do not copy
`build/`, `install/`, `log/`, bags, plots, or camera credentials from the
development machine.

Install the non-ROS Python dependency:

```bash
python3 -m pip install --user -r ~/AMR/requirements-ipc.txt
```

Build the workspace:

```bash
cd ~/AMR
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

Safe smoke test (no trajectory controller is started):

```bash
source /opt/ros/humble/setup.bash
source ~/AMR/install/setup.bash
ros2 launch amr_control ipc_core.launch.py camera_display:=false
```

For automatic startup, copy `amr-ipc.service` to `~/.config/systemd/user/`,
copy `ipc.env.example` to `~/.config/amr/ipc.env`, fill in the real device and
camera values, then protect the credential file with mode `600`. Enable it with
`systemctl --user enable --now amr-ipc.service`. The service starts the serial
bridge, wheel/IMU state bridge, EKF, and camera localization. It deliberately
does not start any trajectory controller or publish `/cmd_vel`.
