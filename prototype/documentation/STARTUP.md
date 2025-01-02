# Starting up the System

Currently, the robot consists of:
- UI_testing.py (ROS node)
- service_test.py (ROS node)
- rosbridge server
- server.py (Mock Flask server)

To run all of the above, one can run the bash script `UI_prototype_launch.sh` in `BladeGuard/prototype/shell_scripts`. This launches the two ROS nodes and rosbridge server using a ROS launch file (explained later) as a background task, followed by running the flask server.

To run just the ROS nodes and rosbridge server, one can use the launch file `/BladeGuard/prototype/ros/motor_controls_ws/src/motors/launch/UI_prototype_launch.py` by:
```bash
source /opt/ros/<DISTRO>/setup.bash
source /BladeGuard/prototype/ros/motor_controls_ws/install/setup.bash
ros2 launch motors UI_prototype_launch.py
```
To run only the server:
```bash
cd /BladeGuard/prototype/web/flask_server
source flask_venv/bin/activate
python server.py
```

The UI_prototype_launch.sh script can be configured to run as the robot boots. To do so, first run
```bash
sudo nano /etc/systemd/system/UI_prototype_launch.service
```

and copy the following code:

```bash
[Unit]
Description=ROS2 Launch File Service
After=network.target

[Service]
ExecStart=/home/<USERNAME>/BladeGuard/prototype/shell_scripts/UI_prototype_l>
WorkingDirectory=/home/<USERNAME>/BladeGuard/prototype/ros/motor_controls_ws
User=<USERNAME>
Restart=always
Environment="PYTHONUNBUFFERED=1"

[Install]
WantedBy=multi-user.target
```

After saving, run

```bash
sudo systemctl enable UI_prototype_launch.service
```

The shell script will now run on system startup. 

To run the service manually, check the status of the service, or manually stop the service, run these three commands, respectively:

```bash
sudo systemctl start UI_prototype_launch.service
sudo systemctl status UI_prototype_launch.service
sudo systemctl stop UI_prototype_launch.service
```