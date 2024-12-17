# BLADEGUARD ROBOTICS PROJECT

This README.md file gives instructions on how to set up, run and maintain the software of the robot. 

## Raspberry Pi
- Username - bladeguard
- OS - Ubuntu 24 LTS (headless)
- Static IP address - 192.168.0.2 // 10.12.2024
- ROS version - ROS2 Jazzy

### Access Using SSH

To gain access to the Raspberry Pi, first connect it to a laptop using an Ethernet cable. The Pi was given a static IP address of 192.168.0.2. To be able to connect to it from a laptop, the following setup is needed:
- **Ubuntu** - In **Settings** &rarr; **Network** look for the **Wired** section. Press the + icon to create a new profile. In the IPv4 tap, select _Manual_ and assign it the address 192.168.0.3 and netmask 255.255.255.0. Finally, click on the profile to make sure it is connected.
- **Windows** - Open **Network Connections** (Win + R and type ncpa.cpl), and right-click on the Ethernet connection to which the Pi is connected. Select **Properties**. Click on **Internet Protocol Version 4 (TCP/IPv4)** and choose **Use the following IP address**. Set the IP address to 192.168.0.3 and the netmask to 255.255.255.0.

After the laptop is set up, open a terminal/command prompt and type `ping 192.168.0.2`. If everything is properly set up, there should be zero packet loss. If on Windows, `ping bladeguard` should also work. To gain access to the bash terminal of the Raspberry Pi, type `ssh bladeguard@192.168.0.2`. If on Windows, `ssh bladeguard@bladeguard.local` shouls also work. You will be prompted to enter the password, after which you will have access to the Pi.

However, with this setup, the Raspberry Pi will not have access to the internet. If this is needed, connect the Pi to a router using the Ethernet cable, and make sure the laptop is connected to the same network as the Pi. Log into the router's web interface to find out the IP address of the Raspberry Pi, and run `ssh bladeguard@<IP from router>`. This way, the Raspberry Pi will have access to the internet.

#### Troubleshooting

If the connection fails, connect the Raspberry Pi to the laptop directly via ethernet cable and try the following:
- **Ubuntu** - run `nmap -sn 192.168.0.3/24`. The IP of the Raspberry Pi should be listed as output. You can use it to SSH into the Pi. If this does not work, go to **Settings** &rarr; **Network** and create a new profile. In the IPv4 tab, select "Shared to other computers", and note the IP address assigned to the connection. Then run `nmap -sn <IP address of connection>/24`, and the IP of the Pi should be shown as output.
- **Windows** - use `ssh bladeguard@bladeguard.local`. If this does not work, run `ipconfig` and make sure that the IP address of the Ethernet connection is indeed 192.168.0.3. If it has multiple addresses, they can be deleted in the **Advanced** tab of the Ethernet connection properties in **Network Connections**. If the IP is not 192.168.0.3, set it again ad described above, and disable any other Ethernet connections. Also, right-click the Ethernet connection used by the Pi, go to **Properties** &rarr; **SHARING** and disable internet connection sharing (ICS). It might be necessary to restart your laptop.

### Setting a Static IP Address

To change the static IP of the Raspberry Pi, run `sudo nano /etc/dhcpcd.conf` and add the following lines:
```java
interface eth0
static ip_address=<IP ADDRESS OF RASPBERRY PI>/24
static routers=<IP SET ON LAPTOP'S ETHERNET CONNECTION>
static domain_name_servers=8.8.8.8
```
Save the file and run `sudo systemctl restart dhcpcd`

## rosbridge_server

To run the rosbridge_server, run the following commands:

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

To connect the user interface to the rosbridge server, make sure to include the line 
```javascript
const ros = new ROSLIB.Ros({ url: "ws://192.168.0.2:9090" });
```

Make sure the IP address is the same as that of the Pi. It also must be an IPv4 address, as IPv6 does not work with rosbridge.

## Arduino
The Arduino Mega should be connected to the Raspberry Pi via USB cable. Messagess can be sent between the two using the serial connection via the PySerial library. To instantiate the connection, the following code is used:
```python
arduino = serial.Serial(port='/dev/arduino_mega', baudrate=115200, timeout=.1)
```

Note that '/dev/arduino_mega' is a symlink which points to the port which the Arduino Mega is connected. This is set up using UDEV rules, and can be set up as follows:
- Open the relevant file with the UDEV rule: `sudo nano /etc/udev/rules.d/99-arduino.rules`
- To change the name of the symlink, or add another symbolic link, modify the line `SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0042", ATTRS{serial}=="434393431343C5101F0D0", SYMLINK+="arduino_mega"`, most importantly the SYMLINK part.

If a different Arduino device is to be used, the UDEV rule will have to be changed:
- Connect the Arduino to the Raspberry Pi
- In the terminal, run `ls -l /dev/ttyACM*`. This will most likely output "/dev/ttyACM0". If there is only one line of output, that is the port of the Arduino. If there are multiple, you can unplug the Arduino and run the command again, and plug it back in and run the command once more, to find out which port belongs ot the Arduino. If there is no output from `ls -l /dev/ttyACM*`, try `ls -l /dev/ttyUSB*`. 
- Once you have found the port, run `udevadm info -a -n <NAME OF ARDUINO PORT>`
- In the output, take note of the attributes `idVendor`, `idProduct`, and `serial`.
- Open the UDEV rule file: `sudo nano /etc/udev/rules.d/99-arduino.rules`, and replace the old attributes with the new values from the previous instruction
- Run the follwing commands, to reload the UDEV rules:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```
- To test if the rule works, run `ls -l /dev/arduino_mega`. It should output the actual port of the Arduino (most likely /dev/ttyACM0)

### Troubleshooting
If there is ever an Arduino-related error cocerning permissions, run the command `sudo chmod a+rw /dev/ttyACM0`, replacing /dev/ttyACM0 with the actual port of the Arduino (or the SYMLINK).