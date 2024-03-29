# JMOAB ROS2

JMOAB package for ROS2. This package is based on [jmoab-ros](https://github.com/rasheeddo/jmoab-ros) package from ROS1. The supported version of PCB is AT_JMOAB06.

![](images/jmoab1.jpg)

## Jetson Nano with ROS2
In order to run ROS2 on Jetson Nano, at the current time there is no official support from NVIDIA for Ubuntu 20.04 which requires for ROS2, there is a docker support for ROS2, but I am not familiar with docker and I prefer to build ROS/ROS2 directly to my filesystem. There are two ways to have Ubuntu 20.04 on Jetson Nano

1. From [Qengineering](https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image) custom image, this you will have CUDA installed already, so recommended for the future of AI integration.

2. From [mr.chrismitchells Xubuntu](https://forums.developer.nvidia.com/t/xubuntu-20-04-focal-fossa-l4t-r32-3-1-custom-image-for-the-jetson-nano/121768) custom image, I have tried with this one, it's working fine, but I noticed that USB3.0 speed got missed, so if you are planning to use high speed USB for some device, you will only get USB2.0 speed instead. And you will need to install CUDA by yourself.

## Update JMOAB firmware

All of available firmware are in [firmwares](./firmwares/) directory.

To flash the firmware, please follow the step below, (must use Windows PC)

- first you will need to have Mini Prog3 [programmer](https://www.digikey.com/catalog/en/partgroup/psoc-miniprog3-programmer-debugger-cy8ckit-002/20080).
- Download PSocC Programmer [software](https://www.infineon.com/cms/en/design-support/tools/programming-testing/psoc-programming-solutions/?utm_source=cypress&utm_medium=referral&utm_campaign=202110_globe_en_all_integration-product_families#!downloads).
- Open the PSoC Programmer, it will shown as here

![](images/open_programmer.jpeg)

- Keep eyes on lower right corner, so it should show 3 green as PASS, POWERED, Connected. So you should plug the Mini Prog3 to the JMOAB J2 header. Make sure that the VTARG pin of the programmer is on the correct pin header on JMOAB (most right pin when facing to Jetson USB port). And also please power on the Jetson as well.

![](images/flash_fw.jpg)

- Once you saw all three greens status, the select which firmware you would like to use, and click on Program button right next to Open button.
- If it's success, you will see the it's showing successful message.

![](images/flash_success.jpeg)

- You will need to power off and on the Jetson again to take affect of new firmware on JMOAB.

## Dependencies

- ros2 debian packages, install from [here](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html).
- `sudo pip3 install smbus2`
- `sudo apt install ros-$ROSDISTRO-tf-transformations`
- `sudo pip3 install transforms3d`
- `sudo pip3 install numpy`
- `sudo apt install python3-colcon-common-extensions`, to build package in workspace

## Install jmoab_ros2 package

Assume you have the ros2 workspace called `dev_ws` at home directory.

```sh
cd ~/dev_ws/src

git clone https://github.com/attraclab/jmoab_ros2.git

cd ..

colcon build --symlink-install --packages-select jmoab_ros2

```

Make sure to source all ros environment setup files.

`source /opt/ros/galactic/setup.bash` for Galactic.

`source ~/dev_ws/install/local_setup.bash` for local workspace name `dev_ws`.

`export ROS_DOMAIN_ID=1`, this will allow all the ROS2 node in network with the same ID could see each other.

or put it in `~/.bashrc` file as below

```sh
source /opt/ros/galactic/setup.bash
source ~/dev_ws/install/local_setup.bash
export ROS_DOMAIN_ID=1
```

## JMOAB atcart_basic

To use JMOAB with ATCart, make sure to have the correct ESC FW by checking from [here](https://github.com/rasheeddo/BrushlessDriveWheels/tree/new-batch-esc#hack-new-batch-esc). The "Model" at the back of ESC should be "MN1 WSDC/3+7F&G-X". 

If you don't know what ATCart is, I recommend to check on [this site](https://stepbystep-robotics.com/hp/robots/atcart-overview/atcart-basic/) or [this video](https://www.youtube.com/watch?v=Y4HL69gDDOU&ab_channel=stepbystep-robotics) about how I hacked it and use it for robots project.

As default the JMOAB is pre-flashed with atcart functions firmware. You can check available firmwares from [firmwares directory](https://github.com/attraclab/jmoab_ros2/tree/master/firmwares), as default it is 20220322_JMOAB06_v1.hex

JMOAB itself can be connected with many peripherals,

![](images/atcart_basic.jpg)

You just need to run

`ros2 run jmoab_ros2 atcart_basic`

### publish

- `/jmoab/sbus_rc_ch` as std_msgs/msg/Int16MultiArray, sbus value of RC transmitter's channels
- `/jmoab/cart_mode` as std_msgs/msg/UInt8, 0: hold, 1: manual, 2: auto modes
- `/jmoab/adc` as std_msgs/msg/Float32MultiArray, ADC value in 12bits maximum voltage is 40.96V

### subscribe

- `/cmd_vel` as geometry_msgs/msg/Twist, you can consider linear.x as throttle (go straight), and angular.z as steering (turning). 
- `/jmoab/wheels_cmd` as std_msgs/msg/Float32MultiArray, this is percentage of left/right wheel power ex. [30.0, -50.0] is left wheel would be spinning forward 30% and right wheel would be spinning reverse 50% . This would be useful when you want to control each wheel speed individually.
- `/jmoab/cart_mode_cmd` as std_msgs/msg/UInt8, this allow you to change cart mode programmatically; 0: hold, 1: manual, 2: auto modes
- `/jmoab/relays` as std_msgs/msg/Int8MultiArray, you can control relays ON/OFF; ex. [1,0] relay1 on relay2 off.
- `/jmoab/servos` as std_msgs/msg/Int16MultiArray, you can control servo motors by pwm value; ex. [1920, 1120, 1520] servo1 is high, servo2 is low, servo3 is neutral.

## JMOAB bno055

Before running imu node, I recommend to do sensor calibration by going to [scripts directory](./scripts/) and run

`python3 bno055_calibrate.py`

You will need to follow these step

1. leave the sensor stays still on table to get gyroscope offset, make sure the `gyr_stat` becomes 3.
2. try put the sensor in 45, 90 -45, -90, -180 degrees to calibrate accelerometer and keep it stay still in each angle for few seconds, make sure the `acc_stat` becomes 3.
3. try swing the sensor into air to calibrate magnetometer, make sure the `mag_stat` becomes 3.
4. once all stat becames 3, the `sys_stat` should turn to 3 as well, then it will end the program by itself, sometimes I got all offset showing 0, you can try run the script again and it will just print the latest offset correctly.
5. You will get `calibration_offset.txt` file in `scripts/`, so keep it there. bno055.py will load this automatically.

After that you just run

`ros2 run jmoab_ros2 bno055`

### publish

- `/imu/data` as sensor_msgs/msg/Imu, all of data would be filled in the message, you can use this for EKF sensor fusion later.
- `/jmoab/ahrs` as std_msgs/msg/Float32MultiArray, the data is [roll, pitch, heading] in degrees 0-360 range.

### subscribe

- `/jmoab/sbus_rc_ch` as std_msgs/msg/Int16MultiArray, it will use throttle stick to recognize that user is moving the cart in straight line.
- `/jmoab/cart_mode` as std_msgs/msg/UInt8, it will calibrate the heading just only in manual control.
- `/fix` as sensor_msgs/msg/NavSatFix, to use GPS data as reference when it's moving in straight line then it will do heading calibration.

## JMOAB pwmcart

In case of other type of cart which needs PWM signal like regular RC car, we have to flash pwm-cart firmware from [firmwares directory](https://github.com/attraclab/jmoab_ros2/tree/master/firmwares), it is 20220729_AT_JMOAB06_fw_v09_PWM-ESC.hex . Please checking on update firmware from step above.

Similar to ATCart it could have other peripherals control, but instead of ATCart interface, it's using PWM1 for left wheel and PWM2 for right wheel ESCs. Vehicles such as ATBuggy, ATCrawler, or ATTLER could be used.

![](images/pwmcart.jpg)

### publish

- `/jmoab/sbus_rc_ch` as std_msgs/msg/Int16MultiArray, sbus value of RC transmitter's channels
- `/jmoab/cart_mode` as std_msgs/msg/UInt8, 0: hold, 1: manual, 2: auto modes
- `/jmoab/adc` as std_msgs/msg/Float32MultiArray, ADC value in 12bits maximum voltage is 40.96V

### subscribe

- `/cmd_vel` as geometry_msgs/msg/Twist, you can consider linear.x as throttle (go straight), and angular.z as steering (turning). 
- `/jmoab/wheels_cmd` as std_msgs/msg/Float32MultiArray, this is percentage of left/right wheel power ex. [30.0, -50.0] is left wheel would be spinning forward 30% and right wheel would be spinning reverse 50% . This would be useful when you want to control each wheel speed individually.
- `/jmoab/cart_mode_cmd` as std_msgs/msg/UInt8, this allow you to change cart mode programmatically; 0: hold, 1: manual, 2: auto modes
- `/jmoab/relays` as std_msgs/msg/Int8MultiArray, you can control relays ON/OFF; ex. [1,0] relay1 on relay2 off.
- `/jmoab/servo` as std_msgs/msg/Int16, you can control only one servo motor by pwm value; ex. in 1120-1520-1920 ranges.

### TODO

It is necessary to know that how much of lowest speed each left/right motors can spin, or in the other meaning we need to know the minimum and maximum of dead band value which makes the wheels start to spin. An ideal is 1024 SBUS value is to stop the motor, and 1025 would make it run forward, and 1023 would be it run backward in very slow speed. But technically things are not perfect like that, there is a dead band of +/- from 1024 which make the wheels to stop. We have to find these dead band first in order to control the motor correctly in future application.

![](images/pwmcart_sbus_db.png)

First start `pwmcart` node as below

```
ros2 run jmoab_ros2 pwmcart --ros-args -p show_log:=True
```

It will be showing log every 1 seconds.

After `pwmcart` node is running, then you can try to publish `/jmoab/wheels_cmd` little by little, for example,

`ros2 topic pub /jmoab/wheels_cmd std_msgs/msg/Float32MultiArray "data: [10.0, 10.0]"`

Then from RC transmitter (ch.5) switch to auto mode.

On `pwmcart` node termnial you will see log info as following,

```
[INFO] [1659088017.413651126] [jmoab_ros2_pwmcart]: vx: 0.00 wz: 0.00 left: 0.00 right: 0.00 x: 0.00 y: 0.00 l_sbus: 1024 r_sbus: 1024
[INFO] [1659088018.461170311] [jmoab_ros2_pwmcart]: vx: 0.00 wz: 0.00 left: 0.00 right: 0.00 x: 0.00 y: 0.00 l_sbus: 1024 r_sbus: 1024
[INFO] [1659088019.512736894] [jmoab_ros2_pwmcart]: vx: 0.00 wz: 0.00 left: 0.00 right: 0.00 x: 0.00 y: 0.00 l_sbus: 1024 r_sbus: 1024
[INFO] [1659088020.561640338] [jmoab_ros2_pwmcart]: vx: 0.00 wz: 0.00 left: 0.00 right: 0.00 x: 0.00 y: 0.00 l_sbus: 1024 r_sbus: 1024

```  

so observe that at which `/jmoab/wheels_cmd` value the wheels stop moving but seems to start moving, left and right could be different values as well. Then check on `l_sbus` and `r_sbus` which printing out, so that is the upper deadband where the motors could start to spin forward.

Then put those value into the parameters file at [config/pwmcart_params.yaml](https://github.com/attraclab/jmoab_ros2/blob/master/config/pwmcart_params.yaml), at `sbus_left_max_db` and `sbus_right_max_db`.

Again try to publish `/jmoab/wheels_cmd` in reverse value like `[-10.0, -10.0]` and keep adjusting until the motors are mostly stop. Then put those values on `sbus_left_min_db` and `sbus_right_min_db`.

In order to use this parameters file, you need to run as

`ros2 run jmoab_ros2 pwmcart --ros-args --params-file ~/dev_ws/src/jmoab_ros2/config/pwmcart_params.yaml`

note that my workspace is `dev_ws`.

Then try publish `/jmoab/wheels_cmd` again, now even 1% or 2% you should notice that the wheel starts to spin slowly.

## JMOAB ATCart8

ATCart8 is an 8 inch encoder built-in wheel motor based platform.

You will need to install this [additiona python package](https://github.com/rasheeddo/ZLAC8015D_python) in order to run this. If you are able to run some examples in [ZLAC8015D_python/examples](https://github.com/rasheeddo/ZLAC8015D_python/tree/master/examples) and the motors can spin correctly then you can continue from this.

![](images/atcart8.png)

ATCart8 is not related JMOAB directly, but in case of some application which requires JMOAB then you can use this as well. 

You will need to make a udev rules, if you're using many USB serial ports in the same time, so in this case I made the USB-RS485 serial port as `/dev/usb_rs485`, so if you're using different name, you have to change it on [this line](https://github.com/attraclab/jmoab_ros2/blob/60af19e06cf2aa8087f8a06c9f12971f9462d050/jmoab_ros2/atcart8.py#L119).

### publish

- `/jmoab/sbus_rc_ch` as std_msgs/msg/Int16MultiArray, sbus value of RC transmitter's channels
- `/jmoab/cart_mode` as std_msgs/msg/UInt8, 0: hold, 1: manual, 2: auto modes
- `/jmoab/adc` as std_msgs/msg/Float32MultiArray, ADC value in 12bits maximum voltage is 40.96V
- `/jmoab/wheels_rpm` as std_msg/msg/Float32MultiArray, this is feedback RPM from ZLAC8015D driver [left_rpm, right_rpm].
- `/atcart8/odom` as nav_msgs/msg/Odometry, this is raw odometry from wheel's encoder calculation, you will need EKF node to run together to have better odometry.

### subscribe

- `/cmd_vel` as geometry_msgs/msg/Twist, you can consider linear.x as throttle (go straight), and angular.z as steering (turning). 
- `/jmoab/cart_rpm_cmd` as std_msgs/msg/Int16MultiArray, this is wheel's RPM control. 
- `/jmoab/cart_deg_cmd` as std_msgs/msg/Int16MultiArray, this is wheel's angle (degree) control, you only need to send one time (not continuously like cmd_vel).
- `/jmoab/cart_dist_cmd` as std_msgs/msg/Int16MultiArray, this is wheel's distance (meters) control, you only need to send one time (not continuously like cmd_vel).
- `/jmoab/cart_mode_cmd` as std_msgs/msg/UInt8, this allow you to change cart mode programmatically; 0: hold, 1: manual, 2: auto modes
- `/jmoab/relays` as std_msgs/msg/Int8MultiArray, you can control relays ON/OFF; ex. [1,0] relay1 on relay2 off.
- `/jmoab/servos` as std_msgs/msg/Int16MultiArray, you can control servo motors by pwm value; ex. [1920, 1120, 1520] servo1 is high, servo2 is low, servo3 is neutral.


# Third party packages.

## JMOAB with F9P GPS

GPS port on JMOAB is connected to pin 8 and 10 of Jetson J41 header and recognized as `/dev/ttyTHS1` (UART_2).

In order to enable this port to use as regular user, please check on this [link](https://forums.developer.nvidia.com/t/read-write-permission-ttyths1/81623/6
) or following the step below.

- We will need to disable nvgetty
	```
	sudo systemctl stop nvgetty
	sudo systemctl disable nvgetty
	```
- Create a udev rule for ttyTHS* to get permission (without sudo)

	`sudo vim  /etc/udev/rules.d/55-tegraserial.rules`
	
- put this content on created udev rule file
	`KERNEL=="ttyTHS*", MODE="0666"`
	
- reload rules and reboot
	```
	sudo udevadm control --reload-rules
	sudo reboot
	```

Then you are able to use `/dev/ttyTHS1` for GPS now.

I recommend to use [nmea_navsat_driver package](https://github.com/ros-drivers/nmea_navsat_driver/tree/ros2), so you will need to

```sh
cd ~/dev_ws/src/
git clone https://github.com/ros-drivers/nmea_navsat_driver.git
cd nmea_navsat_driver
git checkout ros2
```

You will need to edit the params file at [nmea_navsat_driver/config/nmea_serial_driver.yaml](https://github.com/ros-drivers/nmea_navsat_driver.git), and change to as content below,

```
nmea_navsat_driver:
  ros__parameters:
    port: "/dev/ttyTHS1"
    baud: 115200
    frame_id: "gps"
    time_ref_source: "gps"
    useRMC: False
```

Build the package,

```sh
cd ~/dev_ws
colcon build --symlink-install --packages-select nmea_navsat_driver
```

Make a symlink manually in case you change something on params file later,

```sh
cd ~/dev_ws/install/nmea_navsat_driver/share/nmea_navsat_driver/config
ln -s ~/dev_ws/src/nmea_navsat_driver/config/nmea_serial_driver.yaml nmea_serial_driver.yaml 
```

Launch the file,

```sh
cd 
source /opt/ros/foxy/setup.bash
ros2 launch nmea_navsat_driver nmea_serial_driver.launch.py
```

## Robot Localization

In order to have a better odometry, you will to use [robot_localization package](http://docs.ros.org/en/noetic/api/robot_localization/html/index.html) to fuse the `atcart8/odom` with `/imu/data` topics.

Install the package,
`sudo apt install ros-foxy-robot-localization`

The config file is in [jmoab_ros2/config/ekf.yaml](https://github.com/attraclab/jmoab_ros2/blob/master/config/ekf.yaml) which is following [the explanation from original developer](http://docs.ros.org/en/noetic/api/robot_localization/html/configuring_robot_localization.html).

Then you can launch as,

```sh
# Terminal 1
ros2 launch jmoab_ros2 atcart8_imu.launch.py

# Terminal 2
ros2 launch jmoab_ros2 ekf.launch.py
```

or you can try to wrap this up into one launch file in your project.