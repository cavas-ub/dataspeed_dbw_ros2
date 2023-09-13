# dbw_ros
ROS2 interface to Dataspeed drive-by-wire platforms

# Launch

* Joystick demo
    * `ros2 launch ds_dbw_joystick_demo joystick_demo.launch.xml sys:=true`
* Drive-by-wire only
    * `ros2 launch ds_dbw_can dbw.launch.xml`

# Binaries

* ROS buildfarm with infrequent updates:
    * http://repo.ros2.org/status_page/ros_humble_default.html?q=dbw_ros
* Dataspeed buildfarm with frequent updates:
    * https://bitbucket.org/DataspeedInc/ros_binaries/

# One Line Install (binary)

* Use this option to install ROS2 package binaries on a workstation that already has ROS2 installed.
* Paste the following into a terminal to install the binary packages. This script will configure apt-get to connect to the Dataspeed server and install the binary packages.
* ```bash <(wget -q -O - https://bitbucket.org/DataspeedInc/dbw_ros/raw/dbw2/ds_dbw/scripts/sdk_install.bash)```

# One Line ROS2 and Packages Install (binary)

* Use this option to install ROS2 and this SDK on a clean Ubuntu install.
* This should ONLY be run on a fresh install of Ubuntu Desktop [22.04](http://releases.ubuntu.com/22.04/)/[20.04](http://releases.ubuntu.com/20.04/).
* Paste the following into a terminal to install ROS2 and this SDK. This script will change some operating system parameters, install ROS2 [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)/[Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html), install the SDK, and configure the joystick demo to run at startup.
* ```bash <(wget -q -O - https://bitbucket.org/DataspeedInc/dbw_ros/raw/dbw2/ds_dbw/scripts/ros_install.bash)```