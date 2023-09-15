# ds_dbw_joystick_demo
ROS2 interface to Dataspeed drive-by-wire platforms

Launch the drive-by-wire and the demo
```
ros2 launch ds_dbw_joystick_demo joystick_demo.launch.xml sys:=true
```

Logitech F310 Gamepad controls:

* Disable
    * Left Bumper
* Enable
    * Right Bumper
* Brakes
    * Left Trigger
* Throttle
    * Right Trigger
* Steering
    * Left/right of either joystick axis
    * Hold back or start to get full steering range, otherwise half
* Turn Signals
    * Left/Right D-Pad
