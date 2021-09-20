^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dbw_fca_can
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.1 (2021-09-20)
------------------
* Bump firmware versions
* Add reserved bits
* Improve printing of license info
* Add ignition status to ThrottleInfoReport
* Add user control of alert
* Contributors: Kevin Hallenbeck

1.2.0 (2021-05-12)
------------------
* Bump firmware versions
* C++17 and std::clamp()
* Remove ROS Kinetic support
* Populate brake/throttle/steering command values even if enable is false
* Fix socketcan error frame lock up
* Contributors: Kevin Hallenbeck, Robert Maupin

1.1.3 (2021-03-09)
------------------
* Bump firmware versions
* License multiple features individually
* Add option to use socketcan hardware
* Contributors: Kevin Hallenbeck

1.1.2 (2021-01-14)
------------------
* Bump firmware versions
* Fix typo in MiscCmd message
* Update brake type field to match firmware change
* Add Ford Ranger platform
* Add reserved bit to prevent warnings
* Contributors: Kevin Hallenbeck

1.1.1 (2020-08-17)
------------------
* Bump firmware versions
* Contributors: Kevin Hallenbeck

1.1.0 (2020-08-10)
------------------
* Add HVAC control and status
* Contributors: Kevin Hallenbeck, Sun Hwang

1.0.11 (2020-08-05)
-------------------
* Only publish joint states from DbwNode if set to load URDF model
  The joint states messages from DbwNode collide with the joint states published by the Gazebo simulator otherwise.
* Change names of joints to be different from links
  Gazebo 11 doesn't let joint names and link names be the same, which was allowed in earlier versions
* Contributors: Kevin Hallenbeck, Micho Radovnikovich

1.0.10 (2020-07-09)
-------------------
* Add gear reject enumerations
* Increase CMake minimum version to 3.0.2 to avoid warning about CMP0048
  http://wiki.ros.org/noetic/Migration#Increase_required_CMake_version_to_avoid_author_warning
* Remove unused parameter from launch file
* Add sensor reports for wiper, highbeam, and hazard light
* Add sensor reports for gyro, accelerometer, GPS, and tire pressure
* Contributors: Kevin Hallenbeck, Sreedevi Adukkathayar, Sun Hwang

1.0.9 (2020-02-14)
------------------
* Update firmware versions
* Report NAN for signals that are unavailable/faulted
* Add door commands
* Contributors: Kevin Hallenbeck

1.0.8 (2019-10-17)
------------------
* Add launch file argument to switch between Jeep and Pacifica URDF models
* Contributors: Micho Radovnikovich

1.0.7 (2019-09-13)
------------------
* Added argument to enable/disable CAN message filtering on DBW message range
* Contributors: Kevin Hallenbeck

1.0.6 (2019-08-13)
------------------
* Updated firmware versions
* Updated website maintenance link
* Contributors: Kevin Hallenbeck

1.0.5 (2019-07-24)
------------------
* Extend licensing to each module
* Contributors: Kevin Hallenbeck, Sun Hwang

1.0.4 (2019-07-11)
------------------
* Added support for non-hybrid brake report values
* Added gear number to ThrottleInfoReport
* Contributors: Kevin Hallenbeck, Sun Hwang

1.0.3 (2019-05-03)
------------------
* Added warnings for unknown and unsupported command types
* Added FORD_C1 platform
* Added fuel level report message
* Added casts to force single precision floating point math
* Contributors: Kevin Hallenbeck

1.0.2 (2019-03-14)
------------------

1.0.1 (2019-03-01)
------------------
* Updated firmware versions
* Added support for firmware change that uses SVEL resolution of 4 deg/s
* Refactored tcpNoDelay() for subscribers
* Added missing tests for PlatformVersion.h
* Contributors: Kevin Hallenbeck, Micho Radovnikovich

1.0.0 (2018-11-30)
------------------
* Updated firmware versions
* Added BTYPE (brake type) bit
* Added CMD_DECEL brake command type (only for non-hybrid platforms)
* Added dataspeed_ulc_can to dbw.launch
* Added throttlePercentFromPedal lookup table function and corresponding test
* Use the ${catkin_EXPORTED_TARGETS} macro for target dependencies
* Contributors: Kevin Hallenbeck, Micho Radovnikovich

0.0.2 (2018-10-23)
------------------
* Updated firmware versions
* Added platform FCA_WK2 (Jeep Grand Cherokee)
* Force forwarding of brake command type when ABS module is present (instead of BPEC module)
* Disengage on any fault for brake/throttle/steering (change AND to OR)
* Added cruise control buttons
* Latch firmware version on any change (previously only latched once)
* Changed pedal_luts default from true to false (forward command type by default now)
* Disregard overrides on unused subsystems using the TIMEOUT bit
* Removed cruise control related buttons that are not implemented by firmware at this time
* Fixed typo in nodelets.xml of dbw_fca_can
* Renamed steering CMD_TYPE and TMODE
* Set CXX_STANDARD to C++11 only when necessary
* Use sign of wheel speeds to set sign of vehicle speed
* Removed unused dependencies and includes
* Removed steering debug message
* Handle version message with a map/database of several platform/module combinations (ported from dbw_mkz_can)
* Contributors: Kevin Hallenbeck, Micho Radovnikovich

0.0.1 (2018-08-08)
------------------
* Initial release
* Contributors: Kevin Hallenbeck
