std_msgs/Header header

# Current gear enumeration
Gear gear

# Gear command enumeration
Gear cmd

# Gear command enumeration
Gear driver

# Gear reject enumeration
GearReject reject

#
bool power_latched
bool external_control

# Faults
bool ready # Ready to execute commands
bool override_active  # Active override
bool override_other # Override from other system
bool fault # Fault of any kind
bool bad_crc # CRC error on most recent command

##### Start of slower secondary report #####

# Command source
CmdSrc cmd_src

# Degraded
bool degraded
bool degraded_cmd_type
bool degraded_comms
bool degraded_comms_dbw
bool degraded_comms_dbw_gateway
bool degraded_comms_dbw_steer
bool degraded_comms_dbw_brake
bool degraded_comms_dbw_thrtl
bool degraded_comms_vehicle
bool degraded_comms_vehicle_1
bool degraded_comms_vehicle_2
bool degraded_comms_actuator
bool degraded_comms_actuator_1
bool degraded_comms_actuator_2
bool degraded_internal
bool degraded_control_performance
bool degraded_param_mismatch
bool degraded_vehicle
bool degraded_vehicle_speed
bool degraded_actuator
bool degraded_gear_mismatch
bool degraded_power
bool degraded_calibration

# Faults
bool fault_power
bool fault_comms
bool fault_comms_dbw
bool fault_comms_dbw_gateway
bool fault_comms_dbw_steer
bool fault_comms_dbw_brake
bool fault_comms_dbw_thrtl
bool fault_comms_vehicle
bool fault_comms_vehicle_1
bool fault_comms_vehicle_2
bool fault_comms_actuator
bool fault_comms_actuator_1
bool fault_comms_actuator_2
bool fault_internal
bool fault_vehicle
bool fault_vehicle_speed
bool fault_actuator
bool fault_calibration
