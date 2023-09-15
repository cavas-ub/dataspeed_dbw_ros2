std_msgs/Header header

# Brake Pedal
float32 brake_torque_pedal   # Nm
float32 brake_torque_request # Nm
float32 brake_torque_actual  # Nm
Quality brake_pedal_qf

# Anti-lock Brakes
bool abs_active
bool abs_enabled

# Stability Control
bool stab_active
bool stab_enabled

# Traction Control
bool trac_active
bool trac_enabled
