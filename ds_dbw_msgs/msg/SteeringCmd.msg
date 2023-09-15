std_msgs/Header header

# Steering Wheel
float32 cmd       # Command, changes with cmd_type
float32 cmd_rate  # Rate limit, rad/s, range 0 to 17.7, 0 = default, INFINITY = unlimited
float32 cmd_accel # Accel limit, rad/s^2, range 0 to 443, 0 = default, INFINITY = unlimited
uint8 cmd_type

# Enable
bool enable

# Clear latched overrides
bool clear

# Ignore future overrides
bool ignore

# Command types
uint8 CMD_NONE=0        # Command  Rate   Accel    Range        Note
uint8 CMD_TORQUE=1      # Nm       N/A    N/A      <depends>    Torque in Nm
uint8 CMD_ANGLE=2       # rad      rad/s  rad/s^2  <depends>    Angle in rad
uint8 CMD_CURVATURE=3   # 1/m      rad/s  rad/s^2  -0.2 to 0.2  Curvature in 1/m
uint8 CMD_YAW_RATE=4    # rad/s    rad/s  rad/s^2  -8.5 to 8.5  Yaw rate in rad/s
uint8 CMD_PERCENT=14    # %        rad/s  rad/s^2  -100 to 100  Percent of max steering wheel angle
uint8 CMD_CALIBRATE=15  # rad      N/A    N/A      <depends>    Calibrate current steering wheel angle to value in cmd
