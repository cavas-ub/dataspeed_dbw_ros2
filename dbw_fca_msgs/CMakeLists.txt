cmake_minimum_required(VERSION 3.0.2)
project(dbw_fca_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(DIRECTORY msg FILES
  BrakeCmd.msg
  BrakeInfoReport.msg
  BrakeReport.msg
  DoorCmd.msg
  FrontFanSpeed.msg
  FrontTemp.msg
  FuelLevelReport.msg
  Gear.msg
  GearCmd.msg
  GearNum.msg
  GearReject.msg
  GearReport.msg
  HighBeam.msg
  HvacButtonCmd.msg
  HvacSeat.msg
  Ignition.msg
  MiscCmd.msg
  Misc1Report.msg
  Misc2Report.msg
  SteeringCmd.msg
  SteeringReport.msg
  ThrottleCmd.msg
  ThrottleInfoReport.msg
  ThrottleReport.msg
  TirePressureReport.msg
  TurnSignal.msg
  VentMode.msg
  WatchdogCounter.msg
  WheelPositionReport.msg
  WheelSpeedReport.msg
  Wiper.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
  INCLUDE_DIRS include
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY bmr
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        FILES_MATCHING PATTERN "*.bmr"
)
