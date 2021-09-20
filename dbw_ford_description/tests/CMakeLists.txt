### Unit tests
#
#   Only configured when CATKIN_ENABLE_TESTING is true.

# These dependencies are only needed for unit testing
find_package(roslaunch REQUIRED)

# Check all the launch/*.launch files (with rviz as a test dependency)
roslaunch_add_file_check(../launch USE_TEST_DEPENDENCIES)

