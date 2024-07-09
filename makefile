default: build_ws


build_ws:
	colcon build

setup:
	rosdep install -i --from-path src --rosdistro humble -y

pub:
	source ./install/setup.bash
	ros2 run test_pkg talker

sub:
	source ./install/setup.bash
	ros2 run test_pkg listener

param:
	ros2 param list
	ros2 param set /minimal_publisher my_parameter earth

mymsg:
	ros2 pkg create --build-type ament_cmake --license Apache-2.0 test_msgs

.PNOHY: default build_ws setup pub sub param mymsg
