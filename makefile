default: build_ws


build_ws:
	colcon build

setup:
	rosdep install -i --from-path src --rosdistro humble -y

pub:
	source ./install/setup.bash
	ros2 run test_pkg talker

.PNOHY: default build_ws setup pub
