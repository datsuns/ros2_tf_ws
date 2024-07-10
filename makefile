default: build_ws


build_ws:
	colcon build

setup:
	rosdep install -i --from-path src --rosdistro humble -y

lsp:
	go-clsp-build -c lsp.yaml

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

turtle:
	ros2 run turtlesim turtlesim_node

create_pkg_using_tf:
	ros2 pkg create --build-type ament_cmake --license Apache-2.0 tf_pkg --dependencies rclcpp tf2 turtlesim
	colcon build --packages-select tf_pkg

run_pkg_using_tf:
	source ./install/setup.bash
	ros2 run tf_pkg publisher


.PNOHY: default build_ws lsp setup pub sub param mymsg
.PHONY: turtle create_pkg_using_tf run_pkg_using_tf
