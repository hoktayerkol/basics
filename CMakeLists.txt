cmake_minimum_required(VERSION 3.8)
project(basics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(dependencies
	rclcpp
	std_msgs
	geometry_msgs
	sensor_msgs
)

add_executable(simple_node src/simple_node.cpp)
ament_target_dependencies(simple_node ${dependencies})

add_executable(logger src/logger.cpp)
ament_target_dependencies(logger ${dependencies})

add_executable(logger_class src/logger_as_class.cpp)
ament_target_dependencies(logger_class ${dependencies})

add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher ${dependencies})

add_executable(subcriber src/subcriber.cpp)
ament_target_dependencies(subcriber ${dependencies})

add_executable(pubsub src/pubsub.cpp)
ament_target_dependencies(pubsub ${dependencies})

add_executable(turtle_rand_sample src/turtle_rand_sample.cpp)
ament_target_dependencies(turtle_rand_sample ${dependencies})

install(TARGETS
	simple_node
	logger
	logger_class
	publisher
	subcriber
	pubsub
	turtle_rand_sample
	DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
