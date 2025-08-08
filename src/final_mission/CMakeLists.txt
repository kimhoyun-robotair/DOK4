cmake_minimum_required(VERSION 3.8)
project(final_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

add_executable(fsm src/fsm.cpp)
ament_target_dependencies(fsm rclcpp rclcpp_lifecycle lifecycle_msgs std_msgs)
target_compile_features(fsm PUBLIC c_std_99 cxx_std_17)
install(TARGETS fsm DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  final_mission
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  flight_test/geometry_utils.py
  flight_test/vehicle_command.py
  flight_test/autonomous_flight_mission_offboard.py
  flight_test/combined_flight_landing_offboard.py
  flight_test/autonomous_precision_landing.py
  flight_test/test.py
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
