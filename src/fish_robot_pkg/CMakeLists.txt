cmake_minimum_required(VERSION 3.8)
project(fish_robot_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_library(M_LIB m REQUIRED)

# =================================================================
# 1. UART Bridge Node
# =================================================================
add_executable(uart_bridge_node src/uart_bridge_node.cpp)
ament_target_dependencies(uart_bridge_node rclcpp std_msgs geometry_msgs sensor_msgs)
target_link_libraries(uart_bridge_node ${M_LIB})

# =================================================================
# 2. I2C Driver Node
# =================================================================
add_executable(i2c_driver_node src/i2c_driver_node.cpp)
ament_target_dependencies(i2c_driver_node rclcpp std_msgs geometry_msgs sensor_msgs)
target_link_libraries(i2c_driver_node ${M_LIB})

# =================================================================
# 3. State Estimation Node
# =================================================================
add_executable(state_estimation_node src/state_estimation_node.cpp)
ament_target_dependencies(state_estimation_node rclcpp std_msgs geometry_msgs sensor_msgs std_srvs)
target_link_libraries(state_estimation_node ${M_LIB})

# =================================================================
# 4. PID Control Node
# =================================================================
add_executable(pid_control_node src/pid_control_node.cpp)
ament_target_dependencies(pid_control_node rclcpp std_msgs geometry_msgs sensor_msgs)
target_link_libraries(pid_control_node ${M_LIB})

# =================================================================
# 5. RPM Driver Node
# =================================================================
add_executable(rpm_driver_node src/rpm_driver_node.cpp)
ament_target_dependencies(rpm_driver_node rclcpp std_msgs geometry_msgs sensor_msgs)
target_link_libraries(rpm_driver_node gpiodcxx) # [수정] 다시 gpiodcxx로 복구 완료!

# =================================================================
# 6. Data Logger Node
# =================================================================
add_executable(data_logger_node src/data_logger_node.cpp)
ament_target_dependencies(data_logger_node rclcpp std_msgs geometry_msgs sensor_msgs)
target_link_libraries(data_logger_node ${M_LIB})

# =================================================================
# 7. Auto Scenario Node 
# =================================================================
add_executable(auto_scenario_node src/auto_scenario_node.cpp)
ament_target_dependencies(auto_scenario_node rclcpp std_msgs geometry_msgs sensor_msgs)
target_link_libraries(auto_scenario_node ${M_LIB})

install(TARGETS
  uart_bridge_node
  i2c_driver_node
  state_estimation_node
  pid_control_node
  rpm_driver_node
  data_logger_node
  auto_scenario_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()