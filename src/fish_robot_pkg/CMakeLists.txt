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
find_package(Eigen3 3.4 REQUIRED)

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

# =================================================================
# 8. State Estimation EKF Node (신규 - Mahony와 병렬 동작하는 EKF 자세 추정)
# =================================================================
# std_srvs를 일부러 뺐다. 이 노드는 서비스를 광고하지 않는다 —
# /calibrate_mag 를 중복 광고하면 어느 서버가 응답할지 정의되지 않으며,
# 그 핸들러는 보정 파일을 truncate-write 한다. 의존성을 빼두면
# std_srvs 를 include 하는 순간 컴파일 에러가 나므로 규칙이 강제된다.
add_executable(state_estimation_ekf_node src/state_estimation_ekf_node.cpp)
ament_target_dependencies(state_estimation_ekf_node rclcpp std_msgs geometry_msgs sensor_msgs)
# ament_target_dependencies 가 plain 시그니처를 쓰므로 여기도 plain 이어야 한다
# (혼용하면 "The plain signature ... has already been used" 하드 에러).
target_link_libraries(state_estimation_ekf_node ${M_LIB} Eigen3::Eigen)

install(TARGETS
  uart_bridge_node
  i2c_driver_node
  state_estimation_node
  pid_control_node
  rpm_driver_node
  data_logger_node
  auto_scenario_node
  state_estimation_ekf_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 개발 도구 (로봇에 상주하지 않는다. 지자기 보정과 그 검증용)
#   ros2 run fish_robot_pkg magneto_cal.py
#   ros2 run fish_robot_pkg verify_spin.py
install(PROGRAMS
  scripts/magneto_cal.py
  scripts/verify_spin.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()