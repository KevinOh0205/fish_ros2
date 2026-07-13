import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. UART 브릿지 노드 (6축 및 RC 데이터 허브)
        Node(
            package='fish_robot_pkg',
            executable='uart_bridge_node',
            name='uart_bridge_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0
        ),
        
        # 2. I2C 드라이버 노드 (지자기 및 3채널 압력 센서 전송망)
        Node(
            package='fish_robot_pkg',
            executable='i2c_driver_node',
            name='i2c_driver_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0
        ),

        # 3. 상태 추정 노드 (자이로 0점 조정 및 6축/9축 실시간 센서 융합)
        Node(
            package='fish_robot_pkg',
            executable='state_estimation_node',
            name='state_estimation_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0
        ),

        # 4. PID 제어 노드 (목표 각도 추종 PID 제어기 및 모터 믹싱)
        Node(
            package='fish_robot_pkg',
            executable='pid_control_node',
            name='pid_control_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0
        ),

        # 5. RPM 드라이버 노드 (하드웨어 외부 인터럽트 독립 수집가)
        Node(
            package='fish_robot_pkg',
            executable='rpm_driver_node',
            name='rpm_driver_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0
        ),

        # 6. 데이터 로거 노드 (예전 C 펌웨어 규격 매칭 일치형 실시간 영구 파일 로거)
        Node(
            package='fish_robot_pkg',
            executable='data_logger_node',
            name='data_logger_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0
        ),
        
        # 7. 자동 시나리오 노드 (신규 추가됨 - 자율 주행 시나리오 궤적 생성)
        Node(
            package='fish_robot_pkg',
            executable='auto_scenario_node',
            name='auto_scenario_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0
        ),
    ])