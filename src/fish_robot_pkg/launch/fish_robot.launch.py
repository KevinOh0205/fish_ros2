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

        # 8. EKF 자세 추정 노드 (Mahony와 병렬 - 비교/검증용, 제어에는 미사용)
        #    출력: /filtered/attitude_ekf, /filtered/ekf_status
        #    ※ /filtered/attitude 는 절대 발행하지 않는다. 발행자가 둘이 되면
        #      토픽이 200Hz가 되고, pid_control_node의 PID는 dt를 생략한 채
        #      100Hz를 가정하므로 게인이 통째로 틀어진다.
        #      확인: ros2 topic info /filtered/attitude --verbose | grep -c PUBLISHER  -> 1
        #    ※ respawn=False - 수동적 관측 노드다. 죽으면 저널에 스택트레이스
        #      하나를 남기고 조용히 있는 게 맞다. 분당 30회 재시작하며 DDS
        #      디스커버리를 돌리고 /dev/shm을 흘리는 게 아니다.
        #      덤으로 재빌드 없는 비상 정지가 된다 (프로세스만 kill).
        #
        #    use_mag=True 근거 (2026-08-07 실측):
        #      - 6축 yaw 드리프트 +0.155도/분 (합격 기준 2.0) 통과
        #      - 9축에서 교란 중 기울기 오차가 Mahony의 1/2.8 (1.36도 -> 0.49도)
        #      - 지자기 상실 시 500ms 만에 6축으로 떨어지고 자세 계단은 0.09도.
        #        복구 때도 roll/pitch가 0.01도밖에 안 움직인다 (Mahony는 2.7도)
        Node(
            package='fish_robot_pkg',
            executable='state_estimation_ekf_node',
            name='state_estimation_ekf_node',
            output='screen',
            respawn=False,
            parameters=[{
                'use_accel': True,
                'use_mag': True,
                'mag_yaw_sigma': 8.0,
            }]
        ),
    ])