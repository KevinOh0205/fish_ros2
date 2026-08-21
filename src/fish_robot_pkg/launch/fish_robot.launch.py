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

        # 3. PID 제어 노드 (목표 각도 추종 PID 제어기 및 모터 믹싱)
        Node(
            package='fish_robot_pkg',
            executable='pid_control_node',
            name='pid_control_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0
        ),

        # 4. RPM 드라이버 노드 (하드웨어 외부 인터럽트 독립 수집가)
        Node(
            package='fish_robot_pkg',
            executable='rpm_driver_node',
            name='rpm_driver_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0
        ),

        # 5. 데이터 로거 노드 (예전 C 펌웨어 규격 매칭 일치형 실시간 영구 파일 로거)
        Node(
            package='fish_robot_pkg',
            executable='data_logger_node',
            name='data_logger_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0
        ),

        # 6. 자동 시나리오 노드 (자율 주행 시나리오 궤적 생성)
        Node(
            package='fish_robot_pkg',
            executable='auto_scenario_node',
            name='auto_scenario_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0
        ),

        # 7. EKF 자세 추정 노드 — 제어 경로 추정기 (구 state_estimation_node 완전 대체, 2026-08-17)
        #    구 state_estimation_node 블록은 여기서 제거됐다. 같이 띄우면
        #    /filtered/attitude 발행자가 둘이 되어 pid_control_node 가 200Hz로 돌고
        #    (dt 생략 PID라 게인이 통째로 틀어진다) 서로 다른 추정치가 번갈아 들어가며,
        #    /calibrate_mag 서비스도 중복 광고가 된다. 수동 실행으로도 함께 띄우지 말 것.
        #    확인: ros2 topic info /filtered/attitude --verbose | grep -c PUBLISHER  -> 1
        #    롤백: 이 파일의 대체 커밋을 revert 하고 fish-robot 서비스 재시작.
        #
        #    출력: /filtered/attitude (제어 — ±5000 모드 인코딩 + yaw 영점),
        #          /filtered/attitude_ekf (순수값, 비교/진단), /filtered/ekf_status,
        #          /sensor/pressure_calibrated, /CH_IMU, /CH_MEGNET
        #    서비스: /calibrate_mag (magneto_cal.py 실행/조기 종료 토글 — min/max 폐기)
        #
        #    ※ respawn=True — 제어 경로에 들어갔으므로 다른 노드와 동일하게 되살린다.
        #      (관측 전용이던 시절의 respawn=False 근거는 대체와 함께 소멸)
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
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'use_accel': True,
                'use_mag': True,
                'mag_yaw_sigma': 8.0,
            }]
        ),

        # 8. 수심/속도 추정 노드 — 압력 3채널에서 수심(현재)과 전진 속도(다음 단계)
        #    출력: /filtered/hydro [depth, speed, q, q_raw, dP_hydro, static, pitch, flags]
        #    서비스: /hydro/zero_atm (대기압 영점 재포착)
        #
        #    ※ 관측 전용이다 — 어떤 제어 루프도 아직 /filtered/hydro 를 구독하지 않는다.
        #    ※ 수심은 좌우 정압 포트의 산술평균이라 **자세와 무관하게 정확**하다
        #      (압력이 깊이에 선형이므로 두 점의 평균 = 기하학적 중점의 압력).
        #      그래서 IMU가 빠져 있어도, 지자기 보정이 없어도 수심은 나온다.
        #    ※ 포트 역할(front/left/right)은 여기가 아니라 log_csv/port_map.txt 에서
        #      읽는다. 센서 지문(PROM) 대조와 한 곳에 묶어두기 위해서다.
        Node(
            package='fish_robot_pkg',
            executable='hydro_estimator_node',
            name='hydro_estimator_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'water_density': 997.0,      # 담수 25도. 해수는 1025
                'atm_warmup_sec': 180.0,     # EKF의 PRESSURE_WARMUP_SEC 과 같은 값
            }]
        ),
    ])
