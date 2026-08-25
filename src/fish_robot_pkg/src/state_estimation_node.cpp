// =============================================================================
//  state_estimation_node.cpp
//
//  역할 : 여러 원시 센서를 융합해 로봇의 자세(Roll/Pitch/Yaw)를 추정하고,
//         각종 영점 보정(자이로/압력/지자기)과 버튼 UI를 총괄하는 두뇌 노드
//
//   [입력] /raw/imu_6dof         (Imu)                  가속도 3축 + 각속도 3축, 100Hz
//          /raw/magnetometer     (Vector3)              지자기 3축, 100Hz
//          /rc/status            (Int32MultiArray[5])   버튼 입력
//          /sensor/pressure_raw  (Float32MultiArray[6]) 압력 3채널 + 온도 3채널
//   [출력] /filtered/attitude          (Vector3)              자세 + 모드 플래그
//          /sensor/pressure_calibrated (Float32MultiArray[3]) 대기압 영점 제거된 수압
//          /CH_IMU                     (Imu)     축 변환된 IMU  (검증용, 제어에 미사용)
//          /CH_MEGNET                  (Vector3) 축 변환된 지자기 (검증용, 제어에 미사용)
//   [서비스] /calibrate_mag (std_srvs/Trigger)  지자기 캘리브레이션 시작/종료 토글
//
//  핵심 알고리즘 : Mahony 상보 필터
//    자이로는 단기적으로 정확하지만 시간이 지날수록 드리프트하고, 가속도계/지자기계는
//    노이즈가 많지만 장기적으로는 중력/자북이라는 절대 기준을 준다. Mahony 필터는
//    "자이로 적분으로 자세를 진행시키되, 가속도·지자기가 알려주는 방향과의 오차를
//    피드백(Kp, Ki)으로 계속 되먹임"해서 양쪽의 장점만 취한다.
//    지자기가 살아있으면 9축(update_9dof_mahony), 죽었거나 캘리브레이션 중이면
//    자동으로 6축(update_6dof_mahony)으로 떨어진다 — Yaw 절대 기준만 포기하는 셈.
//
//  ※ 좌표계 규약 (필터 내부 기준) : REP-103 FLU — +X 전진 / +Y 왼쪽 / +Z 위
//     세 센서가 물리적으로 서로 다른 방향을 보고 있으므로, 필터에 넣기 전에
//     각각 이 좌표계로 회전시킨다. 상세 근거는 아래 두 콜백의 주석 참조.
//       - nRF52840 IMU (가속도·자이로) -> imu_callback
//       - AK8963 (지자기)              -> mag_callback
//
//  ※ 모드 전달 규약 : 자동 모드일 때 attitude.x(roll)에 +5000을 더해 발행한다.
//     별도 모드 토픽 없이 자세 토픽으로 모드를 함께 실어 나르며,
//     pid_control_node / auto_scenario_node / data_logger_node가 동일 규칙으로 디코딩한다.
//
//  ※ 버튼 UI 요약 (조종기 버튼, /rc/status 100Hz 기준 1틱 = 10ms)
//     btn1 짧게(30ms~1초) : 자동/수동 모드 토글
//     btn1 길게(3초)      : 대기압 영점 재조정
//     btn2 짧게(30ms~1초) : 현재 방향을 Yaw 0도로 정렬
//     btn2 길게(3초)      : 지자기 캘리브레이션 시작
// =============================================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <stdio.h>
#include <math.h>
#include <cmath>       // std::isfinite (dt 실측값 검증)
#include <string.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <vector>
#include <sys/stat.h>
#include <sys/types.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Mahony 필터 피드백 게인
//   Kp : 가속도/지자기가 지적한 오차를 얼마나 빠르게 반영할지 (크면 빠르지만 진동에 민감)
//   Ki : 자이로 바이어스 잔차를 서서히 없애는 적분 게인
#define Kp 2.0f
#define Ki 0.005f

#define GYRO_CALIB_SAMPLES 1000      // 100Hz 기준 10초
#define PRESSURE_CALIB_SAMPLES 100   // 압력 11Hz 기준 약 9초
#define MAG_CALIB_MAX_SAMPLES 1000   // 100Hz 기준 10초 (그 동안 로봇을 8자로 회전)

// [N2] 압력 영점 캡처 전 웜업 대기(초). 웜업 시간 드리프트의 초반 급강하를 지나
// baseline을 잡기 위함. 초반 급드리프트(ch0 ~250초)를 넘기는 값.
#define PRESSURE_WARMUP_SEC 300

class StateEstimationNode : public rclcpp::Node {
public:
    StateEstimationNode() : Node("state_estimation_node"),
                            // q0~q3 = 자세 쿼터니언, (1,0,0,0)은 "회전 없음" 초기 자세
                            dt_(0.01f),
                            last_imu_time_(0, 0, RCL_ROS_TIME), have_prev_imu_time_(false),
                            dt_clamp_count_(0),
                            q0_(1.0f), q1_(0.0f), q2_(0.0f), q3_(0.0f),
                            gyro_calibrated_(false), gyro_sample_count_(0),
                            press_calibrated_(false), press_sample_count_(0),
                            is_mag_calibrating_(false), mag_calib_sample_count_(0), last_progress_(-1),
                            // 지자기 하드아이언/소프트아이언 보정 계수의 기본값.
                            // 아래 load_mag_calibration_file()이 저장된 파일을 찾으면 덮어쓴다.
                            mag_bias_x_(8.025f), mag_bias_y_(21.75f), mag_bias_z_(17.625f),
                            mag_scale_x_(0.858f), mag_scale_y_(0.938f), mag_scale_z_(1.301f),
                            latest_mx_(0.0f), latest_my_(0.0f), latest_mz_(0.0f),
                            raw_yaw_(0.0f), yaw_offset_(0.0f),
                            is_auto_mode_(false), btn1_press_count_(0),
                            btn1_counter_(0), btn2_counter_(0),
                            prev_btn1_(0), prev_btn2_(0),
                            right_btn_pressed_(false),
                            button_press_start_time_(0, 0, RCL_ROS_TIME),
                            left_btn_pressed_(false),
                            left_btn_press_start_time_(0, 0, RCL_ROS_TIME),
                            // [수정] 초기화 리스트에 빠져 있어 첫 /rc/status 수신 전까지
                            // 값이 불확정이었다. 기동 직후 btn1을 누르면 모드 토글이
                            // 엉뚱하게 발동하거나 씹힐 수 있었다.
                            btn1_long_processed_(false),
                            btn2_long_processed_(false),
                            last_mag_time_(0, 0, RCL_ROS_TIME),
                            mag_timeout_(std::chrono::milliseconds(500))   // 지자기 500ms 무수신 = 죽은 것으로 간주
    {
        RCLCPP_INFO(this->get_logger(), "=== [State Estimation] 센서 통합 보정 노드 초기화 ===");

        eInt_[0] = 0.0f; eInt_[1] = 0.0f; eInt_[2] = 0.0f;
        gyro_bias_[0] = 0.0f; gyro_bias_[1] = 0.0f; gyro_bias_[2] = 0.0f;

        for(int i = 0; i < 3; i++) {
            press_sum_[i] = 0.0f;   press_offset_[i] = 0.0f;
            temp_sum_[i] = 0.0f;    temp_offset_[i] = 0.0f;
            press_valid_count_[i] = 0;  press_ch_alive_[i] = false;
            // min/max를 반대 극단으로 초기화해 첫 샘플이 무조건 양쪽을 갱신하게 한다
            mag_max_[i] = -99999.0f;
            mag_min_[i] = 99999.0f;
        }

        // [N2 결론] 20분 특성화(151044 로그) 결과, 압력 영점 드리프트의 원인은
        // 온도가 아니라 웜업 경과 "시간"이었다 (P vs 시간 R²≈0.95, 온도는 20분간 ~0.1°C로
        // 거의 정지). 온도 비례 보정은 무효할 뿐 아니라 온도센서 노이즈를 증폭시키므로
        // 계수를 0으로 고정한다. 드리프트 완화는 웜업 후 재영점(btn1 롱프레스)으로 처리.
        drift_coeff_[0] = 0.0f; drift_coeff_[1] = 0.0f; drift_coeff_[2] = 0.0f;
        last_mag_time_ = this->now();
        node_start_time_ = this->now();
        load_mag_calibration_file();

        attitude_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/filtered/attitude", 10);
        pressure_cal_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/sensor/pressure_calibrated", 10);

        // 축 변환 검증용 발행 토픽. 필터에 실제로 들어가는 값을 그대로 내보내므로
        // /raw/* 와 나란히 echo 하면 회전이 의도대로 걸렸는지 눈으로 대조할 수 있다.
        //   /CH_IMU    <- /raw/imu_6dof     를 FLU로 회전한 값 (단위 그대로: g, 도/초)
        //   /CH_MEGNET <- /raw/magnetometer 를 보정 후 FLU로 재배치한 값
        ch_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/CH_IMU", 10);
        ch_magnet_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/CH_MEGNET", 10);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/raw/imu_6dof", 10, std::bind(&StateEstimationNode::imu_callback, this, std::placeholders::_1));
        mag_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/raw/magnetometer", 10, std::bind(&StateEstimationNode::mag_callback, this, std::placeholders::_1));
        rc_status_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/rc/status", 10, std::bind(&StateEstimationNode::rc_status_callback, this, std::placeholders::_1));
        pressure_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/sensor/pressure_raw", 10, std::bind(&StateEstimationNode::pressure_callback, this, std::placeholders::_1));

        // 버튼 없이 터미널에서도 지자기 캘리브레이션을 걸 수 있게 서비스로도 노출
        //   ros2 service call /calibrate_mag std_srvs/srv/Trigger
        mag_calib_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/calibrate_mag", std::bind(&StateEstimationNode::handle_mag_calibration, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    // 쿼터니언 -> 오일러각(도) 변환. ZYX 순서 기준.
    void get_euler_angles(float *roll, float *pitch, float *yaw) {
        *roll = atan2f(q0_*q1_ + q2_*q3_, 0.5f - q1_*q1_ - q2_*q2_) * (180.0f / M_PI);
        // asinf 정의역은 [-1,1]. 쿼터니언 미세 비정규화로 인자가 이 범위를 벗어나면
        // NaN이 나오므로(±90° 부근) 반드시 클램프한다 (N6: NaN 원천 차단).
        float sin_pitch = std::clamp(-2.0f * (q1_*q3_ - q0_*q2_), -1.0f, 1.0f);
        *pitch = asinf(sin_pitch) * (180.0f / M_PI);
        *yaw = atan2f(q1_*q2_ + q0_*q3_, 0.5f - q2_*q2_ - q3_*q3_) * (180.0f / M_PI);
        raw_yaw_ = *yaw;   // 헤딩 영점(btn2 짧게) 계산에 쓸 보정 전 원시 yaw 보관
    }

    // ---- 9축 Mahony 필터 (가속도 + 자이로 + 지자기) ----
    // 지자기가 자북이라는 절대 기준을 주므로 Yaw 드리프트까지 잡힌다.
    void update_9dof_mahony(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
        float recipNorm;
        // 쿼터니언 곱 항들을 미리 계산해 아래 회전 변환에서 재사용 (연산량 절감)
        float q0q0 = q0_*q0_, q0q1 = q0_*q1_, q0q2 = q0_*q2_, q0q3 = q0_*q3_;
        float q1q1 = q1_*q1_, q1q2 = q1_*q2_, q1q3 = q1_*q3_;
        float q2q2 = q2_*q2_, q2q3 = q2_*q3_, q3q3 = q3_*q3_;

        // 가속도가 전부 0이면 (센서 이상) 보정을 건너뛰고 자이로 적분만 수행
        if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
            // 크기는 버리고 방향만 쓴다 -> 정규화
            recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
            ax *= recipNorm; ay *= recipNorm; az *= recipNorm;
            recipNorm = 1.0f / sqrtf(mx * mx + my * my + mz * mz);
            mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

            // 측정된 지자기를 지구 좌표계로 돌린 뒤, 수평 성분을 모두 bx(북)로 몰아넣어
            // 기준 자기장 방향을 만든다. 자기 편각/복각의 영향을 정규화하는 과정.
            float hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
            float hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
            float bx = sqrtf(hx * hx + hy * hy);
            float bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

            // 현재 추정 자세가 예측하는 중력 방향(halfv*)과 자기장 방향(halfw*)
            float halfvx = q1_ * q3_ - q0_ * q2_;
            float halfvy = q0_ * q1_ + q2_ * q3_;
            float halfvz = q0q0 - 0.5f + q3q3;
            // halfw = R^T * (bx, 0, bz) / 2. 각 성분은 회전행렬의 해당 열에서 온다.
            //   x <- (R00, R20) , y <- (R01, R21) , z <- (R02, R22)
            // [수정 2026-08-06] halfwy의 bz 계수가 R22(0.5-q1q1-q3q3)로 잘못 적혀 있었다.
            //   R21 = q0q1+q2q3 (= halfvy) 이 맞다. 단위 쿼터니언으로 검산하면 halfwy는
            //   0이어야 하는데 옛 식은 0.5*bz를 내놓는다. 지자기 보정이 엉망일 때는 bz가
            //   작아서 증상이 작았고(기울기 오차 5도), 복각이 제대로 잡히자 bz가 5배로
            //   커지면서 14도까지 벌어졌다. 보정이 좋아질수록 더 아파지는 종류의 버그다.
            float halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1_ * q3_ - q0_ * q2_);
            float halfwy = bx * (q1_ * q2_ - q0_ * q3_) + bz * (q0_ * q1_ + q2_ * q3_);
            float halfwz = bx * (q0_ * q2_ + q1_ * q3_) + bz * (0.5f - q1q1 - q2q2);

            // 오차 = (측정 방향) x (예측 방향). 외적이므로 두 방향이 일치하면 0이 된다.
            // 중력 오차와 자기장 오차를 더해 한 번에 보정한다.
            float ex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
            float ey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
            float ez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

            // I항: 오차를 누적해 자이로의 잔여 바이어스를 서서히 상쇄
            if (Ki > 0.0f) {
                eInt_[0] += ex * dt_; eInt_[1] += ey * dt_; eInt_[2] += ez * dt_;
                gx += Ki * eInt_[0]; gy += Ki * eInt_[1]; gz += Ki * eInt_[2];
            }
            // P항: 오차를 자이로 값에 직접 더해 자세를 끌어당긴다
            gx += Kp * ex; gy += Kp * ey; gz += Kp * ez;
        }

        // 보정된 각속도로 쿼터니언을 1스텝 적분 (q_dot = 0.5 * q ⊗ omega)
        gx *= (0.5f * dt_); gy *= (0.5f * dt_); gz *= (0.5f * dt_);
        float qa = q0_, qb = q1_, qc = q2_;   // 갱신 도중 덮어써지므로 원본 백업
        q0_ += (-qb * gx - qc * gy - q3_ * gz); q1_ += (qa * gx + qc * gz - q3_ * gy);
        q2_ += (qa * gy - qb * gz + q3_ * gx); q3_ += (qa * gz + qb * gy - qc * gx);

        // 적분 오차로 쿼터니언 크기가 1에서 벗어나므로 매 스텝 재정규화
        recipNorm = 1.0f / sqrtf(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
        q0_ *= recipNorm; q1_ *= recipNorm; q2_ *= recipNorm; q3_ *= recipNorm;
    }

    // ---- 6축 Mahony 필터 (가속도 + 자이로) ----
    // 지자기 없이 중력만으로 보정하므로 Roll/Pitch는 잡히지만 Yaw는 서서히 드리프트한다.
    // 지자기 센서가 없거나 캘리브레이션 중일 때의 폴백 경로.
    void update_6dof_mahony(float gx, float gy, float gz, float ax, float ay, float az) {
        float recipNorm; float halfvx, halfvy, halfvz; float halfex, halfey, halfez;

        if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
            recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
            ax *= recipNorm; ay *= recipNorm; az *= recipNorm;
            // 현재 자세가 예측하는 중력 방향
            halfvx = q1_ * q3_ - q0_ * q2_; halfvy = q0_ * q1_ + q2_ * q3_; halfvz = q0_ * q0_ - 0.5f + q3_ * q3_;
            // 측정 중력과 예측 중력의 외적 = 보정해야 할 회전 오차
            halfex = (ay * halfvz - az * halfvy); halfey = (az * halfvx - ax * halfvz); halfez = (ax * halfvy - ay * halfvx);

            if (Ki > 0.0f) {
                eInt_[0] += halfex * dt_; eInt_[1] += halfey * dt_; eInt_[2] += halfez * dt_;
                gx += Ki * eInt_[0]; gy += Ki * eInt_[1]; gz += Ki * eInt_[2];
            }
            gx += Kp * halfex; gy += Kp * halfey; gz += Kp * halfez;
        }

        gx *= (0.5f * dt_); gy *= (0.5f * dt_); gz *= (0.5f * dt_);
        float qa = q0_, qb = q1_, qc = q2_;
        q0_ += (-qb * gx - qc * gy - q3_ * gz); q1_ += (qa * gx + qc * gz - q3_ * gy);
        q2_ += (qa * gy - qb * gz + q3_ * gx); q3_ += (qa * gz + qb * gy - qc * gx);

        recipNorm = 1.0f / sqrtf(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
        q0_ *= recipNorm; q1_ *= recipNorm; q2_ *= recipNorm; q3_ *= recipNorm;
    }

    // 압력 센서 처리: 부팅 후 대기압을 기준(영점)으로 잡고, 이후로는 그 차이만 발행한다.
    // => 발행값은 사실상 "수심에 의한 게이지 압력"이 된다.
    void pressure_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 6) return;

        float raw_p[3] = { msg->data[0], msg->data[1], msg->data[2] };
        float raw_t[3] = { msg->data[3], msg->data[4], msg->data[5] };

        // ================= [영점 캘리브레이션 단계] =================
        if (!press_calibrated_) {
            // [N2] 웜업 대기: 전원 직후 압력이 시간에 따라 크게 드리프트하므로,
            // 초반 급강하 구간을 지난 뒤에 대기압 영점을 캡처한다.
            double warmup_elapsed = (this->now() - node_start_time_).seconds();
            if (warmup_elapsed < PRESSURE_WARMUP_SEC) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 30000,
                    "... 압력 센서 웜업 대기 중 (%.0f / %d초). 이후 영점을 자동 캡처합니다 ...",
                    warmup_elapsed, PRESSURE_WARMUP_SEC);
                return;
            }

            // 채널별로 유효한(>=100mbar) 샘플만 누적한다. 죽었거나 mux 실패로 0이 온
            // 채널은 건너뛰어, ch0에 의존하지 않고 살아있는 센서만으로 보정을 완료한다.
            bool any_valid = false;
            for (int i = 0; i < 3; i++) {
                if (raw_p[i] >= 100.0f) {
                    press_sum_[i] += raw_p[i];
                    temp_sum_[i] += raw_t[i];
                    press_valid_count_[i]++;
                    any_valid = true;
                }
            }
            if (!any_valid) return;   // 세 채널 모두 무효면 대기

            press_sample_count_++;

            if (press_sample_count_ % 20 == 0) {
                RCLCPP_INFO(this->get_logger(), "... 압력 센서 대기압 0점 수집 중 (%d / %d) ...", press_sample_count_, PRESSURE_CALIB_SAMPLES);
            }

            if (press_sample_count_ >= PRESSURE_CALIB_SAMPLES) {
                for (int i = 0; i < 3; i++) {
                    // 절반 이상 유효 샘플이 모인 채널만 살아있는 것으로 인정한다.
                    if (press_valid_count_[i] >= PRESSURE_CALIB_SAMPLES / 2) {
                        press_offset_[i] = press_sum_[i] / press_valid_count_[i];   // 대기압 평균 = 영점
                        temp_offset_[i] = temp_sum_[i] / press_valid_count_[i];     // 영점 캡처 당시 온도
                        press_ch_alive_[i] = true;
                    } else {
                        press_ch_alive_[i] = false;
                    }
                }
                press_calibrated_ = true;
                RCLCPP_INFO(this->get_logger(), "=========================================");
                RCLCPP_INFO(this->get_logger(), "[OK] 압력 센서 기준 영점 세팅 완료 (물에 넣으셔도 됩니다)");
                RCLCPP_INFO(this->get_logger(), "     살아있는 채널: [ch0:%s ch1:%s ch2:%s]",
                            press_ch_alive_[0] ? "O" : "X", press_ch_alive_[1] ? "O" : "X", press_ch_alive_[2] ? "O" : "X");
                RCLCPP_INFO(this->get_logger(), "=========================================");
            }
            return;
        }

        // ================= [정상 동작 단계] =================
        auto cal_msg = std_msgs::msg::Float32MultiArray();
        cal_msg.data.resize(3);
        for (int i = 0; i < 3; i++) {
            if (press_ch_alive_[i]) {
                // drift_coeff_는 위 N2 결론에 따라 0으로 고정되어 있어 실질적으로 무효항이다.
                // (온도 보정이 다시 필요해질 때를 위해 구조만 남겨둔 상태)
                float temp_diff = raw_t[i] - temp_offset_[i];
                float drift_correction = temp_diff * drift_coeff_[i];
                cal_msg.data[i] = raw_p[i] - press_offset_[i] - drift_correction;
            } else {
                cal_msg.data[i] = 0.0f;   // 죽은 채널은 0으로 표시
            }
        }
        dynamic_pressures_ = cal_msg.data;
        pressure_cal_pub_->publish(cal_msg);
    }

    // 지자기 보정 계수를 파일에서 읽는다. 파일이 없으면 생성자의 기본값을 그대로 쓴다.
    void load_mag_calibration_file() {
        const char* home_dir = getenv("HOME");
        std::string file_path = home_dir ? (std::string(home_dir) + "/ros2_ws/config/mag_calib_params.txt") : "./config/mag_calib_params.txt";
        std::ifstream infile(file_path);
        if (infile.is_open()) {
            infile >> mag_bias_x_ >> mag_bias_y_ >> mag_bias_z_ >> mag_scale_x_ >> mag_scale_y_ >> mag_scale_z_;
            infile.close();
        }
    }

    // 캘리브레이션 결과를 파일로 저장해 다음 부팅에도 유지되게 한다.
    void save_mag_calibration_file() {
        const char* home_dir = getenv("HOME");
        std::string dir_path = home_dir ? (std::string(home_dir) + "/ros2_ws/log_csv") : "./log_csv";
        std::string file_path = dir_path + "/mag_calib_params.txt";
        struct stat st;
        if (stat(dir_path.c_str(), &st) == -1) mkdir(dir_path.c_str(), 0775);
        std::ofstream outfile(file_path, std::ios::trunc);
        if (outfile.is_open()) {
            outfile << mag_bias_x_ << " " << mag_bias_y_ << " " << mag_bias_z_ << "\n" << mag_scale_x_ << " " << mag_scale_y_ << " " << mag_scale_z_ << "\n";
            outfile.close();
        }
    }

    // 조종기 버튼 2개의 짧게/길게 누름을 판정해 각종 기능을 트리거한다.
    // /rc/status가 100Hz로 오므로 카운터 1틱 = 10ms.
    void rc_status_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) return;
        int32_t rxPkt_btn1 = msg->data[0];
        int32_t rxPkt_btn2 = msg->data[1];

        // 통신두절 시 nRF는 버튼을 255로 보낸다. 이때 1→255 전이를 '뗌(release)'으로
        // 오인해 모드 토글·헤딩 영점이 스퍼리어스하게 트리거되는 것을 막는다(N4).
        // 카운터와 직전 상태를 중립으로 리셋하고 엣지 판정을 건너뛴다.
        if ((rxPkt_btn1 != 0 && rxPkt_btn1 != 1) || (rxPkt_btn2 != 0 && rxPkt_btn2 != 1)) {
            btn1_counter_ = 0; btn1_long_processed_ = false; prev_btn1_ = 0;
            btn2_counter_ = 0; btn2_long_processed_ = false; prev_btn2_ = 0;
            return;
        }

        // 🟥 버튼 1 로직
        if (rxPkt_btn1 == 1) {
            btn1_counter_++;
            if (btn1_counter_ == 300) {   // 정확히 300틱(3초)에서 한 번만 발동 (>=가 아니라 ==)
                btn1_long_processed_ = true;   // 뗄 때 짧은 누름으로 중복 처리되지 않게 표시
                RCLCPP_WARN(this->get_logger(), "======================================================");
                RCLCPP_WARN(this->get_logger(), "[System] 버튼 1 롱프레스 감지! 대기압 영점을 재조정합니다.");
                RCLCPP_WARN(this->get_logger(), "======================================================");
                // 압력 캘리브레이션 상태를 통째로 리셋 -> pressure_callback이 다시 수집 모드로
                // (웜업 대기는 node_start_time_ 기준이라 이미 지났으면 바로 재수집한다)
                press_calibrated_ = false; press_sample_count_ = 0;
                for(int i=0; i<3; i++) { press_sum_[i]=0.0f; temp_sum_[i]=0.0f; press_valid_count_[i]=0; }
            }
        } else {
            if (prev_btn1_ == 1) {   // Falling Edge = 손을 뗀 순간
                // 30ms 이상 1초 미만이고 롱프레스로 처리되지 않았을 때만 짧은 누름
                if (btn1_counter_ > 3 && btn1_counter_ < 100 && !btn1_long_processed_) {
                    is_auto_mode_ = !is_auto_mode_;
                    btn1_press_count_++;
                    RCLCPP_INFO(this->get_logger(), "======================================================");
                    if(is_auto_mode_) {
                        RCLCPP_INFO(this->get_logger(), "[System] 로봇이 자동 모드(AUTO)로 전환되었습니다! (카운트: %d)", btn1_press_count_);
                    } else {
                        RCLCPP_INFO(this->get_logger(), "[System] 로봇이 수동 모드(MANUAL)로 전환되었습니다! (카운트: %d)", btn1_press_count_);
                    }
                    RCLCPP_INFO(this->get_logger(), "======================================================");
                }
            }
            btn1_counter_ = 0; btn1_long_processed_ = false;
        }
        prev_btn1_ = rxPkt_btn1;

        // 🟦 버튼 2 로직
        if (rxPkt_btn2 == 1) {
            btn2_counter_++;
            if (btn2_counter_ == 300) {
                if (!is_mag_calibrating_) {
                    // 지자기 센서가 죽어 있으면 캘리브레이션을 해봐야 의미가 없으므로 거절
                    if ((this->now() - last_mag_time_) >= mag_timeout_) {
                        RCLCPP_ERROR(this->get_logger(), "======================================================");
                        RCLCPP_ERROR(this->get_logger(), "[System] 지자기 센서(AK8963) 응답이 없습니다! 캘리브레이션을 취소합니다.");
                        RCLCPP_ERROR(this->get_logger(), "======================================================");
                    } else {
                        // 서비스 핸들러를 직접 호출한다 (버튼과 서비스가 같은 진입점을 쓰도록)
                        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
                        auto res = std::make_shared<std_srvs::srv::Trigger::Response>();
                        RCLCPP_WARN(this->get_logger(), "======================================================");
                        RCLCPP_WARN(this->get_logger(), "[System] 3초 누름 감지! 지자기 센서 캘리브레이션을 시작합니다. (로봇을 이리저리 회전시켜주세요)");
                        RCLCPP_WARN(this->get_logger(), "======================================================");
                        handle_mag_calibration(req, res);
                    }
                    btn2_long_processed_ = true;
                }
            }
        } else {
            if (prev_btn2_ == 1) {
                if (btn2_counter_ > 3 && btn2_counter_ < 100 && !btn2_long_processed_) {
                    // 헤딩 영점: 현재 바라보는 방향을 Yaw 0도로 삼는다.
                    // 자북이 아니라 "출발 방향" 기준으로 조종하고 싶을 때 사용.
                    yaw_offset_ = raw_yaw_;
                    RCLCPP_WARN(this->get_logger(), "======================================================");
                    RCLCPP_WARN(this->get_logger(), "[System] 헤딩(Yaw) 영점 정렬! 현재 방향을 0도로 설정합니다. (Offset: %.2f)", yaw_offset_);
                    RCLCPP_WARN(this->get_logger(), "======================================================");
                }
            }
            btn2_counter_ = 0; btn2_long_processed_ = false;
        }
        prev_btn2_ = rxPkt_btn2;
    }

    // 지자기 캘리브레이션 토글 진입점 (버튼 롱프레스 / ROS 서비스 공용).
    // 처음 호출 = 수집 시작, 수집 중 재호출 = 즉시 종료 및 계수 확정.
    void handle_mag_calibration(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        (void)req;   // Trigger는 요청 필드가 없다 — 미사용 경고 방어
        // 센서가 죽어 있으면 시작하지 않는다
        if (!is_mag_calibrating_ && (this->now() - last_mag_time_) >= mag_timeout_) {
            res->success = false;
            return;
        }
        if (!is_mag_calibrating_) {
            is_mag_calibrating_ = true; mag_calib_sample_count_ = 0; last_progress_ = -1;
            mag_max_[0]=mag_max_[1]=mag_max_[2]=-99999.0f; mag_min_[0]=mag_min_[1]=mag_min_[2]=99999.0f;
            res->success = true;
        } else { finalize_mag_calibration(); res->success = true; }
    }

    // 수집된 min/max로 하드아이언(bias)과 소프트아이언(scale) 계수를 산출한다.
    //   bias  = 각 축 최대/최소의 중점 -> 주변 금속이 만든 자기장 옵셋 제거
    //   scale = 세 축 반경의 평균 / 해당 축 반경 -> 찌그러진 타원을 구에 가깝게 편다
    void finalize_mag_calibration() {
        is_mag_calibrating_ = false;
        mag_bias_x_ = (mag_max_[0] + mag_min_[0]) / 2.0f; mag_bias_y_ = (mag_max_[1] + mag_min_[1]) / 2.0f; mag_bias_z_ = (mag_max_[2] + mag_min_[2]) / 2.0f;
        float dx = (mag_max_[0] - mag_min_[0]) / 2.0f, dy = (mag_max_[1] - mag_min_[1]) / 2.0f, dz = (mag_max_[2] - mag_min_[2]) / 2.0f, avg = (dx+dy+dz)/3.0f;

        // 0으로 나누기 방지 — 해당 축을 못 돌렸으면 기존 계수를 유지한다
        if (dx != 0.0f) { mag_scale_x_ = avg / dx; }
        if (dy != 0.0f) { mag_scale_y_ = avg / dy; }
        if (dz != 0.0f) { mag_scale_z_ = avg / dz; }
        save_mag_calibration_file();

        RCLCPP_INFO(this->get_logger(), "=========================================");
        RCLCPP_INFO(this->get_logger(), "[OK] 지자기 캘리브레이션 완료 및 저장 성공!");
        RCLCPP_INFO(this->get_logger(), "=========================================");
    }

    // (수정됨: 지자기 캘리브레이션 진행률 출력 추가)
    void mag_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        last_mag_time_ = this->now();   // 생존 확인용 타임스탬프 (500ms 넘으면 6축으로 폴백)
        float raw_mx = msg->x, raw_my = msg->y, raw_mz = msg->z;
        if (is_mag_calibrating_) {
            // 캘리브레이션 중에는 축별 최대/최소만 갱신한다 (로봇을 8자로 돌려주는 동안)
            mag_max_[0] = std::max(mag_max_[0], raw_mx); mag_max_[1] = std::max(mag_max_[1], raw_my); mag_max_[2] = std::max(mag_max_[2], raw_mz);
            mag_min_[0] = std::min(mag_min_[0], raw_mx); mag_min_[1] = std::min(mag_min_[1], raw_my); mag_min_[2] = std::min(mag_min_[2], raw_mz);

            mag_calib_sample_count_++;

            if (mag_calib_sample_count_ % 200 == 0) {
                RCLCPP_INFO(this->get_logger(), "... 지자기 영점 수집 중 (%d / %d) ...", mag_calib_sample_count_, MAG_CALIB_MAX_SAMPLES);
            }

            if (mag_calib_sample_count_ >= MAG_CALIB_MAX_SAMPLES) {
                finalize_mag_calibration();   // 10초 지나면 자동 종료
            }
        }
        // 보정 계수를 적용한 값을 보관 -> imu_callback의 9축 융합에서 사용
        //
        // ── 지자기 좌표계 ────────────────────────────────────────────────────
        // [보드 장착] MPU9250 보드는 실크스크린의 "가속도 축"이 로봇 기준으로
        //     +X = 전진, +Y = 왼쪽, +Z = 위 가 되도록 장착한다.
        //     => 보드의 가속도 좌표계가 곧 로봇 FLU 좌표계다.
        //     (AK8963 자체 축은 기판에 표기되지 않으므로 이렇게 잡을 수밖에 없다)
        //
        // [칩 내부 어긋남] MPU9250은 단일 칩이 아니라 다이 두 개를 한 패키지에
        //     넣은 것이고, 두 다이의 축은 정렬되어 있지 않다.
        //         MPU6500 다이 (InvenSense) : 가속도 + 자이로
        //         AK8963  다이 (아사히카세이) : 지자기
        //     데이터시트 "Orientation of Axes"에 좌표계가 둘 그려져 있으며,
        //     관계는 다음과 같다 (고정값 — 개체·온도에 따라 변하지 않는다).
        //         AK8963 X  <->  MPU6500 Y
        //         AK8963 Y  <->  MPU6500 X
        //         AK8963 Z  =   -MPU6500 Z
        //     따라서 지자기를 (my, mx, -mz)로 재배치하면 가속도 축에 맞고,
        //     보드가 FLU로 장착되어 있으므로 그것이 곧 로봇 FLU다.
        //
        //     R = [0 1  0]   det(R) = +1, R·Rt = I  -> (1,1,0)축 180° 정상 회전.
        //         [1 0  0]   X·Y 스왑만 하면 det=-1(반사)이 되어 부품을 어떻게
        //         [0 0 -1]   돌려도 만들 수 없다. Z 부호 반전은 선택이 아니라
        //                    오른손 좌표계를 지키기 위해 구조적으로 따라온다.
        //
        // ※ 적용 순서 주의: bias/scale은 칩 자체 축 기준으로 산출된다
        //    (finalize_mag_calibration이 raw_m*의 min/max로 계산). 그러므로
        //    보정을 먼저, 축 재배치를 나중에 해야 한다. 순서를 바꾸면 다른 축의
        //    보정값을 쓰게 되고, 기존 mag_calib_params.txt도 무효가 된다.
        // ─────────────────────────────────────────────────────────────────────
        float cal_mx = (raw_mx - mag_bias_x_) * mag_scale_x_;
        float cal_my = (raw_my - mag_bias_y_) * mag_scale_y_;
        float cal_mz = (raw_mz - mag_bias_z_) * mag_scale_z_;
        latest_mx_ =  cal_my;
        latest_my_ =  cal_mx;
        latest_mz_ = -cal_mz;

        // 검증용 발행: 9축 융합에 실제로 들어가는 값 그대로.
        // (하드/소프트아이언 보정까지 적용된 뒤의 값이므로 /raw/magnetometer 와는
        //  축 순서뿐 아니라 크기도 다르다. 축 방향 확인이 목적이면 부호를 본다)
        auto ch_mag_msg = geometry_msgs::msg::Vector3();
        ch_mag_msg.x = latest_mx_;
        ch_mag_msg.y = latest_my_;
        ch_mag_msg.z = latest_mz_;
        ch_magnet_pub_->publish(ch_mag_msg);
    }

    // IMU 수신이 이 노드의 메인 루프다 (100Hz). 자이로 영점 -> 자세 융합 -> 발행.
    //
    // ── IMU 좌표계 ───────────────────────────────────────────────────────────
    // [센서 장착] nRF52840 쪽 IMU는 로봇 기구부에 다음 방향으로 고정한다.
    //     +X = 오른쪽,  +Y = 위,  -Z = 전진        (RUB: Right-Up-Back)
    //
    // [필터 요구] Mahony 필터와 오일러각 추출은 REP-103 FLU를 전제로 한다.
    //     +X = 전진,   +Y = 왼쪽,  +Z = 위         (FLU: Forward-Left-Up)
    //     · "+Z = 위"는 수학적 필수다. 수평 정지 시 가속도가 (0,0,+1)이어야
    //       halfvz(= q0^2 - 0.5 + q3^2)와의 외적 오차가 0이 된다.
    //     · "+X = 전진"은 관례적 선택이다. 이래야 roll이 좌우 기울기,
    //       pitch가 기수 상하가 되어 pid_control_node의 좌/우 서보 차동
    //       믹싱(같은 방향=피치, 반대 방향=롤)이 의도대로 동작한다.
    //
    // [변환] 두 좌표계를 잇는 회전. 가속도와 각속도에 동일하게 적용한다.
    //     전진 = -(센서 Z)  ->  x' = -z
    //     왼쪽 = -(센서 X)  ->  y' = -x
    //     위   = +(센서 Y)  ->  z' = +y
    //
    //     R = [ 0  0 -1 ]   det(R) = +1, R·Rt = I
    //         [-1  0  0 ]   -> 오른손 좌표계를 보존하는 정상 회전이므로,
    //         [ 0  1  0 ]      유사벡터인 각속도에도 같은 행렬을 그대로 쓴다.
    //                          (det=-1이면 각속도 부호가 따로 뒤집혀야 한다)
    //
    // ※ 자이로 영점(gyro_bias_)도 변환된 값으로 수집되므로 좌표계가 일관된다.
    // ※ 지자기는 별개 부품(파이 I2C)이라 이 변환이 적용되지 않는다.
    //    지자기 쪽 축 정렬은 mag_callback 주석 참조.
    // ─────────────────────────────────────────────────────────────────────────
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        float gx = -msg->angular_velocity.z;
        float gy = -msg->angular_velocity.x;
        float gz =  msg->angular_velocity.y;
        const float ax = -msg->linear_acceleration.z;
        const float ay = -msg->linear_acceleration.x;
        const float az =  msg->linear_acceleration.y;

        // 검증용 발행: 회전만 걸린 상태(자이로 바이어스 제거 전, 단위 변환 전)라
        // /raw/imu_6dof 와 성분끼리 1:1로 대조된다. 자이로 영점 잡는 초기 10초
        // 구간에도 발행되도록 아래 early return 앞에 둔다.
        auto ch_imu_msg = sensor_msgs::msg::Imu();
        ch_imu_msg.header.stamp = msg->header.stamp;   // 원본 타임스탬프 유지
        ch_imu_msg.header.frame_id = "base_link";      // 회전 후에는 로봇 FLU 좌표계
        ch_imu_msg.linear_acceleration.x = ax;   // [g]
        ch_imu_msg.linear_acceleration.y = ay;
        ch_imu_msg.linear_acceleration.z = az;
        ch_imu_msg.angular_velocity.x = gx;      // [도/초]
        ch_imu_msg.angular_velocity.y = gy;
        ch_imu_msg.angular_velocity.z = gz;
        ch_imu_pub_->publish(ch_imu_msg);

        // ---- [수정] 적분 스텝을 실측한다 (기존에는 dt_ = 0.01f 고정) ----
        // 자세는 "각속도 x 경과시간"의 누적이다. 경과시간을 상수로 박아두면
        // IMU가 늦게 올 때마다 그만큼 회전을 놓친다. 실제로 /raw/imu_6dof 에
        // 0.5초 공백이 관측됐고, 그 구간은 10ms로 취급되어 49샘플분 회전이 사라졌다.
        // 더 흔하고 더 위험한 쪽은 자잘한 유실이다 — 1%가 빠지면 1% 과소적분이
        // 쉬지 않고 누적되어, 100도/초 선회 중 1도/초씩 계속 모자라게 된다.
        //
        // ※ 상한 클램프를 두는 이유 (EKF 노드와 다른 판단):
        //   Mahony는 게인이 고정(Kp)이라 큰 dt가 그대로 큰 보정 스텝이 된다.
        //   Kp*dt 가 커지면 오버슈트/진동 위험이 있어 0.1초에서 자른다
        //   (Kp*dt/2 = 0.1 -> 스텝당 오차의 약 20% 보정, 안정 영역).
        //   EKF는 공분산이 알아서 신뢰도를 조절하므로 상한을 두지 않았다.
        //   여기서 잘린 만큼은 여전히 손실이지만, 최소한 로그로 드러난다.
        rclcpp::Time imu_time(msg->header.stamp);
        if (imu_time.nanoseconds() == 0) imu_time = this->now();   // 스탬프 미기입 폴백
        if (have_prev_imu_time_) {
            const double measured = (imu_time - last_imu_time_).seconds();
            if (std::isfinite(measured) && measured > 0.0) {
                if (measured > 0.1) {
                    dt_clamp_count_++;
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                        "[System] IMU 공백 %.0f ms (샘플 %.0f개분, 누적 %d회) -> 100ms로 제한하여 적분",
                        measured * 1000.0, measured / 0.01, dt_clamp_count_);
                }
                dt_ = static_cast<float>(std::clamp(measured, 0.001, 0.1));
            }
            // measured <= 0 (시계 역행/중복 스탬프)이면 직전 dt_를 그대로 유지한다
        }
        last_imu_time_ = imu_time;
        have_prev_imu_time_ = true;

        // ---- [1단계] 자이로 영점 캘리브레이션 (기동 후 약 10초) ----
        // 정지 상태에서도 자이로는 0이 아닌 값을 뱉는다(바이어스). 이 평균을 미리 재서
        // 이후 모든 측정값에서 빼주지 않으면 자세가 한 방향으로 계속 흘러간다.
        if (!gyro_calibrated_) {
            if (gyro_sample_count_ == 0) {
                RCLCPP_INFO(this->get_logger(), "=========================================");
                RCLCPP_INFO(this->get_logger(), "[System] 자이로 센서 영점 캘리브레이션을 시작합니다.");
                RCLCPP_INFO(this->get_logger(), "[System] 로봇을 절대 움직이지 마세요! (약 10초 소요)");
                RCLCPP_INFO(this->get_logger(), "=========================================");
            }

            gyro_bias_[0] += gx; gyro_bias_[1] += gy; gyro_bias_[2] += gz;
            gyro_sample_count_++;

            if (gyro_sample_count_ % 200 == 0) {
                RCLCPP_INFO(this->get_logger(), "... 자이로 영점 수집 중 (%d / %d) ...", gyro_sample_count_, GYRO_CALIB_SAMPLES);
            }

            if (gyro_sample_count_ >= GYRO_CALIB_SAMPLES) {
                gyro_bias_[0] /= GYRO_CALIB_SAMPLES;
                gyro_bias_[1] /= GYRO_CALIB_SAMPLES;
                gyro_bias_[2] /= GYRO_CALIB_SAMPLES;
                gyro_calibrated_ = true;

                RCLCPP_INFO(this->get_logger(), "=========================================");
                RCLCPP_INFO(this->get_logger(), "[OK] 자이로 센서 영점 세팅 완료!");
                RCLCPP_INFO(this->get_logger(), "=========================================");
            }
            return;   // 캘리브레이션 중에는 자세를 발행하지 않는다
        }

        // ---- [2단계] 바이어스 제거 후 자세 융합 ----
        gx -= gyro_bias_[0]; gy -= gyro_bias_[1]; gz -= gyro_bias_[2];
        // 지자기가 최근 500ms 안에 도착했고 캘리브레이션 중이 아니면 9축, 아니면 6축으로 폴백.
        // 센서를 도중에 뽑아도 자동으로 6축 모드로 내려가 자세 추정이 끊기지 않는다.
        // (nRF는 각속도를 도/초로 보내므로 라디안으로 환산해서 넘긴다)
        // ax/ay/az, gx/gy/gz는 콜백 진입부에서 이미 FLU로 변환된 값이다.
        if (!is_mag_calibrating_ && (this->now() - last_mag_time_) < mag_timeout_) {
            update_9dof_mahony(gx*(M_PI/180), gy*(M_PI/180), gz*(M_PI/180), ax, ay, az, latest_mx_, latest_my_, latest_mz_);
        } else {
            update_6dof_mahony(gx*(M_PI/180), gy*(M_PI/180), gz*(M_PI/180), ax, ay, az);
        }

        // ---- [3단계] 오일러각 변환 및 발행 ----
        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        get_euler_angles(&roll, &pitch, &yaw);
        // 헤딩 영점 적용 후 ±180도 범위로 되감기 (예: 190도 -> -170도)
        float filtered_yaw = yaw - yaw_offset_;
        if (filtered_yaw > 180.0f) filtered_yaw -= 360.0f; else if (filtered_yaw < -180.0f) filtered_yaw += 360.0f;

        auto attitude_msg = geometry_msgs::msg::Vector3();
        // 자동 모드 플래그를 roll에 +5000 오프셋으로 실어 보낸다 (파일 상단 모드 전달 규약 참고).
        // roll 실제 범위가 ±180도라 수신 측은 |x| > 2500으로 오프셋 유무를 판별할 수 있다.
        attitude_msg.x = is_auto_mode_ ? (roll + 5000.0f) : roll;
        attitude_msg.y = pitch;
        attitude_msg.z = filtered_yaw;
        attitude_pub_->publish(attitude_msg);
    }

    float dt_;                              // 적분 스텝(초). imu_callback에서 매 샘플 실측 (0.001~0.1로 제한)
    rclcpp::Time last_imu_time_;            // dt_ 계산용 직전 IMU 타임스탬프
    bool  have_prev_imu_time_;              // 첫 샘플 판별 (첫 스텝은 초기값 0.01 사용)
    int   dt_clamp_count_;                  // 상한(100ms)에 걸린 횟수 — UART 건전성 지표
    float q0_, q1_, q2_, q3_;               // 현재 자세 쿼터니언
    float eInt_[3];                         // Mahony I항 누산기
    float gyro_bias_[3];                    // 자이로 영점(도/초)
    bool gyro_calibrated_; int gyro_sample_count_;
    bool press_calibrated_; int press_sample_count_;
    bool is_mag_calibrating_; int mag_calib_sample_count_; int last_progress_;
    float mag_bias_x_, mag_bias_y_, mag_bias_z_;      // 하드아이언 보정
    float mag_scale_x_, mag_scale_y_, mag_scale_z_;   // 소프트아이언 보정
    float latest_mx_, latest_my_, latest_mz_;         // 보정 적용된 최신 지자기
    float raw_yaw_;                                   // 헤딩 영점 적용 전 원시 yaw
    float yaw_offset_;                                // btn2 짧게 누름으로 잡은 기준 방향
    bool is_auto_mode_; int btn1_press_count_;
    int btn1_counter_; int btn2_counter_;             // 눌림 지속 틱 (1틱 = 10ms)
    int prev_btn1_; int prev_btn2_;                   // 엣지 판정용 직전 상태
    bool right_btn_pressed_; rclcpp::Time button_press_start_time_; bool left_btn_pressed_; rclcpp::Time left_btn_press_start_time_;
    bool btn1_long_processed_; bool btn2_long_processed_;
    rclcpp::Time last_mag_time_; rclcpp::Duration mag_timeout_;   // 지자기 생존 판정
    rclcpp::Time node_start_time_;   // [N2] 압력 웜업 대기 기준 시각
    float press_sum_[3]; float press_offset_[3]; float temp_sum_[3]; float temp_offset_[3]; float drift_coeff_[3]; float mag_max_[3]; float mag_min_[3];
    int press_valid_count_[3]; bool press_ch_alive_[3];   // 채널별 유효 샘플 수 / 생존 여부 (N3)
    std::vector<float> dynamic_pressures_;

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr attitude_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pressure_cal_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr ch_imu_pub_;          // 축 변환된 IMU (검증용)
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr ch_magnet_pub_; // 축 변환된 지자기 (검증용)
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr mag_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr rc_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pressure_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mag_calib_srv_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateEstimationNode>());
    rclcpp::shutdown();
    return 0;
}
