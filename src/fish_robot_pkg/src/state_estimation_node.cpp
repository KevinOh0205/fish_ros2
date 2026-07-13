#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <stdio.h>
#include <math.h>
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

#define Kp 2.0f   
#define Ki 0.005f 

#define GYRO_CALIB_SAMPLES 1000
#define PRESSURE_CALIB_SAMPLES 100
#define MAG_CALIB_MAX_SAMPLES 1000

// [N2] 압력 영점 캡처 전 웜업 대기(초). 웜업 시간 드리프트의 초반 급강하를 지나
// baseline을 잡기 위함. 초반 급드리프트(ch0 ~250초)를 넘기는 값.
#define PRESSURE_WARMUP_SEC 300

class StateEstimationNode : public rclcpp::Node {
public:
    StateEstimationNode() : Node("state_estimation_node"), 
                            dt_(0.01f), q0_(1.0f), q1_(0.0f), q2_(0.0f), q3_(0.0f),
                            gyro_calibrated_(false), gyro_sample_count_(0),
                            press_calibrated_(false), press_sample_count_(0),
                            is_mag_calibrating_(false), mag_calib_sample_count_(0), last_progress_(-1),
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
                            btn2_long_processed_(false),
                            last_mag_time_(0, 0, RCL_ROS_TIME), 
                            mag_timeout_(std::chrono::milliseconds(500))
    {
        RCLCPP_INFO(this->get_logger(), "=== [State Estimation] 센서 통합 보정 노드 초기화 ===");

        eInt_[0] = 0.0f; eInt_[1] = 0.0f; eInt_[2] = 0.0f;
        gyro_bias_[0] = 0.0f; gyro_bias_[1] = 0.0f; gyro_bias_[2] = 0.0f;
        
        for(int i = 0; i < 3; i++) {
            press_sum_[i] = 0.0f;   press_offset_[i] = 0.0f;
            temp_sum_[i] = 0.0f;    temp_offset_[i] = 0.0f;
            press_valid_count_[i] = 0;  press_ch_alive_[i] = false;
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

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/raw/imu_6dof", 10, std::bind(&StateEstimationNode::imu_callback, this, std::placeholders::_1));
        mag_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/raw/magnetometer", 10, std::bind(&StateEstimationNode::mag_callback, this, std::placeholders::_1));
        rc_status_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/rc/status", 10, std::bind(&StateEstimationNode::rc_status_callback, this, std::placeholders::_1));
        pressure_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/sensor/pressure_raw", 10, std::bind(&StateEstimationNode::pressure_callback, this, std::placeholders::_1));
            
        mag_calib_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/calibrate_mag", std::bind(&StateEstimationNode::handle_mag_calibration, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void get_euler_angles(float *roll, float *pitch, float *yaw) {
        *roll = atan2f(q0_*q1_ + q2_*q3_, 0.5f - q1_*q1_ - q2_*q2_) * (180.0f / M_PI);
        // asinf 정의역은 [-1,1]. 쿼터니언 미세 비정규화로 인자가 이 범위를 벗어나면
        // NaN이 나오므로(±90° 부근) 반드시 클램프한다 (N6: NaN 원천 차단).
        float sin_pitch = std::clamp(-2.0f * (q1_*q3_ - q0_*q2_), -1.0f, 1.0f);
        *pitch = asinf(sin_pitch) * (180.0f / M_PI);
        *yaw = atan2f(q1_*q2_ + q0_*q3_, 0.5f - q2_*q2_ - q3_*q3_) * (180.0f / M_PI);
        raw_yaw_ = *yaw;
    }

    void update_9dof_mahony(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
        float recipNorm;
        float q0q0 = q0_*q0_, q0q1 = q0_*q1_, q0q2 = q0_*q2_, q0q3 = q0_*q3_;
        float q1q1 = q1_*q1_, q1q2 = q1_*q2_, q1q3 = q1_*q3_;
        float q2q2 = q2_*q2_, q2q3 = q2_*q3_, q3q3 = q3_*q3_;

        if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
            recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
            ax *= recipNorm; ay *= recipNorm; az *= recipNorm;
            recipNorm = 1.0f / sqrtf(mx * mx + my * my + mz * mz);
            mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

            float hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
            float hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
            float bx = sqrtf(hx * hx + hy * hy);
            float bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

            float halfvx = q1_ * q3_ - q0_ * q2_; 
            float halfvy = q0_ * q1_ + q2_ * q3_; 
            float halfvz = q0q0 - 0.5f + q3q3;
            float halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1_ * q3_ - q0_ * q2_);
            float halfwy = bx * (q1_ * q2_ - q0_ * q3_) + bz * (0.5f - q1q1 - q3q3);
            float halfwz = bx * (q0_ * q2_ + q1_ * q3_) + bz * (0.5f - q1q1 - q2q2);

            float ex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
            float ey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
            float ez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

            if (Ki > 0.0f) {
                eInt_[0] += ex * dt_; eInt_[1] += ey * dt_; eInt_[2] += ez * dt_;
                gx += Ki * eInt_[0]; gy += Ki * eInt_[1]; gz += Ki * eInt_[2];
            }
            gx += Kp * ex; gy += Kp * ey; gz += Kp * ez;
        }

        gx *= (0.5f * dt_); gy *= (0.5f * dt_); gz *= (0.5f * dt_);
        float qa = q0_, qb = q1_, qc = q2_;
        q0_ += (-qb * gx - qc * gy - q3_ * gz); q1_ += (qa * gx + qc * gz - q3_ * gy);
        q2_ += (qa * gy - qb * gz + q3_ * gx); q3_ += (qa * gz + qb * gy - qc * gx);

        recipNorm = 1.0f / sqrtf(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
        q0_ *= recipNorm; q1_ *= recipNorm; q2_ *= recipNorm; q3_ *= recipNorm;
    }

    void update_6dof_mahony(float gx, float gy, float gz, float ax, float ay, float az) {
        float recipNorm; float halfvx, halfvy, halfvz; float halfex, halfey, halfez;

        if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
            recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
            ax *= recipNorm; ay *= recipNorm; az *= recipNorm;
            halfvx = q1_ * q3_ - q0_ * q2_; halfvy = q0_ * q1_ + q2_ * q3_; halfvz = q0_ * q0_ - 0.5f + q3_ * q3_;
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

    void pressure_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 6) return;

        float raw_p[3] = { msg->data[0], msg->data[1], msg->data[2] };
        float raw_t[3] = { msg->data[3], msg->data[4], msg->data[5] };

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
                        press_offset_[i] = press_sum_[i] / press_valid_count_[i];
                        temp_offset_[i] = temp_sum_[i] / press_valid_count_[i];
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

        auto cal_msg = std_msgs::msg::Float32MultiArray();
        cal_msg.data.resize(3);
        for (int i = 0; i < 3; i++) {
            if (press_ch_alive_[i]) {
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

    void load_mag_calibration_file() {
        const char* home_dir = getenv("HOME");
        std::string file_path = home_dir ? (std::string(home_dir) + "/ros2_ws/log_csv/mag_calib_params.txt") : "./log_csv/mag_calib_params.txt";
        std::ifstream infile(file_path);
        if (infile.is_open()) {
            infile >> mag_bias_x_ >> mag_bias_y_ >> mag_bias_z_ >> mag_scale_x_ >> mag_scale_y_ >> mag_scale_z_;
            infile.close();
        }
    }

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
            if (btn1_counter_ == 300) { 
                btn1_long_processed_ = true; 
                RCLCPP_WARN(this->get_logger(), "======================================================");
                RCLCPP_WARN(this->get_logger(), "[System] 버튼 1 롱프레스 감지! 대기압 영점을 재조정합니다.");
                RCLCPP_WARN(this->get_logger(), "======================================================");
                press_calibrated_ = false; press_sample_count_ = 0;
                for(int i=0; i<3; i++) { press_sum_[i]=0.0f; temp_sum_[i]=0.0f; press_valid_count_[i]=0; }
            }
        } else {
            if (prev_btn1_ == 1) { 
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
                    if ((this->now() - last_mag_time_) >= mag_timeout_) {
                        RCLCPP_ERROR(this->get_logger(), "======================================================");
                        RCLCPP_ERROR(this->get_logger(), "[System] 지자기 센서(AK8963) 응답이 없습니다! 캘리브레이션을 취소합니다.");
                        RCLCPP_ERROR(this->get_logger(), "======================================================");
                    } else {
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

    void handle_mag_calibration(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        (void)req;
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

    void finalize_mag_calibration() {
        is_mag_calibrating_ = false;
        mag_bias_x_ = (mag_max_[0] + mag_min_[0]) / 2.0f; mag_bias_y_ = (mag_max_[1] + mag_min_[1]) / 2.0f; mag_bias_z_ = (mag_max_[2] + mag_min_[2]) / 2.0f;
        float dx = (mag_max_[0] - mag_min_[0]) / 2.0f, dy = (mag_max_[1] - mag_min_[1]) / 2.0f, dz = (mag_max_[2] - mag_min_[2]) / 2.0f, avg = (dx+dy+dz)/3.0f;
        
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
        last_mag_time_ = this->now();
        float raw_mx = msg->x, raw_my = msg->y, raw_mz = msg->z;
        if (is_mag_calibrating_) {
            mag_max_[0] = std::max(mag_max_[0], raw_mx); mag_max_[1] = std::max(mag_max_[1], raw_my); mag_max_[2] = std::max(mag_max_[2], raw_mz);
            mag_min_[0] = std::min(mag_min_[0], raw_mx); mag_min_[1] = std::min(mag_min_[1], raw_my); mag_min_[2] = std::min(mag_min_[2], raw_mz);
            
            mag_calib_sample_count_++;
            
            if (mag_calib_sample_count_ % 200 == 0) {
                RCLCPP_INFO(this->get_logger(), "... 지자기 영점 수집 중 (%d / %d) ...", mag_calib_sample_count_, MAG_CALIB_MAX_SAMPLES);
            }

            if (mag_calib_sample_count_ >= MAG_CALIB_MAX_SAMPLES) {
                finalize_mag_calibration();
            }
        }
        latest_mx_ = (raw_mx - mag_bias_x_) * mag_scale_x_; latest_my_ = (raw_my - mag_bias_y_) * mag_scale_y_; latest_mz_ = (raw_mz - mag_bias_z_) * mag_scale_z_;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        float gx = msg->angular_velocity.x, gy = msg->angular_velocity.y, gz = msg->angular_velocity.z;
        
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
            return;
        }

        gx -= gyro_bias_[0]; gy -= gyro_bias_[1]; gz -= gyro_bias_[2];
        if (!is_mag_calibrating_ && (this->now() - last_mag_time_) < mag_timeout_) {
            update_9dof_mahony(gx*(M_PI/180), gy*(M_PI/180), gz*(M_PI/180), msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z, latest_mx_, latest_my_, latest_mz_);
        } else {
            update_6dof_mahony(gx*(M_PI/180), gy*(M_PI/180), gz*(M_PI/180), msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        }
        
        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        get_euler_angles(&roll, &pitch, &yaw);
        float filtered_yaw = yaw - yaw_offset_;
        if (filtered_yaw > 180.0f) filtered_yaw -= 360.0f; else if (filtered_yaw < -180.0f) filtered_yaw += 360.0f;

        auto attitude_msg = geometry_msgs::msg::Vector3();
        attitude_msg.x = is_auto_mode_ ? (roll + 5000.0f) : roll; 
        attitude_msg.y = pitch; 
        attitude_msg.z = filtered_yaw;
        attitude_pub_->publish(attitude_msg);
    }

    float dt_; float q0_, q1_, q2_, q3_; float eInt_[3]; float gyro_bias_[3];
    bool gyro_calibrated_; int gyro_sample_count_;
    bool press_calibrated_; int press_sample_count_;
    bool is_mag_calibrating_; int mag_calib_sample_count_; int last_progress_;
    float mag_bias_x_, mag_bias_y_, mag_bias_z_; float mag_scale_x_, mag_scale_y_, mag_scale_z_;
    float latest_mx_, latest_my_, latest_mz_; float raw_yaw_; float yaw_offset_;
    bool is_auto_mode_; int btn1_press_count_;
    int btn1_counter_; int btn2_counter_;
    int prev_btn1_; int prev_btn2_;
    bool right_btn_pressed_; rclcpp::Time button_press_start_time_; bool left_btn_pressed_; rclcpp::Time left_btn_press_start_time_;
    bool btn1_long_processed_; bool btn2_long_processed_;
    rclcpp::Time last_mag_time_; rclcpp::Duration mag_timeout_;
    rclcpp::Time node_start_time_;   // [N2] 압력 웜업 대기 기준 시각
    float press_sum_[3]; float press_offset_[3]; float temp_sum_[3]; float temp_offset_[3]; float drift_coeff_[3]; float mag_max_[3]; float mag_min_[3];
    int press_valid_count_[3]; bool press_ch_alive_[3];   // 채널별 유효 샘플 수 / 생존 여부 (N3)
    std::vector<float> dynamic_pressures_;
    
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr attitude_pub_; 
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pressure_cal_pub_; 
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