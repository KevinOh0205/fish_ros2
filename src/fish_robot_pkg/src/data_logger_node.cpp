#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

class DataLoggerNode : public rclcpp::Node {
public:
    DataLoggerNode() : Node("data_logger_node"), fp_(nullptr) {
        RCLCPP_INFO(this->get_logger(), "=== [Data Logger] 누적 카운트 및 동기화 로깅 대기 중 ===");

        if (init_log_file() < 0) {
            RCLCPP_ERROR(this->get_logger(), "CSV 파일 시스템 초기화 실패!");
            return;
        }

        // [추가/수정] 버튼 누적 카운트 및 상태 전이 감지용 변수 초기화
        btn1_cumulative_count_ = 0;
        btn1_hold_counter_ = 0;
        prev_btn1_ = 0;
        btn1_long_processed_ = false;
        is_auto_mode_ = false;

        mode_ = 0;
        roll_ = 0.0f; pitch_ = 0.0f; yaw_ = 0.0f;
        rc_t_ = 0; rc_r_ = 0; rc_p_ = 0; rc_y_ = 0;
        m1_ = 1500; m2_ = 1500; m3_ = 1500; m4_ = 1000;
        rpm_ = 0;
        p0_cal_ = 0.0f; p1_cal_ = 0.0f; p2_cal_ = 0.0f;
        t0_ = 0.0f; t1_ = 0.0f; t2_ = 0.0f;
        rssi_ = 0;

        received_attitude_ = false;
        received_pressure_ = false;
        is_logging_active_ = false;

        attitude_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/filtered/attitude", 10, std::bind(&DataLoggerNode::attitude_callback, this, std::placeholders::_1));

        pressure_cal_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/sensor/pressure_calibrated", 10, std::bind(&DataLoggerNode::pressure_cal_callback, this, std::placeholders::_1));

        // 압력 영점 음의 드리프트(N2) 진단용 원시 온도 로깅. [3..5]가 채널별 온도(C).
        pressure_raw_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/sensor/pressure_raw", 10, std::bind(&DataLoggerNode::pressure_raw_callback, this, std::placeholders::_1));

        rc_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
            "/rc/command", 10, std::bind(&DataLoggerNode::rc_cmd_callback, this, std::placeholders::_1));

        rc_status_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/rc/status", 10, std::bind(&DataLoggerNode::rc_status_callback, this, std::placeholders::_1));

        motor_sub_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
            "/motor/output", 10, std::bind(&DataLoggerNode::motor_callback, this, std::placeholders::_1));

        rpm_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/sensor/tail_rpm", 10, std::bind(&DataLoggerNode::rpm_callback, this, std::placeholders::_1));

        logging_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&DataLoggerNode::write_log_loop, this));
    }

    ~DataLoggerNode() {
        if (fp_ != nullptr) {
            fflush(fp_);
            fclose(fp_);
            RCLCPP_INFO(this->get_logger(), "[Data Logger] 로그 파일 보관 및 스트림 플러시 완료.");
        }
    }

private:
    int init_log_file() {
        const char* log_dir_path = "/home/fish/ros2_ws/log_csv";
        struct stat st;
        memset(&st, 0, sizeof(struct stat));
        
        if (stat(log_dir_path, &st) == -1) {
            if (mkdir(log_dir_path, 0775) < 0) return -1;
        }

        time_t t = time(nullptr);
        struct tm tm_info;
        localtime_r(&t, &tm_info);

        char file_path[512];
        snprintf(file_path, sizeof(file_path), "%s/fish_log_%04d%02d%02d_%02d%02d%02d.csv",
                 log_dir_path, tm_info.tm_year + 1900, tm_info.tm_mon + 1, tm_info.tm_mday,
                 tm_info.tm_hour, tm_info.tm_min, tm_info.tm_sec);

        fp_ = fopen(file_path, "w");
        if (fp_ == nullptr) return -1;

        fprintf(fp_, "Time(s),Btn1_Count,Mode,AutoMode,Roll,Pitch,Yaw,RC_T,RC_R,RC_P,RC_Y,M1,M2,M3,M4,RPM,P0_cal,P1_cal,P2_cal,T0,T1,T2,RSSI\n");
        fflush(fp_);
        
        return 0;
    }

    void attitude_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        received_attitude_ = true;
        // state_estimation이 자동 모드 표시용으로 roll에 실은 ±5000 오프셋을 제거해
        // 실제 자세각을 기록한다 (pid_control_node의 디코딩과 동일 규칙).
        // 동시에 이 오프셋을 로봇의 실제 모드(단일 출처)로 사용한다.
        if (msg->x > 2500.0f) {
            roll_ = msg->x - 5000.0f;
            is_auto_mode_ = true;
        } else if (msg->x < -2500.0f) {
            roll_ = msg->x + 5000.0f;
            is_auto_mode_ = true;
        } else {
            roll_ = msg->x;
            is_auto_mode_ = false;
        }
        pitch_ = msg->y;
        yaw_   = msg->z;
    }

    void pressure_cal_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 3) return;
        received_pressure_ = true;
        p0_cal_ = msg->data[0];
        p1_cal_ = msg->data[1];
        p2_cal_ = msg->data[2];
    }

    // 원시 압력 토픽의 온도 채널([3..5])만 뽑아 저장한다 (압력 드리프트 진단용).
    void pressure_raw_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 6) return;
        t0_ = msg->data[3];
        t1_ = msg->data[4];
        t2_ = msg->data[5];
    }

    void rc_cmd_callback(const geometry_msgs::msg::Quaternion::SharedPtr msg) {
        rc_r_ = static_cast<int16_t>(msg->x);
        rc_p_ = static_cast<int16_t>(msg->y);
        rc_y_ = static_cast<int16_t>(msg->z);
        rc_t_ = static_cast<int16_t>(msg->w);
    }

    void rpm_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        rpm_ = msg->data;
    }

    void rc_status_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 5) return;
        int current_btn1 = msg->data[0];
        rssi_ = msg->data[4];

        // 짧은 누름(Edge Detection)으로 버튼 누적 카운트만 집계한다.
        // 자동/수동 모드 판정은 여기서 하지 않고 state_estimation이 방송한
        // /filtered/attitude 오프셋(attitude_callback)을 단일 출처로 사용한다.
        if (current_btn1 == 1) {
            btn1_hold_counter_++;
            if (btn1_hold_counter_ == 300) {
                btn1_long_processed_ = true;
            }
        } else {
            // 진짜 뗌(current==0)만 짧은 누름으로 인정한다. 통신두절 시 btn=255로 튀는
            // 1→255 전이를 '뗌'으로 오인해 카운트가 증가하는 것을 막는다(N4).
            if (prev_btn1_ == 1 && current_btn1 == 0) {
                // 손을 떼는 순간(Falling Edge), 30ms 이상 1초 이하로 눌렸을 때만 짧은 누름으로 인정
                if (btn1_hold_counter_ > 3 && btn1_hold_counter_ < 100 && !btn1_long_processed_) {
                    btn1_cumulative_count_++; // 누적 카운트 증가!
                }
            }
            btn1_hold_counter_ = 0;
            btn1_long_processed_ = false;
        }
        prev_btn1_ = current_btn1;

        // 통신 상태만 결정한다 (0 = 정상, 2 = 조종기 두절 -9999 센티넬).
        // 자동/수동 여부는 별도 AutoMode 컬럼(is_auto_mode_)으로 분리 기록한다.
        if (rc_t_ == -9999) {
            mode_ = 2;
        } else {
            mode_ = 0;
        }
    }

    void motor_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
        if (msg->data.size() < 4) return;
        m1_ = msg->data[0];
        m2_ = msg->data[1];
        m3_ = msg->data[2];
        m4_ = msg->data[3];
    }

    void write_log_loop() {
        if (!is_logging_active_) {
            if (received_attitude_ && received_pressure_) {
                is_logging_active_ = true;
                start_time_ = this->now(); 
                RCLCPP_INFO(this->get_logger(), "=========================================");
                RCLCPP_INFO(this->get_logger(), "[Data Logger] 센서 0점 보정 완료 감지!");
                RCLCPP_INFO(this->get_logger(), "[Data Logger] 데이터 로깅을 0.0초부터 시작합니다.");
                RCLCPP_INFO(this->get_logger(), "=========================================");
            } else {
                return; 
            }
        }

        if (fp_ == nullptr) return;

        double relative_time_sec = (this->now() - start_time_).seconds();

        // btn1_cumulative_count_ 를 기록합니다.
        fprintf(fp_, "%.3f,%d,%d,%d,%.2f,%.2f,%.2f,%d,%d,%d,%d,%u,%u,%u,%u,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d\n",
                relative_time_sec,
                btn1_cumulative_count_,
                mode_,
                is_auto_mode_ ? 1 : 0,
                roll_, pitch_, yaw_,
                rc_t_, rc_r_, rc_p_, rc_y_,
                m1_, m2_, m3_, m4_,
                rpm_,
                p0_cal_, p1_cal_, p2_cal_,
                t0_, t1_, t2_,
                rssi_);

        static int flush_cnt = 0;
        if (++flush_cnt >= 50) {
            fflush(fp_);
            flush_cnt = 0;
        }
    }

    FILE* fp_;
    
    bool received_attitude_;
    bool received_pressure_;
    bool is_logging_active_;
    rclcpp::Time start_time_;

    // [추가] 버튼 엣지 디텍션 및 상태 관리용 변수들
    int32_t btn1_cumulative_count_;
    int btn1_hold_counter_;
    int prev_btn1_;
    bool btn1_long_processed_;
    bool is_auto_mode_;

    int32_t mode_;
    float roll_, pitch_, yaw_;
    int16_t rc_t_, rc_r_, rc_p_, rc_y_;
    uint16_t m1_, m2_, m3_, m4_;
    int32_t rpm_;
    float p0_cal_, p1_cal_, p2_cal_;
    float t0_, t1_, t2_;   // 채널별 원시 온도(C) — 압력 드리프트 진단용
    int32_t rssi_;

    rclcpp::TimerBase::SharedPtr logging_timer_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr attitude_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pressure_cal_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pressure_raw_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr rc_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr rc_status_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr motor_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rpm_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DataLoggerNode>();
    if (rclcpp::ok()) {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}