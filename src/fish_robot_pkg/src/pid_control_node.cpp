#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>

#include <stdio.h>
#include <stdint.h>
#include <vector>
#include <cmath>

// 서보 동작 범위 (중립 1500us 기준 ±30°). nRF 펌웨어 MotorControl.cpp의
// SERVO_MIN_US/SERVO_MAX_US와 반드시 동일하게 유지할 것 (N5: 범위 불일치 방지).
#define SERVO_MIN_US 1250
#define SERVO_MAX_US 1750

class PidControlNode : public rclcpp::Node {
public:
    PidControlNode() : Node("pid_control_node"),
                       kp_r_(3.5f), ki_r_(0.0f), kd_r_(0.0f),
                       kp_p_(3.5f), ki_p_(0.0f), kd_p_(0.0f), // 회원님 제안대로 지상 테스트를 위해 0.0 유지
                       kp_y_(1.2f), ki_y_(0.0f), kd_y_(0.0f),
                       i_limit_(300.0f),
                       err_sum_roll_(0.0f), err_sum_pitch_(0.0f), err_sum_yaw_(0.0f),
                       prev_err_roll_(0.0f), prev_err_pitch_(0.0f), prev_err_yaw_(0.0f),
                       robot_roll_(0.0f), robot_pitch_(0.0f), robot_yaw_(0.0f),
                       rc_roll_(0), rc_pitch_(0), rc_yaw_(0), rc_throttle_(-9999),
                       auto_roll_(0), auto_pitch_(0), auto_yaw_(0), auto_throttle_(1000),
                       is_auto_mode_(false),
                       current_rpm_(0),
                       failsafe_timeout_(5.0),
                       last_valid_rc_time_(this->now())
    {
        RCLCPP_INFO(this->get_logger(), "=== [PID Control] I_gain 0 세팅 및 자율 주행 연동 완료 ===");

        motor_pub_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("/motor/output", 10);

        attitude_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/filtered/attitude", 10, std::bind(&PidControlNode::attitude_callback, this, std::placeholders::_1));

        rc_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
            "/rc/command", 10, std::bind(&PidControlNode::rc_command_callback, this, std::placeholders::_1));

        auto_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
            "/auto/command", 10, std::bind(&PidControlNode::auto_command_callback, this, std::placeholders::_1));

        rpm_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/sensor/tail_rpm", 10, std::bind(&PidControlNode::rpm_callback, this, std::placeholders::_1));
    }

private:
    void attitude_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        if (msg->x > 2500.0f) {
            robot_roll_ = msg->x - 5000.0f;
            is_auto_mode_ = true;
        } else if (msg->x < -2500.0f) {
            robot_roll_ = msg->x + 5000.0f;
            is_auto_mode_ = true;
        } else {
            robot_roll_ = msg->x;
            is_auto_mode_ = false;
        }
        
        robot_pitch_ = msg->y;
        robot_yaw_ = msg->z;
        calculate_pid_control();
    }

    void rc_command_callback(const geometry_msgs::msg::Quaternion::SharedPtr msg) {
        int16_t incoming_throttle = static_cast<int16_t>(msg->w);
        if (incoming_throttle != -9999) {
            rc_roll_ = static_cast<int16_t>(msg->x);
            rc_pitch_ = static_cast<int16_t>(msg->y);
            rc_yaw_ = static_cast<int16_t>(msg->z);
            rc_throttle_ = incoming_throttle;
            last_valid_rc_time_ = this->now();
        }
    }

    void auto_command_callback(const geometry_msgs::msg::Quaternion::SharedPtr msg) {
        auto_roll_ = static_cast<int16_t>(msg->x);
        auto_pitch_ = static_cast<int16_t>(msg->y);
        auto_yaw_ = static_cast<int16_t>(msg->z);
        auto_throttle_ = static_cast<int16_t>(msg->w);
    }

    // 꼬리 BLDC RPM 수신 (추후 surge speed 제어에서 사용 예정)
    void rpm_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        current_rpm_ = msg->data;
    }

    uint16_t constrain_servo(float val) {
        // NaN은 모든 비교가 false라 클램프를 통과하고 (uint16_t)NaN=UB(보통 0)가 된다.
        // 명시적으로 걸러 중립(1500)으로 폴백한다 (N6).
        if (std::isnan(val)) return 1500;
        if (val < static_cast<float>(SERVO_MIN_US)) return SERVO_MIN_US;
        if (val > static_cast<float>(SERVO_MAX_US)) return SERVO_MAX_US;
        return static_cast<uint16_t>(val);
    }

    uint16_t constrain_pwm(float val) {
        if (std::isnan(val)) return 1000;   // NaN → 최소 스로틀로 안전 폴백 (N6)
        if (val < 1000.0f) return 1000;
        if (val > 2000.0f) return 2000;
        return static_cast<uint16_t>(val);
    }

    void calculate_pid_control() {
        double elapsed_since_last_cmd = (this->now() - last_valid_rc_time_).seconds();

        if (!is_auto_mode_ && (rc_throttle_ == -9999 || elapsed_since_last_cmd > failsafe_timeout_)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "[PID Control] 수동 모드 중 조종기 통신 두절 (Timeout: %.1f초). 모터 정지.", elapsed_since_last_cmd);
            auto motor_output_msg = std_msgs::msg::UInt16MultiArray();
            motor_output_msg.data = {1500, 1500, 1500, 1000};
            motor_pub_->publish(motor_output_msg);
            return;
        }

        auto motor_output_msg = std_msgs::msg::UInt16MultiArray();
        motor_output_msg.data.resize(4);

        float target_roll, target_pitch, target_yaw;
        uint16_t active_throttle;

        if (is_auto_mode_) {
            target_roll = auto_roll_ * 0.1f;
            target_pitch = auto_pitch_ * 0.1f;
            target_yaw = auto_yaw_ * 0.1f;
            active_throttle = constrain_pwm(auto_throttle_);
        } else {
            target_roll = rc_roll_ * 0.1f;
            target_pitch = rc_pitch_ * 0.1f;
            target_yaw = rc_yaw_ * 0.1f;
            int16_t mapped_throttle = static_cast<int16_t>(rc_throttle_ * (1000.0f / 1171.0f));
            active_throttle = constrain_pwm(1000 + mapped_throttle);
        }

        // PID 연산 구역
        float err_roll = target_roll - robot_roll_;
        float err_pitch = target_pitch - robot_pitch_;
        float err_yaw = target_yaw - robot_yaw_;

        // 자세값이 NaN이면(asinf 이상·센서 결함 등) 여기서 적분/미분 상태(err_sum_, prev_err_)에
        // NaN이 스며 재시작 전까지 영구 오염되는 것을 막는다. 상태를 건드리지 않고 안전
        // 출력(서보 중립, 스로틀 유지) 후 반환한다 (N6). 자세 복구 시 깨끗하게 재개된다.
        if (std::isnan(err_roll) || std::isnan(err_pitch) || std::isnan(err_yaw)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[PID Control] 자세값 NaN 감지. 이번 주기 서보 중립 출력 (적분 상태 보존).");
            motor_output_msg.data[0] = 1500;
            motor_output_msg.data[1] = 1500;
            motor_output_msg.data[2] = 1500;
            motor_output_msg.data[3] = active_throttle;
            motor_pub_->publish(motor_output_msg);
            return;
        }

        err_sum_roll_ += err_roll;
        err_sum_pitch_ += err_pitch;
        err_sum_yaw_ += err_yaw;

        if (err_sum_roll_ > i_limit_) err_sum_roll_ = i_limit_;
        else if (err_sum_roll_ < -i_limit_) err_sum_roll_ = -i_limit_;

        if (err_sum_pitch_ > i_limit_) err_sum_pitch_ = i_limit_;
        else if (err_sum_pitch_ < -i_limit_) err_sum_pitch_ = -i_limit_;

        if (err_sum_yaw_ > i_limit_) err_sum_yaw_ = i_limit_;
        else if (err_sum_yaw_ < -i_limit_) err_sum_yaw_ = -i_limit_;

        float d_roll = err_roll - prev_err_roll_;
        float d_pitch = err_pitch - prev_err_pitch_;
        float d_yaw = err_yaw - prev_err_yaw_;

        prev_err_roll_ = err_roll;
        prev_err_pitch_ = err_pitch;
        prev_err_yaw_ = err_yaw;

        float u_roll = (kp_r_ * err_roll) + (ki_r_ * err_sum_roll_) + (kd_r_ * d_roll);
        float u_pitch = (kp_p_ * err_pitch) + (ki_p_ * err_sum_pitch_) + (kd_p_ * d_pitch);
        float u_yaw = (kp_y_ * err_yaw) + (ki_y_ * err_sum_yaw_) + (kd_y_ * d_yaw);

        motor_output_msg.data[0] = constrain_servo(1500 + u_pitch + u_roll);
        motor_output_msg.data[1] = constrain_servo(1500 + u_pitch - u_roll);
        motor_output_msg.data[2] = constrain_servo(1500 + u_yaw);
        motor_output_msg.data[3] = active_throttle;

        if (is_auto_mode_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "[AUTO] Target: R=%.1f, P=%.1f | Out: M1=%d, M2=%d", 
                target_roll, target_pitch, motor_output_msg.data[0], motor_output_msg.data[1]);
        }

        motor_pub_->publish(motor_output_msg);
    }

    float kp_r_, ki_r_, kd_r_;
    float kp_p_, ki_p_, kd_p_;
    float kp_y_, ki_y_, kd_y_;
    float i_limit_;
    float err_sum_roll_, err_sum_pitch_, err_sum_yaw_;
    float prev_err_roll_, prev_err_pitch_, prev_err_yaw_;
    float robot_roll_, robot_pitch_, robot_yaw_;
    
    int16_t rc_roll_, rc_pitch_, rc_yaw_, rc_throttle_;
    int16_t auto_roll_, auto_pitch_, auto_yaw_, auto_throttle_;
    
    bool is_auto_mode_;

    int32_t current_rpm_;  // 최신 꼬리 BLDC RPM (추후 surge speed 제어용)

    double failsafe_timeout_;
    rclcpp::Time last_valid_rc_time_;

    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr motor_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr attitude_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr rc_cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr auto_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rpm_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PidControlNode>());
    rclcpp::shutdown();
    return 0;
}