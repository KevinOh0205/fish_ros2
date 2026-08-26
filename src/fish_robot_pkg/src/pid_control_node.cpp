// =============================================================================
//  pid_control_node.cpp
//
//  역할 : 자세 오차를 PID로 보정해 최종 모터 4채널 출력을 만드는 제어기
//
//   [입력] /filtered/attitude  (Vector3)    <- 현재 자세 + 모드 플래그
//          /rc/command         (Quaternion) <- 조종기 목표값 (수동 모드)
//          /auto/command       (Quaternion) <- 시나리오 목표값 (자동 모드)
//          /sensor/tail_rpm    (Int32)      <- 꼬리 RPM (현재는 수집만)
//   [출력] /motor/output       (UInt16MultiArray[4]) -> uart_bridge_node
//
//  실행 시점 : 타이머가 아니라 /filtered/attitude 수신 콜백에서 PID를 돌린다.
//              즉 제어 주기 = 자세 발행 주기 = IMU 100Hz에 자동으로 동기화된다.
//
//  출력 채널 배치 : [0]=좌 서보, [1]=우 서보, [2]=요 서보, [3]=꼬리 BLDC 스로틀
// =============================================================================

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>

#include <stdio.h>
#include <stdint.h>
#include <vector>
#include <cmath>

// 서보 동작 범위: 혼 기준 약 ±20° (2026-08-26 확정).
//   서보는 HS-5086WP — 표준 RC 신호 ±400us 에 총 91° 를 도는 기종이라
//   0.114°/us 이고, ±20° 는 ±175us, 즉 1325~1675 다.
// nRF 펌웨어 MotorControl.cpp 는 1250~1750 으로 더 넓다. 규칙은 "동일"이 아니라
// **"여기 창이 nRF 창 안에 포함될 것"** 이다 — 여기가 더 좁으면 nRF 클램프에는
// 절대 걸리지 않으므로 안전하고, 여기를 nRF 보다 넓히는 것만 금지다 (N5).
#define SERVO_MIN_US 1325
#define SERVO_MAX_US 1675

class PidControlNode : public rclcpp::Node {
public:
    PidControlNode() : Node("pid_control_node"),
                       // PID 게인: 현재 P만 사용 중 (I/D는 0으로 비활성)
                       // kp 8.75 = 175us / 20°: 오차 1° 당 혼이 약 1° 꺾이고(0.114°/us
                       // 역수), 오차 20° 에서 창 끝(±175us)에 닿는다. 이전 3.5 는
                       // 포화에 오차 71° 가 필요해 벤치에서 "찔끔" 움직였다.
                       // ※ 벤치용 정적 배율이다 — 물에서 출렁이면 낮추는 게 §5 튜닝.
                       kp_r_(8.75f), ki_r_(0.0f), kd_r_(0.0f),
                       kp_p_(8.75f), ki_p_(0.0f), kd_p_(0.0f),
                       kp_y_(1.2f), ki_y_(0.0f), kd_y_(0.0f),
                       i_limit_(300.0f),                       // 적분 와인드업 상한 (ki=0인 지금은 무효)
                       err_sum_roll_(0.0f), err_sum_pitch_(0.0f), err_sum_yaw_(0.0f),
                       prev_err_roll_(0.0f), prev_err_pitch_(0.0f), prev_err_yaw_(0.0f),
                       robot_roll_(0.0f), robot_pitch_(0.0f), robot_yaw_(0.0f),
                       // rc_throttle_를 -9999(두절 센티넬)로 시작 -> 첫 조종기 패킷이
                       // 도착하기 전까지는 페일세이프 상태로 묶어둔다.
                       rc_roll_(0), rc_pitch_(0), rc_yaw_(0), rc_throttle_(-9999),
                       auto_roll_(0), auto_pitch_(0), auto_yaw_(0), auto_throttle_(1000),
                       is_auto_mode_(false),
                       current_rpm_(0),
                       failsafe_timeout_(5.0),                 // 조종기 무수신 허용 시간(초)
                       last_valid_rc_time_(this->now())
    {
        // ── 게인을 ROS 파라미터로 노출 ─────────────────────────────────
        // 물속 튜닝 때 빌드+재시작(약 1분) 없이 즉시 바꾸기 위해서다:
        //   ros2 param set /pid_control_node kp_pitch 10.0
        // 기본값은 위 초기화 목록의 값을 그대로 쓴다 — 진실원은 한 곳이다.
        kp_r_ = (float)this->declare_parameter<double>("kp_roll",  (double)kp_r_);
        ki_r_ = (float)this->declare_parameter<double>("ki_roll",  (double)ki_r_);
        kd_r_ = (float)this->declare_parameter<double>("kd_roll",  (double)kd_r_);
        kp_p_ = (float)this->declare_parameter<double>("kp_pitch", (double)kp_p_);
        ki_p_ = (float)this->declare_parameter<double>("ki_pitch", (double)ki_p_);
        kd_p_ = (float)this->declare_parameter<double>("kd_pitch", (double)kd_p_);
        kp_y_ = (float)this->declare_parameter<double>("kp_yaw",   (double)kp_y_);
        ki_y_ = (float)this->declare_parameter<double>("ki_yaw",   (double)ki_y_);
        kd_y_ = (float)this->declare_parameter<double>("kd_yaw",   (double)kd_y_);
        i_limit_ = (float)this->declare_parameter<double>("i_limit", (double)i_limit_);

        // 런타임 변경 반영. 변경은 WARN 으로 남긴다 — 물속 튜닝에서 "몇 시에 어떤
        // 게인이었나"가 저널에 남아야 CSV 와 대조할 수 있다.
        param_cb_ = this->add_on_set_parameters_callback(
            std::bind(&PidControlNode::on_param_change, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
            "=== [PID Control] 게인: roll %.2f/%.2f/%.2f  pitch %.2f/%.2f/%.2f  yaw %.2f/%.2f/%.2f (kp/ki/kd) ===",
            kp_r_, ki_r_, kd_r_, kp_p_, ki_p_, kd_p_, kp_y_, ki_y_, kd_y_);
        RCLCPP_INFO(this->get_logger(),
            "=== [PID Control] 런타임 튜닝: ros2 param set /pid_control_node kp_pitch <값> ===");

        motor_pub_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("/motor/output", 10);

        // 이 구독이 제어 루프의 심장이다 (콜백 안에서 PID 연산 + 발행까지 수행)
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
        // state_estimation이 roll에 실어 보낸 ±5000 오프셋을 벗겨내 실제 각도를 복원하고,
        // 오프셋 유무 자체를 자동/수동 모드 플래그로 사용한다.
        // (roll 실제 범위는 ±180도라 2500을 넘으면 오프셋이 실린 것이 확실하다)
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
        calculate_pid_control();   // 자세가 갱신될 때마다 즉시 제어 1주기 실행
    }

    void rc_command_callback(const geometry_msgs::msg::Quaternion::SharedPtr msg) {
        // -9999는 nRF52840이 보내는 "조종기 링크 끊김" 센티넬이다.
        // 이 값이 오면 저장된 목표값을 갱신하지 않고 타임스탬프도 찍지 않는다.
        // -> 마지막 정상 수신 시각이 멈추므로 아래 failsafe_timeout_ 검사에 걸린다.
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

    // 런타임 게인 변경. ki 를 바꾸면 그 축의 적분 누산을 0으로 리셋한다 —
    // 누산(err_sum_)은 ki=0 인 동안에도 계속 쌓여 ±i_limit 에 붙어 있으므로,
    // 리셋 없이 ki 를 켜면 그 순간 i_limit×ki 만큼 출력이 점프한다.
    // ※ dt 를 게인에 흡수한 구현이라(아래 주의 참조) I/D 게인은 100Hz 기준 값이다.
    rcl_interfaces::msg::SetParametersResult
    on_param_change(const std::vector<rclcpp::Parameter> &params) {
        for (const auto &prm : params) {
            const std::string &n = prm.get_name();
            const float v = (float)prm.as_double();
            if      (n == "kp_roll")  kp_r_ = v;
            else if (n == "ki_roll")  { ki_r_ = v; err_sum_roll_  = 0.0f; }
            else if (n == "kd_roll")  kd_r_ = v;
            else if (n == "kp_pitch") kp_p_ = v;
            else if (n == "ki_pitch") { ki_p_ = v; err_sum_pitch_ = 0.0f; }
            else if (n == "kd_pitch") kd_p_ = v;
            else if (n == "kp_yaw")   kp_y_ = v;
            else if (n == "ki_yaw")   { ki_y_ = v; err_sum_yaw_   = 0.0f; }
            else if (n == "kd_yaw")   kd_y_ = v;
            else if (n == "i_limit")  i_limit_ = v;
            else continue;
            RCLCPP_WARN(this->get_logger(), "[PID Control] 게인 변경: %s = %.3f", n.c_str(), v);
        }
        rcl_interfaces::msg::SetParametersResult res;
        res.successful = true;
        return res;
    }

    // 서보 출력 클램프: 기구부가 물리적으로 낼 수 있는 각도 밖으로 나가지 않게 막는다.
    uint16_t constrain_servo(float val) {
        // NaN은 모든 비교가 false라 클램프를 통과하고 (uint16_t)NaN=UB(보통 0)가 된다.
        // 명시적으로 걸러 중립(1500)으로 폴백한다 (N6).
        if (std::isnan(val)) return 1500;
        if (val < static_cast<float>(SERVO_MIN_US)) return SERVO_MIN_US;
        if (val > static_cast<float>(SERVO_MAX_US)) return SERVO_MAX_US;
        return static_cast<uint16_t>(val);
    }

    // BLDC 스로틀 클램프: ESC 표준 PWM 범위(1000~2000us)
    uint16_t constrain_pwm(float val) {
        if (std::isnan(val)) return 1000;   // NaN → 최소 스로틀로 안전 폴백 (N6)
        if (val < 1000.0f) return 1000;
        if (val > 2000.0f) return 2000;
        return static_cast<uint16_t>(val);
    }

    void calculate_pid_control() {
        double elapsed_since_last_cmd = (this->now() - last_valid_rc_time_).seconds();

        // ---- [페일세이프] 수동 모드에서 조종기가 끊기면 즉시 중립 출력 후 종료 ----
        // 두 가지 경우를 모두 잡는다:
        //   (1) rc_throttle_ == -9999  : nRF가 명시적으로 "링크 끊김"을 통보
        //   (2) 타임아웃               : nRF 자체가 죽어 /rc/command가 아예 안 오는 경우
        // ※ 자동 모드(is_auto_mode_)에서는 이 검사를 건너뛴다. 조종기 없이 도는 것이
        //    자율 주행의 정상 동작이기 때문이다.
        if (!is_auto_mode_ && (rc_throttle_ == -9999 || elapsed_since_last_cmd > failsafe_timeout_)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "[PID Control] 수동 모드 중 조종기 통신 두절 (Timeout: %.1f초). 모터 정지.", elapsed_since_last_cmd);
            auto motor_output_msg = std_msgs::msg::UInt16MultiArray();
            motor_output_msg.data = {1500, 1500, 1500, 1000};   // 서보 3개 중립 + 추진 정지
            motor_pub_->publish(motor_output_msg);
            return;
        }

        auto motor_output_msg = std_msgs::msg::UInt16MultiArray();
        motor_output_msg.data.resize(4);

        float target_roll, target_pitch, target_yaw;
        uint16_t active_throttle;

        // ---- 목표값 소스 선택 (자동이면 시나리오, 수동이면 조종기) ----
        if (is_auto_mode_) {
            // 0.1도 단위 정수 -> 실제 각도(도)
            target_roll = auto_roll_ * 0.1f;
            target_pitch = auto_pitch_ * 0.1f;
            target_yaw = auto_yaw_ * 0.1f;
            active_throttle = constrain_pwm(auto_throttle_);   // 시나리오는 이미 PWM 단위로 준다
        } else {
            target_roll = rc_roll_ * 0.1f;
            target_pitch = rc_pitch_ * 0.1f;
            target_yaw = rc_yaw_ * 0.1f;
            // 조종기 스로틀 원시 범위(0~1171)를 ESC PWM 범위(1000~2000)로 선형 사상
            int16_t mapped_throttle = static_cast<int16_t>(rc_throttle_ * (1000.0f / 1171.0f));
            active_throttle = constrain_pwm(1000 + mapped_throttle);
        }

        // PID 연산 구역
        // 주의: dt를 곱하지 않는 형태다. 제어 주기가 100Hz로 일정하다는 전제 아래
        //       dt를 게인에 흡수시킨 구현이며, I항은 단순 누산 / D항은 단순 차분이다.
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

        // I항: 오차 누산
        err_sum_roll_ += err_roll;
        err_sum_pitch_ += err_pitch;
        err_sum_yaw_ += err_yaw;

        // 안티 와인드업: 누산값을 ±i_limit_로 잘라 적분항이 폭주하지 않게 한다.
        // (제어가 오래 포화 상태에 머물 때 적분이 무한정 커져 복귀가 늦어지는 현상 방지)
        if (err_sum_roll_ > i_limit_) err_sum_roll_ = i_limit_;
        else if (err_sum_roll_ < -i_limit_) err_sum_roll_ = -i_limit_;

        if (err_sum_pitch_ > i_limit_) err_sum_pitch_ = i_limit_;
        else if (err_sum_pitch_ < -i_limit_) err_sum_pitch_ = -i_limit_;

        if (err_sum_yaw_ > i_limit_) err_sum_yaw_ = i_limit_;
        else if (err_sum_yaw_ < -i_limit_) err_sum_yaw_ = -i_limit_;

        // D항: 직전 주기 대비 오차 변화량
        float d_roll = err_roll - prev_err_roll_;
        float d_pitch = err_pitch - prev_err_pitch_;
        float d_yaw = err_yaw - prev_err_yaw_;

        prev_err_roll_ = err_roll;
        prev_err_pitch_ = err_pitch;
        prev_err_yaw_ = err_yaw;

        // 축별 제어 출력 u = P + I + D
        float u_roll = (kp_r_ * err_roll) + (ki_r_ * err_sum_roll_) + (kd_r_ * d_roll);
        float u_pitch = (kp_p_ * err_pitch) + (ki_p_ * err_sum_pitch_) + (kd_p_ * d_pitch);
        float u_yaw = (kp_y_ * err_yaw) + (ki_y_ * err_sum_yaw_) + (kd_y_ * d_yaw);

        // ---- 모터 믹싱 ----
        // 좌우 가슴지느러미 서보를 차동 구동한다:
        //   두 서보를 같은 방향으로 움직이면 -> 피치(상승/하강)
        //   서로 반대 방향으로 움직이면     -> 롤(좌우 기울기)
        motor_output_msg.data[0] = constrain_servo(1500 + u_pitch + u_roll);   // 좌 서보
        motor_output_msg.data[1] = constrain_servo(1500 + u_pitch - u_roll);   // 우 서보
        motor_output_msg.data[2] = constrain_servo(1500 + u_yaw);              // 요(방향) 서보
        motor_output_msg.data[3] = active_throttle;                            // 꼬리 BLDC 추진

        if (is_auto_mode_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "[AUTO] Target: R=%.1f, P=%.1f | Out: M1=%d, M2=%d",
                target_roll, target_pitch, motor_output_msg.data[0], motor_output_msg.data[1]);
        }

        motor_pub_->publish(motor_output_msg);
    }

    // PID 게인 (r=roll, p=pitch, y=yaw)
    float kp_r_, ki_r_, kd_r_;
    float kp_p_, ki_p_, kd_p_;
    float kp_y_, ki_y_, kd_y_;
    float i_limit_;                                          // 적분 누산 한계 (안티 와인드업)
    float err_sum_roll_, err_sum_pitch_, err_sum_yaw_;       // I항 누산 상태
    float prev_err_roll_, prev_err_pitch_, prev_err_yaw_;    // D항 계산용 직전 오차
    float robot_roll_, robot_pitch_, robot_yaw_;             // 현재 자세(측정값)

    int16_t rc_roll_, rc_pitch_, rc_yaw_, rc_throttle_;          // 수동 목표값 (0.1도 / 스틱 raw)
    int16_t auto_roll_, auto_pitch_, auto_yaw_, auto_throttle_;  // 자동 목표값 (0.1도 / PWM)

    bool is_auto_mode_;

    int32_t current_rpm_;  // 최신 꼬리 BLDC RPM (추후 surge speed 제어용)

    double failsafe_timeout_;          // 조종기 무수신 허용 시간(초)
    rclcpp::Time last_valid_rc_time_;  // 마지막으로 정상 조종기 패킷을 받은 시각

    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr motor_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr attitude_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr rc_cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr auto_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rpm_sub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PidControlNode>());
    rclcpp::shutdown();
    return 0;
}
