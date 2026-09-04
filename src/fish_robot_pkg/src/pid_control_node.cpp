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
                       // 서보 방향: 우측 지느러미는 몸 반대편에 **거울상으로 장착**돼
                       // 같은 PWM 증가가 물리적으로 반대 회전이 된다. 실측(2026-08-26):
                       // 피치 명령에 좌우가 반대로 돌고(=물리 롤), 롤 명령에 같이 돌았다
                       // (=물리 피치) — 명령의 자세↔서보 상관은 정상(−0.93/−0.96)이었으므로
                       // 원인은 믹싱이 아니라 장착 방향이다.
                       //
                       // 2026-09-04 벤치에서 PID·스틱을 빼고 /motor/output 에 고정 PWM 을
                       // 직접 꽂아 확인했다: **좌우에 같은 PWM(1675)을 주면 지느러미가
                       // 서로 반대로 벌어진다** = 거울상 장착이 맞다. 따라서 피치(지느러미
                       // 같은 방향)를 내려면 PWM 은 반대여야 하고, 우측만 반전이 옳다.
                       //
                       // 같은 날 이 값을 false 로 잠깐 뒤집었다가 AUTO 시나리오 2(피치 다운)
                       // 에서 양쪽이 반대로 벌어져 되돌렸다. **수동에서 보이던 이상은 이
                       // 값이 아니라 스틱 롤/피치 스왑이 원인이었다** (swap_rc_rp_ 참조) —
                       // 수동만 보고 이 값을 만지면 AUTO 가 깨진다. 판정은 AUTO 로 할 것.
                       servo_rev_l_(false), servo_rev_r_(true), servo_rev_y_(false),
                       // 축 부호. 피치는 IMU 규약이 항공과 반대(양수 pitch = 기수 아래)라
                       // 뒤집힐 소지가 상시 있다 — 벤치에서 지느러미를 보고 확정할 것.
                       invert_pitch_(false), invert_roll_(false),
                       // 조종기 짐벌 배선 스왑을 파이에서 우회하던 스위치. 조종기 펌웨어를
                       // 고쳤으므로(2026-09-04) 기본값은 false 다 — 켜 두면 두 번 뒤집혀 원위치.
                       swap_rc_rp_(false),
                       // 스틱 부호. 조종기가 스틱 다운에 음수를 보내는데 이 IMU 규약은
                       // 양수가 기수 아래라, 피치만 뒤집어야 다운이 다운이 된다.
                       rc_inv_roll_(false), rc_inv_pitch_(true),
                       // 스틱 데드밴드: 실측(45초 정지)에서 roll 스틱이 중립인데 −12까지
                       // 방황했고(트림 오프셋+잡음), 스로틀이 0→28 스파이크를 냈다.
                       rc_deadband_(15), throttle_deadband_(40),
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
        servo_rev_l_ = this->declare_parameter<bool>("servo_reverse_left",  servo_rev_l_);
        servo_rev_r_ = this->declare_parameter<bool>("servo_reverse_right", servo_rev_r_);
        servo_rev_y_ = this->declare_parameter<bool>("servo_reverse_yaw",   servo_rev_y_);
        // 축 부호 — 서보 반전과 목적이 다르다. 믹싱 주석의 1)/2) 구분을 볼 것.
        invert_pitch_ = this->declare_parameter<bool>("invert_pitch", invert_pitch_);
        invert_roll_  = this->declare_parameter<bool>("invert_roll",  invert_roll_);
        swap_rc_rp_   = this->declare_parameter<bool>("swap_rc_roll_pitch", swap_rc_rp_);
        rc_inv_roll_  = this->declare_parameter<bool>("invert_rc_roll",  rc_inv_roll_);
        rc_inv_pitch_ = this->declare_parameter<bool>("invert_rc_pitch", rc_inv_pitch_);
        rc_deadband_       = (int)this->declare_parameter<int64_t>("rc_deadband",       rc_deadband_);
        throttle_deadband_ = (int)this->declare_parameter<int64_t>("throttle_deadband", throttle_deadband_);

        // 런타임 변경 반영. 변경은 WARN 으로 남긴다 — 물속 튜닝에서 "몇 시에 어떤
        // 게인이었나"가 저널에 남아야 CSV 와 대조할 수 있다.
        param_cb_ = this->add_on_set_parameters_callback(
            std::bind(&PidControlNode::on_param_change, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
            "=== [PID Control] 게인: roll %.2f/%.2f/%.2f  pitch %.2f/%.2f/%.2f  yaw %.2f/%.2f/%.2f (kp/ki/kd) ===",
            kp_r_, ki_r_, kd_r_, kp_p_, ki_p_, kd_p_, kp_y_, ki_y_, kd_y_);
        RCLCPP_INFO(this->get_logger(),
            "=== [PID Control] 서보 반전: 좌 %d / 우 %d / 요 %d  스틱 데드밴드: rpy %d, 스로틀 %d ===",
            servo_rev_l_, servo_rev_r_, servo_rev_y_, rc_deadband_, throttle_deadband_);
        RCLCPP_INFO(this->get_logger(),
            "=== [PID Control] 축 부호 반전: 피치 %d / 롤 %d  (서보 반전과 다른 것 — 축 하나만 뒤집는다) ===",
            invert_pitch_, invert_roll_);
        RCLCPP_INFO(this->get_logger(),
            "=== [PID Control] 수동 스틱 롤<->피치 스왑: %d  (AUTO 경로에는 영향 없음) ===", swap_rc_rp_);
        RCLCPP_INFO(this->get_logger(),
            "=== [PID Control] 수동 스틱 부호 반전: 롤 %d / 피치 %d  (AUTO 는 규약이 달라 제외) ===",
            rc_inv_roll_, rc_inv_pitch_);
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
            // **스틱 축 스왑 우회 스위치. 지금은 꺼져 있는 것이 정상이다.**
            //
            // 2026-09-04 실측: 조종기 스틱을 가로로만 흔들면 /rc/command 의 y(피치)가,
            // 세로로만 흔들면 x(롤)가 움직였다 — 조종기 짐벌 배선이 뒤바뀌어 있었다
            // (세로축 포텐쇼미터가 roll_pin 에). 그래서 피치 스틱이 롤 명령이 되어
            // "피치를 주는데 지느러미가 롤처럼 반대로 움직인다" 가 나왔다.
            //
            // **근본 수정은 조종기 펌웨어에서 했다** (Input_Manager.cpp 의 roll_pin/
            // pitch_pin #define 교체, 2026-09-04). 그래서 이 스위치는 false 다.
            // 조종기를 고친 채로 이걸 켜면 두 번 뒤집혀 다시 망가진다.
            //
            // ※ 이건 **수동 경로만**의 문제였다. AUTO 는 /auto/command 로 따로 오므로
            //   영향이 없다 — 그래서 서보 반전으로 수동을 맞추면 AUTO 가 틀어지고,
            //   AUTO 를 맞추면 수동이 틀어지는 모순이 생겼다. 두 문제가 겹쳐 서로를
            //   가리고 있었던 것이다(다른 하나는 servo_rev_r_ 주석 참조).
            // ※ /rc/command 원시값은 건드리지 않는다 — 로깅·진단이 원본을 봐야 한다.
            //   그래서 이 스위치를 켜면 CSV 의 RC_R/RC_P 열은 뒤바뀐 채로 남는다.
            if (swap_rc_rp_) {
                rc_roll_  = static_cast<int16_t>(msg->y);
                rc_pitch_ = static_cast<int16_t>(msg->x);
            } else {
                rc_roll_  = static_cast<int16_t>(msg->x);
                rc_pitch_ = static_cast<int16_t>(msg->y);
            }
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
            if (n == "servo_reverse_left")  { servo_rev_l_ = prm.as_bool(); RCLCPP_WARN(this->get_logger(), "[PID Control] 서보 반전 변경: 좌 = %d", servo_rev_l_); continue; }
            if (n == "servo_reverse_right") { servo_rev_r_ = prm.as_bool(); RCLCPP_WARN(this->get_logger(), "[PID Control] 서보 반전 변경: 우 = %d", servo_rev_r_); continue; }
            if (n == "servo_reverse_yaw")   { servo_rev_y_ = prm.as_bool(); RCLCPP_WARN(this->get_logger(), "[PID Control] 서보 반전 변경: 요 = %d", servo_rev_y_); continue; }
            if (n == "invert_pitch")        { invert_pitch_ = prm.as_bool(); RCLCPP_WARN(this->get_logger(), "[PID Control] 축 부호 변경: 피치 반전 = %d", invert_pitch_); continue; }
            if (n == "invert_roll")         { invert_roll_  = prm.as_bool(); RCLCPP_WARN(this->get_logger(), "[PID Control] 축 부호 변경: 롤 반전 = %d",  invert_roll_);  continue; }
            if (n == "swap_rc_roll_pitch")  { swap_rc_rp_   = prm.as_bool(); RCLCPP_WARN(this->get_logger(), "[PID Control] 수동 스틱 롤<->피치 스왑 = %d", swap_rc_rp_); continue; }
            if (n == "invert_rc_roll")      { rc_inv_roll_  = prm.as_bool(); RCLCPP_WARN(this->get_logger(), "[PID Control] 수동 스틱 부호: 롤 반전 = %d",  rc_inv_roll_);  continue; }
            if (n == "invert_rc_pitch")     { rc_inv_pitch_ = prm.as_bool(); RCLCPP_WARN(this->get_logger(), "[PID Control] 수동 스틱 부호: 피치 반전 = %d", rc_inv_pitch_); continue; }
            if (n == "rc_deadband")       { rc_deadband_       = (int)prm.as_int(); RCLCPP_WARN(this->get_logger(), "[PID Control] rc_deadband = %d", rc_deadband_); continue; }
            if (n == "throttle_deadband") { throttle_deadband_ = (int)prm.as_int(); RCLCPP_WARN(this->get_logger(), "[PID Control] throttle_deadband = %d", throttle_deadband_); continue; }
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
            // 스틱 데드밴드 — **MANUAL 분기에만** 둔다. AUTO 경로(auto_*)에 새면 시나리오
            // 명령이 깎이고, uart_bridge 에서 깎으면 data_logger 의 원시 스틱 기록이
            // 사라져 오늘 같은 잡음 진단이 불가능해진다. 여기가 유일하게 옳은 자리다.
            // roll/pitch/yaw 는 소프트(재중심): 경계 불연속 없이 중심 부근 미세 트림을
            // 지킨다. 최대 목표각이 25.5°→24.0° 로 주는 것은 무시한다.
            auto soft_db = [this](int16_t st) -> int16_t {
                if (st >  rc_deadband_) return (int16_t)(st - rc_deadband_);
                if (st < -rc_deadband_) return (int16_t)(st + rc_deadband_);
                return 0;
            };
            // **스틱 부호 — MANUAL 에만 건다. AUTO 와 규약이 다르기 때문이다.**
            //
            // 이 IMU 는 **양수 pitch = 기수 아래**다(항공 규약 반대, 실측 확정).
            // AUTO 시나리오는 그 규약대로 쓰여 있다 — dive 가 pitch = +angle 이다.
            // 반면 조종기는 스틱을 내리면 −256 이 나온다. 그대로 목표각으로 쓰면
            // "다운을 줬는데 기수가 올라간다" 가 된다 (2026-09-04 실기 확인).
            //
            // 그래서 뒤집는 자리는 여기, 수동 분기다. 축 부호(invert_pitch_)나
            // 서보 반전으로 고치면 **AUTO 까지 같이 뒤집혀** 멀쩡한 시나리오가 깨진다.
            // 근본 수정은 조종기의 map() 방향이고, 그쪽을 고치면 이 값을 false 로.
            target_roll  = (rc_inv_roll_  ? -1.0f : 1.0f) * soft_db(rc_roll_)  * 0.1f;
            target_pitch = (rc_inv_pitch_ ? -1.0f : 1.0f) * soft_db(rc_pitch_) * 0.1f;
            target_yaw   = soft_db(rc_yaw_)   * 0.1f;
            // 스로틀은 하드 데드밴드: "의도적 입력 전까지 ESC 는 정확히 1000" 이라는
            // 안전 속성이 연속성보다 중요하다. −9999 센티넬은 위 페일세이프가 이미
            // 걸렀으므로 여기 도달한 rc_throttle_ 은 실제 스틱 값이다.
            int16_t thr = (rc_throttle_ < throttle_deadband_) ? 0 : rc_throttle_;
            // 조종기 스로틀 원시 범위(0~1171)를 ESC PWM 범위(1000~2000)로 선형 사상
            int16_t mapped_throttle = static_cast<int16_t>(thr * (1000.0f / 1171.0f));
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
        //
        // **부호가 두 종류이고 고치는 자리가 다르다 — 섞으면 무한히 헛돈다.**
        //
        //   1) 축 부호 (invert_pitch_ / invert_roll_) — 그 축 **하나만** 뒤집는다.
        //      "피치는 반대인데 롤은 맞다" 는 여기서 고친다.
        //   2) 서보 반전 (servo_rev_*) — 그 **서보 하나만** 뒤집는다.
        //      "우측 지느러미만 반대로 간다" 는 여기서 고친다.
        //
        //   양쪽 서보를 동시에 반전하는 것은 **축 부호를 둘 다 뒤집는 것과 같다** —
        //   피치를 고치려고 좌우를 함께 반전하면 롤이 딸려서 뒤집힌다 (2026-09-04
        //   실기에서 실제로 이 함정에 걸렸다). 축 하나만 문제면 반드시 1) 을 쓴다.
        //
        // 2026-08-26 에는 우측만 반전(true)했는데 **물리 확인 없이 PWM 부호로
        // 추론한** 방향이었고 증상이 그대로였다. 2026-09-04 벤치에서 PID·스틱을
        // 빼고 /motor/output 에 고정 PWM 을 직접 꽂아 눈으로 확인해 정정했다.
        // 부호 판정은 반드시 지느러미를 보고 한다 — PWM 부호로 추론하지 말 것.
        float p_term = invert_pitch_ ? -u_pitch : u_pitch;
        float r_term = invert_roll_  ? -u_roll  : u_roll;
        float d_left  = p_term + r_term;
        float d_right = p_term - r_term;
        float d_yaw_s = u_yaw;
        if (servo_rev_l_) d_left  = -d_left;
        if (servo_rev_r_) d_right = -d_right;
        if (servo_rev_y_) d_yaw_s = -d_yaw_s;
        motor_output_msg.data[0] = constrain_servo(1500 + d_left);    // 좌 서보
        motor_output_msg.data[1] = constrain_servo(1500 + d_right);   // 우 서보
        motor_output_msg.data[2] = constrain_servo(1500 + d_yaw_s);   // 요(방향) 서보
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
    bool servo_rev_l_, servo_rev_r_, servo_rev_y_;           // 채널별 장착 방향 반전
    bool invert_pitch_, invert_roll_;                        // 축별 부호 반전 (믹싱 이전)
    bool swap_rc_rp_;                                        // 수동 스틱 롤<->피치 스왑
    bool rc_inv_roll_, rc_inv_pitch_;                        // 수동 스틱 부호 반전 (AUTO 제외)
    int  rc_deadband_, throttle_deadband_;                   // 스틱 데드밴드 [카운트]
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
