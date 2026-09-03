// =============================================================================
//  auto_scenario_node.cpp
//
//  역할 : 자율 주행(AUTO) 모드일 때 시간 기반으로 목표 자세를 생성하는 궤적 엔진
//
//   [입력] /filtered/attitude (Vector3)  <- 자세 + 모드 플래그(±5000 오프셋)
//   [출력] /auto/command     (Quaternion) -> pid_control_node의 목표값
//
//  현재 시나리오 (2026-09-03) : **수평 자세로 20% 추력, 3초 전진 후 정지.**
//         AUTO 로 들어간 순간부터 3초간만 추력을 주고, 그 뒤로는 중립·정지를
//         유지한다. 다시 달리려면 MANUAL 로 나갔다가 AUTO 로 재진입한다
//         (진입 순간 스톱워치가 0으로 리셋되므로 btn1 두 번이면 재실행).
//
//         종전 시나리오는 6초 주기 피치 ±10° 왕복이었다. 물속 첫 검증을 위해
//         "직진만" 으로 단순화했다 — 자세를 흔들면 추력·자세 어느 쪽 문제인지
//         가릴 수 없기 때문이다.
//
//  ※ 시간·추력은 ROS 파라미터다. 물속에서 재빌드 없이 바꾼다:
//       ros2 param set /auto_scenario_node run_sec 5.0
//       ros2 param set /auto_scenario_node throttle_pct 30.0
//     변경은 다음 AUTO 진입부터가 아니라 **즉시** 반영된다(달리는 중에도 바뀐다).
//
//  ※ 모드 전달 규약: state_estimation_ekf_node가 자동 모드일 때 roll 값에 ±5000을
//     더해 발행한다. 별도 모드 토픽 없이 자세 토픽 하나로 모드를 함께 실어 나르는
//     방식이며, 이 노드와 pid_control_node / data_logger_node가 동일하게 디코딩한다.
//
//  ※ 주의: AUTO 에서는 pid_control_node 의 조종기 페일세이프가 통째로 건너뛰어진다
//     (조종기 없이 도는 것이 정상 사용이므로). AUTO 중에는 송신기를 꺼도 멈추지
//     않는다 — 멈추려면 btn1 을 눌러 MANUAL 로 나와야 한다.
// =============================================================================

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

class AutoScenarioNode : public rclcpp::Node {
public:
    AutoScenarioNode() : Node("auto_scenario_node"),
                         is_auto_mode_(false),
                         prev_auto_mode_(false),
                         run_done_logged_(false)
    {
        // ── 시나리오 파라미터 ──────────────────────────────────────────────
        // 기본값을 초기화 목록이 아니라 여기서 정한다 — 진실원은 한 곳이다.
        run_sec_      = this->declare_parameter<double>("run_sec", 3.0);
        throttle_pct_ = this->declare_parameter<double>("throttle_pct", 20.0);

        param_cb_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &ps) {
                rcl_interfaces::msg::SetParametersResult r;
                r.successful = true;
                for (const auto &p : ps) {
                    if (p.get_name() == "run_sec") {
                        run_sec_ = p.as_double();
                        RCLCPP_WARN(this->get_logger(), "[AUTO] run_sec -> %.2f 초", run_sec_);
                    } else if (p.get_name() == "throttle_pct") {
                        throttle_pct_ = p.as_double();
                        RCLCPP_WARN(this->get_logger(), "[AUTO] throttle_pct -> %.1f %% (PWM %u)",
                                    throttle_pct_, pct_to_pwm(throttle_pct_));
                    }
                }
                return r;
            });

        RCLCPP_INFO(this->get_logger(),
            "=== [Auto Scenario] 직진 시나리오: %.1f초 동안 추력 %.0f%% (PWM %u) ===",
            run_sec_, throttle_pct_, pct_to_pwm(throttle_pct_));

        auto_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Quaternion>("/auto/command", 10);

        // 자세 토픽은 모드 감별 용도로만 구독한다 (각도 자체는 PID 노드가 쓴다)
        attitude_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/filtered/attitude", 10, std::bind(&AutoScenarioNode::attitude_callback, this, std::placeholders::_1));

        // 100Hz 궤적 생성 루프
        loop_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&AutoScenarioNode::scenario_loop_callback, this));
    }

private:
    // 추력 % -> ESC PWM(us). 1000 = 정지, 2000 = 최대.
    static uint16_t pct_to_pwm(double pct) {
        if (pct < 0.0)   pct = 0.0;
        if (pct > 100.0) pct = 100.0;
        return static_cast<uint16_t>(1000.0 + pct * 10.0);
    }

    void attitude_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        // 상태 추정 노드에서 보낸 ±5000 오프셋을 통해 모드 감별
        // roll 실제 범위는 ±180도이므로, |x| > 2500이면 오프셋이 실린 것 = 자동 모드
        is_auto_mode_ = (msg->x > 2500.0f || msg->x < -2500.0f);
    }

    void scenario_loop_callback() {
        auto cmd_msg = geometry_msgs::msg::Quaternion();

        // 1. 상태 전이(Rising Edge) 감지: 수동에서 자동으로 막 넘어온 순간.
        //    이 처리가 없으면 노드 기동 시각부터 경과 시간을 세게 되어,
        //    자동 모드로 들어가자마자 시나리오가 이미 끝난 상태로 시작한다.
        if (is_auto_mode_ && !prev_auto_mode_) {
            scenario_start_time_ = this->now();   // 이 순간을 시나리오의 0초로 기록
            run_done_logged_     = false;
            RCLCPP_WARN(this->get_logger(), "=========================================");
            RCLCPP_WARN(this->get_logger(), "[AUTO SCENARIO] 시작! %.1f초간 추력 %.0f%% 전진합니다.",
                        run_sec_, throttle_pct_);
            RCLCPP_WARN(this->get_logger(), "=========================================");
        }

        if (is_auto_mode_) {
            // 2. 시나리오 경과 시간(스톱워치). scenario_start_time_ 은 위 rising edge
            //    블록에서 항상 먼저 설정되므로 미초기화 상태로 여기 도달하지 않는다.
            const double elapsed_sec = (this->now() - scenario_start_time_).seconds();

            // 3. 자세는 전 구간 수평 유지. 직진 검증이 목적이라 자세를 흔들지 않는다.
            //    단위 규약: x/y/z 는 0.1도 단위(PID 노드가 *0.1f 로 환산), w 는 PWM(us).
            cmd_msg.x = 0.0;   // Roll  0도
            cmd_msg.y = 0.0;   // Pitch 0도  (이 로봇은 **양수 = 코가 아래**다)
            cmd_msg.z = 0.0;   // Yaw   0도  (현재 헤딩 유지)

            if (elapsed_sec < run_sec_) {
                cmd_msg.w = static_cast<double>(pct_to_pwm(throttle_pct_));
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "[SCENARIO RUN] %.1f / %.1f초  추력 %.0f%% (PWM %.0f)",
                    elapsed_sec, run_sec_, throttle_pct_, cmd_msg.w);
            } else {
                // 4. 주행 종료 — 추력만 끊고 자세는 계속 수평을 지시한다.
                //    (자세 지시를 놓으면 PID 목표가 사라지는 게 아니라 마지막 값이
                //     남으므로, 명시적으로 수평을 계속 준다.)
                cmd_msg.w = 1000.0;
                if (!run_done_logged_) {
                    run_done_logged_ = true;
                    RCLCPP_WARN(this->get_logger(),
                        "[AUTO SCENARIO] %.1f초 주행 완료 — 추력 정지. "
                        "다시 달리려면 btn1 으로 MANUAL 로 나갔다가 AUTO 로 재진입하세요.",
                        run_sec_);
                }
            }

        } else {
            // [수동 모드]: 중립 대기
            // 자동 모드로 전환되는 순간 직전 값이 남아있지 않도록 계속 중립을 덮어쓴다.
            cmd_msg.x = 0.0;
            cmd_msg.y = 0.0;
            cmd_msg.z = 0.0;
            cmd_msg.w = 1000.0;   // 스로틀 최소(정지)
        }

        // 현재 모드를 '과거'로 저장하여 다음 루프에서 변화를 감지할 수 있게 함
        prev_auto_mode_ = is_auto_mode_;

        auto_cmd_pub_->publish(cmd_msg);
    }

    bool is_auto_mode_;
    bool prev_auto_mode_;      // 상태 전이 판별용 과거 상태 변수
    bool run_done_logged_;     // 종료 로그를 한 번만 찍기 위한 래치

    double run_sec_;           // 주행 시간(초)
    double throttle_pct_;      // 추력(%)

    rclcpp::Time scenario_start_time_;   // 자동 모드 진입 시각 (스톱워치 기준점)

    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr auto_cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr attitude_sub_;
    rclcpp::TimerBase::SharedPtr loop_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoScenarioNode>());
    rclcpp::shutdown();
    return 0;
}
