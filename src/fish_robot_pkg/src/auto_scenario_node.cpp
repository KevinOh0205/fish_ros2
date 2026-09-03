// =============================================================================
//  auto_scenario_node.cpp
//
//  역할 : 자율 주행(AUTO) 모드일 때 시간 기반으로 목표 자세를 생성하는 궤적 엔진
//
//   [입력] /filtered/attitude (Vector3)  <- 자세 + 모드 플래그(±5000 오프셋)
//   [출력] /auto/command     (Quaternion) -> pid_control_node의 목표값
//
//  ── 시나리오 교체 방식 (2026-09-03) ────────────────────────────────────────
//  시나리오는 **단계표(Step 배열)** 하나로 표현한다. 단계마다 지속시간과
//  목표 자세·추력을 적어두면 되고, 새 시나리오를 넣는 일은 표를 하나 더
//  쓰는 것이 전부다(build_steps() 참조).
//
//  선택은 ROS 파라미터로 한다. 재빌드도 재시작도 필요 없다:
//     ros2 param set /auto_scenario_node scenario dive
//     ros2 param set /auto_scenario_node run_sec 5.0
//     ros2 param set /auto_scenario_node throttle_pct 30.0
//     ros2 param set /auto_scenario_node angle_deg 15.0
//
//  **바뀐 값은 다음 AUTO 진입부터 적용된다** (달리는 도중에 궤적이 갈아엎히면
//  물속에서 무슨 일이 일어난 건지 사후에 못 가린다). btn1 로 나갔다 들어오면 된다.
//
//  공통 규칙 두 가지:
//    · 어떤 시나리오든 **run_sec 초가 지나면 추력을 끊고 수평으로 정지**한다.
//      단계표가 그보다 짧으면 표를 처음부터 되풀이한다(왕복 패턴이 이렇게 나온다).
//    · 단계의 추력 -1 은 "throttle_pct 를 쓴다"는 뜻이다. 운용자가 안전 추력을
//      한 곳에서만 낮출 수 있게 하기 위한 것이다.
//
//  ── 시나리오 목록 ─────────────────────────────────────────────────────────
//    straight  (기본) 수평 직진.                     ← 물속 첫 검증용
//    dive      코를 angle_deg 만큼 아래로 하고 전진   (이 로봇은 **양수 피치 = 코 아래**)
//    climb     코를 angle_deg 만큼 위로 하고 전진
//    turn      진입 시점 헤딩에서 angle_deg 만큼 튼 헤딩을 목표로 전진
//    porpoise  3초마다 피치 ±10° 왕복하며 전진        (구 시나리오. run_sec 을 늘려 쓸 것)
//    attitude  추력 0 으로 피치 ±10° 왕복             ← 벤치에서 서보만 볼 때
//
//  ※ turn 의 한계: pid_control_node 의 요 오차는 `target - robot` 이고 **±180
//     경계를 감싸지 않는다.** 진입 헤딩이 ±180 부근이면 목표가 경계를 넘으면서
//     로봇이 먼 쪽으로 돌 수 있다. 경계에서 떨어진 곳에서 쓰거나, PID 쪽에
//     wrap 을 넣은 뒤 쓸 것.
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
#include <vector>
#include <string>
#include <cmath>

class AutoScenarioNode : public rclcpp::Node {
public:
    // 시나리오 한 단계. 각도는 도(deg), 추력은 % (음수면 throttle_pct 를 쓴다).
    // yaw 는 **진입 시점 헤딩 기준 상대각**이다 — 절대 방위가 아니다.
    struct Step {
        double sec;
        double roll;
        double pitch;
        double yaw_rel;
        double thr_pct;
    };

    AutoScenarioNode() : Node("auto_scenario_node"),
                         is_auto_mode_(false),
                         prev_auto_mode_(false),
                         run_done_logged_(false),
                         last_yaw_(0.0f),
                         entry_yaw_(0.0f)
    {
        // ── 파라미터 (기본값의 진실원은 여기 한 곳) ────────────────────────
        scenario_     = this->declare_parameter<std::string>("scenario", "straight");
        run_sec_      = this->declare_parameter<double>("run_sec", 3.0);
        throttle_pct_ = this->declare_parameter<double>("throttle_pct", 20.0);
        angle_deg_    = this->declare_parameter<double>("angle_deg", 15.0);

        param_cb_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &ps) {
                rcl_interfaces::msg::SetParametersResult r;
                r.successful = true;
                for (const auto &p : ps) {
                    const auto &n = p.get_name();
                    if (n == "scenario") {
                        const std::string v = p.as_string();
                        if (!is_known_scenario(v)) {
                            r.successful = false;
                            r.reason = "모르는 시나리오: " + v +
                                       " (straight/dive/climb/turn/porpoise/attitude)";
                            RCLCPP_ERROR(this->get_logger(), "[AUTO] %s", r.reason.c_str());
                            continue;
                        }
                        scenario_ = v;
                    } else if (n == "run_sec")      run_sec_      = p.as_double();
                    else if (n == "throttle_pct")   throttle_pct_ = p.as_double();
                    else if (n == "angle_deg")      angle_deg_    = p.as_double();
                    else continue;

                    RCLCPP_WARN(this->get_logger(),
                        "[AUTO] %s 변경 -> 현재 설정: %s / %.1f초 / 추력 %.0f%% / 각도 %.1f도"
                        "  (**다음 AUTO 진입부터 적용**)",
                        n.c_str(), scenario_.c_str(), run_sec_, throttle_pct_, angle_deg_);
                }
                return r;
            });

        RCLCPP_INFO(this->get_logger(),
            "=== [Auto Scenario] 시나리오 '%s' / %.1f초 / 추력 %.0f%% / 각도 %.1f도 ===",
            scenario_.c_str(), run_sec_, throttle_pct_, angle_deg_);

        auto_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Quaternion>("/auto/command", 10);

        // 자세 토픽은 모드 감별과 진입 헤딩 포착에만 쓴다 (제어각은 PID 노드가 쓴다)
        attitude_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/filtered/attitude", 10, std::bind(&AutoScenarioNode::attitude_callback, this, std::placeholders::_1));

        loop_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&AutoScenarioNode::scenario_loop_callback, this));
    }

private:
    static bool is_known_scenario(const std::string &s) {
        return s == "straight" || s == "dive" || s == "climb"
            || s == "turn"     || s == "porpoise" || s == "attitude";
    }

    // ── 시나리오 정의 — 새 시나리오는 여기에 한 갈래 추가하면 끝난다 ──────
    std::vector<Step> build_steps() const {
        const double A = angle_deg_;
        if (scenario_ == "dive")     return {{run_sec_, 0.0,  A,   0.0, -1.0}};
        if (scenario_ == "climb")    return {{run_sec_, 0.0, -A,   0.0, -1.0}};
        if (scenario_ == "turn")     return {{run_sec_, 0.0,  0.0, A,   -1.0}};
        if (scenario_ == "porpoise") return {{3.0, 0.0,  10.0, 0.0, -1.0},
                                             {3.0, 0.0, -10.0, 0.0, -1.0}};
        if (scenario_ == "attitude") return {{3.0, 0.0,  10.0, 0.0,  0.0},
                                             {3.0, 0.0, -10.0, 0.0,  0.0}};
        return {{run_sec_, 0.0, 0.0, 0.0, -1.0}};    // straight (기본)
    }

    // 추력 % -> ESC PWM(us). 1000 = 정지, 2000 = 최대.
    static uint16_t pct_to_pwm(double pct) {
        if (pct < 0.0)   pct = 0.0;
        if (pct > 100.0) pct = 100.0;
        return static_cast<uint16_t>(1000.0 + pct * 10.0);
    }

    // 헤딩을 (-180, 180] 로 접는다.
    static double wrap180(double d) {
        while (d >  180.0) d -= 360.0;
        while (d <= -180.0) d += 360.0;
        return d;
    }

    void attitude_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        // roll 에 실린 ±5000 오프셋으로 모드 감별.
        // roll 실제 범위는 ±180도이므로 |x| > 2500 이면 오프셋이 실린 것 = 자동 모드.
        is_auto_mode_ = (msg->x > 2500.0f || msg->x < -2500.0f);
        last_yaw_     = msg->z;    // 진입 시점 헤딩을 잡아두기 위해 계속 받아둔다
    }

    void scenario_loop_callback() {
        auto cmd_msg = geometry_msgs::msg::Quaternion();

        // 1. 상태 전이(Rising Edge): 수동에서 자동으로 막 넘어온 순간.
        //    이 처리가 없으면 노드 기동 시각부터 세게 되어, 자동 모드로 들어가자마자
        //    시나리오가 이미 끝난 상태로 시작한다.
        if (is_auto_mode_ && !prev_auto_mode_) {
            scenario_start_time_ = this->now();
            run_done_logged_     = false;
            // **진입 순간의 설정을 통째로 잠근다.** 달리는 도중 파라미터가 바뀌어도
            // 이번 주행은 진입 때 값으로 끝까지 간다 — 사후 분석에서 "무엇을 지시했나"가
            // 한 값으로 확정되게 하기 위해서다.
            active_scenario_ = scenario_;
            active_run_sec_  = run_sec_;
            active_thr_pct_  = throttle_pct_;
            active_steps_    = build_steps();
            entry_yaw_       = last_yaw_;

            double table_sec = 0.0;
            for (const auto &s : active_steps_) table_sec += s.sec;

            RCLCPP_WARN(this->get_logger(), "=========================================");
            RCLCPP_WARN(this->get_logger(),
                "[AUTO SCENARIO] 시작! '%s'  %.1f초  추력 %.0f%%  진입 헤딩 %.1f도",
                active_scenario_.c_str(), active_run_sec_, active_thr_pct_, entry_yaw_);
            if (table_sec > active_run_sec_ + 1e-6) {
                RCLCPP_WARN(this->get_logger(),
                    "[AUTO SCENARIO] 주의: 단계표가 %.1f초인데 run_sec 은 %.1f초 — "
                    "뒷단계는 실행되지 않는다. run_sec 을 늘리세요.", table_sec, active_run_sec_);
            }
            RCLCPP_WARN(this->get_logger(), "=========================================");
        }

        if (is_auto_mode_) {
            const double elapsed = (this->now() - scenario_start_time_).seconds();

            if (elapsed < active_run_sec_ && !active_steps_.empty()) {
                // 2. 단계표에서 지금 단계를 고른다. 표가 run_sec 보다 짧으면 되풀이한다.
                double table_sec = 0.0;
                for (const auto &s : active_steps_) table_sec += s.sec;
                double u = (table_sec > 0.0) ? std::fmod(elapsed, table_sec) : 0.0;

                const Step *cur = &active_steps_.back();
                for (const auto &s : active_steps_) {
                    if (u < s.sec) { cur = &s; break; }
                    u -= s.sec;
                }

                // 3. 단위 규약: x/y/z 는 0.1도 단위(PID 가 *0.1f 로 환산), w 는 PWM(us).
                //    yaw 는 진입 헤딩 기준 상대각이라 여기서 절대각으로 바꿔 보낸다.
                const double thr_pct = (cur->thr_pct < 0.0) ? active_thr_pct_ : cur->thr_pct;
                cmd_msg.x = cur->roll  * 10.0;
                cmd_msg.y = cur->pitch * 10.0;
                cmd_msg.z = wrap180(entry_yaw_ + cur->yaw_rel) * 10.0;
                cmd_msg.w = static_cast<double>(pct_to_pwm(thr_pct));

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "[SCENARIO RUN] '%s' %.1f/%.1f초  롤 %.1f 피치 %.1f 요 %.1f도  추력 %.0f%%",
                    active_scenario_.c_str(), elapsed, active_run_sec_,
                    cmd_msg.x * 0.1, cmd_msg.y * 0.1, cmd_msg.z * 0.1, thr_pct);
            } else {
                // 4. 주행 종료 — 추력을 끊고 수평을 계속 지시한다.
                //    (자세 지시를 놓으면 목표가 사라지는 게 아니라 마지막 값이 남으므로
                //     명시적으로 수평을 계속 준다.)
                cmd_msg.x = 0.0;
                cmd_msg.y = 0.0;
                cmd_msg.z = 0.0;
                cmd_msg.w = 1000.0;
                if (!run_done_logged_) {
                    run_done_logged_ = true;
                    RCLCPP_WARN(this->get_logger(),
                        "[AUTO SCENARIO] '%s' %.1f초 주행 완료 — 추력 정지. "
                        "다시 달리려면 btn1 으로 MANUAL 로 나갔다가 AUTO 로 재진입하세요.",
                        active_scenario_.c_str(), active_run_sec_);
                }
            }

        } else {
            // [수동 모드]: 중립 대기.
            // 자동 모드로 전환되는 순간 직전 값이 남아있지 않도록 계속 중립을 덮어쓴다.
            cmd_msg.x = 0.0;
            cmd_msg.y = 0.0;
            cmd_msg.z = 0.0;
            cmd_msg.w = 1000.0;   // 스로틀 최소(정지)
        }

        prev_auto_mode_ = is_auto_mode_;
        auto_cmd_pub_->publish(cmd_msg);
    }

    bool is_auto_mode_;
    bool prev_auto_mode_;      // 상태 전이 판별용 과거 상태
    bool run_done_logged_;     // 종료 로그를 한 번만 찍기 위한 래치
    float last_yaw_;           // 최신 헤딩 (진입 순간 포착용)
    float entry_yaw_;          // 이번 주행의 기준 헤딩

    // 설정값 (파라미터, 언제든 바뀔 수 있다)
    std::string scenario_;
    double run_sec_, throttle_pct_, angle_deg_;

    // 이번 주행에 잠긴 값 (진입 순간 복사)
    std::string active_scenario_;
    double active_run_sec_ = 3.0, active_thr_pct_ = 20.0;
    std::vector<Step> active_steps_;

    rclcpp::Time scenario_start_time_;

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
