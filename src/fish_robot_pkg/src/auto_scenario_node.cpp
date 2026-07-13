#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

class AutoScenarioNode : public rclcpp::Node {
public:
    AutoScenarioNode() : Node("auto_scenario_node"), 
                         is_auto_mode_(false), 
                         prev_auto_mode_(false) 
    {
        // % 기호 출력을 위한 포맷팅 오류 방지
        RCLCPP_INFO(this->get_logger(), "=== [Auto Scenario] 시간 기반 다단계 궤적 엔진 가동 (안티-와인드업 대응) ===");

        auto_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Quaternion>("/auto/command", 10);

        attitude_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/filtered/attitude", 10, std::bind(&AutoScenarioNode::attitude_callback, this, std::placeholders::_1));

        loop_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&AutoScenarioNode::scenario_loop_callback, this));
    }

private:
    void attitude_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        // 상태 추정 노드에서 보낸 5000.0f 오프셋을 통해 모드 감별
        if (msg->x > 2500.0f || msg->x < -2500.0f) {
            is_auto_mode_ = true;
        } else {
            is_auto_mode_ = false;
        }
    }

    void scenario_loop_callback() {
        auto cmd_msg = geometry_msgs::msg::Quaternion();

        // 1. 상태 전이(Rising Edge) 감지: 수동에서 자동으로 막 넘어온 순간!
        if (is_auto_mode_ && !prev_auto_mode_) {
            scenario_start_time_ = this->now(); // 이 순간을 시나리오의 0초로 기록
            RCLCPP_WARN(this->get_logger(), "=========================================");
            RCLCPP_WARN(this->get_logger(), "[AUTO SCENARIO] 시작! 내부 스톱워치를 0.0초로 초기화합니다.");
            RCLCPP_WARN(this->get_logger(), "=========================================");
        }

        if (is_auto_mode_) {
            // 2. 시나리오 경과 시간(스톱워치) 계산
            double elapsed_sec = (this->now() - scenario_start_time_).seconds();
            
            // 3. 다단계 시나리오 State Machine (예: 6초짜리 반복 루프)
            double cycle_time = fmod(elapsed_sec, 6.0); // 6초 주기로 나머지 연산 (0.0 ~ 5.999...)

            if (cycle_time < 3.0) {
                // [0 ~ 3초 구간]: 피치 +10도, 추력 1500
                cmd_msg.x = 0.0;    // Roll
                cmd_msg.y = 100.0;  // Pitch (단위: 0.1도 -> 10도)
                cmd_msg.z = 0.0;    // Yaw
                cmd_msg.w = 1500.0; // Throttle 50%
            } else {
                // [3 ~ 6초 구간]: 피치 -10도, 추력 1500
                cmd_msg.x = 0.0;    // Roll
                cmd_msg.y = -100.0; // Pitch (단위: 0.1도 -> -10도)
                cmd_msg.z = 0.0;    // Yaw
                cmd_msg.w = 1500.0; // Throttle 50%
            }

            // 터미널 디버깅용: 1초에 한 번씩 현재 어느 지점을 달리고 있는지 출력
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "[SCENARIO RUN] 진행 경과: %.1f초 (현재 사이클: %.1f초) | 지시 Pitch: %.1f도", 
                elapsed_sec, cycle_time, cmd_msg.y * 0.1f);

        } else {
            // [수동 모드]: 중립 대기
            cmd_msg.x = 0.0;
            cmd_msg.y = 0.0;
            cmd_msg.z = 0.0;
            cmd_msg.w = 1000.0;
        }

        // 현재 모드를 '과거'로 저장하여 다음 루프에서 변화를 감지할 수 있게 함
        prev_auto_mode_ = is_auto_mode_;

        auto_cmd_pub_->publish(cmd_msg);
    }

    bool is_auto_mode_;
    bool prev_auto_mode_; // 상태 전이 판별용 과거 상태 변수
    rclcpp::Time scenario_start_time_;

    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr auto_cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr attitude_sub_;
    rclcpp::TimerBase::SharedPtr loop_timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoScenarioNode>());
    rclcpp::shutdown();
    return 0;
}