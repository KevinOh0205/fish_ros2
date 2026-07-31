// =============================================================================
//  rpm_driver_node.cpp
//
//  역할 : 꼬리 BLDC 모터에 달린 홀 센서(엔코더) 펄스를 세어 RPM으로 환산 발행
//
//   [입력] GPIO 21 상승 엣지 인터럽트 (libgpiod)
//   [출력] /sensor/tail_rpm (Int32, 10Hz)
//
//  구조 : 별도 스레드가 커널 이벤트를 블로킹 대기하며 펄스를 누적하고,
//         100ms 타이머가 누적값을 꺼내 RPM으로 환산 후 발행한다.
//         (ROS 콜백 스레드가 GPIO 대기에 묶이지 않도록 분리한 구조)
// =============================================================================

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <gpiod.hpp>   // libgpiod C++ 바인딩 (V1 API)
#include <thread>
#include <atomic>

class RpmDriverNode : public rclcpp::Node {
public:
    RpmDriverNode() : Node("rpm_driver_node"), pulse_count_(0), current_rpm_(0) {
        RCLCPP_INFO(this->get_logger(), "=== [RPM Driver] libgpiod V1 규격 하드웨어 인터럽트 연동 시작 ===");

        try {
            // 라즈베리파이 5는 GPIO가 RP1 칩으로 옮겨져 gpiochip4가 40핀 헤더에 해당한다.
            // (파이 4 이하는 gpiochip0 — 보드 교체 시 이 줄을 반드시 확인할 것)
            chip_ = std::make_unique<gpiod::chip>("gpiochip4");
            line_ = chip_->get_line(21);

            // 상승 엣지에서만 이벤트 발생하도록 요청 (홀 센서 펄스 1개 = 이벤트 1개)
            gpiod::line_request config;
            config.consumer = "rpm_encoder";   // 다른 프로세스가 점유 상태를 보고 알 수 있게 하는 이름표
            config.request_type = gpiod::line_request::EVENT_RISING_EDGE;
            line_.request(config);

            // [오타 수정] running에서 running_로 변경 완료
            running_ = true;
            interrupt_thread_ = std::thread(&RpmDriverNode::interrupt_worker, this);

        } catch (const std::exception& e) {
            // 라인 점유 실패(다른 프로세스가 이미 잡음) 또는 칩 이름 오류 등
            RCLCPP_ERROR(this->get_logger(), "GPIO V1 하드웨어 초기화 실패: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        rpm_pub_ = this->create_publisher<std_msgs::msg::Int32>("/sensor/tail_rpm", 10);
        // 100ms(10Hz) 주기로 펄스 누적값을 RPM으로 환산해 발행
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&RpmDriverNode::publish_rpm, this));
    }

    ~RpmDriverNode() {
        // 워커 스레드에 종료 신호를 주고 완전히 빠져나올 때까지 대기.
        // event_wait 타임아웃이 10ms이므로 최대 10ms 안에 종료된다.
        running_ = false;
        if (interrupt_thread_.joinable()) {
            interrupt_thread_.join();
        }
    }

private:
    // [워커 스레드] 커널 GPIO 이벤트를 기다렸다가 펄스를 누적한다.
    // 10ms 타임아웃을 두는 이유는 펄스가 안 들어와도 주기적으로 깨어나
    // running_ 종료 플래그를 확인하기 위함이다(무한 블로킹 방지).
    void interrupt_worker() {
        while (running_ && rclcpp::ok()) {
            if (line_.event_wait(std::chrono::milliseconds(10))) {
                line_.event_read();   // 이벤트를 큐에서 꺼내야 다음 이벤트가 감지된다
                pulse_count_++;       // atomic — 타이머 스레드와 공유
            }
        }
    }

    // [타이머 스레드] 100ms 동안 쌓인 펄스를 읽고 0으로 리셋한 뒤 RPM 환산
    void publish_rpm() {
        // [하드웨어 스펙 반영] 3클럭 = 1회전 적용
        // (펄스 수 * 10hz * 60초) / 3 펄스 = 펄스 수 * 200
        //
        // 주의: 100ms 창에서 세므로 분해능이 펄스 1개 = 200 RPM으로 거칠다.
        //       저속 구간을 세밀하게 봐야 하면 창을 늘리거나 펄스 간격(주기) 측정 방식으로 바꿔야 한다.
        int pulses = pulse_count_.load();
        pulse_count_ = 0;   // 읽고 즉시 리셋 (load와 store 사이에 들어온 펄스는 유실 — 100Hz 회전에서는 무시 가능)

        current_rpm_ = pulses * 200;

        auto msg = std_msgs::msg::Int32();
        msg.data = current_rpm_;
        rpm_pub_->publish(msg);
    }

    std::unique_ptr<gpiod::chip> chip_;
    gpiod::line line_;
    std::thread interrupt_thread_;
    std::atomic<bool> running_{false};    // 워커 스레드 종료 신호
    std::atomic<int> pulse_count_{0};     // 스레드 간 공유되는 유일한 값 -> atomic 필수
    int32_t current_rpm_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr rpm_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RpmDriverNode>());
    rclcpp::shutdown();
    return 0;
}
