#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <gpiod.hpp>
#include <thread>
#include <atomic>

class RpmDriverNode : public rclcpp::Node {
public:
    RpmDriverNode() : Node("rpm_driver_node"), pulse_count_(0), current_rpm_(0) {
        RCLCPP_INFO(this->get_logger(), "=== [RPM Driver] libgpiod V1 규격 하드웨어 인터럽트 연동 시작 ===");

        try {
            chip_ = std::make_unique<gpiod::chip>("gpiochip4");
            line_ = chip_->get_line(21);

            gpiod::line_request config;
            config.consumer = "rpm_encoder";
            config.request_type = gpiod::line_request::EVENT_RISING_EDGE;
            line_.request(config);

            // [오타 수정] running에서 running_로 변경 완료
            running_ = true; 
            interrupt_thread_ = std::thread(&RpmDriverNode::interrupt_worker, this);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "GPIO V1 하드웨어 초기화 실패: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        rpm_pub_ = this->create_publisher<std_msgs::msg::Int32>("/sensor/tail_rpm", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&RpmDriverNode::publish_rpm, this));
    }

    ~RpmDriverNode() {
        running_ = false;
        if (interrupt_thread_.joinable()) {
            interrupt_thread_.join();
        }
    }

private:
    void interrupt_worker() {
        while (running_ && rclcpp::ok()) {
            if (line_.event_wait(std::chrono::milliseconds(10))) {
                line_.event_read();
                pulse_count_++;
            }
        }
    }

    void publish_rpm() {
        // [하드웨어 스펙 반영] 3클럭 = 1회전 적용
        // (펄스 수 * 10hz * 60초) / 3 펄스 = 펄스 수 * 200
        int pulses = pulse_count_.load();
        pulse_count_ = 0; 

        current_rpm_ = pulses * 200;

        auto msg = std_msgs::msg::Int32();
        msg.data = current_rpm_;
        rpm_pub_->publish(msg);
    }

    std::unique_ptr<gpiod::chip> chip_;
    gpiod::line line_;
    std::thread interrupt_thread_;
    std::atomic<bool> running_{false};
    std::atomic<int> pulse_count_{0};
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