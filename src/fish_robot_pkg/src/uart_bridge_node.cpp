#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#define UART_PORT "/dev/ttyAMA0"
#define BAUDRATE B115200

// --- nRF52840 통신용 수신 데이터 구조체 (46바이트) ---
typedef struct __attribute__((packed)) {
    uint8_t header1;      // 0xBB
    uint8_t header2;      // 0xCC
    float ax; float ay; float az;
    float gx; float gy; float gz;
    int16_t roll; int16_t pitch; int16_t yaw; int16_t throttle;
    uint8_t btn1; uint8_t btn2;
    float vbat1;          
    float vbat2;          
    int8_t rssi;          
    uint8_t checksum;     
} SensorPacket;

// --- nRF52840 통신용 송신 데이터 구조체 (11바이트) ---
typedef struct __attribute__((packed)) {
    uint8_t header;       // 0xAA
    uint16_t motor1;
    uint16_t motor2;
    uint16_t motor3;
    uint16_t motor4;
    uint8_t checksum;     
} MotorPacket;

class UartBridgeNode : public rclcpp::Node {
public:
    UartBridgeNode() : Node("uart_bridge_node"), uart_fd_(-1) {
        RCLCPP_INFO(this->get_logger(), "=== [UART Bridge] 노드 초기화 시작 ===");

        // 하드웨어 UART 포트 활성화
        if (init_uart() < 0) {
            RCLCPP_ERROR(this->get_logger(), "UART 하드웨어 포트(%s) 활성화 실패!", UART_PORT);
            rclcpp::shutdown();
            return;
        }

        // ROS2 퍼블리셔 등록
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/raw/imu_6dof", 10);
        rc_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Quaternion>("/rc/command", 10);
        rc_status_pub = this->create_publisher<std_msgs::msg::Int32MultiArray>("/rc/status", 10);

        // ROS2 서브스크라이버 등록 (최종 제어기 노드에서 계산한 모터 신호 수신)
        motor_sub_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
            "/motor/output", 10, std::bind(&UartBridgeNode::motor_callback, this, std::placeholders::_1));

        // 100Hz 주기 데이터 패치 타이머 가동 (10ms)
        uart_rx_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&UartBridgeNode::uart_rx_loop, this));

        RCLCPP_INFO(this->get_logger(), "[UART Bridge] 100Hz 비동기 스레드 바인딩 성공.");
    }

    ~UartBridgeNode() {
        if (uart_fd_ >= 0) {
            close(uart_fd_);
            RCLCPP_INFO(this->get_logger(), "[UART Bridge] 하드웨어 자원이 안전하게 해제되었습니다.");
        }
    }

private:
    int init_uart() {
        uart_fd_ = open(UART_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
        if (uart_fd_ == -1) return -1;

        struct termios options;
        tcgetattr(uart_fd_, &options);
        
        cfsetispeed(&options, BAUDRATE);
        cfsetospeed(&options, BAUDRATE);
        
        cfmakeraw(&options); 
        options.c_cflag &= ~CRTSCTS; // 라즈베리파이 5 전용 RTS/CTS 하드웨어 흐름제어 차단
        options.c_cflag |= (CS8 | CREAD | CLOCAL);
        
        tcflush(uart_fd_, TCIOFLUSH); 
        tcsetattr(uart_fd_, TCSANOW, &options);
        return uart_fd_;
    }

    // 제어 노드단에서 모터 토픽 요청이 들어오면 패킷 조립 후 nRF52840으로 송신
    void motor_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
        if (msg->data.size() < 4 || uart_fd_ < 0) return;

        MotorPacket pkt;
        pkt.header = 0xAA;
        pkt.motor1 = msg->data[0];
        pkt.motor2 = msg->data[1];
        pkt.motor3 = msg->data[2];
        pkt.motor4 = msg->data[3];

        uint8_t txSum = 0;
        uint8_t* txPtr = (uint8_t*)&pkt;
        for (size_t i = 1; i < sizeof(MotorPacket) - 1; i++) {
            txSum += txPtr[i];
        }
        pkt.checksum = txSum;

        if (write(uart_fd_, &pkt, sizeof(MotorPacket)) < 0) {
            RCLCPP_WARN(this->get_logger(), "nRF52840 측으로 모터 데이터 전송 실패.");
        }
    }

    // 10ms 마다 실행되는 버퍼 긁어오기 및 파싱 루프
    void uart_rx_loop() {
        if (uart_fd_ < 0) return;

        unsigned char byte;
        static uint8_t recvBuf[64];
        static int bufIdx = 0;
        static int state = 0;
        bool packet_received = false;
        SensorPacket rxPkt;

        // OS 로우 버퍼가 완전히 빌 때까지 패킷 스트림 동기 가동
        while (read(uart_fd_, &byte, 1) > 0) {
            switch (state) {
                case 0:
                    if (byte == 0xBB) { state = 1; recvBuf[0] = byte; bufIdx = 1; }
                    break;
                case 1:
                    if (byte == 0xCC) { state = 2; recvBuf[1] = byte; bufIdx = 2; }
                    else if (byte == 0xBB) { state = 1; recvBuf[0] = byte; bufIdx = 1; }
                    else { state = 0; bufIdx = 0; }
                    break;
                case 2:
                    recvBuf[bufIdx++] = byte;
                    if (bufIdx == sizeof(SensorPacket)) {
                        SensorPacket tempPkt;
                        memcpy(&tempPkt, recvBuf, sizeof(SensorPacket));
                        
                        uint8_t rxXor = 0;
                        uint8_t* rxPtr = (uint8_t*)&tempPkt;
                        for (size_t k = 2; k < sizeof(SensorPacket) - 1; k++) {
                            rxXor ^= rxPtr[k];
                        }

                        state = 0;
                        bufIdx = 0;

                        if (rxXor == tempPkt.checksum) {
                            rxPkt = tempPkt;
                            packet_received = true;
                        }
                    }
                    break;
            }
        }

        // 패킷 무결성이 확보되면 ROS2 토픽 발행 처리
        if (packet_received) {
            auto current_time = this->now();

            // 1. nRF발 가속도/각속도 6축 Raw 토픽 브로드캐스트
            auto imu_msg = sensor_msgs::msg::Imu();
            imu_msg.header.stamp = current_time;
            imu_msg.header.frame_id = "imu_link";
            imu_msg.linear_acceleration.x = rxPkt.ax;
            imu_msg.linear_acceleration.y = rxPkt.ay;
            imu_msg.linear_acceleration.z = rxPkt.az;
            imu_msg.angular_velocity.x = rxPkt.gx;
            imu_msg.angular_velocity.y = rxPkt.gy;
            imu_msg.angular_velocity.z = rxPkt.gz;
            imu_pub_->publish(imu_msg);

            // 2. 조종기 스틱 값 마샬링 (Roll, Pitch, Yaw, Throttle)
            auto rc_cmd_msg = geometry_msgs::msg::Quaternion();
            rc_cmd_msg.x = rxPkt.roll;
            rc_cmd_msg.y = rxPkt.pitch;
            rc_cmd_msg.z = rxPkt.yaw;
            rc_cmd_msg.w = rxPkt.throttle;
            rc_cmd_pub_->publish(rc_cmd_msg);

            // 3. 로봇 배터리 및 부가 정보 데이터 정수 변환 스케일 전송
            auto rc_status_msg = std_msgs::msg::Int32MultiArray();
            rc_status_msg.data = {
                rxPkt.btn1, 
                rxPkt.btn2, 
                static_cast<int32_t>(rxPkt.vbat1 * 100), 
                static_cast<int32_t>(rxPkt.vbat2 * 100), 
                rxPkt.rssi
            };
            rc_status_pub->publish(rc_status_msg);
        }
    }

    int uart_fd_;
    rclcpp::TimerBase::SharedPtr uart_rx_timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr rc_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr rc_status_pub;
    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr motor_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UartBridgeNode>());
    rclcpp::shutdown();
    return 0;
}