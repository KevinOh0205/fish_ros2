// =============================================================================
//  uart_bridge_node.cpp
//
//  역할 : 라즈베리파이(ROS2) <-> nRF52840(MCU) 사이의 하드웨어 UART 브릿지
//
//   [수신] nRF52840 --(SensorPacket, 46B)--> /raw/imu_6dof, /rc/command, /rc/status
//   [송신] /motor/output --(MotorPacket, 10B)--> nRF52840
//
//  통신 스펙 : /dev/ttyAMA0, 115200bps, 8N1, 흐름제어 없음(raw 모드)
//  주기      : 100Hz(10ms) 타이머로 수신 버퍼를 폴링 -> 파싱 -> 토픽 발행
//              송신은 /motor/output 콜백이 들어올 때마다 즉시 이벤트 방식으로 처리
// =============================================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>

#include <stdio.h>
#include <string.h>
#include <unistd.h>   // open/read/write/close
#include <fcntl.h>    // O_RDWR, O_NOCTTY, O_NDELAY
#include <termios.h>  // termios 시리얼 설정 구조체

#define UART_PORT "/dev/ttyAMA0"  // 라즈베리파이 PL011 하드웨어 UART (미니 UART 아님)
#define BAUDRATE B115200

// --- nRF52840 통신용 수신 데이터 구조체 (46바이트) ---
// __attribute__((packed)) : 구조체 패딩을 제거해서 MCU가 보낸 바이트 스트림과
//                           메모리 레이아웃을 1:1로 일치시킨다. (memcpy 파싱의 전제 조건)
// 바이트 배치 : 헤더2 + float6(24) + int16 4개(8) + uint8 2개(2) + float2(8) + int8(1) + 체크섬(1) = 46B
typedef struct __attribute__((packed)) {
    uint8_t header1;      // 0xBB  <- 패킷 시작 동기화 바이트 1
    uint8_t header2;      // 0xCC  <- 패킷 시작 동기화 바이트 2
    // ※ 단위 주의: ROS 표준(REP-103)은 m/s^2 / rad/s 지만 nRF 펌웨어는 g / deg/s 로 보낸다.
    //    (실측 정지 시 az≈1.028 -> m/s^2였다면 9.81. state_estimation_node가 자이로에
    //     M_PI/180을 곱해 쓰는 것도 deg/s 입력임을 뒷받침한다.)
    //    이 노드는 단위 변환 없이 그대로 발행하므로, 구독자는 g / deg/s 로 해석해야 한다.
    float ax; float ay; float az;                          // 가속도 3축 [g]
    float gx; float gy; float gz;                          // 각속도 3축 [deg/s]
    int16_t roll; int16_t pitch; int16_t yaw; int16_t throttle;  // 조종기 스틱 raw 값
    uint8_t btn1; uint8_t btn2;                            // 조종기 버튼 상태
    float vbat1;          // 배터리 1 전압 [V]
    float vbat2;          // 배터리 2 전압 [V]
    int8_t rssi;          // 무선 수신 감도 [dBm]
    uint8_t checksum;     // 헤더 2바이트를 제외한 본문의 XOR 누산값
} SensorPacket;

// --- nRF52840 통신용 송신 데이터 구조체 (10바이트) ---
// 헤더(1) + uint16 모터 4채널(8) + 체크섬(1) = 10B
typedef struct __attribute__((packed)) {
    uint8_t header;       // 0xAA  <- 송신 패킷 시작 바이트
    uint16_t motor1;      // 모터 PWM/스로틀 명령 (리틀 엔디안으로 그대로 전송)
    uint16_t motor2;
    uint16_t motor3;
    uint16_t motor4;
    uint8_t checksum;     // 헤더를 제외한 본문의 단순 덧셈(SUM) 누산값
} MotorPacket;

class UartBridgeNode : public rclcpp::Node {
public:
    UartBridgeNode() : Node("uart_bridge_node"), uart_fd_(-1) {
        RCLCPP_INFO(this->get_logger(), "=== [UART Bridge] 노드 초기화 시작 ===");

        // 하드웨어 UART 포트 활성화
        // 포트를 못 열면 브릿지가 아무 일도 할 수 없으므로 즉시 노드 종료
        if (init_uart() < 0) {
            RCLCPP_ERROR(this->get_logger(), "UART 하드웨어 포트(%s) 활성화 실패!", UART_PORT);
            rclcpp::shutdown();
            return;
        }

        // ROS2 퍼블리셔 등록
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/raw/imu_6dof", 10);        // 6축 IMU 원시값
        rc_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Quaternion>("/rc/command", 10); // 조종기 스틱 4채널
        rc_status_pub = this->create_publisher<std_msgs::msg::Int32MultiArray>("/rc/status", 10); // 버튼/배터리/RSSI

        // ROS2 서브스크라이버 등록 (최종 제어기 노드에서 계산한 모터 신호 수신)
        motor_sub_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
            "/motor/output", 10, std::bind(&UartBridgeNode::motor_callback, this, std::placeholders::_1));

        // 100Hz 주기 데이터 패치 타이머 가동 (10ms)
        // 기본 단일 스레드 executor에서는 이 타이머와 motor_callback이 같은 스레드에서
        // 번갈아 실행되므로 uart_fd_ 접근에 별도 락이 필요 없다.
        uart_rx_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&UartBridgeNode::uart_rx_loop, this));

        RCLCPP_INFO(this->get_logger(), "[UART Bridge] 100Hz 비동기 스레드 바인딩 성공.");
    }

    ~UartBridgeNode() {
        // 노드 소멸 시 파일 디스크립터 누수 방지
        if (uart_fd_ >= 0) {
            close(uart_fd_);
            RCLCPP_INFO(this->get_logger(), "[UART Bridge] 하드웨어 자원이 안전하게 해제되었습니다.");
        }
    }

private:
    // UART 포트를 열고 raw / 논블로킹 모드로 설정한다.
    // 반환값: 성공 시 fd(>=0), 실패 시 -1
    int init_uart() {
        // O_NOCTTY : 이 포트를 프로세스의 제어 터미널로 삼지 않음
        // O_NDELAY : 논블로킹 개방 -> read()가 데이터 없을 때 -1로 즉시 반환 (타이머가 멈추지 않음)
        uart_fd_ = open(UART_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
        if (uart_fd_ == -1) return -1;

        struct termios options;
        tcgetattr(uart_fd_, &options);   // 현재 설정을 읽어와서 일부만 덮어쓴다

        cfsetispeed(&options, BAUDRATE); // 입력 보드레이트 115200
        cfsetospeed(&options, BAUDRATE); // 출력 보드레이트 115200

        // raw 모드: 캐노니컬 입력/에코/신호문자/개행 변환 등을 모두 끈다.
        // 바이너리 패킷을 다루므로 커널이 바이트를 가공하면 안 된다.
        cfmakeraw(&options);
        options.c_cflag &= ~CRTSCTS; // 라즈베리파이 5 전용 RTS/CTS 하드웨어 흐름제어 차단
        // CS8    : 데이터 8비트
        // CREAD  : 수신 활성화
        // CLOCAL : 모뎀 제어선(DCD) 무시 -> 3선(TX/RX/GND) 연결에서 필수
        options.c_cflag |= (CS8 | CREAD | CLOCAL);

        tcflush(uart_fd_, TCIOFLUSH);            // 부팅 중 쌓인 쓰레기 바이트 폐기
        tcsetattr(uart_fd_, TCSANOW, &options);  // 설정 즉시 적용
        return uart_fd_;
    }

    // 제어 노드단에서 모터 토픽 요청이 들어오면 패킷 조립 후 nRF52840으로 송신
    void motor_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
        // 4채널 미만이거나 포트가 안 열렸으면 무시 (배열 범위 초과 방지)
        if (msg->data.size() < 4 || uart_fd_ < 0) return;

        MotorPacket pkt;
        pkt.header = 0xAA;
        pkt.motor1 = msg->data[0];
        pkt.motor2 = msg->data[1];
        pkt.motor3 = msg->data[2];
        pkt.motor4 = msg->data[3];

        // 체크섬 계산: 구조체를 바이트 배열로 보고 [1 .. size-2] 구간, 즉
        // 헤더(index 0)와 체크섬 필드(마지막) 자신을 뺀 본문 8바이트를 단순 덧셈.
        // uint8_t 오버플로는 자연스러운 mod-256 누산으로 의도된 동작.
        uint8_t txSum = 0;
        uint8_t* txPtr = (uint8_t*)&pkt;
        for (size_t i = 1; i < sizeof(MotorPacket) - 1; i++) {
            txSum += txPtr[i];
        }
        pkt.checksum = txSum;

        // 10바이트를 한 번에 write. 논블로킹이라 커널 TX 버퍼가 가득 차면 실패할 수 있다.
        if (write(uart_fd_, &pkt, sizeof(MotorPacket)) < 0) {
            RCLCPP_WARN(this->get_logger(), "nRF52840 측으로 모터 데이터 전송 실패.");
        }
    }

    // 10ms 마다 실행되는 버퍼 긁어오기 및 파싱 루프
    void uart_rx_loop() {
        if (uart_fd_ < 0) return;

        unsigned char byte;
        // static: 타이머 호출 사이에 파싱 진행 상태를 유지한다.
        // (한 주기 안에 패킷이 반쪽만 도착해도 다음 주기에서 이어서 파싱)
        static uint8_t recvBuf[64];  // 46바이트 패킷보다 넉넉한 조립용 버퍼
        static int bufIdx = 0;       // 현재까지 쌓인 바이트 수
        static int state = 0;        // 0=헤더1 대기, 1=헤더2 대기, 2=페이로드 수집
        bool packet_received = false;
        SensorPacket rxPkt;

        // OS 로우 버퍼가 완전히 빌 때까지 패킷 스트림 동기 가동
        // 논블로킹이므로 읽을 게 없으면 read()가 -1을 반환하며 루프 탈출.
        // 한 주기에 패킷이 여러 개 들어오면 마지막 정상 패킷만 남아 발행된다(최신값 우선).
        while (read(uart_fd_, &byte, 1) > 0) {
            switch (state) {
                case 0:  // 첫 동기화 바이트 0xBB 탐색
                    if (byte == 0xBB) { state = 1; recvBuf[0] = byte; bufIdx = 1; }
                    break;
                case 1:  // 두 번째 동기화 바이트 0xCC 확인
                    if (byte == 0xCC) { state = 2; recvBuf[1] = byte; bufIdx = 2; }
                    // 0xBB가 연속으로 오면 그것을 새 시작점으로 간주 (0xBB 0xBB 0xCC 대응)
                    else if (byte == 0xBB) { state = 1; recvBuf[0] = byte; bufIdx = 1; }
                    else { state = 0; bufIdx = 0; }  // 동기 실패 -> 처음부터 다시 탐색
                    break;
                case 2:  // 남은 44바이트 페이로드 수집
                    recvBuf[bufIdx++] = byte;
                    if (bufIdx == sizeof(SensorPacket)) {  // 46바이트 다 모임
                        SensorPacket tempPkt;
                        memcpy(&tempPkt, recvBuf, sizeof(SensorPacket));  // packed 구조체로 캐스팅

                        // 무결성 검사: 헤더 2바이트와 체크섬 자신을 제외한
                        // [2 .. size-2] 구간(43바이트)의 XOR 누산.
                        uint8_t rxXor = 0;
                        uint8_t* rxPtr = (uint8_t*)&tempPkt;
                        for (size_t k = 2; k < sizeof(SensorPacket) - 1; k++) {
                            rxXor ^= rxPtr[k];
                        }

                        // 검증 성패와 무관하게 상태 머신은 초기화하고 다음 패킷을 찾는다
                        state = 0;
                        bufIdx = 0;

                        if (rxXor == tempPkt.checksum) {
                            rxPkt = tempPkt;        // 유효 패킷 채택
                            packet_received = true;
                        }
                        // 체크섬 불일치 시 조용히 폐기 (다음 헤더부터 재동기)
                    }
                    break;
            }
        }

        // 패킷 무결성이 확보되면 ROS2 토픽 발행 처리
        if (packet_received) {
            auto current_time = this->now();  // 세 토픽이 같은 타임스탬프를 공유

            // 1. nRF발 가속도/각속도 6축 Raw 토픽 브로드캐스트
            //    (orientation / covariance는 채우지 않음 -> 필터 노드가 후처리한다는 전제)
            auto imu_msg = sensor_msgs::msg::Imu();
            imu_msg.header.stamp = current_time;
            imu_msg.header.frame_id = "imu_link";  // TF 트리상의 IMU 좌표계 이름
            imu_msg.linear_acceleration.x = rxPkt.ax;
            imu_msg.linear_acceleration.y = rxPkt.ay;
            imu_msg.linear_acceleration.z = rxPkt.az;
            imu_msg.angular_velocity.x = rxPkt.gx;
            imu_msg.angular_velocity.y = rxPkt.gy;
            imu_msg.angular_velocity.z = rxPkt.gz;
            imu_pub_->publish(imu_msg);

            // 2. 조종기 스틱 값 마샬링 (Roll, Pitch, Yaw, Throttle)
            //    쿼터니언 타입을 실제 회전이 아닌 float 4개 컨테이너로 전용(轉用)한 형태.
            auto rc_cmd_msg = geometry_msgs::msg::Quaternion();
            rc_cmd_msg.x = rxPkt.roll;
            rc_cmd_msg.y = rxPkt.pitch;
            rc_cmd_msg.z = rxPkt.yaw;
            rc_cmd_msg.w = rxPkt.throttle;
            rc_cmd_pub_->publish(rc_cmd_msg);

            // 3. 로봇 배터리 및 부가 정보 데이터 정수 변환 스케일 전송
            //    Int32MultiArray라 실수를 못 담으므로 전압은 x100 (센티볼트) 정수로 인코딩.
            //    소비 측에서 반드시 /100.0 으로 되돌려야 함.
            //    [0]=btn1, [1]=btn2, [2]=vbat1*100, [3]=vbat2*100, [4]=rssi
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

    int uart_fd_;                                   // UART 파일 디스크립터 (-1 = 미개방)
    rclcpp::TimerBase::SharedPtr uart_rx_timer_;    // 100Hz 수신 폴링 타이머
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr rc_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr rc_status_pub;
    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr motor_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UartBridgeNode>());  // 단일 스레드 executor로 타이머+콜백 처리
    rclcpp::shutdown();
    return 0;
}
