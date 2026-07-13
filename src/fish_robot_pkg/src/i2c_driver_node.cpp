#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#define I2C_BUS "/dev/i2c-1"
#define TCA9548A_ADDR 0x70
#define AK8963_ADDR   0x0C
#define MS5837_ADDR   0x76

class I2cDriverNode : public rclcpp::Node {
public:
    I2cDriverNode() : Node("i2c_driver_node"), i2c_fd_(-1), has_mag_(false) {
        RCLCPP_INFO(this->get_logger(), "=== [I2C Driver] 노드 초기화 시작 ===");

        // 하드웨어 I2C 버스 구동 및 디바이스 연동
        if (init_i2c() < 0) {
            RCLCPP_ERROR(this->get_logger(), "I2C 버스(%s) 초기화 실패!", I2C_BUS);
            rclcpp::shutdown();
            return;
        }

        // ROS2 퍼블리셔 등록
        mag_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/raw/magnetometer", 10);
        pressure_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/sensor/pressure_raw", 10);

        // 100Hz 주기 하드웨어 폴링 제어 타이머 시동 (10ms)
        i2c_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&I2cDriverNode::i2c_polling_loop, this));

        RCLCPP_INFO(this->get_logger(), "[I2C Driver] 100Hz 하드웨어 폴링 루프 작동 성공.");
    }

    ~I2cDriverNode() {
        if (i2c_fd_ >= 0) {
            close(i2c_fd_);
            RCLCPP_INFO(this->get_logger(), "[I2C Driver] I2C 하드웨어 자원 안전 해제 완료.");
        }
    }

private:
    int init_i2c() {
        i2c_fd_ = open(I2C_BUS, O_RDWR);
        if (i2c_fd_ < 0) return -1;

        // 회로도 J4 커넥터: MPU9250 주소 접근 후 Bypass 모드 기동 (하단 AK8963 활성화용)
        if (ioctl(i2c_fd_, I2C_SLAVE, 0x68) >= 0) {
            uint8_t user_ctrl[2] = {0x6A, 0x00}; 
            if (write(i2c_fd_, user_ctrl, 2) < 0) { /* 경고 방어용 무동작 처리 */ }
            
            uint8_t int_config[2] = {0x37, 0x02}; // Bypass Mode Enable
            if (write(i2c_fd_, int_config, 2) == 2) usleep(20000);
        }

        // AK8963 지자기 센서 직접 제어 설정 동기화
        if (ioctl(i2c_fd_, I2C_SLAVE, AK8963_ADDR) >= 0) {
            uint8_t mag_off[2] = {0x0A, 0x00};
            if (write(i2c_fd_, mag_off, 2) == 2) {
                usleep(20000);
                uint8_t mag_on[2] = {0x0A, 0x16}; // Continuous mode 2 (100Hz, 16-bit 출력)
                if (write(i2c_fd_, mag_on, 2) == 2) {
                    RCLCPP_INFO(this->get_logger(), "[AK8963] 지자기 센서 정상 연결 확인 (100Hz).");
                    has_mag_ = true;
                }
            }
        }

        if (!has_mag_) {
            RCLCPP_WARN(this->get_logger(), "[AK8963] 지자기 센서 응답 없음 (상태 추정 시 6축 우회 플래그 유도).");
        }

        // MS5837 압력 센서 전 채널 (TCA9548A 멀티플렉서 채널 0, 1, 2) 초기화 및 내부 PROM 로드
        memset(prom_C_, 0, sizeof(prom_C_));
        for (int i = 0; i < 3; i++) {
            if (!select_mux_channel(i)) continue;
            usleep(5000);

            if (ioctl(i2c_fd_, I2C_SLAVE, MS5837_ADDR) >= 0) {
                bool reset_success = false;
                for (int retry = 0; retry < 3; retry++) {
                    uint8_t reset_cmd = 0x1E;
                    if (write(i2c_fd_, &reset_cmd, 1) == 1) {
                        reset_success = true;
                        break;
                    }
                    usleep(10000);
                }

                if (!reset_success) {
                    RCLCPP_WARN(this->get_logger(), "[MS5837] 채널 %d 압력센서 응답 없음 (Reset 실패).", i);
                    prom_C_[i][1] = 0;
                    continue;
                }
                usleep(15000);

                bool is_valid = true;
                for (int p = 0; p < 7; p++) {
                    uint8_t prom_cmd = 0xA0 + (p * 2);
                    bool read_success = false;
                    for (int retry = 0; retry < 3; retry++) {
                        if (write(i2c_fd_, &prom_cmd, 1) == 1) {
                            uint8_t buf[2];
                            if (read(i2c_fd_, buf, 2) == 2) {
                                prom_C_[i][p] = (buf[0] << 8) | buf[1];
                                if (prom_C_[i][p] != 0x0000 && prom_C_[i][p] != 0xFFFF) {
                                    read_success = true;
                                    break;
                                }
                            }
                        }
                        usleep(5000);
                    }
                    if (!read_success) {
                        is_valid = false;
                        break;
                    }
                }

                if (is_valid) {
                    RCLCPP_INFO(this->get_logger(), "[MS5837] 채널 %d (J%d 커넥터) 압력센서 롬 로드 완료.", i, i + 5);
                } else {
                    RCLCPP_WARN(this->get_logger(), "[MS5837] 채널 %d 압력센서 롬 데이터 손상 (제외 처리).", i);
                    prom_C_[i][1] = 0;
                }
            }
        }
        return 0;
    }

    bool select_mux_channel(uint8_t channel) {
        if (channel > 7) return false;
        if (ioctl(i2c_fd_, I2C_SLAVE, TCA9548A_ADDR) < 0) return false;
        uint8_t mask = 1 << channel;
        return (write(i2c_fd_, &mask, 1) == 1);
    }

    // 멀티플렉서 채널 선택을 짧게 재시도한다. 선택이 실패한 채로 0x76에 접근하면
    // 이전 채널 센서를 엉뚱한 채널 값으로 읽게 되므로, 최종 실패 시 호출부는 건너뛴다.
    bool select_mux_channel_retry(uint8_t channel, int max_retries = 2) {
        for (int attempt = 0; attempt <= max_retries; attempt++) {
            if (select_mux_channel(channel)) return true;
            if (attempt < max_retries) usleep(500);
        }
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[MS5837] 채널 %d 멀티플렉서 선택 실패 (재시도 소진). 이번 사이클 건너뜀.", channel);
        return false;
    }

    // 10ms 마다 정밀 기동되는 하드웨어 I2C 트랜잭션 루프
    void i2c_polling_loop() {
        // 1. 지자기 데이터 판독 및 토픽 송신
        if (has_mag_) {
            float mx = 0.0f, my = 0.0f, mz = 0.0f;
            if (ioctl(i2c_fd_, I2C_SLAVE, AK8963_ADDR) >= 0) {
                uint8_t reg = 0x02; // ST1 상태 검사
                if (write(i2c_fd_, &reg, 1) == 1 && read(i2c_fd_, &reg, 1) == 1 && (reg & 0x01)) {
                    uint8_t data_reg = 0x03; // HXL 데이터 시작점 레지스터 주소
                    uint8_t buf[7];
                    if (write(i2c_fd_, &data_reg, 1) == 1 && read(i2c_fd_, buf, 7) == 7 && !(buf[6] & 0x08)) {
                        mx = static_cast<float>(static_cast<int16_t>((buf[1] << 8) | buf[0])) * 0.15f;
                        my = static_cast<float>(static_cast<int16_t>((buf[3] << 8) | buf[2])) * 0.15f;
                        mz = static_cast<float>(static_cast<int16_t>((buf[5] << 8) | buf[4])) * 0.15f;

                        auto mag_msg = geometry_msgs::msg::Vector3();
                        mag_msg.x = mx; mag_msg.y = my; mag_msg.z = mz;
                        mag_pub_->publish(mag_msg);
                    }
                }
            }
        }

        // 2. 지연 없는 비동기식 압력 센서 파싱 상태머신 (기존 C코드 완전 최적화 이식)
        static int state = 0;
        static int delay_counter = 0;
        static uint32_t D1[3] = {0}, D2[3] = {0};
        static float filtered_P[3] = {0.0f};
        static bool first_read[3] = {true, true, true};
        uint8_t adc_buf[3];

        if (state == 0) {
            // 압력 데이터(D1) 변환 명령 동시 하향
            for (int i = 0; i < 3; i++) {
                if (prom_C_[i][1] == 0) continue;
                if (!select_mux_channel_retry(i)) continue;
                if (ioctl(i2c_fd_, I2C_SLAVE, MS5837_ADDR) >= 0) {
                    uint8_t d1_cmd = 0x4A;
                    if (write(i2c_fd_, &d1_cmd, 1) < 0) { /* 경고 제어용 */ }
                }
            }
            state = 1; delay_counter = 0;
        } 
        else if (state == 1) {
            // 변환 대기 (약 40ms 안정 확보)
            if (++delay_counter >= 4) {
                for (int i = 0; i < 3; i++) {
                    if (prom_C_[i][1] == 0) continue;
                    if (!select_mux_channel_retry(i)) { D1[i] = 0; continue; }
                    if (ioctl(i2c_fd_, I2C_SLAVE, MS5837_ADDR) >= 0) {
                        uint8_t adc_cmd = 0x00;
                        if (write(i2c_fd_, &adc_cmd, 1) == 1 && read(i2c_fd_, adc_buf, 3) == 3) {
                            D1[i] = ((uint32_t)adc_buf[0] << 16) | ((uint32_t)adc_buf[1] << 8) | adc_buf[2];
                        }
                        // 이어서 온도 데이터(D2) 변환 명령 하향
                        uint8_t d2_cmd = 0x5A; 
                        if (write(i2c_fd_, &d2_cmd, 1) < 0) { /* 경고 제어용 */ }
                    }
                }
                state = 2; delay_counter = 0;
            }
        } 
        else if (state == 2) {
            // 변환 대기 (약 40ms 안정 확보)
            if (++delay_counter >= 4) {
                auto out_pressure_msg = std_msgs::msg::Float32MultiArray();
                out_pressure_msg.data.resize(6); // [0..2]: 원시 압력값(mbar), [3..5]: 원시 온도값(C)

                for (int i = 0; i < 3; i++) {
                    if (prom_C_[i][1] == 0) continue;
                    if (!select_mux_channel_retry(i)) continue;
                    if (ioctl(i2c_fd_, I2C_SLAVE, MS5837_ADDR) >= 0) {
                        uint8_t adc_cmd = 0x00;
                        if (write(i2c_fd_, &adc_cmd, 1) == 1 && read(i2c_fd_, adc_buf, 3) == 3) {
                            D2[i] = ((uint32_t)adc_buf[0] << 16) | ((uint32_t)adc_buf[1] << 8) | adc_buf[2];
                        }

                        if (D1[i] != 0xFFFFFF && D2[i] != 0xFFFFFF && D1[i] != 0 && D2[i] != 0) {
                            uint16_t *C = prom_C_[i];
                            int32_t dT = D2[i] - ((uint32_t)C[5] * 256);
                            int32_t TEMP = 2000 + ((int64_t)dT * C[6]) / 8388608;
                            int64_t OFF = ((int64_t)C[2] * 65536) + (((int64_t)C[4] * dT) / 128);
                            int64_t SENS = ((int64_t)C[1] * 32768) + (((int64_t)C[3] * dT) / 256);
                            int32_t P = ((D1[i] * SENS / 2097152) - OFF) / 8192; 

                            float raw_pressure = P / 10.0f;
                            
                            if (first_read[i]) {
                                filtered_P[i] = raw_pressure;
                                first_read[i] = false;
                            } else {
                                filtered_P[i] = (0.1f * raw_pressure) + (0.9f * filtered_P[i]);
                            }

                            out_pressure_msg.data[i] = filtered_P[i];           // 압력 채널 0,1,2 (mbar)
                            out_pressure_msg.data[i + 3] = TEMP / 100.0f;       // 온도 채널 0,1,2 (C)
                        }
                    }
                }
                pressure_pub_->publish(out_pressure_msg);
                state = 0;
            }
        }
    }

    int i2c_fd_;
    bool has_mag_;
    uint16_t prom_C_[3][8];
    rclcpp::TimerBase::SharedPtr i2c_timer_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr mag_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pressure_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<I2cDriverNode>());
    rclcpp::shutdown();
    return 0;
}