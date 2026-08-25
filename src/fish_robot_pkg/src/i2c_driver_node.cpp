// =============================================================================
//  i2c_driver_node.cpp
//
//  역할 : 라즈베리파이 I2C 버스에 직결된 센서 2종을 읽어 토픽으로 발행
//
//   [출력] /raw/magnetometer   (Vector3)            <- AK8963 지자기 3축, 100Hz
//          /sensor/pressure_raw (Float32MultiArray[6]) <- MS5837 압력 3채널 + 온도 3채널, 약 11Hz
//
//  I2C 토폴로지
//    /dev/i2c-1
//      ├─ 0x68 MPU9250  : 자체 데이터는 안 쓰고, Bypass 모드만 켜서 AK8963을 노출시킴
//      ├─ 0x0C AK8963   : MPU9250 패키지 내부의 지자기 센서 (Bypass 상태에서만 접근 가능)
//      └─ 0x70 TCA9548A : I2C 멀티플렉서
//            ├─ ch0 → 0x76 MS5837-02BA (J5 커넥터)   ※ 02BA — 환산식이 30BA와 다름
//            ├─ ch1 → 0x76 MS5837-02BA (J6 커넥터)
//            └─ ch2 → 0x76 MS5837-02BA (J7 커넥터)
//          ※ MS5837 3개가 모두 같은 주소(0x76)라 멀티플렉서로 한 번에 하나씩만 연결한다.
//
//  ※ 주의: 타이머는 100Hz지만 압력 센서는 변환 대기 때문에 약 11Hz로만 갱신된다.
//          (아래 상태 머신 주석 참고). 지자기만 진짜 100Hz다.
// =============================================================================

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fstream>
#include <sstream>
#include <string>
#include <limits>
#include <cmath>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>   // I2C_SLAVE ioctl

#define I2C_BUS "/dev/i2c-1"
#define TCA9548A_ADDR 0x70   // I2C 멀티플렉서
#define AK8963_ADDR   0x0C   // 지자기 센서 (MPU9250 Bypass 시 노출)
#define MS5837_ADDR   0x76   // 수심(압력) 센서 — 3채널 공통 주소

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

        // ##### [임시 2026-08-19 — 압력 특성화 측정용] #####
        // 되돌리는 것을 잊지 않기 위한 경고. 이 줄이 저널에 보이면 아직 측정 설정이다.
        RCLCPP_WARN(this->get_logger(),
                    "[I2C Driver] ※ 압력 IIR 필터 비활성 (특성화 측정용). 운용 전 원복할 것.");
        // ##### 임시 끝 #####
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
        // AK8963은 물리적으로 MPU9250 패키지 안에 있고 보조 I2C 버스에 매달려 있다.
        // Bypass를 켜야 메인 버스에서 0x0C로 직접 보인다.
        if (ioctl(i2c_fd_, I2C_SLAVE, 0x68) >= 0) {
            uint8_t user_ctrl[2] = {0x6A, 0x00};   // USER_CTRL: MPU의 보조 I2C 마스터 기능 해제
            if (write(i2c_fd_, user_ctrl, 2) < 0) { /* 경고 방어용 무동작 처리 */ }

            uint8_t int_config[2] = {0x37, 0x02}; // Bypass Mode Enable
            if (write(i2c_fd_, int_config, 2) == 2) usleep(20000);
        }

        // AK8963 지자기 센서 직접 제어 설정 동기화
        // 모드 전환 전에 반드시 Power-down(0x00)을 거쳐야 한다는 것이 데이터시트 요구사항이다.
        if (ioctl(i2c_fd_, I2C_SLAVE, AK8963_ADDR) >= 0) {
            uint8_t mag_off[2] = {0x0A, 0x00};    // CNTL1 = Power-down
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
            // 지자기가 없으면 state_estimation이 자동으로 6축 모드로 떨어진다(치명적 아님)
            RCLCPP_WARN(this->get_logger(), "[AK8963] 지자기 센서 응답 없음 (상태 추정 시 6축 우회 플래그 유도).");
        }

        // MS5837 압력 센서 전 채널 (TCA9548A 멀티플렉서 채널 0, 1, 2) 초기화 및 내부 PROM 로드
        // MS5837은 공장에서 교정된 계수 6개(C1~C6)를 내부 PROM에 갖고 있고,
        // 이 값 없이는 원시 ADC 값을 실제 압력으로 환산할 수 없다. 부팅 시 1회만 읽으면 된다.
        memset(prom_C_, 0, sizeof(prom_C_));
        for (int i = 0; i < 3; i++) {
            if (!select_mux_channel(i)) continue;
            usleep(5000);

            if (ioctl(i2c_fd_, I2C_SLAVE, MS5837_ADDR) >= 0) {
                // 1) 리셋 명령(0x1E)으로 센서를 알려진 상태로 만든다 (3회 재시도)
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
                    // 이 채널은 죽은 것으로 표시. prom_C_[i][1] == 0 이 "사용 불가" 마커 역할을 한다.
                    RCLCPP_WARN(this->get_logger(), "[MS5837] 채널 %d 압력센서 응답 없음 (Reset 실패).", i);
                    prom_C_[i][1] = 0;
                    continue;
                }
                usleep(15000);   // 리셋 후 PROM 리로드 대기

                // 2) PROM 워드 7개(0xA0~0xAC)를 2바이트씩 읽는다
                bool is_valid = true;
                for (int p = 0; p < 7; p++) {
                    uint8_t prom_cmd = 0xA0 + (p * 2);
                    bool read_success = false;
                    for (int retry = 0; retry < 3; retry++) {
                        if (write(i2c_fd_, &prom_cmd, 1) == 1) {
                            uint8_t buf[2];
                            if (read(i2c_fd_, buf, 2) == 2) {
                                prom_C_[i][p] = (buf[0] << 8) | buf[1];   // 빅엔디안 조합
                                // 0x0000/0xFFFF는 통신 실패나 미장착 시 나오는 전형적 쓰레기값
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
                    prom_C_[i][1] = 0;   // 사용 불가 마커
                }
            }
        }

        verify_port_map();
        return 0;
    }

    // =========================================================================
    //  포트 배정 대조 — 어느 커넥터가 어느 역할인지 매 부팅 자동 확인
    // =========================================================================
    // 메시지에는 채널 신원 정보가 없다. 배열 위치가 곧 신원이고, 그걸 보증하는
    // 건 배선뿐이다. 배선이 바뀌면 앞/옆이 뒤바뀐 채로 조용히 큰 음수 동압이
    // 나오고, 좌/우가 바뀌면 레버암 부호가 뒤집힌다.
    //
    // 그래서 저장하는 데이터를 "선언문"이 아니라 **센서 지문(PROM C1~C6)**으로
    // 한다. 선언문은 하드웨어에 대한 *주장*이라 정비 중에 조용히 거짓이 되지만,
    // PROM은 센서마다 고유해서 기계가 스스로 대조할 수 있다.
    // 근거: 2026-08-21 J5를 "접촉 불량"으로 오진했다가 PROM 비교(0x4003.. vs
    // 0xD001..)로 실은 센서가 교체된 것임이 드러났다.
    //
    // ※ PROM이 잡는 것은 **센서 교체**다. 같은 센서에 튜브만 다른 포트로 옮긴
    //   경우는 못 잡는다 — 그건 압력값으로만 구별되므로 hydro_estimator_node 의
    //   런타임 감시(주행 중 q_raw 지속 음수 -> 배정 뒤바뀜 의심)가 맡는다.
    void verify_port_map() {
        const char *home = getenv("HOME");
        const std::string path = home ? (std::string(home) + "/ros2_ws/log_csv/port_map.txt")
                                      : "./log_csv/port_map.txt";
        static const char *ROLE[3] = {"front", "left", "right"};   // 기본 배정: ch0/ch1/ch2

        std::ifstream in(path);
        if (!in.is_open()) {
            // 파일이 없으면 지금 읽은 PROM으로 만들어 둔다. 역할은 **추정값**이므로
            // verified: no 로 적고, 물리 확인 전까지 매 부팅 경고한다.
            in.close();
            FILE *fp = fopen(path.c_str(), "w");
            if (!fp) {
                RCLCPP_WARN(this->get_logger(), "[포트] 배정 파일 생성 실패 (%s)", path.c_str());
                return;
            }
            fprintf(fp, "# 포트 배정 및 센서 지문 (MS5837 PROM C1~C6) — i2c_driver_node 가 자동 생성\n");
            fprintf(fp, "#\n");
            fprintf(fp, "# verified: no      <- 물리 확인 후 yes 로 바꾸십시오\n");
            fprintf(fp, "#   앞 확정 : 노즈 포트에 입으로 바람을 불어 그 채널만 오르는지\n");
            fprintf(fp, "#   좌우 확정: 우현을 아래로 30도 기울여 우 포트가 약 4.3 mbar 높은지\n");
            fprintf(fp, "#\n");
            fprintf(fp, "# 역할   채널  C1     C2     C3     C4     C5     C6\n");
            for (int i = 0; i < 3; i++) {
                if (prom_C_[i][1] == 0) continue;      // 죽은 채널은 적지 않는다
                fprintf(fp, "%-7s %d    ", ROLE[i], i);
                for (int c = 1; c <= 6; c++) fprintf(fp, " %5u", prom_C_[i][c]);
                fprintf(fp, "\n");
            }
            fclose(fp);
            RCLCPP_WARN(this->get_logger(),
                        "[포트] 배정 파일을 새로 만들었습니다 (%s). 역할은 **추정값**입니다 — "
                        "물리 확인 후 verified 를 yes 로 바꾸십시오.", path.c_str());
            return;
        }

        bool verified = false, mismatch = false, any = false;
        std::string line;
        while (std::getline(in, line)) {
            if (line.find("verified:") != std::string::npos && line.find("yes") != std::string::npos)
                verified = true;
            if (line.empty() || line[0] == '#') continue;
            std::istringstream ss(line);
            std::string role; int ch = -1; unsigned c[7] = {0};
            if (!(ss >> role >> ch)) continue;
            if (ch < 0 || ch > 2) continue;
            bool got = true;
            for (int k = 1; k <= 6; k++) if (!(ss >> c[k])) { got = false; break; }
            if (!got) continue;
            any = true;

            if (prom_C_[ch][1] == 0) {
                RCLCPP_ERROR(this->get_logger(), "[포트] %s(ch%d) 무응답 — 센서가 죽었거나 빠졌습니다.",
                             role.c_str(), ch);
                mismatch = true;
                continue;
            }
            bool same = true;
            for (int k = 1; k <= 6; k++) if (prom_C_[ch][k] != c[k]) { same = false; break; }
            if (same) {
                RCLCPP_INFO(this->get_logger(), "[포트] %s(ch%d) 확인", role.c_str(), ch);
            } else {
                RCLCPP_ERROR(this->get_logger(),
                             "[포트] %s(ch%d) **불일치 — 그 자리 센서가 바뀌었습니다.** "
                             "기대 C1=%u, 실제 C1=%u", role.c_str(), ch, c[1], prom_C_[ch][1]);
                mismatch = true;
            }
        }
        in.close();

        if (!any)
            RCLCPP_WARN(this->get_logger(), "[포트] 배정 파일에 읽을 항목이 없습니다 (%s)", path.c_str());
        else if (mismatch)
            RCLCPP_ERROR(this->get_logger(),
                         "[포트] 배정이 파일과 다릅니다. 수심·속도가 엉뚱한 채널로 계산됩니다 — "
                         "배선을 확인하거나, 의도한 교체라면 %s 를 지워 다시 만드십시오.", path.c_str());
        else if (!verified)
            RCLCPP_WARN(this->get_logger(),
                        "[포트] 지문은 일치하지만 역할이 물리적으로 확인되지 않았습니다 "
                        "(%s 의 verified: no).", path.c_str());
    }

    // TCA9548A에 비트마스크 1바이트를 써서 해당 채널만 메인 버스에 연결한다.
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
        // 1. 지자기 데이터 판독 및 토픽 송신 (매 주기 = 100Hz)
        if (has_mag_) {
            float mx = 0.0f, my = 0.0f, mz = 0.0f;
            if (ioctl(i2c_fd_, I2C_SLAVE, AK8963_ADDR) >= 0) {
                uint8_t reg = 0x02; // ST1 상태 검사
                // ST1의 DRDY(비트0)가 1일 때만 새 데이터가 준비된 것이다
                if (write(i2c_fd_, &reg, 1) == 1 && read(i2c_fd_, &reg, 1) == 1 && (reg & 0x01)) {
                    uint8_t data_reg = 0x03; // HXL 데이터 시작점 레지스터 주소
                    uint8_t buf[7];
                    // X/Y/Z 6바이트 + ST2 1바이트를 연속으로 읽는다.
                    // ST2까지 읽어야 센서가 다음 측정을 시작하며, HOFL(비트3)이 서면 자기장 포화 = 폐기.
                    if (write(i2c_fd_, &data_reg, 1) == 1 && read(i2c_fd_, buf, 7) == 7 && !(buf[6] & 0x08)) {
                        // AK8963은 리틀엔디안(하위 바이트 먼저), 16bit 해상도에서 0.15 uT/LSB
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
        //
        //    MS5837은 "변환 명령 -> 대기 -> 결과 읽기" 방식이고, OSR 8192에서 변환에
        //    약 17ms가 걸린다. usleep으로 기다리면 100Hz 타이머가 통째로 막히므로,
        //    타이머 주기(10ms)를 카운트하는 상태 머신으로 대기를 쪼갰다.
        //
        //    state 0 : D1(압력) 변환 시작 명령을 3채널에 하달           (10ms)
        //    state 1 : 40ms 대기 후 D1 수거 + D2(온도) 변환 명령 하달   (40ms)
        //    state 2 : 40ms 대기 후 D2 수거 + 압력 환산 + 발행          (40ms)
        //    => 한 바퀴 약 90ms, 즉 압력 토픽은 실질 약 11Hz로 발행된다.
        static int state = 0;
        static int delay_counter = 0;
        static uint32_t D1[3] = {0}, D2[3] = {0};      // 채널별 원시 ADC (압력/온도)
        static float filtered_P[3] = {0.0f};           // 채널별 저역통과 필터 상태
        static bool first_read[3] = {true, true, true};
        uint8_t adc_buf[3];

        if (state == 0) {
            // 압력 데이터(D1) 변환 명령 동시 하향
            for (int i = 0; i < 3; i++) {
                if (prom_C_[i][1] == 0) continue;                 // 죽은 채널 건너뜀
                if (!select_mux_channel_retry(i)) continue;
                if (ioctl(i2c_fd_, I2C_SLAVE, MS5837_ADDR) >= 0) {
                    uint8_t d1_cmd = 0x4A;   // D1(압력) 변환 시작, OSR=8192 (최고 정밀도)
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
                    // mux 선택에 실패하면 엉뚱한 채널 값을 읽게 되므로 D1을 0으로 무효화
                    if (!select_mux_channel_retry(i)) { D1[i] = 0; continue; }
                    if (ioctl(i2c_fd_, I2C_SLAVE, MS5837_ADDR) >= 0) {
                        uint8_t adc_cmd = 0x00;   // ADC Read: 24bit 결과를 3바이트로 회수
                        if (write(i2c_fd_, &adc_cmd, 1) == 1 && read(i2c_fd_, adc_buf, 3) == 3) {
                            D1[i] = ((uint32_t)adc_buf[0] << 16) | ((uint32_t)adc_buf[1] << 8) | adc_buf[2];
                        } else {
                            D1[i] = 0;   // [필수] 실패 시 반드시 무효화 — 아래 주석 참조
                        }
                        // 이어서 온도 데이터(D2) 변환 명령 하향
                        uint8_t d2_cmd = 0x5A;   // D2(온도) 변환 시작, OSR=8192
                        if (write(i2c_fd_, &d2_cmd, 1) < 0) { /* 경고 제어용 */ }
                    } else {
                        D1[i] = 0;
                    }
                }
                state = 2; delay_counter = 0;
            }
        }
        else if (state == 2) {
            // 변환 대기 (약 40ms 안정 확보)
            if (++delay_counter >= 4) {
                auto out_pressure_msg = std_msgs::msg::Float32MultiArray();
                // [0..2] 압력(mbar), [3..5] 온도(C).
                //
                // **데이터 없음은 0.0이 아니라 NaN이다** (2026-08-20 변경).
                // 전에는 resize가 채운 0.0이 그대로 나갔는데, 영점을 뺀 하류
                // (/sensor/pressure_calibrated)에서 0.0은 "수면"이라는 지극히 정상적인
                // 값이라 **버스가 죽은 것과 수면에 떠 있는 것이 구분되지 않았다**.
                // 수심 제어가 붙으면 잠긴 버스를 "수면"으로 읽고 계속 잠수를 시도하게 된다.
                //
                // NaN을 쓰는 이유:
                //   - 어떤 정상값과도 겹치지 않는다 (수심 0이든 -5든 NaN은 아니다)
                //   - 모든 비교가 false라 기존 ">= 100mbar" 유효성 검사에 그대로 걸린다
                //     (하류를 한 줄도 안 고쳐도 무효 판정이 유지된다)
                //   - 계산에 전염된다. 확인을 빼먹은 소비자도 조용히 틀린 답을 내는 대신
                //     결과가 NaN이 되어 눈에 띄게 망가진다
                //   - CSV에 "nan"으로 남아 사후 분석에서 결측이 자동 구분된다
                // 확인은 반드시 std::isnan(x) 로 한다. NaN은 자기 자신과도 같지 않아
                // x == NAN 은 **항상 false**다.
                out_pressure_msg.data.assign(6, std::numeric_limits<float>::quiet_NaN());

                for (int i = 0; i < 3; i++) {
                    if (prom_C_[i][1] == 0) continue;
                    // [필수] 어떤 경로로 실패하든 D1/D2를 0으로 무효화한다.
                    //
                    // D1/D2는 static이라 갱신에 실패하면 **직전 성공값이 그대로 남는다**.
                    // 예전에는 그 상태로 환산이 돌아, 센서를 뽑아도 마지막 값이 소수점까지
                    // 똑같이 계속 발행됐다 (2026-08-21 실측: J5를 뽑았는데 1005.7000이
                    // 8샘플 연속 동일. 살아있는 채널은 잡음으로 매번 달랐다).
                    // 0.0이나 NaN보다 훨씬 위험하다 — 완벽하게 정상으로 보이기 때문이다.
                    // 잠수 중에 센서가 죽으면 수심이 그 값에 얼어붙고 아무도 모른다.
                    // 0은 아래 환산 가드가 무효로 걸러내고, 결국 NaN으로 발행된다.
                    if (!select_mux_channel_retry(i)) { D2[i] = 0; continue; }
                    if (ioctl(i2c_fd_, I2C_SLAVE, MS5837_ADDR) >= 0) {
                        uint8_t adc_cmd = 0x00;
                        if (write(i2c_fd_, &adc_cmd, 1) == 1 && read(i2c_fd_, adc_buf, 3) == 3) {
                            D2[i] = ((uint32_t)adc_buf[0] << 16) | ((uint32_t)adc_buf[1] << 8) | adc_buf[2];
                        } else {
                            D2[i] = 0;
                        }

                        // 0xFFFFFF(통신 실패)와 0(미수거)은 환산에서 제외
                        if (D1[i] != 0xFFFFFF && D2[i] != 0xFFFFFF && D1[i] != 0 && D2[i] != 0) {
                            // ---- MS5837-02BA 데이터시트 1차 온도 보상 환산식 ----
                            // C[1]=SENS_T1, C[2]=OFF_T1, C[3]=TCS, C[4]=TCO, C[5]=T_REF, C[6]=TEMPSENS
                            //
                            // ※ 상수가 모델마다 다르다. 30BA와 02BA는 OFF/SENS/최종 나눗셈이
                            //   전부 2배 또는 4배씩 다르고 결과 단위도 0.1 vs 0.01 mbar다.
                            //   틀린 식을 쓰면 정확히 20배 어긋난 값이 나오는데(2026-08-20 실측:
                            //   30BA 식 20132.60 vs 02BA 식 1006.63), 하류의 유효성 검사가
                            //   "100mbar 이상"이라 그대로 통과해 조용히 수심을 20배로 만든다.
                            //   센서를 교체할 때는 이 블록을 반드시 같이 확인할 것.
                            //
                            //            30BA            02BA(현재)
                            //   OFF   C2*2^16 + C4dT/2^7   C2*2^17 + C4dT/2^6
                            //   SENS  C1*2^15 + C3dT/2^8   C1*2^16 + C3dT/2^7
                            //   P     (...)/2^13 [0.1mbar] (...)/2^15 [0.01mbar]
                            uint16_t *C = prom_C_[i];
                            int32_t dT = D2[i] - ((uint32_t)C[5] * 256);              // 기준 온도와의 차 (모델 공통)
                            int32_t TEMP = 2000 + ((int64_t)dT * C[6]) / 8388608;     // 실제 온도 (0.01°C, 모델 공통)
                            int64_t OFF = ((int64_t)C[2] * 131072) + (((int64_t)C[4] * dT) / 64);    // 온도 보상된 오프셋
                            int64_t SENS = ((int64_t)C[1] * 65536) + (((int64_t)C[3] * dT) / 128);   // 온도 보상된 감도
                            int32_t P = ((D1[i] * SENS / 2097152) - OFF) / 32768;     // 압력 (0.01 mbar 단위)

                            float raw_pressure = P / 100.0f;   // 0.01mbar -> mbar

                            // 모델 불일치 조기 경보. 공기 중 대기압은 어디서든 300~1200mbar
                            // 안에 들어온다(02BA 정격 범위와 동일). 여기를 벗어나면 환산식이
                            // 센서 모델과 안 맞거나 센서가 고장난 것이다. 위에서 설명한
                            // "조용히 20배" 사고를 여기서 잡는다. 판정만 하고 값은 그대로 흘린다.
                            if (raw_pressure < 300.0f || raw_pressure > 1200.0f) {
                                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                                    "[MS5837] 채널 %d 압력 %.1f mbar — 대기압 범위(300~1200) 밖입니다. "
                                    "센서 모델과 환산식이 맞는지 확인하세요 (현재 코드는 02BA용).",
                                    i, raw_pressure);
                            }

                            // 1차 IIR 저역통과 필터 (계수 0.1 = 시정수 약 0.8초 @11Hz).
                            //
                            // ※ 이 필터는 발행 전에 걸리므로 원본은 어디에도 남지 않는다.
                            //   시정수 0.86초는 코너 주파수 0.19Hz로, 수심 제어를 붙이면
                            //   루프 대역폭과 같은 영역이라 위상 여유를 깎을 수 있다.
                            //   그때 계수를 재검토할 것 (잡음은 무필터에서도 0.20mbar =
                            //   수심 2mm로 여유가 크다 — 맞바꿀 여지는 지연 쪽에 있다).
                            //
                            // 첫 샘플은 필터 상태를 그대로 세팅해 0에서 서서히 올라오는 것을 막는다.
                            if (first_read[i]) {
                                filtered_P[i] = raw_pressure;
                                first_read[i] = false;
                            } else {
                                // ##### [임시 2026-08-19 — 압력 특성화 측정용] #####
                                // 필터를 통과시킨다. 잡음 실측(press_char.py --s3)이
                                // 필터 뒤에서는 4.4배 작게 나오기 때문이다.
                                // 측정이 끝나면 아래 줄을 지우고 그 아래 원본을 되살릴 것.
                                filtered_P[i] = raw_pressure;
                                // filtered_P[i] = (0.1f * raw_pressure) + (0.9f * filtered_P[i]);
                                // ##### 임시 끝 #####
                            }

                            out_pressure_msg.data[i] = filtered_P[i];           // 압력 채널 0,1,2 (mbar)
                            out_pressure_msg.data[i + 3] = TEMP / 100.0f;       // 온도 채널 0,1,2 (C)
                        } else {
                            // 환산 불가 — 이 채널은 NaN인 채로 나간다. 다음 사이클이
                            // 깨끗하게 시작되도록 첫 샘플 플래그도 되돌린다.
                            first_read[i] = true;
                        }
                    } else {
                        D2[i] = 0;
                    }
                }
                // 연속 실패 감지. 예전에는 먹스 선택 실패도 ADC 읽기 실패도 로그 없이
                // 지나가서, 버스가 잠겨도 저널만 봐서는 알 수 없었다 (2026-08-20 실측:
                // t=42~45초 3초 결측이 아무 흔적도 남기지 않았다).
                int alive_now = 0;
                for (int i = 0; i < 3; i++)
                    if (std::isfinite(out_pressure_msg.data[i])) alive_now++;

                if (alive_now == 0) {
                    if (++press_fail_streak_ == 30) {          // 약 3초 (11Hz 기준)
                        RCLCPP_ERROR(this->get_logger(),
                            "[MS5837] 전 채널 %d사이클 연속 실패 — I2C 버스 잠김 의심. "
                            "압력은 NaN으로 발행 중(수심 0이 아님). 전원 재인가가 필요할 수 있음.",
                            press_fail_streak_);
                    } else if (press_fail_streak_ > 30 && press_fail_streak_ % 300 == 0) {
                        RCLCPP_ERROR(this->get_logger(),
                            "[MS5837] 전 채널 실패 지속 (%d사이클, 약 %d초).",
                            press_fail_streak_, press_fail_streak_ / 11);
                    }
                } else {
                    if (press_fail_streak_ >= 30) {
                        RCLCPP_WARN(this->get_logger(),
                            "[MS5837] 압력 복구됨 (%d사이클 만에, 살아있는 채널 %d개).",
                            press_fail_streak_, alive_now);
                    }
                    press_fail_streak_ = 0;
                }

                pressure_pub_->publish(out_pressure_msg);
                state = 0;   // 다음 사이클 시작
            }
        }
    }

    int i2c_fd_;
    int press_fail_streak_ = 0;   // 압력 전 채널 연속 실패 사이클 수 (버스 잠김 감지)
    bool has_mag_;
    uint16_t prom_C_[3][8];   // [채널][PROM 워드] — [1]이 0이면 "해당 채널 사용 불가" 마커
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
