// =============================================================================
//  data_logger_node.cpp
//
//  역할 : 시스템 전 토픽을 한 곳에서 구독해 100Hz로 CSV 한 줄씩 기록하는 블랙박스
//
//   [입력] /filtered/attitude          (Vector3)             자세 + 모드 플래그
//          /sensor/pressure_calibrated (Float32MultiArray[3]) 영점 보정된 압력
//          /sensor/pressure_raw        (Float32MultiArray[6]) 원시 절대압[0..2] + 온도[3..5]
//          /rc/command                 (Quaternion)          조종기 스틱
//          /rc/status                  (Int32MultiArray[5])  버튼/배터리/RSSI
//          /motor/output               (UInt16MultiArray[4]) 최종 모터 출력
//          /sensor/tail_rpm            (Int32)               꼬리 RPM
//          /raw/imu_6dof               (Imu)                  원시 6축 — g, 도/초 그대로
//          /raw/magnetometer           (Vector3)              원시 지자기[µT], 보정 전 칩 축
//          /filtered/ekf_status        (Float32MultiArray[12]) [0..2] 자이로 바이어스, [10] 플래그
//   [출력] ~/ros2_ws/log_csv/fish_log_YYYYMMDD_HHMMSS.csv
//
//  설계 : 콜백은 값을 멤버 변수에 "저장만" 하고, 10ms 타이머가 그 순간의
//         최신 스냅샷을 한 줄로 기록한다. 토픽마다 주기가 달라도(압력 11Hz,
//         자세 100Hz) 시간축이 일정한 표가 만들어진다.
//
//  원시값 3종을 상시 기록하는 이유(2026-08-18): 물속 운용은 재현이 안 된다.
//  융합 결과만 남으면 자세 이상이 "센서 탓인지 필터 탓인지" 사후 규명이 불가능하다.
//
//  ※ 크기: 100Hz x 39컬럼 ≈ 시간당 84MB. 파일당 200MB(약 2.4시간)에서 새 파일로
//     회전하며, Time(s) 축은 회전을 넘어 이어진다(기동 시각 기준 유지). 오래된
//     파일은 지우지 않는다 — log_csv/에는 실험 CSV와 지자기 보정 파일이 같이
//     살므로 자동 삭제는 위험하다. 총량 관리는 사람 몫.
// =============================================================================

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <limits>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

class DataLoggerNode : public rclcpp::Node {
public:
    DataLoggerNode() : Node("data_logger_node"), fp_(nullptr) {
        RCLCPP_INFO(this->get_logger(), "=== [Data Logger] 누적 카운트 및 동기화 로깅 대기 중 ===");

        if (init_log_file() < 0) {
            RCLCPP_ERROR(this->get_logger(), "CSV 파일 시스템 초기화 실패!");
            return;
        }

        // [추가/수정] 버튼 누적 카운트 및 상태 전이 감지용 변수 초기화
        btn1_cumulative_count_ = 0;
        btn1_hold_counter_ = 0;
        prev_btn1_ = 0;
        btn1_long_processed_ = false;
        is_auto_mode_ = false;

        // 각 컬럼의 초기값. 해당 센서가 아직 안 붙은 구간은 이 값이 그대로 기록된다.
        mode_ = 0;
        roll_ = 0.0f; pitch_ = 0.0f; yaw_ = 0.0f;
        rc_t_ = 0; rc_r_ = 0; rc_p_ = 0; rc_y_ = 0;
        m1_ = 1500; m2_ = 1500; m3_ = 1500; m4_ = 1000;   // 서보 중립 + 추진 정지
        rpm_ = 0;
        p0_cal_ = 0.0f; p1_cal_ = 0.0f; p2_cal_ = 0.0f;
        t0_ = 0.0f; t1_ = 0.0f; t2_ = 0.0f;
        // 원시 절대압은 0.0이 아니라 NaN으로 시작한다. 대기압은 ~1013mbar라 0.0이
        // 물리적으로 불가능하긴 하지만, 드라이버가 죽은 채널을 NaN으로 표시하므로
        // (i2c_driver_node.cpp:282-296) "아직 안 받음"도 같은 표기로 두어야
        // 분석 스크립트가 한 가지 규칙(isnan)만 알면 되게 한다.
        {
            const float nan_f = std::numeric_limits<float>::quiet_NaN();
            p0_raw_ = nan_f; p1_raw_ = nan_f; p2_raw_ = nan_f;
        }
        rssi_ = 0;
        ax_ = 0.0f; ay_ = 0.0f; az_ = 0.0f;             // 원시 가속도 [g]
        gx_ = 0.0f; gy_ = 0.0f; gz_ = 0.0f;             // 원시 자이로 [도/초]
        mx_ = 0.0f; my_ = 0.0f; mz_ = 0.0f;             // 원시 지자기 [µT]
        bias_x_ = 0.0f; bias_y_ = 0.0f; bias_z_ = 0.0f; // EKF 자이로 바이어스 추정 [도/초]
        ekf_flags_ = 0;

        received_attitude_ = false;
        received_pressure_ = false;
        is_logging_active_ = false;

        attitude_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/filtered/attitude", 10, std::bind(&DataLoggerNode::attitude_callback, this, std::placeholders::_1));

        pressure_cal_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/sensor/pressure_calibrated", 10, std::bind(&DataLoggerNode::pressure_cal_callback, this, std::placeholders::_1));

        // 원시 압력([0..2] 절대압 mbar)과 원시 온도([3..5] C)를 함께 기록한다.
        // 원시 압력은 수심·속도(피토 차압)를 사후에 다시 계산하기 위한 원본이다 —
        // 보정본(P*_cal)은 EKF가 잡은 영점이 이미 빠져 있어 역산이 되지 않는다.
        pressure_raw_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/sensor/pressure_raw", 10, std::bind(&DataLoggerNode::pressure_raw_callback, this, std::placeholders::_1));

        rc_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
            "/rc/command", 10, std::bind(&DataLoggerNode::rc_cmd_callback, this, std::placeholders::_1));

        rc_status_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/rc/status", 10, std::bind(&DataLoggerNode::rc_status_callback, this, std::placeholders::_1));

        motor_sub_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
            "/motor/output", 10, std::bind(&DataLoggerNode::motor_callback, this, std::placeholders::_1));

        rpm_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/sensor/tail_rpm", 10, std::bind(&DataLoggerNode::rpm_callback, this, std::placeholders::_1));

        // ---- 원시값 3종 (2026-08-18 추가) — 사후 진단용 상시 기록 ----
        imu_raw_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/raw/imu_6dof", 10, std::bind(&DataLoggerNode::imu_raw_callback, this, std::placeholders::_1));

        mag_raw_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/raw/magnetometer", 10, std::bind(&DataLoggerNode::mag_raw_callback, this, std::placeholders::_1));

        ekf_status_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/filtered/ekf_status", 10, std::bind(&DataLoggerNode::ekf_status_callback, this, std::placeholders::_1));

        // 100Hz 기록 타이머 (모든 토픽의 최신 스냅샷을 한 줄로)
        logging_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&DataLoggerNode::write_log_loop, this));
    }

    ~DataLoggerNode() {
        // 소멸 시 버퍼에 남은 마지막 몇 줄까지 디스크에 확실히 내려쓴다.
        if (fp_ != nullptr) {
            fflush(fp_);
            fclose(fp_);
            RCLCPP_INFO(this->get_logger(), "[Data Logger] 로그 파일 보관 및 스트림 플러시 완료.");
        }
    }

private:
    int init_log_file() {
        const char* log_dir_path = "/home/fish/ros2_ws/log_csv";
        struct stat st;
        memset(&st, 0, sizeof(struct stat));

        // 디렉터리가 없으면 생성
        if (stat(log_dir_path, &st) == -1) {
            if (mkdir(log_dir_path, 0775) < 0) return -1;
        }

        // 파일명에 기동 시각을 박아 실행마다 새 파일이 생기게 한다 (덮어쓰기 사고 방지)
        time_t t = time(nullptr);
        struct tm tm_info;
        localtime_r(&t, &tm_info);   // _r 버전: 스레드 안전

        char file_path[512];
        snprintf(file_path, sizeof(file_path), "%s/fish_log_%04d%02d%02d_%02d%02d%02d.csv",
                 log_dir_path, tm_info.tm_year + 1900, tm_info.tm_mon + 1, tm_info.tm_mday,
                 tm_info.tm_hour, tm_info.tm_min, tm_info.tm_sec);

        fp_ = fopen(file_path, "w");
        if (fp_ == nullptr) return -1;

        // 새 컬럼은 반드시 기존 컬럼 "뒤에" 덧붙인다 — 열 번호로 읽는 기존
        // 분석 스크립트(예전 C 펌웨어 규격 포함)가 깨지지 않게 하는 규칙이다.
        // 현재 36컬럼까지가 고정 구간이고, 37열부터가 이번 확장분이다.
        // AX..AZ[g], GX..GZ[도/초] — nRF 원시 규약 그대로 (REP-103 아님)
        // MX..MZ[µT] — 보정 전 칩 축. 원시로 남겨야 나중에 새 보정 계수를
        //              오프라인으로 다시 적용할 수 있다 (보정 후 값은 역산 불가)
        // BiasX..Z[도/초] — EKF 온라인 추정. EKF_Flags bit5(0x20)=지자기 두절(6축 폴백)
        // ※ 축 주의: AX..GZ·MX..MZ는 "칩 축", BiasX..Z는 "FLU 몸체 축"이다.
        //    원시 자이로에서 바이어스를 직접 빼면 틀린다 — 칩→몸체 변환
        //    (bx,by,bz) = (-gz, -gx, +gy) 를 거친 뒤 빼야 한다 (실측 대조 완료:
        //    BiasZ -3.06 이 원시 GY -3.0 에 대응).
        // P0_raw..P2_raw[mbar] — 영점 전 절대압. 보정본(P0_cal..)과 나란히 있어야
        //    (원시 - 보정) = EKF가 잡은 대기압 영점이 사후에 그대로 복원된다.
        //    수심·피토 차압을 나중에 다른 상수로 다시 계산하려면 이 원본이 필요하다.
        //    죽은 채널은 nan으로 찍힌다 (0.0 아님).
        fprintf(fp_, "Time(s),Btn1_Count,Mode,AutoMode,Roll,Pitch,Yaw,RC_T,RC_R,RC_P,RC_Y,M1,M2,M3,M4,RPM,P0_cal,P1_cal,P2_cal,T0,T1,T2,RSSI,"
                     "AX,AY,AZ,GX,GY,GZ,MX,MY,MZ,BiasX,BiasY,BiasZ,EKF_Flags,"
                     "P0_raw,P1_raw,P2_raw\n");
        fflush(fp_);   // 헤더는 즉시 기록해 파일이 비어 보이지 않게 한다

        return 0;
    }

    // 각 센서가 처음 붙는 시점을 로그 시간축 기준으로 남긴다. 로깅이 기동 즉시
    // 시작되므로, 초기값 구간과 실제 데이터 구간의 경계를 이걸로 찾을 수 있다.
    void note_first_arrival(bool &flag, const char *what) {
        if (flag) return;
        flag = true;
        double t = is_logging_active_ ? (this->now() - start_time_).seconds() : 0.0;
        RCLCPP_INFO(this->get_logger(), "[Data Logger] %s 수신 시작 (t = %.3f초).", what, t);
    }

    void attitude_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        note_first_arrival(received_attitude_, "자세(attitude)");
        // state_estimation이 자동 모드 표시용으로 roll에 실은 ±5000 오프셋을 제거해
        // 실제 자세각을 기록한다 (pid_control_node의 디코딩과 동일 규칙).
        // 동시에 이 오프셋을 로봇의 실제 모드(단일 출처)로 사용한다.
        if (msg->x > 2500.0f) {
            roll_ = msg->x - 5000.0f;
            is_auto_mode_ = true;
        } else if (msg->x < -2500.0f) {
            roll_ = msg->x + 5000.0f;
            is_auto_mode_ = true;
        } else {
            roll_ = msg->x;
            is_auto_mode_ = false;
        }
        pitch_ = msg->y;
        yaw_   = msg->z;
    }

    void pressure_cal_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 3) return;
        note_first_arrival(received_pressure_, "보정 압력(pressure_calibrated)");
        p0_cal_ = msg->data[0];
        p1_cal_ = msg->data[1];
        p2_cal_ = msg->data[2];
    }

    // 원시 압력 토픽: [0..2] 절대압(mbar, 영점 전), [3..5] 채널별 온도(C).
    // 죽은 채널은 드라이버가 NaN으로 채워 보내므로 그대로 통과시킨다 — 여기서
    // 거르면 CSV에서 "고장"과 "정상적으로 0"을 구별할 수 없게 된다.
    void pressure_raw_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 6) return;
        p0_raw_ = msg->data[0];
        p1_raw_ = msg->data[1];
        p2_raw_ = msg->data[2];
        t0_ = msg->data[3];
        t1_ = msg->data[4];
        t2_ = msg->data[5];
    }

    // 조종기 스틱 값. -9999(두절 센티넬)도 걸러내지 않고 그대로 기록해
    // 나중에 로그에서 두절 구간을 눈으로 찾을 수 있게 한다.
    void rc_cmd_callback(const geometry_msgs::msg::Quaternion::SharedPtr msg) {
        rc_r_ = static_cast<int16_t>(msg->x);
        rc_p_ = static_cast<int16_t>(msg->y);
        rc_y_ = static_cast<int16_t>(msg->z);
        rc_t_ = static_cast<int16_t>(msg->w);
    }

    void rpm_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        rpm_ = msg->data;
    }

    void rc_status_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 5) return;
        int current_btn1 = msg->data[0];
        rssi_ = msg->data[4];

        // 짧은 누름(Edge Detection)으로 버튼 누적 카운트만 집계한다.
        // 자동/수동 모드 판정은 여기서 하지 않고 state_estimation이 방송한
        // /filtered/attitude 오프셋(attitude_callback)을 단일 출처로 사용한다.
        //
        // /rc/status는 100Hz로 오므로 카운터 1틱 = 10ms다.
        if (current_btn1 == 1) {
            btn1_hold_counter_++;
            if (btn1_hold_counter_ == 300) {   // 3초 이상 = 롱프레스 -> 짧은 누름 집계에서 제외
                btn1_long_processed_ = true;
            }
        } else {
            // 진짜 뗌(current==0)만 짧은 누름으로 인정한다. 통신두절 시 btn=255로 튀는
            // 1→255 전이를 '뗌'으로 오인해 카운트가 증가하는 것을 막는다(N4).
            if (prev_btn1_ == 1 && current_btn1 == 0) {
                // 손을 떼는 순간(Falling Edge), 30ms 이상 1초 이하로 눌렸을 때만 짧은 누름으로 인정
                // (하한: 채터링 제거 / 상한: 롱프레스와 구분)
                if (btn1_hold_counter_ > 3 && btn1_hold_counter_ < 100 && !btn1_long_processed_) {
                    btn1_cumulative_count_++; // 누적 카운트 증가!
                }
            }
            btn1_hold_counter_ = 0;
            btn1_long_processed_ = false;
        }
        prev_btn1_ = current_btn1;

        // 통신 상태만 결정한다 (0 = 정상, 2 = 조종기 두절 -9999 센티넬).
        // 자동/수동 여부는 별도 AutoMode 컬럼(is_auto_mode_)으로 분리 기록한다.
        if (rc_t_ == -9999) {
            mode_ = 2;
        } else {
            mode_ = 0;
        }
    }

    void motor_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
        if (msg->data.size() < 4) return;
        m1_ = msg->data[0];
        m2_ = msg->data[1];
        m3_ = msg->data[2];
        m4_ = msg->data[3];
    }

    // 원시 6축. nRF가 보낸 g·도/초 단위 그대로다(REP-103 아님) — 변환 없이
    // 기록해야 발행 당시 값과 1:1로 대조된다.
    void imu_raw_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        ax_ = static_cast<float>(msg->linear_acceleration.x);
        ay_ = static_cast<float>(msg->linear_acceleration.y);
        az_ = static_cast<float>(msg->linear_acceleration.z);
        gx_ = static_cast<float>(msg->angular_velocity.x);
        gy_ = static_cast<float>(msg->angular_velocity.y);
        gz_ = static_cast<float>(msg->angular_velocity.z);
    }

    void mag_raw_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        mx_ = static_cast<float>(msg->x);
        my_ = static_cast<float>(msg->y);
        mz_ = static_cast<float>(msg->z);
    }

    // EKF 진단(10Hz 발행이라 CSV에는 같은 값이 ~10행씩 반복된다 — 정상).
    // 바이어스는 세션 간 50배씩 다르고 열로도 흐르므로(2시간에 0.06도/초 실측)
    // 자세 이상 시 "바이어스 폭주였나"를 이 열이 즉답한다.
    void ekf_status_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 12) return;
        bias_x_ = msg->data[0];
        bias_y_ = msg->data[1];
        bias_z_ = msg->data[2];
        ekf_flags_ = static_cast<int32_t>(msg->data[10]);
    }

    void write_log_loop() {
        // 센서 연결 여부와 무관하게 노드 기동 즉시 로깅을 시작한다.
        // (이전에는 received_attitude_ && received_pressure_ 를 요구했는데, 압력센서를
        //  물리적으로 떼고 벤치 테스트하면 /sensor/pressure_calibrated가 영원히 발행되지
        //  않아 CSV에 헤더만 남고 데이터가 0행이었다. 각 센서가 붙기 전 구간은 초기값으로
        //  기록되며, 실제 수신 시작 시점은 아래 콜백들이 INFO 로그로 남긴다.)
        if (!is_logging_active_) {
            is_logging_active_ = true;
            start_time_ = this->now();   // 이 시각이 Time(s) 컬럼의 0초
            RCLCPP_INFO(this->get_logger(), "=========================================");
            RCLCPP_INFO(this->get_logger(), "[Data Logger] 데이터 로깅을 0.0초부터 시작합니다.");
            RCLCPP_INFO(this->get_logger(), "[Data Logger] (센서 미연결 구간은 초기값으로 기록됩니다)");
            RCLCPP_INFO(this->get_logger(), "=========================================");
        }

        if (fp_ == nullptr) return;

        double relative_time_sec = (this->now() - start_time_).seconds();

        // btn1_cumulative_count_ 를 기록합니다.
        // 자릿수: 가속도/바이어스 %.4f (바이어스 불안정성 실측이 0.001도/초 수준),
        //         자이로 %.3f, 지자기 %.2f (노이즈 ~0.1µT 아래는 의미 없음)
        //         원시 압력 %.3f — 0.001mbar = 수심 0.01mm. 실측 잡음이 0.13mbar라
        //         기존 %.2f로도 버티지만, 사후 재계산의 원본이라 잘라 둘 이유가 없다.
        fprintf(fp_, "%.3f,%d,%d,%d,%.2f,%.2f,%.2f,%d,%d,%d,%d,%u,%u,%u,%u,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,"
                     "%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%d,"
                     "%.3f,%.3f,%.3f\n",
                relative_time_sec,
                btn1_cumulative_count_,
                mode_,
                is_auto_mode_ ? 1 : 0,
                roll_, pitch_, yaw_,
                rc_t_, rc_r_, rc_p_, rc_y_,
                m1_, m2_, m3_, m4_,
                rpm_,
                p0_cal_, p1_cal_, p2_cal_,
                t0_, t1_, t2_,
                rssi_,
                ax_, ay_, az_,
                gx_, gy_, gz_,
                mx_, my_, mz_,
                bias_x_, bias_y_, bias_z_,
                ekf_flags_,
                p0_raw_, p1_raw_, p2_raw_);

        // 50줄(0.5초)마다 디스크로 강제 플러시.
        // 매 줄 플러시하면 SD카드 I/O로 100Hz 루프가 밀리고, 아예 안 하면
        // 전원이 갑자기 끊길 때 마지막 수 초가 통째로 날아간다. 그 절충점.
        static int flush_cnt = 0;
        if (++flush_cnt >= 50) {
            fflush(fp_);
            flush_cnt = 0;

            // 파일당 200MB 회전. 판정은 플러시 직후에만 하므로 ftell이 곧 실제
            // 파일 크기다. start_time_은 건드리지 않는다 — Time(s)가 파일 경계를
            // 넘어 이어져야 회전된 파일들을 그대로 이어붙여 분석할 수 있다.
            if (ftell(fp_) >= ROTATE_BYTES) {
                fclose(fp_);
                fp_ = nullptr;
                if (init_log_file() < 0) {
                    RCLCPP_ERROR(this->get_logger(),
                                 "[Data Logger] 로그 회전 실패 — 기록 중단 (디스크 용량 확인 필요)");
                    return;
                }
                RCLCPP_INFO(this->get_logger(),
                            "[Data Logger] 200MB 도달 — 새 파일로 회전 (t = %.1f초, Time축 연속).",
                            relative_time_sec);
            }
        }
    }

    FILE* fp_;

    bool received_attitude_;    // 최초 수신 로그를 한 번만 찍기 위한 래치
    bool received_pressure_;
    bool is_logging_active_;
    rclcpp::Time start_time_;   // Time(s) 컬럼의 기준 시각

    // [추가] 버튼 엣지 디텍션 및 상태 관리용 변수들
    int32_t btn1_cumulative_count_;   // 짧은 누름 누적 횟수 (CSV Btn1_Count 컬럼)
    int btn1_hold_counter_;           // 현재 눌림 지속 틱 수 (1틱 = 10ms)
    int prev_btn1_;                   // 직전 버튼 상태 (엣지 판정용)
    bool btn1_long_processed_;        // 롱프레스로 이미 처리됨 -> 뗄 때 짧은 누름 집계 안 함
    bool is_auto_mode_;

    // ---- 이하가 CSV 한 줄을 구성하는 최신 스냅샷 ----
    int32_t mode_;                             // 0=정상, 2=조종기 두절
    float roll_, pitch_, yaw_;
    int16_t rc_t_, rc_r_, rc_p_, rc_y_;
    uint16_t m1_, m2_, m3_, m4_;
    int32_t rpm_;
    float p0_cal_, p1_cal_, p2_cal_;
    float t0_, t1_, t2_;      // 채널별 원시 온도(C) — 압력 드리프트 진단용
    float p0_raw_, p1_raw_, p2_raw_;  // 채널별 원시 절대압(mbar, 영점 전). 고장/미수신 = NaN
    int32_t rssi_;
    float ax_, ay_, az_, gx_, gy_, gz_;      // 원시 6축 [g, 도/초]
    float mx_, my_, mz_;                     // 원시 지자기 [µT], 보정 전 칩 축
    float bias_x_, bias_y_, bias_z_;         // EKF 자이로 바이어스 추정 [도/초]
    int32_t ekf_flags_;                      // EKF 상태 비트 (bit5 = 6축 폴백)

    static constexpr long ROTATE_BYTES = 200L * 1024 * 1024;   // 파일당 회전 문턱

    rclcpp::TimerBase::SharedPtr logging_timer_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr attitude_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pressure_cal_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pressure_raw_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr rc_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr rc_status_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr motor_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rpm_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_raw_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr mag_raw_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr ekf_status_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DataLoggerNode>();
    // 생성자에서 파일 열기에 실패하면 rclcpp::shutdown()이 호출된 상태일 수 있으므로
    // spin 전에 한 번 확인한다.
    if (rclcpp::ok()) {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}
