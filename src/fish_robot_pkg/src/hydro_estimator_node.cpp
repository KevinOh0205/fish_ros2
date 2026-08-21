// =============================================================================
//  hydro_estimator_node.cpp
//
//  역할 : 압력 3채널에서 **수심**과 (이후 단계에서) **전진 속도**를 만드는 노드.
//         이번 단계에서 구현된 것은 수심까지다 — 속도 관련 필드는 NaN으로 나간다.
//
//   [입력] /sensor/pressure_raw (Float32MultiArray[6]) 절대압 3채널[mbar] + 온도 3채널[°C], ~11Hz
//   [출력] /filtered/hydro      (Float32MultiArray[8])  아래 배열 규격 참조
//   [서비스] /hydro/zero_atm (std_srvs/Trigger)  대기압 영점 재포착
//
//  ── 왜 새 노드인가 ──────────────────────────────────────────────────────────
//  기존 파이프라인은 게이지 mbar에서 끝난다. mbar->m 변환이 실행 노드 어디에도
//  없었다 (press_char.py 의 MBAR_TO_MM 만 존재, 오프라인 전용). 그 변환을
//  state_estimation_ekf_node 에 얹지 않은 이유는 그 노드가 이미 1600줄에
//  자세 경로를 쥐고 있어서다. 수심/속도 필터를 튜닝하다 죽으면 자세까지 죽는다.
//
//  ── 왜 /sensor/pressure_calibrated 가 아니라 원시본을 쓰는가 ────────────────
//  1) EKF의 press_ch_alive_[i] 는 **영구 래치**다. 100샘플 영점 구간에 죽어
//     있던 채널은 하드웨어가 복구돼도(i2c_driver_node 가 "압력 복구됨"을 찍어도)
//     btn1 재영점 전까지 영원히 NaN이다. 그 결함을 물려받을 이유가 없다.
//  2) 보정본은 3열이라 온도가 없다. 2차 온도보상을 나중에 붙일 수 없다.
//  3) 압력 경로가 자세 경로에 종속되는 방향이 거꾸로다.
//  대가: 시스템에 대기압 영점이 둘이 된다(여기 것과 EKF 것). 캡처 시각이 달라
//  수백분의 1 mbar 어긋나는 건 정상이다 — 아래 캡처 로그에 그렇게 적어둔다.
//
//  ── 수심이 자세와 무관한 이유 (근사가 아니라 정확히) ────────────────────────
//  압력은 깊이에 선형이므로 **두 정압 포트의 산술평균 = 두 점의 기하학적 중점의
//  압력**이다. 좌우가 ±y 대칭이면 롤이 정확히 상쇄되고(한쪽 −3cm면 반대쪽 +3cm),
//  둘이 같은 x면 피치는 공통으로 움직인다. 그래서 이 노드는 수심을 내는 데
//  /filtered/attitude 를 구독조차 하지 않는다. IMU가 빠져 있어도 수심은 나온다.
//  ※ 정압 포트가 하나만 살아남으면 이 성질이 깨진다 → 플래그로 알린다.
//  ※ 앞(정체압) 포트는 정면을 향해 동압이 섞이므로 **수심에 쓰면 안 된다**
//    (빨리 갈수록 깊다고 착각한다). 속도 단계에서 쓴다.
//
//  ── /filtered/hydro 배열 규격 (8) ──────────────────────────────────────────
//    [0] depth        [m]     수심. 영점 미포착·정압 전멸이면 NaN
//    [1] speed        [m/s]   ※ 이번 단계 미구현 — NaN
//    [2] q            [mbar]  ※ 이번 단계 미구현 — NaN
//    [3] q_raw        [mbar]  ※ 이번 단계 미구현 — NaN
//    [4] dP_hydro     [mbar]  ※ 이번 단계 미구현 — NaN
//    [5] static_gauge [mbar]  살아있는 정압 포트의 게이지압 평균
//    [6] pitch_used   [도]    ※ 이번 단계 미구현 — NaN
//    [7] flags        비트    아래 FLAG_* 참조
//  커스텀 메시지를 만들지 않는 프로젝트 관례를 따른다. flags를 float에 싣는 것도
//  /filtered/ekf_status data[10] 과 같은 방식이다 (float32는 2^24까지 정수 정확).
// =============================================================================

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

// 배열 인덱스 — 숫자를 코드 여기저기 흩뿌리지 않는다
enum { H_DEPTH = 0, H_SPEED, H_Q, H_QRAW, H_DPHYDRO, H_STATIC, H_PITCH, H_FLAGS, H_LEN };

// 플래그. 앞쪽(0x001~0x020)은 "정상 동작 중"을 켜는 비트, 뒤쪽은 고장 비트다.
static constexpr uint32_t FLAG_DEPTH_OK   = 0x001;   // 수심 유효
static constexpr uint32_t FLAG_SPEED_OK   = 0x002;   // 속도/q 유효 (이번 단계에선 항상 0)
static constexpr uint32_t FLAG_ATM_ZERO   = 0x004;   // 대기압 영점 확보
static constexpr uint32_t FLAG_Q_ZERO     = 0x008;   // 물속 q 영점 확보 (이번 단계 미사용)
static constexpr uint32_t FLAG_ATT_FRESH  = 0x010;   // 자세 신선 (이번 단계 미사용)
static constexpr uint32_t FLAG_ATT_FALLBK = 0x020;   // 자세 폴백 (이번 단계 미사용)
static constexpr uint32_t FLAG_FRONT_DEAD = 0x040;   // 앞(정체압) 포트 사망
static constexpr uint32_t FLAG_LEFT_DEAD  = 0x080;   // 좌 정압 포트 사망
static constexpr uint32_t FLAG_RIGHT_DEAD = 0x100;   // 우 정압 포트 사망
static constexpr uint32_t FLAG_Q_DEADBAND = 0x200;   // (이번 단계 미사용)
static constexpr uint32_t FLAG_Q_NEGATIVE = 0x400;   // (이번 단계 미사용)
static constexpr uint32_t FLAG_PRESS_STOP = 0x800;   // 압력 스트림 정지

class HydroEstimatorNode : public rclcpp::Node {
public:
    HydroEstimatorNode() : Node("hydro_estimator_node") {
        declare_params();
        load_port_map();

        node_start_time_ = this->now();
        last_press_time_ = this->now();

        for (int i = 0; i < 3; i++) atm_offset_[i] = NAN;

        // 저장된 영점이 신선하면 그대로 쓴다 — respawn 되어도 수심이 안 끊긴다.
        load_zero_file();

        pressure_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/sensor/pressure_raw", 10,
            std::bind(&HydroEstimatorNode::pressure_callback, this, std::placeholders::_1));

        hydro_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/filtered/hydro", 10);

        zero_atm_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/hydro/zero_atm",
            std::bind(&HydroEstimatorNode::handle_zero_atm, this,
                      std::placeholders::_1, std::placeholders::_2));

        // 압력이 끊긴 것을 알리는 워치독. 압력 콜백 안에서만 판정하면 스트림이
        // 아예 멈췄을 때 판정 자체가 안 돌아 영원히 침묵한다.
        watchdog_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&HydroEstimatorNode::watchdog, this));

        log_effective_params();
    }

private:
    // =========================================================================
    //  파라미터
    // =========================================================================
    void declare_params() {
        rho_        = this->declare_parameter<double>("water_density", 997.0);   // kg/m^3 담수 25도. 해수 1025
        gravity_    = this->declare_parameter<double>("gravity", 9.80665);
        // 압력[mbar] -> 수심[m] 환산 계수. rho*g/100. 담수에서 97.77 mbar/m,
        // 즉 1 mbar = 10.23 mm. 이 한 줄이 이 노드의 존재 이유다.
        mbar_per_m_ = rho_ * gravity_ / 100.0;

        // 웜업은 EKF(PRESSURE_WARMUP_SEC=180)와 같은 값으로 둔다. 같은 물리량에
        // 시스템 안에 다른 숫자가 두 개 생기는 쪽이 더 나쁘고, 어차피 수심만
        // 기다리는 것이라(속도는 영점이 필요 없다) 기다림의 실제 비용이 없다.
        warmup_sec_ = this->declare_parameter<double>("atm_warmup_sec", 180.0);

        atm_window_     = this->declare_parameter<int>("atm_window_samples", 101);
        atm_spread_max_ = this->declare_parameter<double>("atm_spread_max_mbar", 1.5);
        atm_min_mbar_   = this->declare_parameter<double>("atm_valid_min_mbar", 900.0);
        atm_max_mbar_   = this->declare_parameter<double>("atm_valid_max_mbar", 1100.0);

        // 저장 영점의 유효 기간. **길게 잡으면 안 된다** — 기압은 전선이 지나갈 때
        // 시간당 1~2 hPa 움직이고 그건 수심 1~2 cm다. 이 저장의 목적은 respawn
        // 생존(수 초)이지 세션 간 재사용이 아니다.
        zero_max_age_   = this->declare_parameter<double>("zero_max_age_sec", 600.0);

        press_timeout_  = this->declare_parameter<double>("pressure_timeout_sec", 0.5);

        const char *home = getenv("HOME");
        const std::string base = home ? (std::string(home) + "/ros2_ws/log_csv/") : "./log_csv/";
        port_map_path_ = this->declare_parameter<std::string>("port_map_path", base + "port_map.txt");
        zero_path_     = this->declare_parameter<std::string>("zero_file_path", base + "hydro_zero.txt");
    }

    void log_effective_params() {
        RCLCPP_INFO(this->get_logger(), "=========================================");
        RCLCPP_INFO(this->get_logger(), "[Hydro] 수심 추정 노드 기동 (속도는 다음 단계 — NaN으로 발행)");
        RCLCPP_INFO(this->get_logger(), "[Hydro]   포트 배정: 정체압=ch%d, 정압좌=ch%d, 정압우=ch%d  (%s)",
                    ch_front_, ch_left_, ch_right_, port_map_note_.c_str());
        RCLCPP_INFO(this->get_logger(), "[Hydro]   rho=%.1f kg/m^3, g=%.5f -> %.2f mbar/m (1 mbar = %.2f mm)",
                    rho_, gravity_, mbar_per_m_, 1000.0 / mbar_per_m_);
        RCLCPP_INFO(this->get_logger(), "[Hydro]   대기압 영점: 웜업 %.0f초, 창 %d샘플(중앙값), 산포한계 %.2f mbar, 타당범위 [%.0f, %.0f]",
                    warmup_sec_, atm_window_, atm_spread_max_, atm_min_mbar_, atm_max_mbar_);
        if (atm_captured_) {
            RCLCPP_INFO(this->get_logger(), "[Hydro]   저장 영점 적재됨: %.3f / %.3f / %.3f mbar",
                        atm_offset_[0], atm_offset_[1], atm_offset_[2]);
        }
        RCLCPP_INFO(this->get_logger(), "[Hydro] 출력: /filtered/hydro [depth, speed, q, q_raw, dP_hydro, static, pitch, flags]");
        RCLCPP_INFO(this->get_logger(), "[Hydro] 서비스: /hydro/zero_atm (대기압 영점 재포착)");
        RCLCPP_INFO(this->get_logger(), "=========================================");
    }

    // =========================================================================
    //  포트 배정 파일 — 역할(front/left/right) <-> 먹스 채널
    // =========================================================================
    // 배열 위치가 곧 채널 신원이고(메시지에 식별 정보가 없다), 어느 커넥터의
    // 튜브가 노즈로 가느냐는 소프트웨어가 알 방법이 없다. 그래서 역할을 코드에
    // 박지 않고 파일로 뺀다 — 재배선이 코드 변경을 부르지 않게.
    // 센서 지문(PROM) 대조는 i2c_driver_node 가 한다 (거기가 PROM을 읽는 곳).
    void load_port_map() {
        ch_front_ = 0; ch_left_ = 1; ch_right_ = 2;     // 파일이 없을 때의 기본값
        port_map_note_ = "기본값";

        std::ifstream f(port_map_path_);
        if (!f.is_open()) {
            RCLCPP_WARN(this->get_logger(),
                        "[Hydro] 포트 배정 파일 없음 -> 기본값 front=ch0/left=ch1/right=ch2 (%s)",
                        port_map_path_.c_str());
            return;
        }
        bool verified = false;
        std::string line;
        while (std::getline(f, line)) {
            if (line.find("verified:") != std::string::npos && line.find("yes") != std::string::npos)
                verified = true;
            if (line.empty() || line[0] == '#') continue;
            std::istringstream ss(line);
            std::string role; int ch = -1;
            if (!(ss >> role >> ch)) continue;
            if (ch < 0 || ch > 2) continue;
            if      (role == "front") ch_front_ = ch;
            else if (role == "left")  ch_left_  = ch;
            else if (role == "right") ch_right_ = ch;
        }
        f.close();
        port_map_note_ = verified ? "파일, 물리확인 완료" : "파일, **물리확인 안 됨**";
        if (!verified) {
            RCLCPP_WARN(this->get_logger(),
                        "[Hydro] 포트 배정이 물리적으로 확인되지 않았습니다. 포트를 하나씩 눌러 "
                        "배열 원소가 맞는지 본 뒤 %s 의 'verified: no' 를 yes 로 바꾸십시오.",
                        port_map_path_.c_str());
        }
    }

    // =========================================================================
    //  영점 파일 — respawn 생존용
    // =========================================================================
    // launch 의 모든 노드가 respawn=True 다. 잠수 중 이 노드가 죽으면 영점이
    // 사라진 채 되살아나고, 수심이 통째로 틀어지는데 **아무 표시가 없다**.
    bool load_zero_file() {
        std::ifstream f(zero_path_);
        if (!f.is_open()) return false;
        std::string line;
        double epoch = 0.0; float o[3] = {NAN, NAN, NAN};
        bool got = false;
        while (std::getline(f, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::istringstream ss(line);
            if (ss >> epoch >> o[0] >> o[1] >> o[2]) { got = true; break; }
        }
        f.close();
        if (!got) return false;

        const double age = this->now().seconds() - epoch;
        if (age < 0.0 || age > zero_max_age_) {
            RCLCPP_WARN(this->get_logger(),
                        "[Hydro] 저장된 영점이 %.0f초 지났습니다 (한계 %.0f초) -> 버리고 새로 잡습니다. "
                        "기압은 시간당 1~2 hPa(수심 1~2 cm) 움직입니다.", age, zero_max_age_);
            return false;
        }
        for (int i = 0; i < 3; i++) atm_offset_[i] = o[i];
        atm_captured_ = true;
        RCLCPP_INFO(this->get_logger(), "[Hydro] 저장된 영점 적재 (%.0f초 전): %.3f / %.3f / %.3f mbar",
                    age, o[0], o[1], o[2]);
        return true;
    }

    void save_zero_file() {
        // 원자적 쓰기 — 쓰다 죽어도 반쯤 쓰인 파일이 남지 않는다 (magneto_cal.py 방식)
        const std::string tmp = zero_path_ + ".tmp";
        FILE *fp = fopen(tmp.c_str(), "w");
        if (!fp) {
            RCLCPP_WARN(this->get_logger(), "[Hydro] 영점 저장 실패 (%s)", tmp.c_str());
            return;
        }
        fprintf(fp, "# hydro_estimator_node 대기압 영점 (자동 생성 — 손으로 고치지 말 것)\n");
        fprintf(fp, "# epoch_sec  ch0_mbar  ch1_mbar  ch2_mbar\n");
        fprintf(fp, "%.3f %.4f %.4f %.4f\n", this->now().seconds(),
                atm_offset_[0], atm_offset_[1], atm_offset_[2]);
        fclose(fp);
        if (rename(tmp.c_str(), zero_path_.c_str()) != 0)
            RCLCPP_WARN(this->get_logger(), "[Hydro] 영점 파일 교체 실패 (%s)", zero_path_.c_str());
    }

    // =========================================================================
    //  대기압 영점 포착
    // =========================================================================
    // EKF(state_estimation_ekf_node.cpp:612-649)의 구현을 그대로 베끼지 않는다.
    // 거기엔 게이트가 `raw_p >= 100 mbar` 하나뿐이라, **물속에서 respawn 되면
    // ~1100 mbar를 "대기압"으로 평균 내고 1 m 깊이에서 영원히 0.0 = "수면"을
    // 보고한다.** 여기서는 세 가지를 더 본다:
    //   1) 평균이 아니라 중앙값  — 글리치 한 샘플에 면역
    //   2) 산포 한계             — 벤치는 조용하고 물에 떠 있으면 시끄럽다
    //   3) [900,1100] 타당 범위  — 잠긴 상태를 직접 거부
    // 거부해도 포기하지 않고 계속 다시 시도한다. 이유를 로그로 남기므로,
    // 게이트가 빡빡해서 못 잡는 경우에도 "왜 수심이 안 나오는지"를 알 수 있다.
    struct Stat { float med, p05, p95; };

    static Stat stat_of(std::vector<float> v) {
        std::sort(v.begin(), v.end());
        const size_t n = v.size();
        auto at = [&](double q) { return v[std::min(n - 1, (size_t)(q * (n - 1) + 0.5))]; };
        return Stat{ at(0.50), at(0.05), at(0.95) };
    }

    void collect_atm(const float p[3], const bool alive[3]) {
        // 서비스로 강제한 경우가 아니면 웜업을 기다린다
        if (!force_recapture_) {
            const double el = (this->now() - node_start_time_).seconds();
            if (el < warmup_sec_) {
                if (el - last_warmup_log_ >= 30.0) {
                    last_warmup_log_ = el;
                    RCLCPP_INFO(this->get_logger(),
                                "[Hydro] 대기압 영점 웜업 대기 (%.0f / %.0f초) — 수심은 아직 NaN입니다",
                                el, warmup_sec_);
                }
                return;
            }
        }

        for (int i = 0; i < 3; i++) if (alive[i]) acc_[i].push_back(p[i]);
        collect_msgs_++;
        if (collect_msgs_ < atm_window_) return;

        // 채널별로 독립 판정한다 — 한 채널이 죽어도 나머지는 영점을 잡는다
        int ok_count = 0;
        std::string report;
        for (int i = 0; i < 3; i++) {
            char buf[160];
            if ((int)acc_[i].size() < atm_window_ / 2) {
                snprintf(buf, sizeof(buf), "  ch%d: 유효샘플 %zu개 부족 -> 영점 없음\n", i, acc_[i].size());
                report += buf;
                atm_offset_[i] = NAN;
                continue;
            }
            const Stat s = stat_of(acc_[i]);
            const float spread = s.p95 - s.p05;
            if (s.med < atm_min_mbar_ || s.med > atm_max_mbar_) {
                snprintf(buf, sizeof(buf),
                         "  ch%d: 중앙값 %.2f mbar 가 [%.0f, %.0f] 밖 -> 거부 (물속에서 잡으려는 것 아닙니까?)\n",
                         i, s.med, atm_min_mbar_, atm_max_mbar_);
                report += buf;
                atm_offset_[i] = NAN;
                continue;
            }
            if (spread > atm_spread_max_) {
                snprintf(buf, sizeof(buf),
                         "  ch%d: 산포 %.3f mbar (한계 %.2f) -> 거부 (로봇이 흔들리거나 물에 떠 있습니다)\n",
                         i, spread, atm_spread_max_);
                report += buf;
                atm_offset_[i] = NAN;
                continue;
            }
            atm_offset_[i] = s.med;
            ok_count++;
            snprintf(buf, sizeof(buf), "  ch%d: %.4f mbar  (산포 %.3f, 유효 %zu샘플)\n",
                     i, s.med, spread, acc_[i].size());
            report += buf;
        }

        reset_collection();

        if (ok_count == 0) {
            RCLCPP_ERROR(this->get_logger(), "[Hydro] 대기압 영점 포착 실패 — 다시 시도합니다\n%s", report.c_str());
            return;
        }

        atm_captured_    = true;
        force_recapture_ = false;
        RCLCPP_INFO(this->get_logger(), "=========================================");
        RCLCPP_INFO(this->get_logger(), "[Hydro] 대기압 영점 포착 완료 (%d채널)\n%s", ok_count, report.c_str());
        RCLCPP_INFO(this->get_logger(), "[Hydro] ※ EKF의 P*_cal 영점과 수백분의 1 mbar 다른 것은 정상입니다 —");
        RCLCPP_INFO(this->get_logger(), "[Hydro]   서로 다른 시각에 독립적으로 잡은 값이라 그렇습니다.");
        RCLCPP_INFO(this->get_logger(), "=========================================");
        if (this->get_parameter("zero_max_age_sec").as_double() > 0.0) save_zero_file();
    }

    void reset_collection() {
        for (int i = 0; i < 3; i++) { acc_[i].clear(); acc_[i].reserve(atm_window_); }
        collect_msgs_ = 0;
    }

    // =========================================================================
    //  압력 콜백 — 이 노드의 본체
    // =========================================================================
    void pressure_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 6) return;
        last_press_time_ = this->now();
        got_first_press_ = true;

        const float p[3] = { msg->data[0], msg->data[1], msg->data[2] };

        // 채널 유효성. 드라이버는 죽은 채널을 NaN으로 채운다 —
        // **isnan 으로 판정해야 한다. x == NAN 은 항상 false다.**
        // >= 100 은 NaN에도 자동으로 false가 되므로 두 검사가 겹쳐도 안전하다.
        bool alive[3];
        for (int i = 0; i < 3; i++) alive[i] = std::isfinite(p[i]) && p[i] >= 100.0f;

        if (!atm_captured_ || force_recapture_) collect_atm(p, alive);

        publish(p, alive);
    }

    void publish(const float p[3], const bool alive[3]) {
        const float nan_f = std::numeric_limits<float>::quiet_NaN();
        uint32_t flags = 0;

        if (!alive[ch_front_]) flags |= FLAG_FRONT_DEAD;
        if (!alive[ch_left_])  flags |= FLAG_LEFT_DEAD;
        if (!alive[ch_right_]) flags |= FLAG_RIGHT_DEAD;
        if (atm_captured_)     flags |= FLAG_ATM_ZERO;
        if (press_stopped_)    flags |= FLAG_PRESS_STOP;

        // 정압 포트의 게이지압 평균. 채널마다 자기 영점을 빼므로, 한쪽이 죽어도
        // 남은 쪽이 자기 영점으로 계속 계산된다.
        double sum = 0.0; int n = 0;
        for (int ch : {ch_left_, ch_right_}) {
            if (!alive[ch] || !std::isfinite(atm_offset_[ch])) continue;
            sum += (double)p[ch] - (double)atm_offset_[ch];
            n++;
        }

        float static_gauge = nan_f, depth = nan_f;
        if (n > 0) {
            static_gauge = (float)(sum / n);
            depth        = (float)(sum / n / mbar_per_m_);
            flags |= FLAG_DEPTH_OK;
            // 하나만 살아남으면 좌우 평균의 롤 상쇄가 깨진다 — 수심이 자세
            // 의존이 되므로 알려야 한다. 반경 0.06 m, 롤 30도에서 약 3 cm.
            if (n == 1 && !warned_single_) {
                warned_single_ = true;
                RCLCPP_WARN(this->get_logger(),
                            "[Hydro] 정압 포트가 하나뿐입니다 — 롤 상쇄가 깨져 수심이 자세에 의존합니다 "
                            "(롤 30도에서 약 3 cm).");
            } else if (n == 2) {
                warned_single_ = false;
            }
        }

        auto out = std_msgs::msg::Float32MultiArray();
        out.data.resize(H_LEN);
        out.data[H_DEPTH]    = depth;
        out.data[H_SPEED]    = nan_f;   // 다음 단계
        out.data[H_Q]        = nan_f;   // 다음 단계
        out.data[H_QRAW]     = nan_f;   // 다음 단계
        out.data[H_DPHYDRO]  = nan_f;   // 다음 단계
        out.data[H_STATIC]   = static_gauge;
        out.data[H_PITCH]    = nan_f;   // 다음 단계
        out.data[H_FLAGS]    = (float)flags;
        hydro_pub_->publish(out);
    }

    void watchdog() {
        // 첫 수신 전에는 판정하지 않는다. DDS 발견에 0.5초 넘게 걸리는 일이 흔해서,
        // 그냥 두면 기동할 때마다 "압력 스트림 정지" 오류가 헛발동한다.
        // 대신 한 번도 못 받는 경우는 따로 알린다 — 조용히 죽어 있으면 안 되므로.
        if (!got_first_press_) {
            const double since_start = (this->now() - node_start_time_).seconds();
            if (since_start > 5.0)
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                                      "[Hydro] 기동 후 %.0f초간 압력을 한 번도 받지 못했습니다 — "
                                      "i2c_driver_node 가 떠 있습니까?", since_start);
            return;
        }
        const double age = (this->now() - last_press_time_).seconds();
        const bool stopped = (age > press_timeout_);
        if (stopped && !press_stopped_)
            RCLCPP_ERROR(this->get_logger(), "[Hydro] 압력 스트림 정지 (%.1f초) — i2c_driver_node 를 확인하십시오.", age);
        else if (!stopped && press_stopped_)
            RCLCPP_WARN(this->get_logger(), "[Hydro] 압력 스트림 복구됨.");
        press_stopped_ = stopped;
    }

    // =========================================================================
    //  서비스 — 대기압 영점 재포착
    // =========================================================================
    void handle_zero_atm(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        (void)req;   // Trigger 는 요청 필드가 없다 — 미사용 경고 방어
        if ((this->now() - last_press_time_).seconds() > press_timeout_) {
            res->success = false;
            res->message = "압력 스트림이 정지 상태입니다.";
            return;
        }
        reset_collection();
        force_recapture_ = true;
        atm_captured_    = false;
        RCLCPP_WARN(this->get_logger(), "[Hydro] 대기압 영점 재포착 시작 (%d샘플, 약 %.0f초)",
                    atm_window_, atm_window_ / 11.0);
        res->success = true;
        res->message = "재포착 시작 — 결과는 저널에 남습니다. 로봇을 흔들지 마십시오.";
    }

    // ---- 파라미터 ----
    double rho_, gravity_, mbar_per_m_;
    double warmup_sec_, atm_spread_max_, atm_min_mbar_, atm_max_mbar_;
    double zero_max_age_, press_timeout_;
    int    atm_window_;
    std::string port_map_path_, zero_path_, port_map_note_;

    // ---- 포트 배정 ----
    int ch_front_ = 0, ch_left_ = 1, ch_right_ = 2;

    // ---- 영점 상태 ----
    float  atm_offset_[3];
    bool   atm_captured_    = false;
    bool   force_recapture_ = false;
    std::vector<float> acc_[3];
    int    collect_msgs_    = 0;
    double last_warmup_log_ = -1e9;

    // ---- 런타임 ----
    bool   press_stopped_    = false;
    bool   got_first_press_  = false;
    bool   warned_single_  = false;
    rclcpp::Time node_start_time_, last_press_time_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pressure_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr    hydro_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                zero_atm_srv_;
    rclcpp::TimerBase::SharedPtr                                      watchdog_timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HydroEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}
