// =============================================================================
//  hydro_estimator_node.cpp
//
//  역할 : 압력 3채널에서 **수심**과 **전진 속도**를 만드는 노드. 둘 다 구현돼 있다.
//         수심 = 좌우 정압 평균 (자세와 무관), 속도 = 피토-정압 차압 (자세 보정 필요).
//         다만 속도의 절대 기준(q_offset)과 배율(speed_k)은 **물속 실측으로만** 정해진다 —
//         그전까지 속도는 상대값이고, 플래그와 기동 경고가 그것을 알린다.
//
//   [입력] /sensor/pressure_raw    (Float32MultiArray[6]) 절대압 3채널[mbar] + 온도 3채널[°C], ~11Hz
//          /filtered/attitude_ekf (Vector3)              순수 자세[도] — 100Hz, 링버퍼로 지연 보정
//          /rc/status             (Int32MultiArray[5])   버튼 (btn1 롱프레스 = 대기압 재영점)
//          /sensor/tail_rpm       (Int32)                포트 배정 뒤바뀜 감시용
//   [출력] /filtered/hydro             (Float32MultiArray[8]) 아래 배열 규격 참조
//          /sensor/pressure_calibrated (Float32MultiArray[3]) 대기압 영점 제거된 게이지압[mbar]
//   [서비스] /hydro/zero_atm (std_srvs/Trigger)  대기압 영점 재포착 (공기 중)
//            /hydro/zero_q   (std_srvs/Trigger)  물속 정지 동압 영점
//
//  ── 왜 새 노드인가 ──────────────────────────────────────────────────────────
//  기존 파이프라인은 게이지 mbar에서 끝난다. mbar->m 변환이 실행 노드 어디에도
//  없었다 (press_char.py 의 MBAR_TO_MM 만 존재, 오프라인 전용). 그 변환을
//  state_estimation_ekf_node 에 얹지 않은 이유는 그 노드가 이미 1600줄에
//  자세 경로를 쥐고 있어서다. 수심/속도 필터를 튜닝하다 죽으면 자세까지 죽는다.
//
//  ── 압력 서브시스템 전체를 이 노드가 소유한다 (2026-08-21 이관) ────────────
//  대기압 영점과 /sensor/pressure_calibrated 는 원래 state_estimation_ekf_node 에
//  있었다. 거기 있던 이유는 없다 — 2026-08-17 Mahony -> EKF 이관 때 옛 노드의
//  세간이 통째로 딸려온 것이고, **압력은 EKF 상태벡터에 들어가지도 않는다**
//  (상태는 쿼터니언과 자이로 바이어스뿐). 같은 프로세스에 세 들어 살던 셈이다.
//
//  그대로 뒀다면 같은 일을 하는 영점이 시스템에 둘이 되고, btn1 롱프레스가
//  둘 중 하나만 다시 잡아 "btn1 길게 = 대기압 재영점" 규약이 절반만 참이 된다.
//  그래서 영점·발행·버튼을 한 덩어리로 여기로 옮겼다. 얻은 것:
//    - 영점이 하나다
//    - EKF의 물속 respawn 버그가 패치가 아니라 삭제로 사라진다 (아래 collect_atm 참조)
//    - 채널이 죽었다 살아나면 그냥 복구된다 (EKF의 press_ch_alive_ 는 영구 래치였다)
//    - EKF 노드가 자세만 하게 된다
//  /sensor/pressure_calibrated 는 토픽 이름·형식·의미를 그대로 유지하므로
//  data_logger_node(CSV 17~19열)는 손댈 필요가 없다. **발행자는 반드시 하나여야
//  한다** — EKF 쪽 발행은 같은 커밋에서 제거했다.
//
//  원시본(/sensor/pressure_raw)을 입력으로 쓰는 이유는 그 안에 온도가 함께 오기
//  때문이다. 2차 온도보상을 붙일 자리가 여기 말고는 없다.
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
//  ── 속도: 피토-정압 차압 ────────────────────────────────────────────────────
//  앞 포트는 정면을 향해 물이 그 앞에서 멈추므로 전압(정압+동압)을, 옆 포트는
//  흐름과 나란해 정압만 읽는다. 그 차이가 동압이다.
//
//    q_raw = P_front − mean(P_left, P_right)        ← 원시 절대압. **대기압 영점 불필요**
//    q     = q_raw − dP_hydro − q_offset
//    v     = k · sign(q) · sqrt(2|q| / rho)
//
//  q 가 대기압 영점을 안 쓰는 이유: 채널별 대기압 오프셋은 차압에 **상수**로만
//  들어가고 그건 전부 q_offset 에 흡수된다. 그래서 속도는 t=0부터 나온다 —
//  180초 웜업도, 재영점 중 침묵도 속도와 무관하다.
//
//  ── dP_hydro: 앞 포트가 옆 포트보다 깊이 잠긴 만큼 ─────────────────────────
//  기체가 기울면 앞 포트와 옆 포트 중점의 수심이 달라진다. 그 정수압 차이는
//  속도와 무관하므로 빼야 한다. û 를 **월드 위쪽(UP)** 을 FLU 동체 프레임으로
//  표현한 것이라 하면 (theta=피치, phi=롤, 둘 다 발행값[도]):
//
//    û = ( −sin θ ,  cos θ·sin φ ,  cos θ·cos φ )          ← yaw 는 안 들어간다
//    d_front − d_mid = −(L · û) = Lx·sin θ − Ly·cos θ·sin φ − Lz·cos θ·cos φ
//    dP_hydro[mbar]  = (rho·g/100) · (d_front − d_mid)
//
//  검산: 수평이면 û=(0,0,1). 코 아래(theta>0)면 Lx·sin theta > 0 → 앞이 더 깊다 → 압력 상승. ✓
//
//  ※ 함정 1 — 피치 부호. 발행 pitch 는 **코 아래가 양수**다(항공 규약 반대).
//    항공 규약으로 머릿속 변환부터 하고 −Lx·sin θ 라고 쓰면 기준 피치 7.9도에서
//    4.7 mbar 가 어긋난다 — 1.0 m/s 동압(4.99)과 맞먹는다.
//  ※ 함정 2 — û 인가 ĝ 인가. EKF 의 g_b(= q.conjugate()*UnitZ, :1038)는 이름과 달리
//    **UP 벡터**다(정지 시 위를 가리키는 비력과 비교하므로). 그 줄을 복사하면
//    식 전체가 뒤집힌다. 여기서는 변수명을 u_up 으로 쓰고 g 라는 이름을 쓰지 않는다.
//  ※ 함정 3 — 두 정압을 역분산 가중하지 말 것. ch2 가 시끄럽다고 가중하면 유효점이
//    기하학적 중점을 벗어나 1차 롤 감도가 되살아난다. 단순 평균을 쓴다.
//  ※ Lz 는 수평에서 상수(−K·Lz)라 q_offset 이 흡수하지만, cos theta 변화 때문에
//    ±20도에서 2차 효과가 남는다. Lz=5cm 면 0.29 mbar ≈ 피치 1도. 무시하지 말 것.
//
//  ── 압력 샘플 지연 ─────────────────────────────────────────────────────────
//  압력 한 샘플은 발행 시점의 값이 아니다. i2c_driver_node 의 3상태 기계에서
//  t0 에 변환 명령 → OSR8192 는 ~17ms 에 끝남 → t0+40ms 에 수거 → t0+80ms 에 발행.
//  적분 중심이 t0+8.5ms 이므로 **압력이 대표하는 시각은 발행보다 약 72ms 과거**다.
//  2Hz 꼬리치기·피치 진폭 3도면 그동안 자세가 1.5~3도 어긋나 0.45~0.9 mbar —
//  0.8 m/s 동압의 1/4 이다. 그래서 자세를 100Hz 링버퍼에 담아 과거 값으로 보간한다.
//  ※ 세 채널이 같은 10ms 틱에서 변환을 시작하므로 앞↔옆 사이엔 지연 차가 없다(q는 안전).
//  ※ i2c_driver_node 의 IIR(현재 우회)을 되살리면 군지연 ~860ms 가 붙어 이 값이
//    통째로 무의미해진다. 되살릴 때 pressure_lag_ms 를 반드시 다시 정할 것.
//
//  ── /filtered/hydro 배열 규격 (8) ──────────────────────────────────────────
//    [0] depth        [m]     수심. 영점 미포착·정압 전멸이면 NaN
//    [1] speed        [m/s]   부호 있음. 앞 포트 사망·자세 없음이면 NaN
//    [2] q            [mbar]  보정·영점 후, 저역통과·데드밴드 적용
//    [3] q_raw        [mbar]  **보정 전 순수 차압** (오프라인 재계산용)
//    [4] dP_hydro     [mbar]  자세 보정으로 뺀 양
//    [5] static_gauge [mbar]  살아있는 정압 포트의 게이지압 평균
//    [6] pitch_used   [도]    **실제로 보간해 쓴 피치** (5~7열 스냅샷과 다르다)
//    [7] flags        비트    아래 FLAG_* 참조
//  커스텀 메시지를 만들지 않는 프로젝트 관례를 따른다. flags를 float에 싣는 것도
//  /filtered/ekf_status data[10] 과 같은 방식이다 (float32는 2^24까지 정수 정확).
// =============================================================================

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <deque>
#include <vector>

// 배열 인덱스 — 숫자를 코드 여기저기 흩뿌리지 않는다
enum { H_DEPTH = 0, H_SPEED, H_Q, H_QRAW, H_DPHYDRO, H_STATIC, H_PITCH, H_FLAGS, H_LEN };

// 플래그. 앞쪽(0x001~0x020)은 "정상 동작 중"을 켜는 비트, 뒤쪽은 고장 비트다.
static constexpr uint32_t FLAG_DEPTH_OK   = 0x001;   // 수심 유효
static constexpr uint32_t FLAG_SPEED_OK   = 0x002;   // 속도/q 유효
// 정상값: 육상 대기 0x015 (수심+대기영점+자세), 물속 주행 0x01F (+속도+q영점)
static constexpr uint32_t FLAG_ATM_ZERO   = 0x004;   // 대기압 영점 확보
static constexpr uint32_t FLAG_Q_ZERO     = 0x008;   // 물속 q 영점 확보
static constexpr uint32_t FLAG_ATT_FRESH  = 0x010;   // 자세 신선 (지연 보간 성공)
static constexpr uint32_t FLAG_ATT_FALLBK = 0x020;   // 자세 없음 -> 기준 피치 상수로 폴백
static constexpr uint32_t FLAG_FRONT_DEAD = 0x040;   // 앞(정체압) 포트 사망
static constexpr uint32_t FLAG_LEFT_DEAD  = 0x080;   // 좌 정압 포트 사망
static constexpr uint32_t FLAG_RIGHT_DEAD = 0x100;   // 우 정압 포트 사망
static constexpr uint32_t FLAG_Q_DEADBAND = 0x200;   // q 데드밴드 내 -> 속도 0 으로 보고
static constexpr uint32_t FLAG_Q_NEGATIVE = 0x400;   // q<-3sigma 지속: 배정 뒤바뀜/기포/막힘 의심
static constexpr uint32_t FLAG_PRESS_STOP = 0x800;   // 압력 스트림 정지

class HydroEstimatorNode : public rclcpp::Node {
public:
    HydroEstimatorNode() : Node("hydro_estimator_node") {
        declare_params();
        load_port_map();

        node_start_time_ = this->now();
        last_press_time_ = this->now();
        last_att_time_   = this->now() - rclcpp::Duration::from_seconds(3600.0);   // "아직 없음"

        for (int i = 0; i < 3; i++) atm_offset_[i] = NAN;

        // 저장된 영점이 신선하면 그대로 쓴다 — respawn 되어도 수심이 안 끊긴다.
        load_zero_file();

        pressure_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/sensor/pressure_raw", 10,
            std::bind(&HydroEstimatorNode::pressure_callback, this, std::placeholders::_1));

        hydro_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/filtered/hydro", 10);

        // 구 state_estimation_ekf_node 에서 이관. 규격(3열, 게이지 mbar, 죽은 채널 NaN)
        // 을 그대로 유지하므로 data_logger_node 는 바뀐 것을 모른다.
        pressure_cal_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/sensor/pressure_calibrated", 10);

        // btn1 롱프레스(정확히 3초) = 대기압 재영점. EKF에서 이관했다 —
        // 버튼을 해석하는 곳이 늘어난 게 아니라 옮겨간 것이다.
        rc_status_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/rc/status", 10,
            std::bind(&HydroEstimatorNode::rc_status_callback, this, std::placeholders::_1));

        // 자세는 **순수값** 토픽을 쓴다 — /filtered/attitude 는 AUTO 모드에서 roll 에
        // +5000 이 실리고 yaw 에 btn2 오프셋이 적용된다. 여기서 필요한 건 물리량이다.
        attitude_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/filtered/attitude_ekf", 50,
            std::bind(&HydroEstimatorNode::attitude_callback, this, std::placeholders::_1));

        // 포트 배정 뒤바뀜 감시용. 꼬리가 도는데 q 가 계속 음수면 앞/옆이 바뀐 것이다.
        rpm_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/sensor/tail_rpm", 10,
            std::bind(&HydroEstimatorNode::rpm_callback, this, std::placeholders::_1));

        zero_q_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/hydro/zero_q",
            std::bind(&HydroEstimatorNode::handle_zero_q, this,
                      std::placeholders::_1, std::placeholders::_2));

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
        mbar_per_m_ = rho_ * gravity_ / 100.0;   // = K_. 수심 경로에서 쓰는 이름

        // [N2] 대기압 영점 캡처 전 웜업 대기(초). 구 EKF의 PRESSURE_WARMUP_SEC
        // 이 여기로 왔다 — 근거도 함께 옮긴다.
        //
        // 300초였다 -> 180초 (2026-08-20). 300초는 30BA 시절 "ch0 초반 250초
        // 급강하"를 넘기려고 정한 값인데, 02BA로 교체한 뒤 실측하니 **그 급강하
        // 자체가 없었다**:
        //   - 25분 기록에서 t=30초 시점부터 기울기가 이미 ±0.05mbar/분 이내
        //     (기울기 추정의 잡음 하한 ±0.019와 같은 급)
        //   - 25분 총 이동량 ch0 -0.08 / ch1 -0.41 / ch2 -0.11 mbar (0.8~4.2mm)
        //   - 평가 구간을 고정해 캡처 시각만 바꿔보면 180초와 300초의 수심 오차
        //     차이가 0.03~0.17mm 로, 센서 잡음(0.16mm)과 구분되지 않는다
        // ch1만 25분 내내 꾸준히 흐르는데, 이건 잦아드는 전이가 아니라서 웜업을
        // 늘려도 해결되지 않는다(재영점이나 센서 교체로 다뤄야 할 문제다).
        //
        // ※ 이 측정은 **최종 조립 전** 상태였다. 하우징이 닫히면 열·기밀 조건이
        //   달라져 급강하가 되살아날 수 있으므로, 최종 조립 후 press_char.py 로
        //   재확인할 것. 180초는 그때까지의 안전 마진을 겸한다 (실측상 0초여도
        //   ch0/ch2는 차이가 없었다).
        //
        // ※ 이 기다림이 늦추는 것은 **수심뿐**이다. 속도(피토 차압)는 영점이
        //   필요 없어 t=0부터 나온다 — 채널별 대기압 오프셋이 차압에 상수로만
        //   들어가고 그건 물속 q 영점에 흡수되기 때문이다.
        warmup_sec_ = this->declare_parameter<double>("atm_warmup_sec", 180.0);

        atm_window_     = this->declare_parameter<int>("atm_window_samples", 101);
        atm_spread_max_ = this->declare_parameter<double>("atm_spread_max_mbar", 1.5);
        atm_min_mbar_   = this->declare_parameter<double>("atm_valid_min_mbar", 900.0);
        atm_max_mbar_   = this->declare_parameter<double>("atm_valid_max_mbar", 1100.0);

        // 두 영점의 유효기간은 **다르다**. 같은 파일에 저장하지만 성질이 반대다.
        //
        // 대기압 영점은 절대압이라 날씨를 그대로 맞는다. 3.87일 벤치 실측:
        //   1시간 최악 1.50 / 8시간 최악 3.42 / 하루 최악 3.02 mbar (= 수심 3.5 cm)
        // 그래서 오래된 값을 그대로 쓰면 안 된다. 다만 **버리는 대신 잠정값으로 쓰고
        // 웜업 뒤 다시 잡아 교체한다** — 그러면 기동 직후 수심이 NaN 인 구간이 없어지고
        // 정확도도 잃지 않는다. 유효기간은 "잠정값으로 쓸 만한가"의 의미다.
        atm_zero_max_age_ = this->declare_parameter<double>("atm_zero_max_age_sec", 86400.0);
        //
        // q 영점은 **차압**이라 날씨가 세 채널에 똑같이 걸려 지워진다. 같은 3.87일
        // 기록에서 차압의 블록평균 표준편차가 10분~6시간 내내 0.053 mbar 로 **평평했다**
        // (다시 커지지 않았다 = random walk 없음). 즉 하루를 가도 0.053 mbar 이내이고
        // 그건 1.0 m/s 에서 0.53 % 다. 그리고 재포착에 물속 정지가 필요해 자동으로 못 한다.
        q_zero_max_age_   = this->declare_parameter<double>("q_zero_max_age_sec", 86400.0);

        press_timeout_  = this->declare_parameter<double>("pressure_timeout_sec", 0.5);

        // ---- 속도(피토) ----
        // 기하는 **실측에서 역산한 값을 넣는다.** 자로 잰 값이 아니라 물속 틸트
        // 시험에서 q_raw 를 sin(theta) 에 회귀해 얻은 기울기다. 실효 압력탭 위치는
        // 국소 유동 때문에 기하학적 위치와 다르다.
        //   기울기 = K·Lx,  절편 = q_offset  (K = rho·g/100 = 97.77 mbar/m)
        // 초기값 0.176 m 는 설계 치수(앞 포트 ~ 옆 포트 중점 앞뒤 거리)다.
        lever_x_    = this->declare_parameter<double>("lever_x_m", 0.176);
        lever_y_    = this->declare_parameter<double>("lever_y_m", 0.0);
        lever_z_    = this->declare_parameter<double>("lever_z_m", 0.0);
        half_span_  = this->declare_parameter<double>("port_half_span_m", 0.044);  // 좌우 8.8cm 의 절반

        // k = 1/sqrt(Cp_front − Cp_static). 옆구리 플러시 포트는 동체 주위 가속 때문에
        // 자유류 정압보다 낮게 읽으므로(Cp<0) 차압이 부풀려진다. k 가 그걸 흡수한다.
        // **예인으로 맞추면 안 된다** — 예인은 옆미끄러짐 beta=0 인데 운용은 꼬리치기로
        // beta 가 계속 붙어 정체압이 cos^2(beta) 만큼 깎인다. 자력 주행으로 맞출 것.
        speed_k_    = this->declare_parameter<double>("speed_k", 1.0);

        pressure_lag_ms_ = this->declare_parameter<double>("pressure_lag_ms", 72.0);
        att_timeout_ms_  = this->declare_parameter<double>("attitude_timeout_ms", 300.0);
        // 자세가 없을 때 쓰는 피치. NaN 대신 이걸 쓰는 이유는 1차 목적이 압력 데이터
        // 확보이고, 직진·수평이면 상수 가정의 비용이 실제 편차 ±2도에서 ±0.6 mbar 라
        // 나쁘지만 무용하진 않기 때문이다. 플래그가 신뢰도를 알려준다.
        mount_pitch_deg_ = this->declare_parameter<double>("mount_baseline_pitch_deg", 7.9);

        // **q 를 먼저 필터링하고 그 다음에 제곱근**을 취한다. 반대로 하면 부호 채터를
        // 데드밴드로 정리할 수 없고 Jensen 부등식으로 저속에서 편향이 생긴다.
        // tau 는 꼬리치기 주기의 2배 이상이어야 한다 — 좌우 평균이 못 잡는 O(beta^2)
        // 성분이 박자 주파수로 남기 때문이다. 1~3Hz 면 1.0초가 2~3주기를 덮는다.
        q_lpf_tau_   = this->declare_parameter<double>("q_lpf_tau_sec", 1.0);
        q_sigma_     = this->declare_parameter<double>("q_sigma_mbar", 0.189);   // 실측 샘플당
        q_deadband_n_= this->declare_parameter<double>("q_deadband_sigma", 3.0);
        q_offset_max_= this->declare_parameter<double>("q_offset_max_mbar", 10.0);
        q_window_    = this->declare_parameter<int>("q_zero_window_samples", 33); // ~3초 @11Hz
        q_spread_max_= this->declare_parameter<double>("q_zero_spread_max_mbar", 0.5);
        min_submerged_ = this->declare_parameter<double>("min_submerged_mbar", 3.0);

        K_ = rho_ * gravity_ / 100.0;

        const char *home = getenv("HOME");
        const std::string base = home ? (std::string(home) + "/ros2_ws/log_csv/") : "./log_csv/";
        port_map_path_ = this->declare_parameter<std::string>("port_map_path", base + "port_map.txt");
        zero_path_     = this->declare_parameter<std::string>("zero_file_path", base + "hydro_zero.txt");
    }

    void log_effective_params() {
        RCLCPP_INFO(this->get_logger(), "=========================================");
        RCLCPP_INFO(this->get_logger(), "[Hydro] 수심/속도 추정 노드 기동");
        RCLCPP_INFO(this->get_logger(), "[Hydro]   포트 배정: 정체압=ch%d, 정압좌=ch%d, 정압우=ch%d  (%s)",
                    ch_front_, ch_left_, ch_right_, port_map_note_.c_str());
        RCLCPP_INFO(this->get_logger(), "[Hydro]   rho=%.1f kg/m^3, g=%.5f -> %.2f mbar/m (1 mbar = %.2f mm)",
                    rho_, gravity_, mbar_per_m_, 1000.0 / mbar_per_m_);
        RCLCPP_INFO(this->get_logger(), "[Hydro]   대기압 영점: 웜업 %.0f초, 창 %d샘플(중앙값), 산포한계 %.2f mbar, 타당범위 [%.0f, %.0f]",
                    warmup_sec_, atm_window_, atm_spread_max_, atm_min_mbar_, atm_max_mbar_);
        if (atm_captured_) {
            RCLCPP_INFO(this->get_logger(), "[Hydro]   대기압 영점: %.3f / %.3f / %.3f mbar%s",
                        atm_offset_[0], atm_offset_[1], atm_offset_[2],
                        atm_provisional_ ? "  (**잠정** — 웜업 뒤 교체 예정)" : "");
        }
        RCLCPP_INFO(this->get_logger(), "[Hydro]   영점 유효기간: 대기압 %.0f시간(잠정 적재용), q %.0f시간",
                    atm_zero_max_age_ / 3600.0, q_zero_max_age_ / 3600.0);
        RCLCPP_INFO(this->get_logger(), "[Hydro] 출력: /filtered/hydro [depth, speed, q, q_raw, dP_hydro, static, pitch, flags]");
        RCLCPP_INFO(this->get_logger(), "[Hydro]       /sensor/pressure_calibrated (EKF에서 이관 — 발행자는 반드시 1개)");
        RCLCPP_INFO(this->get_logger(), "[Hydro] 재영점: btn1 롱프레스 3초, 또는 서비스 /hydro/zero_atm");
        RCLCPP_INFO(this->get_logger(), "[Hydro] 물속 q 영점: 서비스 /hydro/zero_q  (%s)",
                    q_zero_captured_ ? "확보됨" : "**미확보 — 속도는 상대값**");
        RCLCPP_INFO(this->get_logger(),
                    "[Hydro]   기하: lever=(%.3f, %.3f, %.3f) m, 좌우반경 %.3f m, k=%.3f, 지연 %.0f ms",
                    lever_x_, lever_y_, lever_z_, half_span_, speed_k_, pressure_lag_ms_);
        if (std::abs(speed_k_ - 1.0) < 1e-9)
            RCLCPP_WARN(this->get_logger(),
                "[Hydro] ※ speed_k=1.0 — **속도가 보정되지 않았습니다.** 자력 주행 실측으로 맞출 것 "
                "(예인으로 맞추면 안 됨: 예인은 옆미끄러짐 0이라 정체압이 온전하다).");

        // 부호 자가검증 — 함정 1(피치 부호)과 함정 2(u_up vs g)를 매 부팅마다 잡는다.
        // 코를 아래로 기울이면 앞 포트가 더 깊이 잠기므로 **양수**가 나와야 한다.
        {
            const double d = dP_hydro_of(0.0, mount_pitch_deg_);
            RCLCPP_INFO(this->get_logger(),
                "[Hydro] 부호 검증: pitch=+%.1f도(기수 아래), roll=0 -> dP_hydro=%+.3f mbar "
                "(앞 포트가 더 깊음 -> 더 높은 압력)", mount_pitch_deg_, d);
            if (d <= 0.0)
                RCLCPP_ERROR(this->get_logger(),
                    "[Hydro] ** 부호가 음수입니다 — 축/부호 오류. 이 상태의 속도는 신뢰할 수 없습니다. **");
        }
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
        double q_off = 0.0; int q_ok = 0;
        bool got = false;
        while (std::getline(f, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::istringstream ss(line);
            if (ss >> epoch >> o[0] >> o[1] >> o[2]) {
                got = true;
                ss >> q_off >> q_ok;   // 옛 형식(4칸)이면 여기서 실패해도 q_ok=0 으로 남는다
                break;
            }
        }
        f.close();
        if (!got) return false;

        const double age = this->now().seconds() - epoch;
        if (age < 0.0) {
            RCLCPP_WARN(this->get_logger(), "[Hydro] 저장된 영점의 시각이 미래입니다 (%.0f초) -> 무시합니다.", age);
            return false;
        }

        // ── 대기압: 잠정 적재. 웜업 뒤 다시 잡아 교체한다 ──
        if (age <= atm_zero_max_age_) {
            for (int i = 0; i < 3; i++) atm_offset_[i] = o[i];
            atm_captured_    = true;
            atm_provisional_ = true;     // 아직 이번 세션에서 잡은 값이 아니다
            RCLCPP_WARN(this->get_logger(),
                "[Hydro] 대기압 영점을 **잠정값으로** 적재 (%.0f시간 전): %.3f / %.3f / %.3f mbar. "
                "웜업 뒤 다시 잡아 교체합니다 — 그때까지 수심에 최대 수 cm 오차가 있을 수 있습니다.",
                age / 3600.0, o[0], o[1], o[2]);
        } else {
            RCLCPP_WARN(this->get_logger(),
                "[Hydro] 대기압 영점이 %.0f시간 지나 잠정값으로도 쓸 수 없습니다 (한계 %.0f시간).",
                age / 3600.0, atm_zero_max_age_ / 3600.0);
        }

        // ── q 영점: 확정 적재. 재포착에 물속 정지가 필요해 자동으로 못 잡는다 ──
        if (q_ok && age <= q_zero_max_age_) {
            q_offset_ = q_off; q_zero_captured_ = true;
            RCLCPP_INFO(this->get_logger(),
                "[Hydro] q 영점 적재 (%.0f시간 전): %.4f mbar — 차압은 날씨가 상쇄되어 "
                "하루를 가도 0.053 mbar 이내다(3.87일 실측).", age / 3600.0, q_off);
        } else if (q_ok) {
            RCLCPP_WARN(this->get_logger(),
                "[Hydro] q 영점이 %.0f시간 지나 버립니다 (한계 %.0f시간) -> 속도는 상대값입니다.",
                age / 3600.0, q_zero_max_age_ / 3600.0);
        }
        return atm_captured_;
    }

    void save_zero_file() {
        // 원자적 쓰기 — 쓰다 죽어도 반쯤 쓰인 파일이 남지 않는다 (magneto_cal.py 방식)
        const std::string tmp = zero_path_ + ".tmp";
        FILE *fp = fopen(tmp.c_str(), "w");
        if (!fp) {
            RCLCPP_WARN(this->get_logger(), "[Hydro] 영점 저장 실패 (%s)", tmp.c_str());
            return;
        }
        fprintf(fp, "# hydro_estimator_node 영점 (자동 생성 — 손으로 고치지 말 것)\n");
        fprintf(fp, "# epoch_sec  ch0_mbar  ch1_mbar  ch2_mbar  q_offset_mbar  q_valid\n");
        fprintf(fp, "%.3f %.4f %.4f %.4f %.4f %d\n", this->now().seconds(),
                atm_offset_[0], atm_offset_[1], atm_offset_[2],
                q_offset_, q_zero_captured_ ? 1 : 0);
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
            // 잠정값을 들고 있으면 실패해도 치명적이지 않다 — 물속 respawn 이 정확히
            // 이 경우다(게이트가 물속 포착을 막아준다). 그래서 잠정값이 있을 때는
            // 몇 번만 시도하고 조용해진다. 없으면 성공할 때까지 계속 시도한다.
            if (atm_provisional_) {
                if (++atm_retry_ <= 3) {
                    RCLCPP_WARN(this->get_logger(),
                        "[Hydro] 대기압 재포착 실패 (%d/3) — 잠정값을 유지합니다. 물속이면 정상입니다.\n%s",
                        atm_retry_, report.c_str());
                } else if (atm_retry_ == 4) {
                    RCLCPP_WARN(this->get_logger(),
                        "[Hydro] 대기압 재포착을 포기하고 잠정값으로 계속 갑니다. "
                        "공기 중으로 꺼낸 뒤 btn1 롱프레스로 다시 잡으십시오.");
                }
                if (atm_retry_ > 3) atm_provisional_ = false;   // 더 시도하지 않는다
            } else {
                RCLCPP_ERROR(this->get_logger(), "[Hydro] 대기압 영점 포착 실패 — 다시 시도합니다\n%s", report.c_str());
            }
            return;
        }

        const bool was_provisional = atm_provisional_;
        atm_captured_    = true;
        atm_provisional_ = false;
        atm_retry_       = 0;
        force_recapture_ = false;
        RCLCPP_INFO(this->get_logger(), "=========================================");
        RCLCPP_INFO(this->get_logger(), "[Hydro] 대기압 영점 포착 완료 (%d채널)%s\n%s",
                    ok_count, was_provisional ? " — 잠정값을 교체했습니다" : "", report.c_str());
        RCLCPP_INFO(this->get_logger(), "[Hydro] ※ EKF의 P*_cal 영점과 수백분의 1 mbar 다른 것은 정상입니다 —");
        RCLCPP_INFO(this->get_logger(), "[Hydro]   서로 다른 시각에 독립적으로 잡은 값이라 그렇습니다.");
        RCLCPP_INFO(this->get_logger(), "=========================================");
        save_zero_file();
    }

    void reset_collection() {
        for (int i = 0; i < 3; i++) { acc_[i].clear(); acc_[i].reserve(atm_window_); }
        collect_msgs_ = 0;
    }

    // =========================================================================
    //  자세 링버퍼 — 압력이 대표하는 "과거 시각"의 자세를 꺼내 쓴다
    // =========================================================================
    // 최신 자세를 그냥 쓰면 안 된다. 압력 샘플은 발행보다 ~72ms 과거의 값이고,
    // 꼬리치기 중에는 그 사이 자세가 수 도 움직인다. 2Hz·진폭 3도면 0.45~0.9 mbar
    // 오차인데 이는 0.8 m/s 동압의 1/4 이다.
    struct AttSample { rclcpp::Time t; double roll, pitch; };

    void attitude_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        // /filtered/attitude_ekf 는 순수값이라 ±5000 인코딩도 yaw 오프셋도 없다.
        att_buf_.push_back(AttSample{this->now(), msg->x, msg->y});
        // 0.64초치면 충분하다 (지연 72ms 의 8배). 넘치면 오래된 것부터 버린다.
        while (att_buf_.size() > 64) att_buf_.pop_front();
        last_att_time_ = this->now();
    }

    void rpm_callback(const std_msgs::msg::Int32::SharedPtr msg) { tail_rpm_ = msg->data; }

    // 두 각도 사이를 최단 경로로 보간한다. ±180 경계를 넘는 롤에서 값이 튀지 않게.
    static double lerp_angle(double a, double b, double f) {
        double d = std::fmod(b - a + 540.0, 360.0) - 180.0;
        return a + d * f;
    }

    // 목표 시각의 자세를 선형 보간해 돌려준다. 버퍼가 비었거나 낡았으면 false.
    bool attitude_at(const rclcpp::Time &target, double &roll, double &pitch) const {
        if (att_buf_.empty()) return false;
        if ((this->now() - last_att_time_).seconds() > att_timeout_ms_ * 1e-3) return false;

        // 목표가 버퍼보다 과거/미래면 끝값으로 포화시킨다 (외삽하지 않는다).
        if (target <= att_buf_.front().t) {
            roll = att_buf_.front().roll; pitch = att_buf_.front().pitch; return true;
        }
        if (target >= att_buf_.back().t) {
            roll = att_buf_.back().roll; pitch = att_buf_.back().pitch; return true;
        }
        for (size_t i = 1; i < att_buf_.size(); i++) {
            const AttSample &a = att_buf_[i - 1], &b = att_buf_[i];
            if (target < b.t) {
                const double span = (b.t - a.t).seconds();
                const double f = (span > 1e-9) ? (target - a.t).seconds() / span : 0.0;
                roll  = lerp_angle(a.roll,  b.roll,  f);
                pitch = lerp_angle(a.pitch, b.pitch, f);
                return true;
            }
        }
        roll = att_buf_.back().roll; pitch = att_buf_.back().pitch;
        return true;
    }

    // 앞 포트가 옆 포트 중점보다 깊이 잠긴 만큼의 정수압 [mbar].
    // 변수명을 u_up 으로 쓴다 — ĝ(아래)와 헷갈리면 식 전체가 뒤집힌다(함정 2).
    double dP_hydro_of(double roll_deg, double pitch_deg) const {
        const double D2R = M_PI / 180.0;
        const double th = pitch_deg * D2R, ph = roll_deg * D2R;
        const double ux = -std::sin(th);
        const double uy =  std::cos(th) * std::sin(ph);
        const double uz =  std::cos(th) * std::cos(ph);
        // d_front − d_mid = −(L · u_up)
        const double dd = -(lever_x_ * ux + lever_y_ * uy + lever_z_ * uz);
        return K_ * dd;
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

        // 잠정 적재 상태에서도 재포착을 시도한다. 성공하면 교체하고 잠정 표시를 뗀다.
        if (!atm_captured_ || atm_provisional_ || force_recapture_) collect_atm(p, alive);

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

        // ── 속도(피토) ────────────────────────────────────────────────────
        // **원시 절대압을 그대로 뺀다.** 대기압 영점을 쓰지 않는 이유는 채널별
        // 오프셋이 차압에 상수로만 들어가고 그건 q_offset 이 흡수하기 때문이다.
        // (실측 뒷받침: 3.87일 벤치에서 절대압은 날씨로 5.6~6.3 mbar 흘렀지만
        //  차압의 바닥은 0.053 mbar 였다 — 100배 이상 공통모드 상쇄.)
        float q_raw = nan_f, dph = nan_f, pitch_used = nan_f, q_out = nan_f, speed = nan_f;

        const bool have_front = alive[ch_front_];
        int ns = 0; double ssum = 0.0;
        for (int ch : {ch_left_, ch_right_}) { if (alive[ch]) { ssum += p[ch]; ns++; } }

        if (have_front && ns > 0) {
            q_raw = (float)((double)p[ch_front_] - ssum / ns);

            // 자세를 **압력이 대표하는 과거 시각**에서 꺼낸다.
            double roll_u, pitch_u;
            const rclcpp::Time t_eff = last_press_time_ - rclcpp::Duration::from_seconds(pressure_lag_ms_ * 1e-3);
            if (attitude_at(t_eff, roll_u, pitch_u)) {
                flags |= FLAG_ATT_FRESH;
            } else {
                // 자세가 없으면 장착 기준 피치로 폴백한다. 기동 직후 수 초간
                // /filtered/attitude_ekf 가 아예 없는 것은 **정상**이므로 ERROR 를 찍지 않는다.
                roll_u = 0.0; pitch_u = mount_pitch_deg_;
                flags |= FLAG_ATT_FALLBK;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                    "[Hydro] 자세 없음 -> 피치 %.1f도 상수로 폴백. 속도 신뢰도가 떨어집니다.", mount_pitch_deg_);
            }
            pitch_used = (float)pitch_u;
            dph = (float)dP_hydro_of(roll_u, pitch_u);
            last_pitch_used_ = pitch_used; last_dph_ = dph;

            if (q_zero_captured_) {
                flags |= FLAG_Q_ZERO;
                const double q_corr = (double)q_raw - (double)dph - q_offset_;

                // 필터를 먼저, 제곱근은 나중. NaN 한 개가 IIR 상태를 영구히 오염시키므로
                // 유한성 검사를 통과한 값만 넣고, 첫 유효 샘플로 재시드한다.
                if (std::isfinite(q_corr)) {
                    if (!q_lpf_seeded_) { q_lpf_ = q_corr; q_lpf_seeded_ = true; }
                    else {
                        const double dt = 1.0 / 11.0;                 // 압력 실효 주기
                        const double a  = dt / std::max(1e-6, q_lpf_tau_ + dt);
                        q_lpf_ = a * q_corr + (1.0 - a) * q_lpf_;
                    }
                }
                q_out = (float)q_lpf_;

                // 데드밴드는 q 공간에서. 필터 후 sigma = sigma/sqrt(2*tau/dt) 근사.
                const double sig_f = q_sigma_ / std::sqrt(std::max(1.0, 2.0 * q_lpf_tau_ * 11.0));
                const double band  = q_deadband_n_ * sig_f;
                if (std::abs(q_lpf_) < band) {
                    speed = 0.0f;
                    flags |= FLAG_Q_DEADBAND;
                } else {
                    const double sgn = (q_lpf_ > 0.0) ? 1.0 : -1.0;
                    speed = (float)(speed_k_ * sgn * std::sqrt(2.0 * std::abs(q_lpf_) * 100.0 / rho_));
                }
                flags |= FLAG_SPEED_OK;

                // 배정 뒤바뀜 감시 — 꼬리가 도는데 q 가 계속 큰 음수면 앞/옆이 바뀐 것이다.
                // PROM 지문(i2c_driver_node)은 **센서 교체**를 잡지만 튜브만 옮긴 경우는
                // 압력값으로만 구별된다. 그 구멍을 이 감시가 메운다.
                if (tail_rpm_ > 0 && q_lpf_ < -3.0 * q_sigma_) {
                    if (++q_neg_run_ == 55) {   // ~5초
                        flags |= FLAG_Q_NEGATIVE;
                        RCLCPP_ERROR(this->get_logger(),
                            "[Hydro] 꼬리가 도는데 q 가 %.3f mbar 로 계속 음수입니다 — "
                            "앞/옆 포트 배정이 뒤바뀌었거나, 정체압 포트에 기포가 갇혔거나, 막혔습니다.",
                            q_lpf_);
                    } else if (q_neg_run_ > 55) { flags |= FLAG_Q_NEGATIVE; }
                } else { q_neg_run_ = 0; }
            }

            // 물속 q 영점 자동 포착 (기본 꺼짐). 켤 때의 게이트는 collect_q_zero 참조.
            if (q_zero_collecting_) collect_q_zero(q_raw, dph, static_gauge);
        }

        auto out = std_msgs::msg::Float32MultiArray();
        out.data.resize(H_LEN);
        out.data[H_DEPTH]    = depth;
        out.data[H_SPEED]    = speed;
        out.data[H_Q]        = q_out;
        out.data[H_QRAW]     = q_raw;
        out.data[H_DPHYDRO]  = dph;
        out.data[H_STATIC]   = static_gauge;
        out.data[H_PITCH]    = pitch_used;
        out.data[H_FLAGS]    = (float)flags;
        hydro_pub_->publish(out);

        // 이관된 /sensor/pressure_calibrated — 영점 포착 전에는 발행하지 않는다.
        // (구 동작 그대로다. data_logger_node 는 그 구간을 초기값 0.0으로 기록하고,
        //  docs/csv_format.md 가 그렇게 설명하고 있다.)
        if (!atm_captured_) return;
        auto cal = std_msgs::msg::Float32MultiArray();
        cal.data.resize(3);
        for (int i = 0; i < 3; i++) {
            // 죽은 채널은 0.0이 아니라 NaN이다 — 이 토픽에서 0.0은 "수면"이라는
            // 지극히 정상적인 값이라, 0.0을 센티넬로 쓰면 "버스 사망"과
            // "수면에 떠 있음"이 같은 값이 된다 (2026-08-20 규약).
            cal.data[i] = (alive[i] && std::isfinite(atm_offset_[i]))
                        ? (float)((double)p[i] - (double)atm_offset_[i])
                        : nan_f;
        }
        pressure_cal_pub_->publish(cal);
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
        start_recapture("서비스");
        res->success = true;
        res->message = "재포착 시작 — 결과는 저널에 남습니다. 로봇을 흔들지 마십시오.";
    }

    // 재포착 진입점 하나 — 서비스와 버튼이 같은 경로를 쓴다.
    // 웜업을 건너뛴다: 이미 켜져 있던 센서라 예열이 끝났고, 재영점을 누른 사람은
    // 지금 다시 잡히기를 기대한다.
    void start_recapture(const char *who) {
        reset_collection();
        force_recapture_ = true;
        atm_captured_    = false;
        RCLCPP_WARN(this->get_logger(), "[Hydro] 대기압 영점 재포착 시작 (%s, %d샘플 약 %.0f초) — 로봇을 흔들지 마십시오.",
                    who, atm_window_, atm_window_ / 11.0);
    }

    // =========================================================================
    //  물속 q 영점 — 속도의 절대 기준
    // =========================================================================
    // 왜 필수인가: 장착 기준 피치 7.9도(보정 없는 알려진 갭)만으로 dP_hydro 가
    // 2.365 mbar 다. 1.0 m/s 동압(4.99)의 절반이다. 여기에 포트 간 z 오프셋,
    // 센서별 절대 바이어스, 공기->물 온도차가 더해진다. 이 영점 하나가 그걸
    // **한꺼번에** 뺀다. 남는 것은 그 자세에서 벗어난 만큼이고 dP_hydro 가 처리한다.
    //
    // 유효기간: 3.87일 벤치 실측에서 차압의 블록평균 표준편차가 10분~6시간 구간에
    // 0.053 mbar 로 **평평했다**(다시 커지지 않았다 = random walk 없음). 즉 한 번
    // 잡으면 하루를 가도 0.053 mbar 이내다 — 1.0 m/s 에서 0.53 %.
    void handle_zero_q(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        (void)req;
        if ((this->now() - last_press_time_).seconds() > press_timeout_) {
            res->success = false; res->message = "압력 스트림이 정지 상태입니다."; return;
        }
        q_acc_.clear();
        q_zero_collecting_ = true;
        RCLCPP_WARN(this->get_logger(),
            "[Hydro] 물속 q 영점 수집 시작 (%d샘플 약 %.0f초) — 로봇을 물속에 정지시켜 두십시오.",
            q_window_, q_window_ / 11.0);
        res->success = true;
        res->message = "수집 시작 — 결과는 저널에 남습니다. 꼬리를 멈추고 흐름이 없게 하십시오.";
    }

    void collect_q_zero(float q_raw, float dph, float static_gauge) {
        if (!std::isfinite(q_raw) || !std::isfinite(dph)) return;
        q_acc_.push_back((double)q_raw - (double)dph);
        if ((int)q_acc_.size() < q_window_) return;

        q_zero_collecting_ = false;
        std::vector<double> v = q_acc_;
        std::sort(v.begin(), v.end());
        const double med    = v[v.size() / 2];
        const double spread = v[(size_t)(0.95 * (v.size() - 1))] - v[(size_t)(0.05 * (v.size() - 1))];

        // 거부 조건. 실패해도 조용히 넘어가지 않고 이유를 남긴다.
        const char *why = nullptr;
        if (!std::isfinite(static_gauge) || static_gauge < min_submerged_)
            why = "잠기지 않았습니다 (게이지압이 낮습니다)";
        else if (spread > q_spread_max_)
            why = "산포가 큽니다 (흐르고 있거나 꼬리가 돌고 있습니다)";
        else if (std::abs(med) > q_offset_max_)
            why = "값이 너무 큽니다 (포트 배정이나 기하가 틀렸을 수 있습니다)";
        if (why) {
            RCLCPP_ERROR(this->get_logger(),
                "[Hydro] q 영점 거부 — %s. 중앙값 %.4f mbar, 산포 %.4f, 게이지압 %.2f mbar",
                why, med, spread, static_gauge);
            return;
        }

        q_offset_        = med;
        q_zero_captured_ = true;
        q_lpf_seeded_    = false;   // 영점이 바뀌었으니 필터를 다시 시드한다
        RCLCPP_WARN(this->get_logger(), "=========================================");
        RCLCPP_WARN(this->get_logger(),
            "[Hydro] q 영점 확보: %.4f mbar (산포 %.4f, %zu샘플)", q_offset_, spread, q_acc_.size());
        RCLCPP_WARN(this->get_logger(),
            "[Hydro]   당시 수심 %.3f m, 피치 %.2f도, dP_hydro %.4f mbar",
            static_gauge / mbar_per_m_, (double)last_pitch_used_, (double)last_dph_);
        RCLCPP_WARN(this->get_logger(), "=========================================");
        save_zero_file();
    }

    // =========================================================================
    //  버튼 — btn1 롱프레스(정확히 3초)로 대기압 재영점
    // =========================================================================
    // 판정 규칙은 EKF/구 노드와 **똑같이** 맞춘다 (1틱 = 10ms, == 300 에서 1회).
    // 다르게 두면 같은 버튼이 두 노드에서 다른 시점에 반응한다.
    void rc_status_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.empty()) return;
        const int32_t btn1 = msg->data[0];

        // 링크 두절 시 nRF는 버튼을 255로 보낸다. 1->255 전이를 누름의 연장으로
        // 오인하지 않도록 0/1 이 아니면 카운터를 중립으로 되돌린다 (N4와 같은 방어).
        if (btn1 != 0 && btn1 != 1) { btn1_counter_ = 0; return; }

        if (btn1 == 1) {
            btn1_counter_++;
            if (btn1_counter_ == 300) start_recapture("버튼1 롱프레스");   // >= 가 아니라 == : 1회만
        } else {
            btn1_counter_ = 0;
        }
    }

    // ---- 파라미터 ----
    double rho_, gravity_, mbar_per_m_, K_;
    double lever_x_, lever_y_, lever_z_, half_span_, speed_k_;
    double pressure_lag_ms_, att_timeout_ms_, mount_pitch_deg_;
    double q_lpf_tau_, q_sigma_, q_deadband_n_, q_offset_max_, q_spread_max_, min_submerged_;
    int    q_window_;
    double warmup_sec_, atm_spread_max_, atm_min_mbar_, atm_max_mbar_;
    double atm_zero_max_age_, q_zero_max_age_, press_timeout_;
    int    atm_window_;
    std::string port_map_path_, zero_path_, port_map_note_;

    // ---- 포트 배정 ----
    int ch_front_ = 0, ch_left_ = 1, ch_right_ = 2;

    // ---- 영점 상태 ----
    float  atm_offset_[3];
    bool   atm_captured_    = false;
    bool   atm_provisional_ = false;   // 파일에서 적재만 했고 이번 세션에서 안 잡은 상태
    int    atm_retry_       = 0;
    bool   force_recapture_ = false;
    std::vector<float> acc_[3];
    int    collect_msgs_    = 0;
    int    btn1_counter_    = 0;   // 1틱 = 10ms
    double last_warmup_log_ = -1e9;

    // ---- 런타임 ----
    bool   press_stopped_    = false;
    bool   got_first_press_  = false;

    // ---- 속도 상태 ----
    std::deque<AttSample> att_buf_;
    rclcpp::Time  last_att_time_;
    int32_t       tail_rpm_          = 0;
    double        q_offset_          = 0.0;
    bool          q_zero_captured_   = false;
    bool          q_zero_collecting_ = false;
    std::vector<double> q_acc_;
    double        q_lpf_             = 0.0;
    bool          q_lpf_seeded_      = false;
    int           q_neg_run_         = 0;
    float         last_pitch_used_   = 0.0f, last_dph_ = 0.0f;
    bool   warned_single_  = false;
    rclcpp::Time node_start_time_, last_press_time_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pressure_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr   rc_status_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr      attitude_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr             rpm_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr    hydro_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr    pressure_cal_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                zero_atm_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                zero_q_srv_;
    rclcpp::TimerBase::SharedPtr                                      watchdog_timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HydroEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}
