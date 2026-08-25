// =============================================================================
//  state_estimation_ekf_node.cpp
//
//  역할 : **제어 경로의 자세 추정기** — 구 state_estimation_node(Mahony)의 완전
//         대체 (2026-08-17 이관). 자이로를 예측 입력으로, 가속도·지자기를 관측으로
//         삼아 MEKF로 자세를 추정하고, 구 노드가 쥐고 있던 /filtered/attitude 발행
//         (±5000 모드 인코딩 + yaw 영점)·버튼 UI·지자기 캘리브레이션
//         트리거까지 전부 승계한다. 이관 근거는 실측이다: 교란 중 기울기 오차
//         1/2.8 (1.36도 -> 0.49도), 지자기 복구 시 자세 요동 1/54 수준,
//         6축 yaw 드리프트 +0.155도/분 (합격 기준 2.0도/분 통과).
//
//   [입력] /raw/imu_6dof         (Imu)                  가속도 3축[g] + 각속도 3축[도/초], 100Hz
//          /raw/magnetometer     (Vector3)              지자기 3축[uT], 95Hz  (use_mag=true일 때만 구독)
//          /rc/status            (Int32MultiArray[5])   버튼 입력 (1틱 = 10ms)
//   [출력] /filtered/attitude          (Vector3) 자세 + 모드 플래그 — 제어 소비 3곳이 구독
//                                       x = roll (+5000 = AUTO), y = pitch,
//                                       z = yaw − yaw_offset_ (±180도 랩)
//          /filtered/attitude_ekf      (Vector3) 순수 EKF 자세 — 오프셋·인코딩 없음
//                                       (비교 스크립트 호환 + 진단용으로 그대로 유지)
//          /filtered/ekf_status        (Float32MultiArray[12]) 진단, 10Hz
//          /CH_IMU                     (Imu)     축 변환된 IMU  (검증용, 제어에 미사용)
//          /CH_MEGNET                  (Vector3) 축 변환된 지자기 (검증용, 제어에 미사용)
//   [서비스] /calibrate_mag (std_srvs/Trigger)  magneto_cal.py 실행/조기 종료 토글
//
//  ※ 압력은 이 노드와 무관하다 (2026-08-21 이관). 대기압 영점,
//     /sensor/pressure_calibrated 발행, btn1 롱프레스 재영점을 전부
//     hydro_estimator_node 로 옮겼다. 애초에 여기 있을 이유가 없었다 —
//     2026-08-17 Mahony -> EKF 이관 때 옛 노드의 세간이 딸려온 것이고,
//     **압력은 이 필터의 상태벡터에 들어가지도 않는다**(상태는 쿼터니언과
//     자이로 바이어스뿐). 같은 프로세스에 세 들어 살던 셈이다.
//
//  ※ 9축/6축 전환 : 구 노드와 **같은 500ms 워치독**으로 자동 전환한다. 지자기가
//     살아있으면 9축, 끊기면 6축, 돌아오면 다시 9축. 캘리브레이션 중에도 강제
//     6축이다 (구 노드와 동일). 판정은 세 단계로 더 촘촘하다 (update_mag_yaw 참조):
//       1) 복각 게이트 — 지자기·중력 사잇각은 장소 상수(한국 약 143도)다.
//          이게 흔들리면 자기 교란. **크기만 보는 검사가 못 잡는 것을 잡는다**
//       2) 크기 게이트 — 런타임 실측 |m| 대비 +-20%
//       3) 나이 비례 R 팽창 — 오래된 샘플을 절벽처럼 버리지 않고 연속적으로 덜 믿는다
//     그리고 폴백 자체는 공분산이 알아서 처리한다. 지자기가 없는 동안 yaw는
//     관측 불가라 P가 자라고, 돌아오는 순간 게인이 그만큼 커져 **쌓인 드리프트에
//     비례해** 되돌린다. Mahony의 고정 Kp는 1초를 쉬든 10분을 쉬든 똑같이 당긴다.
//
//  ※※ 구 state_estimation_node 와 동시 실행 절대 금지 ※※
//     이 노드가 이제 /filtered/attitude 의 **유일한** 발행자다. 그 토픽은
//     pid_control_node / auto_scenario_node / data_logger_node 가 구독하며,
//     pid_control_node는 타이머 없이 그 콜백 안에서 PID를 돌린다. 구 노드를 함께
//     띄우면 발행자가 둘 = 제어 주기 200Hz가 되는데, PID 게인은 dt를 생략하고
//     100Hz를 가정하므로 제어 법칙이 곧바로 깨진다. 게다가 서로 다른 추정치가
//     번갈아 들어가고, /calibrate_mag 서비스도 중복 광고가 된다 (어느 서버가
//     응답할지 ROS2 명세상 정의되지 않는다). 구 노드는 저장소에 롤백 경로로만
//     남기고 launch 에서 뺐다 — 수동으로도 같이 띄우지 말 것.
//     기동 전 확인: ros2 topic info /filtered/attitude --verbose | grep -c PUBLISHER -> 1
//
//  ── 모드 전달 규약 (구 노드에서 승계, CLAUDE.md §1) ─────────────────────────
//    자동 모드일 때 attitude.x(roll)에 +5000을 더해 발행한다. 별도 모드 토픽 없이
//    자세 토픽으로 모드를 실어 나르며, 소비 3곳이 |x| > 2500 으로 해독한다.
//    인코딩을 바꾸면 해독 3곳 + 이 인코딩 1곳, 네 파일을 함께 바꿔야 한다.
//
//  ── 지자기 캘리브레이션 : min/max 폐기, magneto_cal.py 이관 ─────────────────
//    btn2 길게 / /calibrate_mag → scripts/magneto_cal.py(텀블 + 타원체 피팅)를
//    서브프로세스로 실행한다. 구 노드의 min/max는 실측에서 무보정보다 나빴다
//    (흩어짐 38.1% vs 22.1%, 2026-08-06). 진행 중 재트리거 = SIGINT 조기 종료.
//    exit 0(품질 게이트 통과·저장)이면 계수를 즉시 재로드한다 — 재시작 불필요.
//    이 노드는 mag_calib_params.txt 를 **읽기만** 한다. 쓰기는 magneto_cal.py
//    단독이며, 스크립트가 자체 백업(.bak_날짜)과 품질 게이트(흩어짐 8%)를 갖는다.
//
//  ── 핵심 알고리즘 : MEKF (Multiplicative EKF) ───────────────────────────────
//    공칭 상태 : 쿼터니언 q (body->world) + 자이로 바이어스 b [rad/s]
//    오차 상태 : dx = [dtheta(3) ; db(3)]  (6차원, 이게 P가 기술하는 대상)
//                q_true = q_hat (x) dq(dtheta)   <- LOCAL(body) 우측 곱 규약
//                b_true = b_hat + db
//
//    쿼터니언 4개를 그대로 상태로 쓰지 않는 이유: qTq=1 제약 때문에 4x4 자세
//    공분산은 q 방향으로 **구조적으로 특이**하다. Riccati 재귀가 그 널 방향을
//    강제하지 않으므로 반올림 오차가 계속 쌓여 P가 비양정이 되고, 수십 분 뒤
//    S = HPHt+R 분해가 실패한다. 조용히 진행되다 갑자기 터지는 고전적 실패다.
//    MEKF는 공분산을 제약 없는 3차원 회전벡터 위에서만 다루므로 이 문제가 없다.
//
//  ── Mahony 대비 이 노드가 노리는 것 ─────────────────────────────────────────
//    (1) 자이로 바이어스를 상태에 넣어 실시간으로 계속 추정한다.
//        (기존 노드는 부팅 시 1000샘플 평균으로 한 번 잡고 끝)
//    (2) 가속도가 못 믿을 순간(꼬리치기 등)에 R을 부풀려 자동으로 덜 믿는다.
//        (기존 노드의 Kp=2.0은 상황과 무관한 상수)
//    합격 기준: 6축 yaw 드리프트 <= 2도/분 (Mahony 실측 -8도/분 대비 4배)
//
//  ※ 좌표계 규약 : REP-103 FLU — +X 전진 / +Y 왼쪽 / +Z 위
//     축 변환과 오일러각 추출은 state_estimation_node.cpp 의 것을 **그대로**
//     복사했다. 다르면 두 토픽의 차이가 필터 차이인지 좌표계 차이인지 알 수 없다.
//     원본 근거 주석: state_estimation_node.cpp:542-567(IMU), :495-521(지자기)
//     부호 규약도 동일 — roll+ = 우현 아래, pitch+ = 기수 **아래**, yaw+ = 좌선회
// =============================================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <math.h>
#include <algorithm>
#include <fstream>
#include <string>
#include <vector>

#include <csignal>      // kill(SIGINT) — magneto_cal.py 조기 종료
#include <sys/wait.h>   // waitpid(WNOHANG) — 서브프로세스 회수
#include <unistd.h>     // waitpid 보조
#include <spawn.h>      // posix_spawnp — MT 프로세스에서 안전한 스폰
extern char **environ;  // posix_spawnp 에 물려줄 환경

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 필터 내부는 전부 double. float은 ROS 메시지 경계에서만 쓴다.
//   P의 동적 범위가 ~1e6이고 Riccati가 조건수를 제곱한다. float 24비트 가수로는
//   Q_theta(7.6e-9)를 P_theta(3e-6)에 더할 때 유효숫자가 2자리만 남아 프로세스
//   노이즈 주입이 양자화된다. Cortex-A76은 배정밀도 FPU가 full-rate이고
//   한 스텝이 5~10us(코어의 0.1%)라 성능상 float을 쓸 이유가 없다.
using S    = double;
using Vec3 = Eigen::Matrix<S, 3, 1>;
using Mat3 = Eigen::Matrix<S, 3, 3>;
using Vec6 = Eigen::Matrix<S, 6, 1>;
using Mat6 = Eigen::Matrix<S, 6, 6>;
using Quat = Eigen::Quaternion<S>;

static constexpr S D2R = M_PI / 180.0;
static constexpr S R2D = 180.0 / M_PI;

#define GYRO_CALIB_SAMPLES 1000   // 100Hz 기준 10초. 기존 노드와 동일한 창 크기
#define STILLNESS_MAX_RETRY 3     // 정지 판정 실패 시 재시도 횟수

// dt 구간 구분 (100Hz 공칭 = 10ms)
static constexpr S DT_LONG_SEC  = 0.05;   // 5샘플 유실. 여기부터는 로그를 남긴다
static constexpr S DT_BREAK_SEC = 0.50;   // 50샘플 유실. 스트림 단절로 보고 적분 포기

// 카이제곱 임계값 (NIS 게이트)
static constexpr S CHI2_3_999 = 16.27;   // 자유도 3, 99.9%
static constexpr S CHI2_1_99  =  6.63;   // 자유도 1, 99%

// ---- 작은 수학 유틸 --------------------------------------------------------

static inline Mat3 skew(const Vec3 &v) {
    Mat3 M;
    M <<     0.0, -v.z(),  v.y(),
          v.z(),     0.0, -v.x(),
         -v.y(),  v.x(),     0.0;
    return M;
}

// exp : R^3 -> SO(3). theta가 0에 가까우면 sin(t)/t 가 0/0이 되므로 테일러로 분기.
static inline Mat3 expSO3(const Vec3 &w) {
    const S th2 = w.squaredNorm();
    const S th  = std::sqrt(th2);
    const Mat3 W = skew(w);
    if (th < 1e-8) return Mat3::Identity() + W + 0.5 * W * W;
    return Mat3::Identity() + (std::sin(th) / th) * W + ((1.0 - std::cos(th)) / th2) * W * W;
}

// exp : R^3 -> 단위 쿼터니언.
//   Mahony식 q += 0.5*dt*q(x)w + 정규화는 스텝당 theta^3/12 만큼 각도가 부족하다.
//   200도/초에서 이 오차는 0.020도/초 — 추정하려는 바이어스와 같은 크기다.
//   게다가 EKF가 그 잔차를 바이어스로 흡수해 상태를 오염시킨다. sin/cos 한 번,
//   약 30ns. 근사할 이유가 없다.
static inline Quat expQuat(const Vec3 &w) {
    const S th = w.norm();
    if (th < 1e-8) {
        Quat q(1.0, 0.5 * w.x(), 0.5 * w.y(), 0.5 * w.z());
        q.normalize();
        return q;
    }
    const S s = std::sin(0.5 * th) / th;
    return Quat(std::cos(0.5 * th), s * w.x(), s * w.y(), s * w.z());
}

static inline S wrap_pi(S a) {
    a = std::fmod(a + M_PI, 2.0 * M_PI);
    if (a < 0.0) a += 2.0 * M_PI;
    return a - M_PI;
}

class StateEstimationEkfNode : public rclcpp::Node {
public:
    StateEstimationEkfNode()
        : Node("state_estimation_ekf_node"),
          q_(1.0, 0.0, 0.0, 0.0),
          b_(Vec3::Zero()),
          P_(Mat6::Zero()),
          initialized_(false),
          init_count_(0),
          init_accel_sum_(Vec3::Zero()),
          init_mag_sum_(Vec3::Zero()),
          init_mag_count_(0),
          win_count_(0),
          win_gyro_sum_(Vec3::Zero()),
          win_gyro_sumsq_(Vec3::Zero()),
          win_anorm_sum_(0.0),
          win_anorm_min_(1e9),
          win_anorm_max_(-1e9),
          win_retry_(0),
          bias_seeded_(false),
          a_ref_(1.0),
          mag_ref_(0.0),
          mag_ref_valid_(false),
          latest_m_(Vec3::Zero()),
          mag_fresh_(false),
          mag_far_count_(0),
          is_auto_mode_(false),
          btn1_press_count_(0),
          btn1_counter_(0), btn2_counter_(0),
          prev_btn1_(0), prev_btn2_(0),
          btn1_long_processed_(false), btn2_long_processed_(false),
          raw_yaw_(0.0f), yaw_offset_(0.0f),
          is_mag_calibrating_(false),
          calib_pid_(-1),
          have_prev_stamp_(false),
          t_prev_(0, 0, RCL_ROS_TIME),
          last_mag_time_(0, 0, RCL_ROS_TIME),
          tick_(0),
          flags_(0),
          innov_accel_(Vec3::Zero()),
          innov_mag_deg_(0.0),
          dt_used_(0.0),
          accel_rej_run_(0),
          mag_rej_run_(0),
          numeric_fault_(0),
          logged_first_sample_(false)
    {
        declare_params();

        // 지자기 보정 계수 로드. 캘리브레이션 성공 시(poll_mag_calibration)에도
        // 같은 함수로 재로드한다 — 이 파일을 읽는 경로는 이 한 곳뿐이다.
        load_mag_calibration_file();

        attitude_ekf_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/filtered/attitude_ekf", 10);
        status_pub_   = this->create_publisher<std_msgs::msg::Float32MultiArray>("/filtered/ekf_status", 10);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/raw/imu_6dof", 10,
            std::bind(&StateEstimationEkfNode::imu_callback, this, std::placeholders::_1));

        // use_mag=false면 구독 자체를 만들지 않는다 (불필요한 DDS 매칭 회피)
        if (use_mag_) {
            mag_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
                "/raw/magnetometer", 10,
                std::bind(&StateEstimationEkfNode::mag_callback, this, std::placeholders::_1));
        }

        // ── 제어 경로 이관분 (구 state_estimation_node 에서 승계) ──────────────
        attitude_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/filtered/attitude", 10);

        // 축 변환 검증용 발행 토픽. 필터에 실제로 들어가는 값을 그대로 내보내므로
        // /raw/* 와 나란히 echo 하면 회전이 의도대로 걸렸는지 눈으로 대조할 수 있다.
        //   /CH_IMU    <- /raw/imu_6dof     를 FLU로 회전한 값 (단위 그대로: g, 도/초)
        //   /CH_MEGNET <- /raw/magnetometer 를 보정 후 FLU로 재배치한 값
        ch_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/CH_IMU", 10);
        ch_magnet_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/CH_MEGNET", 10);

        rc_status_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/rc/status", 10,
            std::bind(&StateEstimationEkfNode::rc_status_callback, this, std::placeholders::_1));

        // 버튼 없이 터미널에서도 지자기 캘리브레이션을 걸 수 있게 서비스로도 노출
        //   ros2 service call /calibrate_mag std_srvs/srv/Trigger   (재호출 = 조기 종료)
        mag_calib_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/calibrate_mag",
            std::bind(&StateEstimationEkfNode::handle_mag_calibration, this,
                      std::placeholders::_1, std::placeholders::_2));

        // 워치독 기준 시각은 반드시 "지금"으로 잡는다. 기본 생성된 Time(=epoch 0)과
        // 차분하면 첫 메시지 전까지 타임아웃 판정이 엉망이 된다.
        last_mag_time_ = this->now();

        log_effective_params();
    }

private:
    // =========================================================================
    //  파라미터
    // =========================================================================
    // 집 스타일(#define, 파라미터 0개)에서 의도적으로 벗어난다. SERVO_MIN_US 같은
    // 값은 펌웨어와 맞춰야 하는 물리 상수라 거의 안 바뀌지만, Q와 R은 경험적으로
    // 발견하는 튜닝량이라 수십 번 바뀐다. use_accel/use_mag는 애초에 ablation
    // 스위치라 #define이면 바이너리를 4개 관리해야 한다.
    // 생성자에서 한 번만 읽고 동적 콜백은 두지 않는다 — 공분산 전파 도중 Q가
    // 바뀌면 P가 비양정이 되는 길이다. 재튜닝은 노드 재시작으로.
    void declare_params() {
        use_accel_ = this->declare_parameter<bool>("use_accel", true);
        // true = 자동. 지자기가 살아있으면 9축, mag_timeout_ms 넘게 끊기면 6축으로
        // 자동 폴백하고 돌아오면 다시 9축이 된다 (기존 노드와 동일한 판정 기준).
        // false = 강제 6축. ablation 시험용.
        use_mag_   = this->declare_parameter<bool>("use_mag", true);

        const S sg  = this->declare_parameter<double>("gyro_noise_sigma",    0.05);   // 도/초/sqrt(Hz)
        const S sbg = this->declare_parameter<double>("gyro_bias_rw_sigma",  0.002);  // 도/초/sqrt(s)
        bias_tau_   = this->declare_parameter<double>("bias_tau",         1000.0);    // 초
        const S sa  = this->declare_parameter<double>("accel_noise_sigma",   0.05);   // 무차원(정규화 벡터)
        const S sm  = this->declare_parameter<double>("mag_yaw_sigma",       8.0);    // 도

        accel_soft_   = this->declare_parameter<double>("accel_soft_knee", 0.02);     // g
        accel_reject_ = this->declare_parameter<double>("accel_reject",    0.30);     // g
        const S bmax  = this->declare_parameter<double>("bias_max",       10.0);      // 도/초
        const S dipg  = this->declare_parameter<double>("mag_dip_gate_deg", 15.0);    // 도
        // 지자기 벡터와 '위' 방향이 이루는 각. 한국(복각 53도)에서 90+53 = 143도.
        // 90도 미만이면 자기장이 위를 향한다는 뜻이라 북반구에서 물리적으로 불가능하다.
        const S dipe  = this->declare_parameter<double>("mag_dip_expected_deg", 143.0);

        mag_timeout_ms_  = this->declare_parameter<int>("mag_timeout_ms", 500);
        init_samples_    = this->declare_parameter<int>("init_samples", 50);
        status_divisor_  = this->declare_parameter<int>("status_publish_divisor", 10);

        // 파라미터는 도 단위로 선언하고(데이터시트가 그렇게 준다) 여기서 한 번만
        // rad로 바꾼다. 이 변환을 빠뜨리면 Q에 (pi/180)^2 = 3.05e-4 배가 통째로
        // 틀어지고, 필터가 과신 상태가 된 이유를 일주일 찾게 된다.
        sg2_  = (sg  * D2R) * (sg  * D2R);   // (rad/s)^2/Hz
        sbg2_ = (sbg * D2R) * (sbg * D2R);   // (rad/s)^2/s
        sa2_  = sa * sa;
        sm2_  = (sm * D2R) * (sm * D2R);     // rad^2
        bias_max_ = bmax * D2R;              // rad/s
        mag_dip_gate_ = dipg * D2R;          // rad
        dip_expected_ = dipe * D2R;          // rad

        // Gauss-Markov 이산 계수 exp(-dt/tau)는 predict()에서 **실제 dt로** 계산한다.
        // 여기서 공칭 dt로 한 번 구해 재사용하면 dt가 흔들릴 때 바이어스 감쇠만
        // 다른 시간축을 쓰게 된다. tau=1000s에서는 무시할 수준이지만, tau를 짧게
        // 잡는 순간(예: 10s) 스텝당 0.1% 어긋나므로 파라미터가 문서대로 동작하지 않는다.
    }

    void log_effective_params() {
        RCLCPP_INFO(this->get_logger(), "=========================================");
        RCLCPP_INFO(this->get_logger(), "[EKF] MEKF 자세 추정 노드 기동");
        RCLCPP_INFO(this->get_logger(), "[EKF]   관측: accel=%s, mag=%s",
                    use_accel_ ? "ON" : "OFF", use_mag_ ? "ON" : "OFF");
        RCLCPP_INFO(this->get_logger(), "[EKF]   sigma_g=%.4f deg/s/sqrt(Hz), sigma_bg=%.5f deg/s/sqrt(s), tau_b=%.0f s",
                    std::sqrt(sg2_) * R2D, std::sqrt(sbg2_) * R2D, bias_tau_);
        RCLCPP_INFO(this->get_logger(), "[EKF]   sigma_a=%.3f, sigma_mag_yaw=%.1f deg",
                    std::sqrt(sa2_), std::sqrt(sm2_) * R2D);
        RCLCPP_INFO(this->get_logger(), "[EKF]   accel gate: soft=%.3f g, reject=%.3f g",
                    accel_soft_, accel_reject_);
        RCLCPP_INFO(this->get_logger(), "[EKF]   mag gate: 복각 +-%.1f도, 크기 +-20%%, 워치독 %dms(-> 6축 폴백)",
                    mag_dip_gate_ * R2D, mag_timeout_ms_);
        RCLCPP_INFO(this->get_logger(), "[EKF]   지자기 보정: bias(%.3f %.3f %.3f) scale(%.4f %.4f %.4f)",
                    mag_bias_x_, mag_bias_y_, mag_bias_z_, mag_scale_x_, mag_scale_y_, mag_scale_z_);
        RCLCPP_INFO(this->get_logger(), "[EKF] 출력: /filtered/attitude(제어, ±5000 모드 + yaw 영점), /filtered/attitude_ekf(순수값), /filtered/ekf_status");
        RCLCPP_INFO(this->get_logger(), "[EKF] 부가: /CH_IMU, /CH_MEGNET, 서비스 /calibrate_mag (magneto_cal.py)");
        RCLCPP_INFO(this->get_logger(), "[EKF] ※ 구 state_estimation_node 와 동시 실행 금지 — /filtered/attitude 발행자 2개 = PID 200Hz");
        RCLCPP_INFO(this->get_logger(), "=========================================");
    }

    // 구 노드와 동일한 경로/순서/폴백. 이 노드에 쓰기 경로는 존재하지 않는다 —
    // mag_calib_params.txt 를 쓰는 것은 magneto_cal.py 단독이다 (백업·품질 게이트 내장).
    void load_mag_calibration_file() {
        // 폴백값도 state_estimation_node.cpp 의 초기화 리스트와 같은 값을 쓴다
        mag_bias_x_ = 8.025f;  mag_bias_y_ = 21.75f; mag_bias_z_ = 17.625f;
        mag_scale_x_ = 0.858f; mag_scale_y_ = 0.938f; mag_scale_z_ = 1.301f;

        const char *home_dir = getenv("HOME");
        std::string file_path = home_dir ? (std::string(home_dir) + "/ros2_ws/config/mag_calib_params.txt")
                                         : "./config/mag_calib_params.txt";
        std::ifstream infile(file_path);
        if (infile.is_open()) {
            infile >> mag_bias_x_ >> mag_bias_y_ >> mag_bias_z_
                   >> mag_scale_x_ >> mag_scale_y_ >> mag_scale_z_;
            infile.close();
        } else {
            RCLCPP_WARN(this->get_logger(), "[EKF] 지자기 보정 파일 없음 -> 기본값 사용 (%s)", file_path.c_str());
        }
    }

    // =========================================================================
    //  버튼 UI (/rc/status 100Hz, 1틱 = 10ms) — 구 state_estimation_node 이식
    // =========================================================================
    // 짧게 = 30ms~1초 (>3 && <100 틱), 길게 = 정확히 300틱에서 1회 발화 (== 300).
    //   btn1 짧게 : 자동/수동 모드 토글 (+5000 인코딩의 근원)
    //   btn1 길게 : 대기압 영점 재수집
    //   btn2 짧게 : 현재 방향을 Yaw 0도로 (yaw_offset_ — /filtered/attitude 에만 적용)
    //   btn2 길게 : magneto_cal.py 실행 / 진행 중이면 SIGINT 조기 종료
    void rc_status_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) return;
        const int32_t btn1 = msg->data[0];
        const int32_t btn2 = msg->data[1];

        // 통신두절 시 nRF는 버튼을 255로 보낸다. 이때 1→255 전이를 '뗌(release)'으로
        // 오인해 모드 토글·헤딩 영점이 스퍼리어스하게 트리거되는 것을 막는다(N4).
        // 양쪽 카운터와 직전 상태를 중립으로 리셋하고 엣지 판정을 건너뛴다.
        if ((btn1 != 0 && btn1 != 1) || (btn2 != 0 && btn2 != 1)) {
            btn1_counter_ = 0; btn1_long_processed_ = false; prev_btn1_ = 0;
            btn2_counter_ = 0; btn2_long_processed_ = false; prev_btn2_ = 0;
            return;
        }

        // 버튼 1
        if (btn1 == 1) {
            btn1_counter_++;
            if (btn1_counter_ == 300) {   // 정확히 300틱(3초)에서 한 번만 발동 (>=가 아니라 ==)
                // 대기압 재영점 동작은 hydro_estimator_node 로 이관됐다(2026-08-21).
                // 여기서는 **감지만** 남긴다 — 이 표시가 없으면 손을 뗄 때 짧은
                // 누름으로 오인되어 AUTO/MANUAL 이 제멋대로 토글된다.
                btn1_long_processed_ = true;
            }
        } else {
            if (prev_btn1_ == 1) {   // Falling Edge = 손을 뗀 순간
                // 30ms 이상 1초 미만이고 롱프레스로 처리되지 않았을 때만 짧은 누름
                if (btn1_counter_ > 3 && btn1_counter_ < 100 && !btn1_long_processed_) {
                    is_auto_mode_ = !is_auto_mode_;
                    btn1_press_count_++;
                    RCLCPP_INFO(this->get_logger(), "======================================================");
                    RCLCPP_INFO(this->get_logger(), "[EKF] 로봇이 %s 모드로 전환되었습니다! (카운트: %d)",
                                is_auto_mode_ ? "자동(AUTO)" : "수동(MANUAL)", btn1_press_count_);
                    RCLCPP_INFO(this->get_logger(), "======================================================");
                }
            }
            btn1_counter_ = 0; btn1_long_processed_ = false;
        }
        prev_btn1_ = btn1;

        // 버튼 2
        if (btn2 == 1) {
            btn2_counter_++;
            if (btn2_counter_ == 300) {
                btn2_long_processed_ = true;
                // 시작/조기 종료/거절을 함수가 알아서 판단하고 로그를 남긴다
                start_or_stop_mag_calibration(nullptr);
            }
        } else {
            if (prev_btn2_ == 1) {
                if (btn2_counter_ > 3 && btn2_counter_ < 100 && !btn2_long_processed_) {
                    // 헤딩 영점: 현재 바라보는 방향을 Yaw 0도로 삼는다.
                    // 자북이 아니라 "출발 방향" 기준으로 조종하고 싶을 때 사용.
                    // /filtered/attitude_ekf 는 순수값이라 이 오프셋이 붙지 않는다.
                    yaw_offset_ = raw_yaw_;
                    RCLCPP_WARN(this->get_logger(), "======================================================");
                    RCLCPP_WARN(this->get_logger(), "[EKF] 헤딩(Yaw) 영점 정렬! 현재 방향을 0도로 설정합니다. (Offset: %.2f)", yaw_offset_);
                    RCLCPP_WARN(this->get_logger(), "======================================================");
                }
            }
            btn2_counter_ = 0; btn2_long_processed_ = false;
        }
        prev_btn2_ = btn2;
    }

    // =========================================================================
    //  지자기 캘리브레이션 — magneto_cal.py 서브프로세스 (min/max 방식 폐기)
    // =========================================================================
    // 구 노드의 min/max 수집은 실측에서 무보정보다 나빴다 (흩어짐 38.1% vs 22.1%,
    // 2026-08-06). 여기서는 타원체 피팅 도구를 그대로 서브프로세스로 실행한다.
    //   · 시작: fork + execlp("python3", ..., scripts/magneto_cal.py)
    //           stdout/stderr 는 부모 것을 물려받아 저널로 흘러간다 (진행 UI 포함)
    //   · 재트리거: SIGINT — 스크립트는 그때까지의 데이터로 분석·판정을 진행한다
    //   · 종료 감지: imu_callback 의 status 감쇠 주기에서 waitpid(WNOHANG) 폴링
    //     exit 0 = 품질 게이트 통과·저장 → 계수 즉시 재로드 + 게이트 기준 재수집
    //     exit ≠ 0 = 저장 거부/실패 → 기존 계수 유지
    // 반환값은 서비스 응답용 성공 여부. why 는 nullptr 허용 (버튼 경로).
    bool start_or_stop_mag_calibration(std::string *why) {
        if (is_mag_calibrating_) {
            // 진행 중 재트리거 = 수집 조기 종료 (구 서비스의 "2회 호출 = 종료" 시맨틱 대응)
            if (calib_pid_ > 0) kill(calib_pid_, SIGINT);
            RCLCPP_WARN(this->get_logger(),
                        "[EKF] 지자기 캘리브레이션 조기 종료 요청 (SIGINT) — 수집분으로 분석을 진행합니다");
            if (why) *why = "조기 종료 요청 — 수집분으로 분석 진행";
            return true;
        }
        // 지자기 센서가 죽어 있으면 시작하지 않는다 (구 노드와 동일한 판정 기준)
        if ((this->now() - last_mag_time_).seconds() >= mag_timeout_ms_ * 1e-3) {
            RCLCPP_ERROR(this->get_logger(), "======================================================");
            RCLCPP_ERROR(this->get_logger(), "[EKF] 지자기 센서(AK8963) 응답이 없습니다! 캘리브레이션을 취소합니다.");
            RCLCPP_ERROR(this->get_logger(), "======================================================");
            if (why) *why = "지자기 무응답 — 캘리브레이션 거절";
            return false;
        }

        // 경로 조립과 로깅은 fork 전에 끝낸다. 멀티스레드 프로세스(DDS 스레드)의
        // fork+exec 대신 posix_spawnp를 쓴다. DDS 스레드가 여럿 도는 프로세스에서
        // fork 자식이 exec 전에 malloc 계열을 건드리면 (execlp의 PATH 탐색 포함)
        // 다른 스레드가 쥔 락과 데드락할 수 있다 — posix_spawnp는 표준이 이 상황을
        // 보장하는 유일한 스폰 방식이라 그 계급의 문제가 아예 없다.
        const char *home = getenv("HOME");
        const std::string script = std::string(home ? home : "") +
                                   "/ros2_ws/src/fish_robot_pkg/scripts/magneto_cal.py";

        pid_t pid = -1;
        char arg0[] = "python3";
        std::vector<char> arg1(script.begin(), script.end());
        arg1.push_back('\0');
        char argu[] = "-u";   // 무버퍼 출력 — 저널로 진행 UI가 실시간으로 흘러가게 한다
        char *argv_[] = { arg0, argu, arg1.data(), nullptr };
        const int rc = posix_spawnp(&pid, "python3", nullptr, nullptr, argv_, environ);
        if (rc != 0) {
            RCLCPP_ERROR(this->get_logger(),
                         "[EKF] posix_spawnp 실패 (%d) — 캘리브레이션을 시작하지 못했습니다", rc);
            if (why) *why = "spawn 실패";
            return false;
        }
        // ── 부모 ──
        calib_pid_ = pid;
        is_mag_calibrating_ = true;   // 이 동안 지자기 관측은 강제 6축으로 빠진다
        RCLCPP_WARN(this->get_logger(), "======================================================");
        RCLCPP_WARN(this->get_logger(),
                    "[EKF] 지자기 캘리브레이션 시작 (magneto_cal.py, pid %d)", static_cast<int>(pid));
        RCLCPP_WARN(this->get_logger(),
                    "[EKF]   6가지 자세로 로봇을 한 바퀴씩 돌려주세요. 진행 UI는 저널로 흘러갑니다");
        RCLCPP_WARN(this->get_logger(), "======================================================");
        if (why) *why = "magneto_cal.py 시작 (pid " + std::to_string(pid) + ")";
        return true;
    }

    // 버튼 없이 터미널에서도 걸 수 있는 서비스 진입점 (버튼과 같은 함수를 쓴다)
    void handle_mag_calibration(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        (void)req;   // Trigger 는 요청 필드가 없다 — 미사용 경고 방어
        std::string why;
        res->success = start_or_stop_mag_calibration(&why);
        res->message = why;
    }

    // magneto_cal.py 종료 회수. imu_callback 의 status 감쇠 주기(~10Hz)로 폴링된다.
    void poll_mag_calibration() {
        int st = 0;
        const pid_t r = waitpid(calib_pid_, &st, WNOHANG);
        if (r == 0) return;   // 아직 실행 중

        if (r == calib_pid_ && WIFEXITED(st) && WEXITSTATUS(st) == 0) {
            // 품질 게이트 통과·저장 성공 → 새 계수를 즉시 적용한다 (재시작 불필요).
            // 게이트 기준(|m| 중앙값·복각)은 옛 보정으로 잰 값이라 새 계수와 안 맞는다.
            // 무효화해서 재수집시킨다 — 안 하면 새 보정의 샘플들이 옛 기준에 걸려 거부된다.
            load_mag_calibration_file();
            mag_ref_valid_ = false;
            dip_ref_valid_ = false;
            win_mag_norms_.clear();
            RCLCPP_INFO(this->get_logger(), "=========================================");
            RCLCPP_INFO(this->get_logger(), "[EKF] 지자기 캘리브레이션 성공 — 새 보정 즉시 적용");
            RCLCPP_INFO(this->get_logger(), "[EKF]   bias(%.3f %.3f %.3f) scale(%.4f %.4f %.4f)",
                        mag_bias_x_, mag_bias_y_, mag_bias_z_, mag_scale_x_, mag_scale_y_, mag_scale_z_);
            RCLCPP_INFO(this->get_logger(), "=========================================");
        } else if (r == calib_pid_ && WIFEXITED(st)) {
            RCLCPP_WARN(this->get_logger(),
                        "[EKF] 지자기 캘리브레이션 저장 거부/실패 (exit %d) — 기존 계수를 유지합니다"
                        " (127 = python3/스크립트 실행 실패)", WEXITSTATUS(st));
        } else {
            // 시그널 종료(r == pid, !WIFEXITED) 또는 waitpid 오류(r < 0, 예: ECHILD)
            RCLCPP_WARN(this->get_logger(),
                        "[EKF] magneto_cal.py 비정상 종료 — 기존 계수를 유지합니다");
        }
        is_mag_calibrating_ = false;
        calib_pid_ = -1;
    }

    // =========================================================================
    //  지자기 콜백
    // =========================================================================
    void mag_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        const S raw_mx = msg->x, raw_my = msg->y, raw_mz = msg->z;
        if (!std::isfinite(raw_mx) || !std::isfinite(raw_my) || !std::isfinite(raw_mz)) return;

        last_mag_time_ = this->now();

        // 보정 먼저, 축 재배치 나중. 순서가 load-bearing이다 — bias/scale은
        // 칩 자체 축의 min/max로 산출되므로 재배치 후에 적용하면 다른 축의
        // 보정값을 쓰게 되고 기존 mag_calib_params.txt 가 통째로 무효가 된다.
        // (근거 상세: state_estimation_node.cpp:495-521)
        const S cal_mx = (raw_mx - mag_bias_x_) * mag_scale_x_;
        const S cal_my = (raw_my - mag_bias_y_) * mag_scale_y_;
        const S cal_mz = (raw_mz - mag_bias_z_) * mag_scale_z_;
        latest_m_ = Vec3(cal_my, cal_mx, -cal_mz);   // R = [0 1 0; 1 0 0; 0 0 -1], det=+1

        // 검증용 발행 (구 노드의 /CH_MEGNET 승계): 융합에 실제로 들어가는 값 그대로.
        // (하드/소프트아이언 보정까지 적용된 뒤의 값이므로 /raw/magnetometer 와는
        //  축 순서뿐 아니라 크기도 다르다. 축 방향 확인이 목적이면 부호를 본다)
        auto ch_mag_msg = geometry_msgs::msg::Vector3();
        ch_mag_msg.x = latest_m_.x();
        ch_mag_msg.y = latest_m_.y();
        ch_mag_msg.z = latest_m_.z();
        ch_magnet_pub_->publish(ch_mag_msg);

        // 지자기 95Hz, IMU 100Hz. 매 IMU 틱마다 융합하면 약 5%가 이중 계산되어
        // 상관된 노이즈를 두 번 세고 헤딩에 과신이 생긴다. 신선한 샘플만 쓴다.
        mag_fresh_ = true;

        // 초기화 창 동안 |m| 을 모아 런타임 게이트 기준을 만든다.
        // 45.4uT로 하드코딩하면 보정 파일이 나쁠 때(현재 |m|~57) 지자기가 영원히
        // 잠겨버린다. 런타임 기준이면 *상수* 보정 오차는 통과하고(헤딩 오프셋일 뿐)
        // *변화*(스러스터 전류, 철물 접근)만 잡힌다 — 원하는 판별이 정확히 이것이다.
        if (!mag_ref_valid_) {
            const S n = latest_m_.norm();
            if (std::isfinite(n) && n > 1e-6) win_mag_norms_.push_back(n);
            init_mag_sum_ += latest_m_;
            init_mag_count_++;
            // 재보정 성공 직후의 기준 재수집 경로: 정지 창(finalize_stationary_window)은
            // 부팅 때 한 번만 돌므로, bias_seeded_ 이후에 무효화된 |m| 기준은 여기서
            // ~5초치(500샘플) 중앙값으로 다시 세운다. 이 분기가 없으면 크기 게이트가
            // 영영 꺼진 채 win_mag_norms_ 만 무한히 자란다.
            if (bias_seeded_ && win_mag_norms_.size() >= 500) {
                const size_t mid = win_mag_norms_.size() / 2;
                std::nth_element(win_mag_norms_.begin(), win_mag_norms_.begin() + mid, win_mag_norms_.end());
                mag_ref_ = win_mag_norms_[mid];
                mag_ref_valid_ = true;
                win_mag_norms_.clear();
                RCLCPP_INFO(this->get_logger(), "[EKF] |m| 게이트 기준 재확보: %.2f uT (새 보정 기준)", mag_ref_);
            }
        }
    }

    // =========================================================================
    //  IMU 콜백 = 메인 루프 (100Hz)
    // =========================================================================
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // ---- 축 변환 (센서 RUB -> 로봇 FLU) ----
        // state_estimation_node.cpp:568-574 와 반드시 동일해야 한다.
        //     R = [ 0  0 -1 ]   det(R) = +1, R·Rt = I
        //         [-1  0  0 ]   정상 회전이므로 유사벡터인 각속도에도 같은 행렬을 쓴다
        //         [ 0  1  0 ]
        const Vec3 g_flu(-msg->angular_velocity.z, -msg->angular_velocity.x,  msg->angular_velocity.y);
        const Vec3 a_flu(-msg->linear_acceleration.z, -msg->linear_acceleration.x, msg->linear_acceleration.y);

        if (!g_flu.allFinite() || !a_flu.allFinite()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "[EKF] IMU에 비유한값 수신 -> 이번 샘플 무시");
            return;
        }

        // 검증용 발행 (구 노드의 /CH_IMU 승계): 회전만 걸린 상태(단위 변환 전)라
        // /raw/imu_6dof 와 성분끼리 1:1로 대조된다. 아래 초기화 게이트보다 앞에
        // 두어 기동 직후 초기화 창(init_samples)에서도 발행된다.
        auto ch_imu_msg = sensor_msgs::msg::Imu();
        ch_imu_msg.header.stamp = msg->header.stamp;   // 원본 타임스탬프 유지
        ch_imu_msg.header.frame_id = "base_link";      // 회전 후에는 로봇 FLU 좌표계
        ch_imu_msg.linear_acceleration.x = a_flu.x();  // [g]
        ch_imu_msg.linear_acceleration.y = a_flu.y();
        ch_imu_msg.linear_acceleration.z = a_flu.z();
        ch_imu_msg.angular_velocity.x = g_flu.x();     // [도/초]
        ch_imu_msg.angular_velocity.y = g_flu.y();
        ch_imu_msg.angular_velocity.z = g_flu.z();
        ch_imu_pub_->publish(ch_imu_msg);

        // 도/초 -> rad/s. 이 한 곳에서만 변환한다. 가속도는 g 단위 그대로 두는데,
        // 정규화해서 방향만 쓰므로 스케일이 결과에 영향을 주지 않기 때문이다.
        const Vec3 gyro_rad = g_flu * D2R;
        const Vec3 accel_g  = a_flu;

        // 축 변환을 눈으로 대조할 수 있게 첫 샘플만 찍는다. /CH_IMU 는 이제 이 노드가
        // 직접 발행하므로 echo 값과 이 로그가 일치해야 정상이다.
        if (!logged_first_sample_) {
            logged_first_sample_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "[EKF] 첫 변환 샘플  accel=(%.3f %.3f %.3f)[g]  gyro=(%.2f %.2f %.2f)[도/초]"
                        "  <- 'ros2 topic echo /CH_IMU --once' 와 대조할 것",
                        a_flu.x(), a_flu.y(), a_flu.z(), g_flu.x(), g_flu.y(), g_flu.z());
        }

        // ---- dt (고정 0.01이 아니라 실측) ----
        // EKF는 P <- FPFt + Q*dt 에서 dt 오차가 바이어스 관측성 속도를 틀리게
        // 만들고, 그게 잘못된 바이어스로, 다시 roll/pitch로 번진다.
        // ※ uart_bridge_node:219 는 this->now()로 찍으므로 이 스탬프는 nRF의
        //   샘플 시각이 아니라 Pi의 도착 시각이다. 그래도 고정 0.01보다 낫다 —
        //   실제 레이트가 99.5Hz면 고정값은 0.5% 과소적분인데, 도착 시각은
        //   장기 평균이 구조적으로 맞다. 대신 지터가 있으므로 sigma_g는 부풀려 둔다.
        rclcpp::Time t(msg->header.stamp);
        if (t.nanoseconds() == 0) t = this->now();     // 스탬프 미기입 폴백
        S dt = 0.01;
        if (have_prev_stamp_) dt = (t - t_prev_).seconds();
        t_prev_ = t;
        have_prev_stamp_ = true;

        dt_used_ = dt;   // 진단에는 클램프 전 실측값을 싣는다 (공백 자체가 정보다)

        if (!std::isfinite(dt) || dt <= 0.0 || dt > DT_BREAK_SEC) {
            // 스트림 단절 / 시계 점프. 이 정도 공백이면 자이로 한 샘플로 구간 전체를
            // 대표시킬 수 없다 — 그 사이 자세가 어떻게 됐는지 알 방법이 없으므로
            // 적분을 **시도하지 않고** 모른다는 사실만 P에 반영한다. 그러면 다음
            // 가속도/지자기 관측의 게인이 커져 평소보다 빠르게 되돌아온다.
            P_.block<3, 3>(0, 0) += (sg2_  * DT_BREAK_SEC) * Mat3::Identity();
            P_.block<3, 3>(3, 3) += (sbg2_ * DT_BREAK_SEC) * Mat3::Identity();
            symmetrize(P_);
            dt_break_count_++;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "[EKF] IMU 스트림 단절 (dt=%.3f s, 누적 %d회) -> 예측 건너뜀",
                                 dt, dt_break_count_);
            return;
        }

        // 상한 클램프는 두지 않는다. 예전에 0.05s로 잘랐더니 0.05~0.5s 구간의 공백이
        // 조용히 0.05s로 취급되어 **최대 0.45초치 회전이 로그도 없이 사라졌다**.
        // 이는 Mahony의 dt=0.01 하드코딩과 정도만 다른 같은 종류의 오류다.
        // 실측 dt로 적분하면 Q = sigma^2*dt 가 함께 커져 불확실성도 자동으로 반영된다.
        // 아래 하한(dt>0)은 위 분기가 이미 보장하고, 코드 어디에도 dt로 나누는 곳은 없다.
        if (dt > DT_LONG_SEC) {
            dt_long_count_++;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "[EKF] IMU 샘플 %.0f개분 유실 (dt=%.1f ms, 누적 %d회) -> 실측 dt로 적분",
                                 dt / 0.01, dt * 1000.0, dt_long_count_);
        }

        // ---- 정지 구간 통계 누적 (초기화와 병렬로 계속 돈다) ----
        accumulate_stationary_window(gyro_rad, accel_g);

        // ---- 초기화: 첫 init_samples 동안은 평균만 모으고 발행하지 않는다 ----
        if (!initialized_) {
            init_accel_sum_ += accel_g;
            init_count_++;
            // use_mag일 때는 지자기 샘플이 최소 하나 올 때까지 기다린다. 없이 TRIAD를
            // 돌리면 yaw가 0에서 출발해 기동할 때마다 초기 헤딩이 달라지고, 두 추정기
            // 비교의 재현성이 떨어진다. 다만 센서가 죽어 있을 수 있으므로 구간의
            // 2배까지만 기다리고 포기한다 (죽은 센서로 영원히 멈추면 안 된다).
            const bool wait_for_mag = use_mag_ && init_mag_count_ == 0
                                      && init_count_ < init_samples_ * 2;
            if (init_count_ < init_samples_ || wait_for_mag) return;
            finish_initialization();
            return;
        }

        flags_ = 0;
        flags_ |= 0x01;   // bit0 초기화 완료

        // ---- 예측 ----
        predict(gyro_rad, dt);

        // ---- 갱신 (가속도 먼저, 그다음 지자기) ----
        // 순서가 중요하다: 지자기의 H는 g_b에 의존하는데, 가속도 보정 뒤의 g_b가
        // 더 신선하다. 순차 처리라 각 관측을 독립적으로 게이팅할 수 있고
        // 재선형화도 공짜로 따라온다.
        const Vec3 w_hat = gyro_rad - b_;
        if (use_accel_) {
            if (update_accel(accel_g, w_hat)) { flags_ |= 0x02; accel_rej_run_ = 0; }
            else                              { flags_ |= 0x08; accel_rej_run_++; }
        }
        if (use_mag_) {
            // 지자기 생존 판정은 구 노드와 **같은 500ms 기준**을 쓴다. 캘리브레이션
            // 중에는 구 노드와 동일하게 강제 6축 — magneto_cal.py 가 도는 동안
            // 로봇을 6자세로 뒤집는데, 그 지자기를 낡은(어쩌면 틀린) 계수로 관측에
            // 쓰면 yaw 를 엉뚱하게 끌고 다닌다. 그 동안 P가 자라므로 복귀는 자동이다.
            const S mag_age = (this->now() - last_mag_time_).seconds();
            const bool stale = mag_age > (mag_timeout_ms_ * 1e-3);
            set_mag_active(!stale && !is_mag_calibrating_);
            if (stale) {
                flags_ |= 0x20;
                mag_rej_run_++;
            } else if (is_mag_calibrating_) {
                mag_fresh_ = false;   // 캘리브레이션 중에는 소비만 하고 융합하지 않는다
            } else if (mag_fresh_) {
                mag_fresh_ = false;
                if (update_mag_yaw(latest_m_, mag_age, w_hat)) { flags_ |= 0x04; mag_rej_run_ = 0; }
                else                                          { flags_ |= 0x10; mag_rej_run_++; }
            }
        }

        // ---- 정지 구간이 끝났으면 바이어스 유사관측 주입 (1회) ----
        if (!bias_seeded_ && win_count_ >= GYRO_CALIB_SAMPLES) finalize_stationary_window();

        // ---- 건전성 검사 ----
        if (!health_ok()) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                  "[EKF] 상태 비유한값 감지 -> 재초기화");
            hard_reset(accel_g);
            return;
        }

        publish();

        // magneto_cal.py 종료 회수 — status 감쇠 주기(~10Hz)에 맞춰 waitpid(WNOHANG).
        // (tick_ 은 publish() 안에서 매 샘플 증가한다)
        if (calib_pid_ > 0 && (tick_ % status_divisor_) == 0) poll_mag_calibration();

        // 게이트가 오래 걸려 있으면 알린다. 조용히 자이로로만 몇 분을 버티는 게
        // 가장 보기 싫은 실패다.
        if (accel_rej_run_ > 200)
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "[EKF] 가속도 게이트가 %d틱째 닫혀 있음", accel_rej_run_);
        if (use_mag_ && mag_rej_run_ > 200)
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "[EKF] 지자기 게이트가 %d틱째 닫혀 있음", mag_rej_run_);
    }

    // =========================================================================
    //  예측 단계
    // =========================================================================
    void predict(const Vec3 &gyro_rad, S dt) {
        const Vec3 w = gyro_rad - b_;   // 바이어스 보정된 각속도.
                                        // 원시값으로 적분하면 바이어스 상태가 아무 일도 안 한다.

        // 공칭 쿼터니언: LOCAL 오차 규약이므로 우측 곱, 정확한 지수사상
        q_ = q_ * expQuat(w * dt);
        q_.normalize();
        if (q_.w() < 0) q_.coeffs() = -q_.coeffs();

        // 오차 상태 전이 행렬
        //   Phi11 = exp(-[w]x dt)          (정확, 로드리게스)
        //   Phi12 = -I dt + 0.5 dt^2 [w]x  (2차. 버린 항은 100Hz에서 상대 1e-7)
        //   Phi22 = exp(-dt/tau_b)
        const Mat3 Rinc = expSO3(w * dt);
        Mat6 F = Mat6::Zero();
        F.block<3, 3>(0, 0) =  Rinc.transpose();
        F.block<3, 3>(0, 3) = -Mat3::Identity() * dt + (0.5 * dt * dt) * skew(w);
        F.block<3, 3>(3, 3) =  Mat3::Identity() * std::exp(-dt / bias_tau_);   // 실제 dt 사용

        // Q는 대각만 쓴다. 정확한 Van Loan 형태의 dt^3/3, dt^2/2 항은 dt=0.01에서
        // 각각 상대 5e-8, 2e-4 수준이라 sigma_g 튜닝 불확실성보다 훨씬 작다.
        // 지배적인 theta<->b 상관은 위의 Phi12 = -I*dt 가 **정확히** 처리한다.
        // ※ sigma_g는 밀도(rad/s/sqrt(Hz))다. 각도 분산은 sigma^2*dt 이지
        //   sigma^2*dt^2 도 sigma^2/dt 도 아니다. 가장 흔한 튜닝 버그.
        Mat6 Q = Mat6::Zero();
        Q.block<3, 3>(0, 0) = (sg2_  * dt) * Mat3::Identity();
        Q.block<3, 3>(3, 3) = (sbg2_ * dt) * Mat3::Identity();

        // 행렬곱은 Eigen이 임시객체를 만들므로 alias 안전하다.
        // .noalias()를 붙이는 "최적화"는 그 임시객체를 없애서 P를 조용히 망가뜨린다.
        P_ = F * P_ * F.transpose() + Q;
        symmetrize(P_);
    }

    // =========================================================================
    //  공용 갱신 루틴 (Joseph 형태)
    // =========================================================================
    // (I-KH)P 는 **최적** 게인에서만 유효하다. 우리 게인은 최적이 아니다 —
    // R을 매 틱 적응적으로 부풀리고, 순차 갱신을 하고, H는 1차 선형화다.
    // 준최적 K에서 (I-KH)P 는 대칭성도 PSD도 보장하지 않아 P가 비양정이 될 수 있다.
    // Joseph 형태는 합동변환된 PSD 행렬 두 개의 합이라 **어떤 K에서도** PSD다.
    // 6x6에서 추가 비용 약 650 flop (~0.2us). 사실상 공짜.
    //
    // null_dir : 이 방향으로는 **보정하지 않는다** (단위벡터, body 프레임).
    //   그 관측이 원리적으로 볼 수 없는 방향을 넘기면 된다. 가속도라면 월드 수직
    //   방향(g_b)이다 — 중력은 어느 방향을 보든 똑같으므로 yaw 정보가 전혀 없다.
    //
    //   왜 필요한가 (2026-08-06 실측으로 확인된 버그):
    //     H_a 의 널공간이 g_b 라 이론상 가속도는 yaw 를 못 건드린다. 그러나 게인은
    //     K = P Ht S^-1 이고, P 에 yaw <-> 기울기 상관이 조금이라도 쌓이면 그 통로로
    //     보정이 새어 yaw 가 돌아간다. 초기 P_yaw 가 60도로 크면 그 누설이 증폭된다.
    //     6축에서 로봇을 yaw 로 흔들었더니 한 번에 -33도, 재현 시 -37도가 샜다.
    //     같은 구간 Mahony 는 -0.75도 (외적 구조상 yaw 성분이 애초에 0이라 안전하다).
    //     결정적 증거: 6축인데 yaw 1시그마가 40 -> 17 -> 7도로 **줄었다**. yaw 를
    //     관측하는 센서가 하나도 없는데 확신이 늘어난 것 자체가 누설의 증거다.
    //
    //   보정값(dx)만 잘라내면 반쪽짜리다. 그러면 P 는 여전히 "yaw 정보를 얻었다"고
    //   갱신되어 확신이 계속 줄고, 증상만 가린 채 원인이 남는다. 그래서 **게인 K 를**
    //   자른다. 그러면 dx 와 Joseph 공분산 갱신이 같은 K 를 쓰므로 둘이 자동으로
    //   일관되고, P_yaw 는 예측 단계의 Q 만큼 정직하게 커진다.
    //   (Joseph 형태는 어떤 K 에서도 PSD 라 잘린 K 를 넣어도 안전하다.)
    //
    //   자세 블록과 바이어스 블록을 **둘 다** 자른다. 자이로 바이어스의 g_b 성분은
    //   월드 수직축 둘레의 회전을 만들므로 그것도 중력으로는 관측 불가다.
    template <int M>
    bool update(const Eigen::Matrix<S, M, 6> &H,
                const Eigen::Matrix<S, M, 1> &nu,
                const Eigen::Matrix<S, M, M> &R,
                S nis_gate,
                const Vec3 *null_dir = nullptr)
    {
        Eigen::Matrix<S, M, M> Sm = H * P_ * H.transpose() + R;
        Sm = (0.5 * (Sm + Sm.transpose())).eval();

        // LDLT가 아니라 LLT(촐레스키)를 쓴다. S = HPHt + R 은 **양정이어야만** 하고,
        // 양정이 아니면 그것 자체가 잡아내야 할 수치 고장이다. LDLT는 부정부호
        // 행렬도 성공으로 처리해 그 고장을 숨긴다. 덤으로 LLT는 피벗 치환이 없어
        // GCC의 -Warray-bounds 오탐(M=1 경로)도 따라 사라진다.
        Eigen::LLT<Eigen::Matrix<S, M, M>> llt(Sm);
        if (llt.info() != Eigen::Success) { numeric_fault_++; return false; }

        const Eigen::Matrix<S, M, 1> Sinv_nu = llt.solve(nu);
        const S nis = nu.dot(Sinv_nu);
        if (!std::isfinite(nis) || nis > nis_gate) return false;

        // K = P Ht S^-1. S와 P가 대칭이므로 (P Ht S^-1)t = S^-1 H P.
        const Eigen::Matrix<S, M, 6> Sinv_HP = llt.solve(H * P_);
        Eigen::Matrix<S, 6, M> K = Sinv_HP.transpose();

        // 관측 불가 방향 사영 — 위 주석의 누설 차단. 이 아래는 전부 잘린 K를 쓴다.
        if (null_dir != nullptr) {
            const Mat3 Pp = Mat3::Identity() - (*null_dir) * null_dir->transpose();
            K.template topRows<3>()    = (Pp * K.template topRows<3>()).eval();
            K.template bottomRows<3>() = (Pp * K.template bottomRows<3>()).eval();
        }

        const Vec6 dx = K * nu;
        if (!dx.allFinite()) { numeric_fault_++; return false; }

        const Mat6 IKH = Mat6::Identity() - K * H;
        const Mat6 T   = IKH * P_ * IKH.transpose() + K * R * K.transpose();
        P_ = T;
        symmetrize(P_);

        inject(dx);
        return true;
    }

    // =========================================================================
    //  가속도 관측
    // =========================================================================
    // h(x) = R(q)t * [0,0,1]t = g_b,  H = [ [g_b]x   0 ],  nu = a_hat - g_b
    //
    // 부호 검증: q=I 이면 g_b=(0,0,1). 실제 roll이 +eps(우현 아래)면 월드 up이
    // body +Y(왼쪽)로 기울어 실측은 (0,eps,1)이다.
    // 공식은 (0,0,1) + [0,0,1]x*(eps,0,0) = (0,0,1)+(0,eps,0) = (0,eps,1). OK
    //
    // [g_b]x 는 rank 2이고 널공간이 g_b 방향이다 — 중력은 yaw 정보를 담지 않으니 옳다.
    bool update_accel(const Vec3 &a_g, const Vec3 &w_hat) {
        const S an = a_g.norm();
        if (!std::isfinite(an) || an < 1e-3) return false;

        // 적응형 R — 이 설계에서 가장 load-bearing한 부분.
        //   2Hz로 +-30도 요잉하고 IMU가 회전중심에서 10cm 떨어져 있으면
        //   원심 가속도가 0.44g이고, 이건 **항상 바깥쪽이라 평균해도 안 없어진다**.
        //   그대로 융합하면 atan(0.44) ~ 24도의 정적 기울기 편향이 생긴다.
        //   하드 게이트만 쓰면 꼬리 주파수로 채터링해서 보정이 duty-cycle로 들어가고
        //   꼬리 운동과 맥놀이를 일으키므로, 연속 팽창 + 바깥쪽 하드컷을 쓴다.
        // ※ 1.0g가 아니라 초기화 때 실측한 a_ref_(~1.03)로 재는 것이 중요하다.
        //   3% 스케일 오차는 정규화 때문에 자세엔 영향이 없지만 게이트엔 영향을 준다.
        const S r = std::abs(an / a_ref_ - 1.0);
        if (r > accel_reject_) return false;

        const Vec3 a_hat = a_g / an;
        const Vec3 g_b   = q_.conjugate() * Vec3::UnitZ();   // R(q)t * [0,0,1]

        Eigen::Matrix<S, 3, 6> H = Eigen::Matrix<S, 3, 6>::Zero();
        H.block<3, 3>(0, 0) = skew(g_b);

        const Vec3 nu = a_hat - g_b;
        innov_accel_ = nu;

        const S rn   = r / accel_soft_;
        const S wn   = 0.02 * w_hat.norm();   // 각속도 비례 항: 원심~w^2r, 접선~alpha*r
        const S infl = 1.0 + rn * rn + (wn * wn) / sa2_;
        const Mat3 R = (sa2_ * infl) * Mat3::Identity();

        // g_b 방향(월드 수직)으로는 보정하지 않는다. 중력은 yaw 정보를 담지 않으므로
        // 그 방향 성분은 관측이 준 정보가 아니라 P의 상관을 타고 들어온 찌꺼기다.
        // 이걸 빼야 6축에서 흔들 때 yaw가 -33도씩 새던 문제가 사라진다.
        return update<3>(H, nu, R, CHI2_3_999, &g_b);
    }

    // =========================================================================
    //  지자기 관측 — yaw 전용 수평 투영 (3축 벡터 갱신이 아니다)
    // =========================================================================
    // 왜 3축이 아닌가:
    //   한국은 복각 53도라 자기장의 수직 성분(36.3uT)이 수평(27.3uT)보다 크다.
    //   3축 벡터 갱신은 세 잔차를 동등하게 신뢰하므로
    //     · 가정한 복각의 오차가 roll/pitch로 **직접 주입**되고
    //     · 하드아이언 오차(현 파일은 45.4uT 자기장에 44.7uT X 바이어스를 뺀다.
    //       가설이 아니라 실측된 문제다)가 자세 의존적 잔차를 만들어 필터가 그걸
    //       **기울기 오차로 해석**하며 가속도계와 싸운다.
    //   roll/pitch는 PID가 실제로 제어하는 값이다. 오염되면 제어 권한 상실이다.
    //   지자기가 필요한 유일한 것은 yaw이므로(roll/pitch는 중력이 이미 준다),
    //   yaw만 주도록 구조적으로 강제한다.
    //
    // 유도:
    //   m_w = R(q) m_b,  nu = -atan2(m_w.y, m_w.x)
    //   dtheta_wz = e3t R dtheta = (Rt e3)t dtheta 이고 Rt e3 가 바로 g_b 다.
    //   따라서  H = [ g_b^T   0 ]  (1x6)
    //   보정 K*nu 의 자세 성분은 P_theta * g_b 에 비례 = **월드 수직축 둘레로만**
    //   회전한다. 어떤 자세에서도 roll/pitch를 만들 수 없다. 그리고 벡터라서
    //   오일러 미분이 없고 -> pitch +-90도에서도 짐벌 특이점이 없다.
    //
    // 역방향 누출(roll/pitch 오차 -> 헤딩, 이득 tan53 = 1.33)은 존재하지만 무해하다.
    // roll/pitch는 가속도계가 독립적으로 강하게 관측하기 때문이다.
    bool update_mag_yaw(const Vec3 &m_cal, S mag_age, const Vec3 &w_hat) {
        const S mn = m_cal.norm();
        if (!std::isfinite(mn) || mn < 1e-6) return false;

        const Vec3 m_hat = m_cal / mn;
        const Vec3 g_b   = q_.conjugate() * Vec3::UnitZ();

        // ── [게이트 1] 복각(dip) 검사 ─────────────────────────────────────────
        // 지자기 벡터와 중력 벡터가 이루는 각은 그 장소의 물리 상수다(한국 약 143도).
        // 로봇이 어떻게 돌아가든 **두 벡터의 상호 각도는 변하지 않는다**.
        // 따라서 이 각이 흔들리면 자기장이 교란된 것이다 — 크기 검사보다 강력하다:
        //   · 크기는 그대로인데 방향만 휘는 교란(근처 철물, 서보 전류)을 잡는다
        //   · 순수 yaw 오차에는 반응하지 않는다 (두 벡터가 같이 돌므로).
        //     즉 우리가 고치려는 그 오차에는 면역이라 게이트로 쓰기에 안전하다
        // 예측 중력 g_b를 쓰는 이유: 측정 가속도를 쓰면 로봇이 가속할 때마다
        // 지자기 게이트가 같이 닫혀버린다. roll/pitch는 강하게 관측되므로 g_b가 낫다.
        const S dip = std::acos(std::clamp(m_hat.dot(g_b), -1.0, 1.0));
        if (!dip_ref_valid_) {
            dip_ref_ = dip;                 // 첫 성공 샘플로 부트스트랩
            dip_ref_valid_ = true;
            const S err = std::abs(dip - dip_expected_);
            if (err > 30.0 * D2R) {
                // 절대 물리 검사. 기준을 첫 샘플로 부트스트랩하므로 게이트 자체는
                // 자기 일관적이라 이 오류를 절대 못 잡는다. 그래서 여기서 한 번
                // 실제 지구 자기장과 대조한다.
                RCLCPP_ERROR(this->get_logger(), "==========================================================");
                RCLCPP_ERROR(this->get_logger(),
                             "[EKF] 복각이 물리적으로 불가능합니다: 실측 %.1f도, 기대 %.1f도",
                             dip * R2D, dip_expected_ * R2D);
                RCLCPP_ERROR(this->get_logger(),
                             "[EKF]   90도 미만 = 자기장이 '위'를 향한다는 뜻 -> 북반구에서 불가능.");
                RCLCPP_ERROR(this->get_logger(),
                             "[EKF]   원인 후보: (1) 지자기 축 재배치의 Z 부호, (2) 하드아이언 보정값.");
                RCLCPP_ERROR(this->get_logger(),
                             "[EKF]   ※ 주의: 가속도/자이로는 nRF52840 IMU, 지자기는 Pi의 AK8963 —");
                RCLCPP_ERROR(this->get_logger(),
                             "[EKF]     서로 다른 부품이므로 MPU9250 데이터시트의 다이 정렬 관계로는");
                RCLCPP_ERROR(this->get_logger(),
                             "[EKF]     둘의 정렬을 유도할 수 없다. 실측으로 정해야 한다.");
                RCLCPP_ERROR(this->get_logger(),
                             "[EKF]   이 상태에서 yaw는 신뢰할 수 없습니다. use_mag:=false 권장.");
                RCLCPP_ERROR(this->get_logger(), "==========================================================");
            } else {
                RCLCPP_INFO(this->get_logger(), "[EKF] 복각 기준 확보: %.1f도 (기대 %.1f도, 편차 %.1f도)",
                            dip * R2D, dip_expected_ * R2D, err * R2D);
            }
        } else if (std::abs(dip - dip_ref_) > mag_dip_gate_) {
            mag_dip_bad_++;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "[EKF] 복각 이상 %.1f도 (기준 %.1f도) -> 지자기 교란으로 보고 거부",
                                 dip * R2D, dip_ref_ * R2D);
            return false;
        }

        // ── [게이트 2] 자기장 크기 ────────────────────────────────────────────
        if (mag_ref_valid_ && std::abs(mn / mag_ref_ - 1.0) > 0.20) return false;

        // ── [게이트 3] 수평 성분 붕괴 (atan2 보호) ────────────────────────────
        const Vec3 m_w = q_ * m_hat;
        if (std::hypot(m_w.x(), m_w.y()) < 0.15) return false;

        Eigen::Matrix<S, 1, 1> nu;
        nu(0) = wrap_pi(-std::atan2(m_w.y(), m_w.x()));
        innov_mag_deg_ = nu(0) * R2D;

        // 헤딩 재획득: 잔차가 크면 게인이 기어가서 몇 분씩 걸린다. 지속되면 스냅.
        if (std::abs(nu(0)) > 60.0 * D2R) {
            if (++mag_far_count_ > 200) { heading_snap(nu(0)); mag_far_count_ = 0; }
        } else {
            mag_far_count_ = 0;
        }

        Eigen::Matrix<S, 1, 6> H = Eigen::Matrix<S, 1, 6>::Zero();
        H.block<1, 3>(0, 0) = g_b.transpose();

        // 오래된 샘플은 절벽처럼 버리지 않고 **연속적으로 덜 믿는다**.
        // 나이 age인 헤딩 측정은 대략 |w|*age 만큼 틀어져 있으므로 그만큼 R에 더한다.
        // 지자기가 띄엄띄엄 오는(I2C 먹통) 구간에서 이진 on/off로 채터링하는 대신
        // 부드럽게 열화된다.
        const S slip = w_hat.norm() * mag_age;
        Eigen::Matrix<S, 1, 1> R;
        R(0, 0) = sm2_ + slip * slip;

        // 여기는 null_dir 을 넘기지 않는다 — 넘기면 안 된다. 가속도와 정반대로,
        // 이 관측이 주는 정보는 **월드 수직축 둘레 회전(yaw) 그 자체**다.
        // g_b 를 사영해 버리면 지자기 갱신이 통째로 0이 된다.
        return update<1>(H, nu, R, CHI2_1_99);
    }

    // 9축 <-> 6축 전환을 로그로 남긴다. 기존 노드는 조용히 전환하므로 사후에
    // "그때 6축이었나?"를 알 방법이 없다. 백에는 ekf_status bit5로도 남는다.
    void set_mag_active(bool active) {
        if (active == mag_active_) return;
        mag_active_ = active;
        if (active) {
            // 공분산이 폴백을 자동으로 처리한다 — 이게 Mahony 대비 구조적 이득이다.
            // 지자기가 없는 동안 yaw는 관측 불가라 P가 계속 자라고, 돌아오는 순간
            // 칼만 게인이 그만큼 커져서 **쌓인 드리프트에 비례해** 되돌린다.
            // Mahony의 고정 Kp는 1초를 쉬었든 10분을 쉬었든 같은 속도로 당긴다.
            RCLCPP_INFO(this->get_logger(),
                        "[EKF] 지자기 복귀 -> 9축 (yaw 1sig=%.1f도, 그만큼 빠르게 재획득)",
                        std::sqrt(std::max(0.0, P_(2, 2))) * R2D);
        } else if (is_mag_calibrating_) {
            RCLCPP_WARN(this->get_logger(), "[EKF] 지자기 캘리브레이션 중 -> 강제 6축 (완료 시 자동 복귀)");
        } else {
            RCLCPP_WARN(this->get_logger(), "[EKF] 지자기 %dms 무응답 -> 6축 폴백 (yaw 절대 기준 상실)",
                        mag_timeout_ms_);
        }
    }

    // 의도적인 월드 프레임 회전이므로 **좌측** 곱이다.
    // (보정은 body 프레임이라 우측 곱, 스냅은 월드 프레임이라 좌측 곱)
    void heading_snap(S nu) {
        q_ = Quat(Eigen::AngleAxis<S>(nu, Vec3::UnitZ())) * q_;
        q_.normalize();
        const Vec3 g_b = q_.conjugate() * Vec3::UnitZ();
        const S s = 30.0 * D2R;
        // 월드 수직축 방향으로만 불확실성을 다시 연다 (rank-1). roll/pitch 신뢰도는 유지.
        P_.block<3, 3>(0, 0) += (s * s) * (g_b * g_b.transpose());
        symmetrize(P_);
        RCLCPP_WARN(this->get_logger(), "[EKF] 헤딩 스냅 %.1f도 (지자기 잔차 지속)", nu * R2D);
    }

    // =========================================================================
    //  오차 주입과 공분산 리셋
    // =========================================================================
    void inject(const Vec6 &dx) {
        const Vec3 dth = dx.head<3>();

        q_ = q_ * expQuat(dth);   // 우측 곱 (LOCAL 규약)
        q_.normalize();
        if (q_.w() < 0) q_.coeffs() = -q_.coeffs();

        b_ += dx.tail<3>();
        b_ = b_.cwiseMax(-bias_max_).cwiseMin(bias_max_);

        // 리셋 야코비안. 주입 후 새 오차는 dq(dth+) = dq(dth_hat)^-1 (x) dq(dth) 이고,
        // BCH로 log(exp(-a)exp(b)) ~ b - a - 0.5(a x b) 이므로 G_tt = I - 0.5[dth]x.
        // 정상 상태에서 |dth| ~ 1e-3 이라 영향은 0.1%지만, 초기화 직후와 헤딩 스냅
        // 직후에는 |dth|가 0.1~1 rad이라 5~50% 효과다. 세 줄이니 넣는다.
        Mat6 G = Mat6::Identity();
        G.block<3, 3>(0, 0) -= 0.5 * skew(dth);
        const Mat6 T = G * P_ * G.transpose();
        P_ = T;
        symmetrize(P_);
    }

    // P = 0.5(P + Pt) 는 aliasing 위험이라 명시적 임시객체를 거쳐야 한다.
    // (Eigen은 A = A.transpose() 는 잡아주지만 복합식은 보장하지 않는다)
    // 대각 하한을 두어 반올림으로 음의 분산이 생겨도 다음 LDLT를 오염시키지 않게 한다.
    static void symmetrize(Mat6 &P) {
        const Mat6 T = 0.5 * (P + P.transpose());
        P = T;
        for (int i = 0; i < 6; ++i) if (P(i, i) < 1e-14) P(i, i) = 1e-14;
    }

    // =========================================================================
    //  초기화
    // =========================================================================
    // 구 노드는 10초간 아무것도 발행하지 않아 모터 출력이 끊겼다(블로킹 자이로
    // 캘리브레이션). 여기서는 그걸 재현하지 않는다:
    //   1) init_samples(0.5초) 평균으로 TRIAD -> 해석적 자세 초기화 후 즉시 발행 시작
    //   2) b_ = 0, 넉넉한 초기 공분산 -> 바이어스가 0에서 수렴하는 곡선 자체가
    //      필터가 동작한다는 1차 증거가 된다
    //   3) 1000샘플 정지 창은 **병렬로** 계속 돌고, 끝나면 유사관측으로 합류한다
    //      (finalize_stationary_window 참조)
    // 제어 경로가 된 지금도 이 트레이드는 유지한다 — 발행이 ~0.5초에 시작되는 것은
    // 의도된 개선이다. 대신 바이어스가 수렴하기 전 처음 ~10초는 yaw 가 미세하게
    // 덜 정확할 수 있다 (roll/pitch 는 가속도가 곧바로 잡아준다).
    //
    // identity로 시작하지 않는 이유: 실제 자세가 30도면 초기 dtheta=0.52 rad로
    // 1차 선형화 영역 밖이고 수렴에 수 초가 걸린다. 가속도가 이미 있는데
    // identity로 출발할 이유가 없다.
    void finish_initialization() {
        const Vec3 a_avg = init_accel_sum_ / static_cast<S>(init_count_);
        Vec3 m_avg = Vec3::Zero();
        const bool have_mag = use_mag_ && init_mag_count_ > 0;
        if (have_mag) m_avg = init_mag_sum_ / static_cast<S>(init_mag_count_);

        q_ = triad(a_avg, m_avg, have_mag);
        b_.setZero();

        // 자세 불확실성은 월드 프레임으로 기술하는 게 자연스럽고, 오차 상태는
        // body 프레임이다. 한 줄로 변환한다.
        // ※ "yaw 모름"에 pi^2을 넣지 말 것. 선형화 필터에서 거대한 분산은 의미가
        //   없고(소각 가정이 이미 60도에서 깨진다) 게인이 1에 가까워져 사실상
        //   스냅이 된다. 스냅이 필요하면 heading_snap()으로 명시적으로 한다.
        Mat3 P0w = Mat3::Zero();
        P0w(0, 0) = P0w(1, 1) = std::pow(5.0 * D2R, 2);
        P0w(2, 2) = have_mag ? std::pow(20.0 * D2R, 2) : std::pow(60.0 * D2R, 2);
        const Mat3 Rwb = q_.toRotationMatrix();
        P_.setZero();
        P_.block<3, 3>(0, 0) = Rwb.transpose() * P0w * Rwb;
        P_.block<3, 3>(3, 3) = Mat3::Identity() * std::pow(5.0 * D2R, 2);   // 바이어스 1σ 5도/초

        a_ref_ = a_avg.norm();
        if (!std::isfinite(a_ref_) || a_ref_ < 0.5 || a_ref_ > 2.0) a_ref_ = 1.0;

        initialized_ = true;

        S roll, pitch, yaw;
        get_euler_angles(&roll, &pitch, &yaw);
        RCLCPP_INFO(this->get_logger(),
                    "[EKF] 초기화 완료 (%d샘플): RPY=(%.2f, %.2f, %.2f)도, |a|기준=%.4f g, 지자기=%s",
                    init_count_, roll, pitch, yaw, a_ref_, have_mag ? "사용" : "미사용");
    }

    // TRIAD: 중력으로 월드 +Z를, 자기장 수평 성분으로 월드 +X를 잡는다.
    static Quat triad(const Vec3 &a_b, const Vec3 &m_b, bool have_mag) {
        if (a_b.norm() < 1e-6) return Quat::Identity();
        const Vec3 z = a_b.normalized();   // 월드 +Z(위)를 body 좌표로 본 것

        if (!have_mag || m_b.norm() < 1e-6)
            return Quat::FromTwoVectors(z, Vec3::UnitZ());   // yaw=0, 특이점 안전

        Vec3 x = m_b - m_b.dot(z) * z;     // 자북의 수평 성분 (body 좌표)
        if (x.norm() < 0.1 * m_b.norm())   // 자기장이 거의 수직 -> 축퇴
            return Quat::FromTwoVectors(z, Vec3::UnitZ());
        x.normalize();
        const Vec3 y = z.cross(x);         // 오른손 좌표계

        Mat3 Rbw;                          // 열 = 월드 축을 body로 본 것 => world->body
        Rbw.col(0) = x; Rbw.col(1) = y; Rbw.col(2) = z;
        Quat q(Rbw.transpose());           // body->world
        q.normalize();
        return q;
    }

    void hard_reset(const Vec3 &a_g) {
        q_ = triad(a_g, Vec3::Zero(), false);
        if (!b_.allFinite()) b_.setZero();
        P_.setZero();
        P_.block<3, 3>(0, 0) = Mat3::Identity() * std::pow(10.0 * D2R, 2);
        P_.block<3, 3>(3, 3) = Mat3::Identity() * std::pow(5.0  * D2R, 2);
        mag_far_count_ = 0;
    }

    bool health_ok() const {
        if (!q_.coeffs().allFinite() || !b_.allFinite() || !P_.allFinite()) return false;
        const S n = q_.norm();
        return std::isfinite(n) && n > 1e-6;
    }

    // =========================================================================
    //  정지 구간 통계 -> 바이어스 유사관측
    // =========================================================================
    void accumulate_stationary_window(const Vec3 &gyro_rad, const Vec3 &accel_g) {
        if (bias_seeded_ || win_count_ >= GYRO_CALIB_SAMPLES) return;
        win_gyro_sum_   += gyro_rad;
        win_gyro_sumsq_ += gyro_rad.cwiseProduct(gyro_rad);
        const S an = accel_g.norm();
        win_anorm_sum_ += an;
        win_anorm_min_ = std::min(win_anorm_min_, an);
        win_anorm_max_ = std::max(win_anorm_max_, an);
        win_count_++;
    }

    void reset_stationary_window() {
        win_count_ = 0;
        win_gyro_sum_.setZero();
        win_gyro_sumsq_.setZero();
        win_anorm_sum_ = 0.0;
        win_anorm_min_ =  1e9;
        win_anorm_max_ = -1e9;
        win_mag_norms_.clear();
    }

    // 정지 판정을 하는 이유: 기존 노드는 검사 없이 평균만 내므로, 그 10초 동안
    // 로봇을 건드리면 움직임이 조용히 바이어스에 섞여 세션 내내 남는다.
    void finalize_stationary_window() {
        const S n = static_cast<S>(win_count_);
        const Vec3 mean = win_gyro_sum_ / n;
        const Vec3 var  = (win_gyro_sumsq_ / n) - mean.cwiseProduct(mean);
        const S anorm_mean = win_anorm_sum_ / n;
        const S anorm_dev  = std::max(win_anorm_max_ - anorm_mean, anorm_mean - win_anorm_min_);

        S gyro_std_max = 0.0;
        for (int i = 0; i < 3; ++i) gyro_std_max = std::max(gyro_std_max, std::sqrt(std::max(0.0, var(i))));

        const bool still = (gyro_std_max * R2D < 1.0) && (anorm_dev < 0.05);

        S sigma_bcal = 0.05 * D2R;   // 정지 평균 자체의 불확실성
        if (!still) {
            if (win_retry_ < STILLNESS_MAX_RETRY) {
                win_retry_++;
                RCLCPP_WARN(this->get_logger(),
                            "[EKF] 정지 판정 실패 (자이로 std=%.2f도/초, |a| 변동=%.3f g) -> 창 재시작 (%d/%d)",
                            gyro_std_max * R2D, anorm_dev, win_retry_, STILLNESS_MAX_RETRY);
                reset_stationary_window();
                return;
            }
            // 무한 대기를 막기 위해 부풀린 불확실성으로 수용한다
            sigma_bcal *= 10.0;
            RCLCPP_ERROR(this->get_logger(),
                         "[EKF] 정지 판정 %d회 실패 -> 불확실성 10배로 수용 (바이어스 초기값 신뢰 낮음)",
                         STILLNESS_MAX_RETRY);
        }

        // 상태를 덮어쓰지 않고 **유사관측**으로 흘려보낸다.
        //   H = [0 I], nu = b_stationary - b_hat
        // 불연속이 없고, 평균 자체의 노이즈를 올바르게 반영하며, 그 10초간 EKF가
        // 학습한 것을 버리지 않고 합성한다.
        Eigen::Matrix<S, 3, 6> H = Eigen::Matrix<S, 3, 6>::Zero();
        H.block<3, 3>(0, 3) = Mat3::Identity();
        const Vec3 nu = mean - b_;
        const Mat3 R = (sigma_bcal * sigma_bcal) * Mat3::Identity();
        // 게이팅하지 않는다 (게이트하면 존재 이유가 없다). 사영도 하지 않는다 —
        // 정지 창의 자이로 평균은 세 축 바이어스를 모두 직접 잰 값이므로
        // g_b 방향 성분도 정당한 관측이다.
        update<3>(H, nu, R, 1e9);

        bias_seeded_ = true;
        a_ref_ = anorm_mean;
        if (!std::isfinite(a_ref_) || a_ref_ < 0.5 || a_ref_ > 2.0) a_ref_ = 1.0;

        // 지자기 게이트 기준: 평균이 아니라 중앙값. 순간적인 외부 자기 교란
        // (예전에 관측된 136uT 스파이크 같은 것)에 기준이 끌려가지 않게 한다.
        if (!win_mag_norms_.empty()) {
            const size_t mid = win_mag_norms_.size() / 2;
            std::nth_element(win_mag_norms_.begin(), win_mag_norms_.begin() + mid, win_mag_norms_.end());
            mag_ref_ = win_mag_norms_[mid];
            mag_ref_valid_ = true;
        }

        // 실측 sigma_g 를 설정값과 나란히 찍어둔다. 가장 중요한 튜닝 파라미터에
        // 대한 공짜 sanity check다. 자동 적용은 하지 않는다 — 정지 노이즈는
        // 운동 중 노이즈를 과소평가하므로 그대로 쓰면 과신 필터가 된다.
        const S fs = 100.0;
        const S sg_meas = gyro_std_max / std::sqrt(fs);
        RCLCPP_INFO(this->get_logger(),
                    "[EKF] 정지 창 완료: 바이어스(%.3f %.3f %.3f)도/초, |a|=%.4f g, |m|기준=%.2f uT",
                    mean.x() * R2D, mean.y() * R2D, mean.z() * R2D, a_ref_,
                    mag_ref_valid_ ? mag_ref_ : 0.0);
        RCLCPP_INFO(this->get_logger(),
                    "[EKF]   실측 sigma_g=%.4f도/초/sqrt(Hz) (설정값 %.4f) — 참고용, 자동 적용 안 함",
                    sg_meas * R2D, std::sqrt(sg2_) * R2D);
    }

    // =========================================================================
    //  출력
    // =========================================================================
    // state_estimation_node.cpp:152-160 과 **동일한** ZYX 추출. 그래야 두 토픽의
    // 차이가 단순 뺄셈이 된다. asin 클램프도 그대로 유지한다(쿼터니언 미세
    // 비정규화로 인자가 +-1을 넘으면 +-90도 부근에서 NaN이 나온다).
    void get_euler_angles(S *roll, S *pitch, S *yaw) const {
        const S q0 = q_.w(), q1 = q_.x(), q2 = q_.y(), q3 = q_.z();
        *roll  = std::atan2(q0 * q1 + q2 * q3, 0.5 - q1 * q1 - q2 * q2) * R2D;
        const S sin_pitch = std::clamp(-2.0 * (q1 * q3 - q0 * q2), -1.0, 1.0);
        *pitch = std::asin(sin_pitch) * R2D;
        *yaw   = std::atan2(q1 * q2 + q0 * q3, 0.5 - q2 * q2 - q3 * q3) * R2D;
    }

    void publish() {
        S roll, pitch, yaw;
        get_euler_angles(&roll, &pitch, &yaw);
        if (!std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(yaw)) return;

        // 헤딩 영점(btn2 짧게)에 쓸 원시 yaw 보관. EKF의 get_euler_angles 는 const라
        // 구 노드처럼 안에서 부수효과로 저장할 수 없으므로 추출 직후 여기서 잡는다.
        raw_yaw_ = static_cast<float>(yaw);

        // ── 제어 토픽 /filtered/attitude — 구 노드의 인코딩 규약 그대로 ──
        //   x = roll, 자동 모드면 +5000 (수신 3곳이 |x| > 2500 으로 해독)
        //   z = yaw - yaw_offset_ 을 ±180도 범위로 1회 되감기 (예: 190도 -> -170도)
        float filtered_yaw = static_cast<float>(yaw) - yaw_offset_;
        if (filtered_yaw > 180.0f) filtered_yaw -= 360.0f;
        else if (filtered_yaw < -180.0f) filtered_yaw += 360.0f;

        auto att = geometry_msgs::msg::Vector3();
        att.x = is_auto_mode_ ? (roll + 5000.0) : roll;
        att.y = pitch;
        att.z = static_cast<double>(filtered_yaw);
        attitude_pub_->publish(att);

        // ── 진단 토픽 /filtered/attitude_ekf — 오프셋도 모드 인코딩도 없는 순수값 ──
        // 비교 스크립트(exp_b.py 등)가 단순 뺄셈으로 쓰므로 이 규약은 유지한다.
        auto att_ekf = geometry_msgs::msg::Vector3();
        att_ekf.x = static_cast<double>(roll);
        att_ekf.y = static_cast<double>(pitch);
        att_ekf.z = static_cast<double>(yaw);    // yaw_offset_ 미적용, 절대값
        attitude_ekf_pub_->publish(att_ekf);

        // 진단은 10Hz로 솎아 발행한다. 페이로드가 자세 메시지의 4배라
        // 100Hz로 내보내면 사람도 백 분석도 필요 없는 속도로 대역만 먹는다.
        if (++tick_ % status_divisor_ != 0) return;

        auto st = std_msgs::msg::Float32MultiArray();
        st.data.resize(12);
        st.data[0] = static_cast<float>(b_.x() * R2D);
        st.data[1] = static_cast<float>(b_.y() * R2D);
        st.data[2] = static_cast<float>(b_.z() * R2D);
        // 자세 1σ. P는 body 프레임 오차이므로 엄밀히는 roll/pitch/yaw와 1:1이
        // 아니지만, 크기 감각을 보는 용도로는 대각 제곱근으로 충분하다.
        st.data[3] = static_cast<float>(std::sqrt(std::max(0.0, P_(0, 0))) * R2D);
        st.data[4] = static_cast<float>(std::sqrt(std::max(0.0, P_(1, 1))) * R2D);
        st.data[5] = static_cast<float>(std::sqrt(std::max(0.0, P_(2, 2))) * R2D);
        st.data[6] = static_cast<float>(innov_accel_.x());
        st.data[7] = static_cast<float>(innov_accel_.y());
        st.data[8] = static_cast<float>(innov_accel_.z());
        st.data[9] = static_cast<float>(innov_mag_deg_);
        st.data[10] = static_cast<float>(flags_);
        st.data[11] = static_cast<float>(dt_used_);
        status_pub_->publish(st);
    }

    // =========================================================================
    //  멤버
    // =========================================================================
    // ---- 필터 상태 ----
    Quat q_;        // 공칭 자세 (body -> world)
    Vec3 b_;        // 자이로 바이어스 [rad/s], FLU body 프레임
    Mat6 P_;        // 오차 상태 [dtheta(3); db(3)] 의 공분산

    // ---- 파라미터 (생성자에서 한 번만 읽는다) ----
    bool use_accel_ = true, use_mag_ = false;
    S sg2_ = 0.0, sbg2_ = 0.0, sa2_ = 0.0, sm2_ = 0.0;   // 전부 SI 제곱 단위
    S bias_tau_ = 1000.0, bias_max_ = 0.0;
    S accel_soft_ = 0.02, accel_reject_ = 0.30, mag_dip_gate_ = 0.0, dip_expected_ = 0.0;
    int mag_timeout_ms_ = 500, init_samples_ = 50, status_divisor_ = 10;

    // ---- 초기화 ----
    bool initialized_;
    int  init_count_;
    Vec3 init_accel_sum_;
    Vec3 init_mag_sum_;
    int  init_mag_count_;

    // ---- 정지 구간 통계 (바이어스 유사관측용) ----
    int  win_count_;
    Vec3 win_gyro_sum_, win_gyro_sumsq_;
    S    win_anorm_sum_, win_anorm_min_, win_anorm_max_;
    std::vector<S> win_mag_norms_;
    int  win_retry_;
    bool bias_seeded_;

    // ---- 게이트 기준값 (런타임 실측) ----
    S    a_ref_;          // 정지 시 |a| [g] — 1.0이 아니라 실측(~1.03)을 쓴다
    S    mag_ref_;        // 정지 시 |m| 중앙값 [uT]
    bool mag_ref_valid_;
    S    dip_ref_ = 0.0;      // 지자기-중력 사잇각 기준 [rad] (한국 약 143도)
    bool dip_ref_valid_ = false;
    int  mag_dip_bad_ = 0;    // 복각 게이트에 걸린 횟수 (보정 품질 지표)
    bool mag_active_ = true;  // 현재 9축인지 6축인지 (전환 로깅용)

    // ---- 지자기 ----
    float mag_bias_x_ = 0.0f, mag_bias_y_ = 0.0f, mag_bias_z_ = 0.0f;
    float mag_scale_x_ = 1.0f, mag_scale_y_ = 1.0f, mag_scale_z_ = 1.0f;
    Vec3  latest_m_;      // 보정 + 축 재배치까지 끝난 최신 지자기 [uT], FLU
    bool  mag_fresh_;     // 이중 계산 방지
    int   mag_far_count_;

    // ---- 버튼 UI / 모드 (구 state_estimation_node 이식) ----
    bool is_auto_mode_;                    // +5000 인코딩의 근원 (btn1 짧게로 토글)
    int  btn1_press_count_;
    int  btn1_counter_; int btn2_counter_; // 눌림 지속 틱 (1틱 = 10ms)
    int  prev_btn1_; int prev_btn2_;       // 엣지 판정용 직전 상태
    bool btn1_long_processed_; bool btn2_long_processed_;

    // ---- yaw 영점 (btn2 짧게) ----
    float raw_yaw_;      // 헤딩 영점 적용 전 원시 yaw [도] — publish()에서 매 샘플 갱신
    float yaw_offset_;   // btn2 짧게 누름으로 잡은 기준 방향 [도]

    // ---- 지자기 캘리브레이션 (magneto_cal.py 서브프로세스) ----
    bool  is_mag_calibrating_;   // true 인 동안 강제 6축
    pid_t calib_pid_;            // 실행 중인 magneto_cal.py 의 pid (-1 = 없음)

    // ---- 시간 ----
    bool          have_prev_stamp_;
    rclcpp::Time  t_prev_;
    rclcpp::Time  last_mag_time_;

    // ---- 진단 ----
    uint32_t tick_;
    uint32_t flags_;
    Vec3     innov_accel_;
    S        innov_mag_deg_;
    S        dt_used_;
    int      dt_long_count_ = 0;    // 5샘플 이상 유실 횟수 (UART 건전성 지표)
    int      dt_break_count_ = 0;   // 스트림 단절 횟수
    int      accel_rej_run_, mag_rej_run_;
    int      numeric_fault_;
    bool     logged_first_sample_;

    // ---- ROS 핸들 ----
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr attitude_ekf_pub_;      // /filtered/attitude_ekf (순수값)
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr status_pub_;       // /filtered/ekf_status
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr attitude_pub_;          // /filtered/attitude (제어)
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr ch_imu_pub_;                  // /CH_IMU (검증용)
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr ch_magnet_pub_;         // /CH_MEGNET (검증용)
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr mag_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr rc_status_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mag_calib_srv_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateEstimationEkfNode>());
    rclcpp::shutdown();
    return 0;
}
