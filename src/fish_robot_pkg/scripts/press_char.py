#!/usr/bin/env python3
# 압력 센서(MS5837-02BA) 특성화 — 안정화 시간 / 영점 / 잡음을 한 번의 기록으로 잰다.
#
#   python3 press_char.py [초]        기록 (기본 3600초). launch가 부팅과 함께 자동 실행.
#   python3 press_char.py --s1 <csv>  1단계: 센서가 안정되는 데 얼마나 걸리나
#   python3 press_char.py --s2 <csv>  2단계: 공기 중에서 빼줘야 할 값(영점)과 현행 절차의 오차
#   python3 press_char.py --s3 <csv>  3단계: 얼마나 떨리나(잡음)와 최대-최소 폭
#
#   --after <초> 로 2·3단계의 "안정 구간 시작점"을 직접 지정할 수 있다.
#   생략하면 1단계 판정값을 내부에서 다시 계산해 쓴다.
#
# 전제: 측정 중에는 i2c_driver_node의 IIR 필터가 꺼져 있어야 한다. 필터가 살아 있으면
#       잡음이 4.4배 작게 나온다(3단계가 무의미해진다). 1·2단계는 느린 현상이라
#       필터가 있어도 결과가 같다.
#
# 주의: 기존 시험 스크립트들과 달리 50행마다 증분 저장한다. 60분짜리 기록을 끝에
#       한 번만 저장하면 중간에 서비스가 멈출 때 전부 날아간다.
import sys
import csv
import time

OUT_DIR = "/home/fish/ros2_ws/log_csv"
HDR = ['t', 'p0', 'p1', 'p2', 'tc0', 'tc1', 'tc2']

# 데이터시트 값 — 이 측정이 검증하려는 대상
# MS5837-02BA, OSR 8192 기준. 30BA는 0.20mbar(2mm)로 12.5배 나쁘므로
# 센서 모델을 바꾸면 이 값도 같이 바꿔야 한다.
DS_SIGMA_MBAR = 0.016
# 이 하드웨어의 실측 기준선 (2026-08-20, 60분, 필터 우회, 3채널):
# 0.13 / 0.13 / 0.24 mbar = 데이터시트의 8~15배. 채널 간 무상관(r≈-0.005)이라
# 실제 기압 변동이나 공통 전원 잡음이 아니라 센서 개별 전기 잡음이다.
# 앞으로의 측정은 데이터시트가 아니라 이 값과 비교해서 판정한다.
MEASURED_SIGMA_MBAR = 0.13
MBAR_TO_MM = 10.197        # 담수 1mbar = 1.0197cm = 10.197mm


# =============================================================================
#  기록
# =============================================================================
def record(dur):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray

    out = f"{OUT_DIR}/presschar_{time.strftime('%Y%m%d_%H%M%S')}.csv"

    class Rec(Node):
        def __init__(self):
            super().__init__('press_char')
            self.t0 = None
            self.n = 0
            self.last = -999.0
            self.buf = []
            self.f = open(out, 'w', newline='')
            self.w = csv.writer(self.f)
            self.w.writerow(HDR)
            self.f.flush()
            self.create_subscription(Float32MultiArray, '/sensor/pressure_raw', self.cb, 50)

        def cb(self, m):
            if len(m.data) < 6:
                return
            now = time.monotonic()
            if self.t0 is None:
                self.t0 = now
            el = now - self.t0

            # csv.writer는 float를 repr로 쓴다 -> 정밀도 손실 없음.
            # (로거의 %.2f를 거치면 0.01mbar로 뭉개져 잡음 측정이 불가능하다)
            self.buf.append([el] + [float(x) for x in m.data[:6]])
            self.n += 1

            if len(self.buf) >= 50:          # 약 4.5초마다 디스크로
                self.w.writerows(self.buf)
                self.f.flush()
                self.buf.clear()

            if el - self.last >= 30.0:
                self.last = el
                p = m.data[:3]
                sys.stdout.write(
                    f"\r[압력 특성화] {el:6.0f}/{dur:.0f}초  {self.n}샘플  "
                    f"ch0={p[0]:8.2f}  ch1={p[1]:8.2f}  ch2={p[2]:8.2f} mbar   ")
                sys.stdout.flush()

            if el >= dur:
                raise SystemExit

        def close(self):
            if self.buf:
                self.w.writerows(self.buf)
            self.f.flush()
            self.f.close()

    rclpy.init()
    n = Rec()
    print(f"\n=== 압력 센서 특성화 기록 · {dur/60:.0f}분 ===")
    print("로봇을 흔들리지 않는 곳에 두고 건드리지 마세요. 모터 전원은 꺼두세요.")
    print(f"저장 위치: {out}\n")
    try:
        rclpy.spin(n)
    except (SystemExit, KeyboardInterrupt):
        pass
    n.close()
    print(f"\n\n기록 종료: {out}  ({n.n} 샘플)")
    print("분석:  python3 press_char.py --s1 " + out)
    rclpy.shutdown()


# =============================================================================
#  공통 — 로드와 기울기
# =============================================================================
def load(path):
    """CSV를 읽고 (t, P, T, alive)를 돌려준다. 무효 구간은 잘라낸다.

    두 단계로 거른다:
      1) 죽은 채널 — 드라이버가 PROM 적재 실패 시 0을 내보낸다. 중앙값으로 판정
         (평균은 뒤에 붙은 0 무더기에 끌려간다).
      2) 무효 행 — 기록 도중 I2C 버스가 죽으면 그 시점부터 0이 쌓인다. 이걸 안
         걸러내면 (0 - 1006)mbar가 수심 -10m 오차로 둔갑해 분석이 통째로 망가진다.
         (2026-08-20 실측에서 실제로 겪음) 살아있는 채널이 하나라도 무효인 행은
         버리고, 구간이 끊기면 **가장 긴** 연속 구간을 쓴다(이유는 아래 주석).
    """
    import numpy as np
    A = np.genfromtxt(path, delimiter=',', names=True)
    t = A['t']
    P = np.vstack([A['p0'], A['p1'], A['p2']]).T
    T = np.vstack([A['tc0'], A['tc1'], A['tc2']]).T

    alive = [i for i in range(3) if np.median(P[:, i]) >= 100.0]
    if not alive:
        return t, P, T, alive

    good = np.all(P[:, alive] >= 100.0, axis=1)
    if not good.all():
        # 가장 긴 연속 유효 구간을 취한다. "첫 구간"이 아니라 "가장 긴 구간"인 이유:
        # 버스가 끝에서 죽으면 앞쪽이 최장이고(2026-08-20 25분 기록), 초반에 몇 초
        # 끊겼다 살아나면 뒤쪽이 최장이다(같은 날 60분 기록의 t=42~45초 3초 결측).
        # 첫 구간만 쓰면 후자에서 59분을 통째로 버리게 된다.
        idx = np.flatnonzero(np.diff(np.concatenate(([0], good.view(np.int8), [0]))))
        starts, ends = idx[0::2], idx[1::2]          # [start, end) 쌍
        k = int(np.argmax(ends - starts))
        a, b = int(starts[k]), int(ends[k])
        print(f" ※ 무효(0mbar) {int((~good).sum())}행 발견 — 최장 연속 유효 구간"
              f" t={t[a]:.1f}~{t[b-1]:.1f}초 ({b-a}행)만 분석합니다.")
        t, P, T = t[a:b], P[a:b], T[a:b]
        t = t - t[0]                                  # 구간 시작을 0초로 재기준
    return t, P, T, alive


def slope_mbar_per_min(t, p):
    """구간 [t, p]의 1차 회귀 기울기를 mbar/분으로."""
    import numpy as np
    if len(t) < 10:
        return float('nan')
    a = np.polyfit(t, p, 1)[0]      # mbar/초
    return a * 60.0


def settle_time(t, P, alive, win=60.0, thresh=0.2):
    """|기울기|가 thresh(mbar/분) 아래로 내려가 그 뒤로 계속 유지되는 첫 시각.

    '처음 내려간 시각'이 아니라 '내려간 뒤 되돌아오지 않는 시각'을 쓴다.
    드리프트 곡선이 잡음 때문에 잠깐 평평해 보이는 구간에 속지 않기 위해서다.
    """
    import numpy as np
    worst = 0.0
    for ch in alive:
        ok_from = None
        edges = np.arange(0.0, t[-1] - win, win / 2.0)
        for i in range(len(edges) - 1, -1, -1):
            s = edges[i]
            m = (t >= s) & (t < s + win)
            if m.sum() < 10:
                continue
            if abs(slope_mbar_per_min(t[m], P[m, ch])) <= thresh:
                ok_from = s
            else:
                break
        if ok_from is None:
            return None            # 기록 끝까지 안정되지 않음
        worst = max(worst, ok_from)
    return worst


def resolve_after(t, P, alive, argv):
    """--after 가 있으면 그 값, 없으면 1단계 판정값."""
    if '--after' in argv:
        return float(argv[argv.index('--after') + 1])
    s = settle_time(t, P, alive)
    return s if s is not None else t[-1] * 0.5


# =============================================================================
#  1단계 — 안정화 시간
# =============================================================================
def stage1(path):
    import numpy as np
    t, P, T, alive = load(path)
    print("\n" + "=" * 66)
    print(" [1단계] 센서 안정화에 얼마나 걸리나")
    print("=" * 66)
    print(f" 기록 {t[-1]/60:.1f}분, {len(t)}샘플, 살아있는 채널 {alive}")
    if not alive:
        print(" 살아있는 채널이 없습니다. 압력 센서 연결을 확인하세요.")
        return
    print(f" ※ 센서 예열은 파이 전원 인가부터 시작됩니다. 부팅~기록시작 지연만큼")
    print(f"   아래 시각에 더해서 해석하세요.\n")

    # 시간대별 기울기
    print(" 60초 창의 기울기 [mbar/분]  (음수 = 값이 내려가는 중)")
    print(" " + "-" * 64)
    print(f" {'시각[초]':>10} " + "".join([f"{'ch'+str(c):>12}" for c in alive]))
    marks = [30, 60, 120, 180, 240, 300, 420, 600, 900, 1200, 1800, 2400, 3000, 3300]
    for s in marks:
        if s + 60 > t[-1]:
            break
        m = (t >= s) & (t < s + 60)
        if m.sum() < 10:
            continue
        row = "".join([f"{slope_mbar_per_min(t[m], P[m, c]):>12.3f}" for c in alive])
        print(f" {s:>10} {row}")

    # 기울기 추정 자체의 잡음 하한 — 이보다 낮은 문턱은 의미가 없다.
    # 잡음 sigma인 신호를 창 W초, N샘플로 회귀할 때 기울기의 표준오차는
    # sigma / (sigma_t * sqrt(N)),  sigma_t = W/sqrt(12).
    win = 60.0
    m0 = t >= (t[-1] * 0.5)                 # 안정 구간에서 잡음을 대충 잡는다
    n_win = max(int(win * len(t) / t[-1]), 10)
    noise = float(np.median([np.std(np.diff(P[m0, c])) / np.sqrt(2) for c in alive]))
    se = noise / ((win / np.sqrt(12.0)) * np.sqrt(n_win)) * 60.0   # mbar/분

    # 문턱 통과 시각
    print("\n 안정 판정 (그 뒤로 되돌아오지 않는 첫 시각)")
    print(f" 기울기 추정의 잡음 하한 ±{se:.3f} mbar/분 — 이보다 촘촘한 문턱은 무의미")
    print(" " + "-" * 64)
    for th in (0.5, 0.2, 0.05):
        mm = th * MBAR_TO_MM
        if th < 2.0 * se:
            print(f"   |기울기| < {th:.2f} mbar/분 ({mm:.1f}mm/분): 판정 불가 "
                  f"(잡음 하한 {se:.3f}에 묻힘)")
            continue
        s = settle_time(t, P, alive, thresh=th)
        if s is None:
            print(f"   |기울기| < {th:.2f} mbar/분 ({mm:.1f}mm/분): 기록 끝까지 미달성")
        else:
            print(f"   |기울기| < {th:.2f} mbar/분 ({mm:.1f}mm/분): {s:6.0f}초  ({s/60:.1f}분)")

    # 총 드리프트
    print("\n 총 드리프트 (기록 시작 -> 안정 구간 평균)")
    print(" " + "-" * 64)
    after = settle_time(t, P, alive) or t[-1] * 0.5
    m_end = t >= after
    for c in alive:
        first = np.mean(P[t < 10.0, c])
        last = np.mean(P[m_end, c])
        d = last - first
        print(f"   ch{c}: {first:9.3f} -> {last:9.3f} mbar   변화 {d:+7.3f} mbar "
              f"({d*MBAR_TO_MM:+7.1f}mm)")

    # 300초 판정 — 이 측정의 핵심 숫자
    print("\n 현행 PRESSURE_WARMUP_SEC = 300초 판정")
    print(" " + "-" * 64)
    ok = True
    for c in alive:
        m = (t >= 300) & (t < 360)
        if m.sum() < 10:
            print(f"   ch{c}: 기록이 짧아 판정 불가")
            continue
        sl = slope_mbar_per_min(t[m], P[m, c])
        # 영점을 300초에 잡고 1시간 운용했을 때 누적되는 수심 오차
        err_1h = abs(sl) * 60.0 * MBAR_TO_MM / 10.0   # mm/분 * 60분 -> mm, /10 -> cm
        print(f"   ch{c}: t=300초의 기울기 {sl:+7.3f} mbar/분 "
              f"-> 1시간 뒤 수심 오차 {err_1h:6.1f} cm")
        if err_1h > 5.0:
            ok = False
    print()
    if ok:
        print("   판정: 300초로 충분합니다.")
    else:
        print("   판정: 300초로는 부족합니다. 위 '안정 판정' 표의 시각으로 늘리는 것을 검토하세요.")

    print("\n 온도 변화 (드리프트 원인이 온도인지 확인 — §N2는 '아니다'였음)")
    print(" " + "-" * 64)
    for c in alive:
        print(f"   ch{c}: {T[0, c]:6.2f} -> {T[-1, c]:6.2f} °C   변화 {T[-1, c]-T[0, c]:+5.2f}")
    print("=" * 66)
    print(f"\n다음: python3 press_char.py --s2 {path}\n")


# =============================================================================
#  2단계 — 빼줘야 하는 값(영점)
# =============================================================================
def stage2(path):
    import numpy as np
    t, P, T, alive = load(path)
    after = resolve_after(t, P, alive, sys.argv)
    m = t >= after

    print("\n" + "=" * 66)
    print(" [2단계] 공기 중에서 빼줘야 하는 값 (영점)")
    print("=" * 66)
    print(f" 안정 구간: t >= {after:.0f}초, {m.sum()}샘플 ({(t[-1]-after)/60:.1f}분)\n")

    print(" 채널별 영점 (안정 구간 평균)")
    print(" " + "-" * 64)
    ref = {}
    for c in alive:
        v = float(np.mean(P[m, c]))
        ref[c] = v
        print(f"   ch{c}: {v:9.3f} mbar")

    # 현행 절차가 실제로 잡는 값 — EKF는 t=300초부터 100샘플을 평균한다
    print("\n 현행 절차(300초 대기 후 100샘플)가 잡는 영점과의 차이")
    print(" " + "-" * 64)
    idx = np.where(t >= 300.0)[0]
    if len(idx) < 100:
        print("   기록이 짧아 시뮬레이션 불가")
    else:
        sel = idx[:100]
        print(f"   시뮬레이션 구간: t = {t[sel[0]]:.1f} ~ {t[sel[-1]]:.1f}초")
        for c in alive:
            cap = float(np.mean(P[sel, c]))
            err = cap - ref[c]
            print(f"   ch{c}: 캡처 {cap:9.3f}  참값 {ref[c]:9.3f}  "
                  f"오차 {err:+7.3f} mbar ({err*MBAR_TO_MM:+7.1f}mm)")
        print("\n   이 오차는 세션 내내 모든 수심에 고정으로 실립니다.")

    # 채널 간 차이 — 차압으로 자세를 볼 때의 기준선
    if len(alive) >= 2:
        print("\n 채널 간 차이 (차압 활용 시의 기준선)")
        print(" " + "-" * 64)
        for i in range(len(alive)):
            for j in range(i + 1, len(alive)):
                a, b = alive[i], alive[j]
                d = ref[a] - ref[b]
                print(f"   ch{a} - ch{b}: {d:+8.3f} mbar ({d*MBAR_TO_MM:+8.1f}mm)")
        print("\n   02BA의 절대 정확도는 ±0.5mbar(600~1000mbar, 20도) / ±2mbar(넓은 조건)이므로")
        print("   개체 간 1~2mbar 차이는 정상 범위다. 공기 중이라 높이차 효과는 무시할 수준이다")
        print("   (공기는 물보다 800배 가벼워 1m 높이차가 0.12mbar = 수심 1.2mm).")
        print("   즉 이 값은 순수한 개체차이며, 차압으로 자세를 보려면 반드시 먼저 빼야 한다.")
    print("=" * 66)
    print(f"\n다음: python3 press_char.py --s3 {path}\n")


# =============================================================================
#  3단계 — 잡음과 최대-최소
# =============================================================================
def stage3(path):
    import numpy as np
    t, P, T, alive = load(path)
    after = resolve_after(t, P, alive, sys.argv)
    m = t >= after
    ts, Ps = t[m], P[m]

    print("\n" + "=" * 66)
    print(" [3단계] 얼마나 떨리나 (잡음)")
    print("=" * 66)
    print(f" 안정 구간: t >= {after:.0f}초, {len(ts)}샘플\n")

    # 2초 창 표준편차의 중앙값 = 드리프트가 안 섞인 잡음
    print(" 잡음 sigma  — 2초 창의 표준편차, 창들의 중앙값")
    print(" (짧은 창이라 드리프트가 섞이지 않는다. 긴 창을 쓰면 잡음이 과대평가된다)")
    print(" " + "-" * 64)
    sig = {}
    for c in alive:
        stds = []
        s = ts[0]
        while s + 2.0 <= ts[-1]:
            w = (ts >= s) & (ts < s + 2.0)
            if w.sum() >= 8:
                stds.append(np.std(Ps[w, c], ddof=1))
            s += 2.0
        if not stds:
            continue
        v = float(np.median(stds))
        sig[c] = v
        ratio = v / DS_SIGMA_MBAR
        print(f"   ch{c}: {v:.4f} mbar = {v*MBAR_TO_MM:5.2f} mm   "
              f"(데이터시트 {DS_SIGMA_MBAR} mbar 대비 {ratio:4.2f}배, 창 {len(stds)}개)")

    if sig:
        print()
        worst = max(sig.values()) / MEASURED_SIGMA_MBAR
        ds = max(sig.values()) / DS_SIGMA_MBAR
        print(f"   (데이터시트 대비 {ds:.1f}배 — 이 하드웨어의 기존 실측도 8~15배였으므로 정상 범위)")
        if worst <= 2.0:
            print(f"   판정: 기존 실측 기준선({MEASURED_SIGMA_MBAR} mbar)과 같은 수준입니다.")
        elif worst <= 5.0:
            print(f"   판정: 기존 실측보다 {worst:.1f}배 나쁩니다. 배선·전원·진동을 의심하세요.")
        else:
            print(f"   판정: 기존 실측보다 {worst:.1f}배 나쁩니다. "
                  f"IIR 필터가 켜져 있으면 오히려 작게 나오므로, 이 정도면 하드웨어 문제입니다.")

    # 최대-최소는 관측 시간에 따라 자란다 — 그래서 스펙으로 못 쓴다
    print("\n 최대-최소 폭 — 구간 길이에 따라 자란다")
    print(" " + "-" * 64)
    print(f" {'구간':>10} " + "".join([f"{'ch'+str(c):>16}" for c in alive]))
    for dur, name in ((1.0, "1초"), (10.0, "10초"), (60.0, "1분"),
                      (600.0, "10분"), (ts[-1] - ts[0], "전체")):
        if dur > ts[-1] - ts[0]:
            continue
        cells = ""
        for c in alive:
            p2p = []
            s = ts[0]
            while s + dur <= ts[-1]:
                w = (ts >= s) & (ts < s + dur)
                if w.sum() >= 5:
                    p2p.append(np.ptp(Ps[w, c]))
                s += max(dur, 1.0)
            if p2p:
                v = float(np.median(p2p))
                cells += f"{v*MBAR_TO_MM:14.1f}mm"
            else:
                cells += f"{'-':>16}"
        print(f" {name:>8} {cells}")

    if sig:
        c0 = alive[0]
        print(f"\n   같은 신호인데 sigma는 {sig[c0]*MBAR_TO_MM:.1f}mm, 전체 최대-최소는 그 몇 배입니다.")
        print("   최대-최소는 오래 볼수록 커지므로 센서의 성질이 될 수 없습니다.")
        print("   '잡음이 얼마인가'에 답할 때는 반드시 sigma를 쓰세요.")
    print("=" * 66 + "\n")


# =============================================================================
if __name__ == '__main__':
    if len(sys.argv) > 2 and sys.argv[1] == '--s1':
        stage1(sys.argv[2])
    elif len(sys.argv) > 2 and sys.argv[1] == '--s2':
        stage2(sys.argv[2])
    elif len(sys.argv) > 2 and sys.argv[1] == '--s3':
        stage3(sys.argv[2])
    else:
        record(float(sys.argv[1]) if len(sys.argv) > 1 else 3600.0)
