#!/usr/bin/env python3
# 먹스 채널·압력 센서를 단계별로 분리해서 진단한다.
#
#   python3 ~/ros2_ws/src/fish_robot_pkg/scripts/mux_probe.py       기본 (채널당 5회)
#   python3 ... mux_probe.py 20                                     채널당 20회 (접촉 불량 잡을 때)
#
#   ※ I2C 버스를 독점하므로 먼저 시스템을 내릴 것:
#        sudo systemctl stop fish-robot
#
# 네 단계를 따로 판정한다. 앞 단계가 통과해도 뒤 단계에서 떨어질 수 있다:
#   1) 채널 전기 상태 — 그 채널을 열었을 때 버스가 살아있는가
#   2) 센서 응답·PROM — 리셋에 답하는가, 보정 계수가 온전한가 (CRC 검증)
#   3) **ADC 변환**    — 실제로 압력을 뽑아내는가, 그 값이 대기압 범위인가
#   4) **배정 대조**   — config/port_map.txt 에 적힌 개체가 지금 그 자리에 있는가
#
# 3)이 왜 필요한가 (2026-09-04에 두 개체에서 실측):
#   리셋(0x1E)도 받고 PROM도 CRC까지 맞게 읽히는데 **변환 명령(0x40~0x4A)만
#   전 OSR에서 NACK** 되는 고장이 있다. 2)까지만 보면 "정상"으로 나오지만
#   압력은 한 샘플도 안 나온다. 게다가 i2c_driver_node 의 실패 ERROR 는
#   **전 채널이 죽었을 때만** 뜨므로(i2c_driver_node.cpp 의 press_fail_streak_),
#   셋 중 하나만 이러면 저널에 아무 흔적도 남지 않는다. 이 단계가 그 사각지대다.
#
# 4)를 도구가 하는 이유: 지문을 숫자로만 찍으면 사람이 port_map.txt 를 열어 눈으로
#   대조해야 한다. 센서를 자주 바꿔 끼우는 선별 작업에서 그게 제일 헷갈리는 지점이다.
#   ※ 지문은 C1~C6 여섯 개를 다 본다. C1 은 일련번호가 아니라 보정 계수라 유일성이
#     보장되지 않고, i2c_driver_node 의 판정 기준도 여섯 개 전부다 — 기준을 맞춰 둔다.
#
# ※ 탐지는 반드시 "쓰기"(리셋 0x1E)로 한다. MS5837은 선행 명령 없이 읽으면
#   응답하지 않아, i2cdetect/i2cget 으로는 멀쩡한 센서도 못 찾는다.
#   (2026-08-20에 이걸로 J5를 "무응답"으로 오진했다)
import fcntl, os, sys, time

I2C_SLAVE = 0x0703
BUS, MUX, SENS = '/dev/i2c-1', 0x70, 0x76

D1_CMD, D2_CMD = 0x4A, 0x5A   # 압력/온도 변환, OSR=8192 — i2c_driver_node 와 같은 설정
OSR_WAIT = 0.025              # 데이터시트 최대 18ms + 여유


def talk(fd, addr, wr=None, rd=0):
    """한 번의 I2C 거래. 성공하면 읽은 바이트(또는 b''), 실패하면 None."""
    try:
        fcntl.ioctl(fd, I2C_SLAVE, addr)
        if wr is not None:
            os.write(fd, bytes(wr))
        return os.read(fd, rd) if rd else b''
    except OSError:
        return None


def crc4(prom):
    """MS5837 데이터시트의 PROM CRC-4. prom = 워드 7개 리스트."""
    n = list(prom) + [0]
    n[0] &= 0x0FFF                      # CRC 자리를 0으로
    rem = 0
    for cnt in range(16):
        rem ^= (n[cnt >> 1] & 0xFF) if cnt % 2 else (n[cnt >> 1] >> 8)
        for _ in range(8):
            rem = ((rem << 1) ^ 0x3000) & 0xFFFF if rem & 0x8000 else (rem << 1) & 0xFFFF
    return (rem >> 12) & 0x0F


def read_prom(fd):
    """PROM 7워드를 빅엔디안으로 읽는다. 실패하면 None."""
    words = []
    for p in range(7):
        b = talk(fd, SENS, [0xA0 + p * 2], 2)
        if b is None or len(b) != 2:
            return None
        words.append((b[0] << 8) | b[1])
    return words


def read_adc(fd, cmd):
    """변환 명령 -> 대기 -> 24비트 회수. (값, 실패사유). 값이 None이면 사유를 본다.

    'nack'    변환 명령 자체를 거부 — ADC 사망 (PROM은 멀쩡해도 이럴 수 있다)
    'read'    변환은 받았는데 결과 회수 실패
    'zero'    0 또는 0xFFFFFF — 미수거/통신 실패. 드라이버도 이 둘을 버린다
    """
    if talk(fd, SENS, [cmd]) is None:
        return None, 'nack'
    time.sleep(OSR_WAIT)
    b = talk(fd, SENS, [0x00], 3)
    if b is None or len(b) != 3:
        return None, 'read'
    v = (b[0] << 16) | (b[1] << 8) | b[2]
    if v in (0, 0xFFFFFF):
        return None, 'zero'
    return v, None


def idiv(a, b):
    """C 의 정수 나눗셈(0 쪽으로 절단). 파이썬 // 는 내림이라 음수에서 1 LSB 어긋난다."""
    q = abs(a) // abs(b)
    return q if (a < 0) == (b < 0) else -q


def convert_02ba(C, d1, d2):
    """MS5837-**02BA** 환산식. i2c_driver_node.cpp 와 같은 식이어야 한다. -> (mbar, °C)

    ※ 30BA 식을 쓰면 정확히 20배 어긋난 값이 나온다(2026-08-20 실측:
      30BA 식 20132.60 vs 02BA 식 1006.63). 아래 '대기압 범위' 판정이 그걸 잡는다.
    """
    dT = d2 - C[5] * 256
    temp = 2000 + idiv(dT * C[6], 8388608)
    off = C[2] * 131072 + idiv(C[4] * dT, 64)
    sens = C[1] * 65536 + idiv(C[3] * dT, 128)
    if temp < 2000:                                   # 2차 보상 — 20°C 미만에서만 발동
        d2sq = (temp - 2000) ** 2
        temp -= idiv(11 * dT * dT, 17179869184)
        off -= idiv(31 * d2sq, 8)
        sens -= idiv(63 * d2sq, 32)
    p = idiv(idiv(d1 * sens, 2097152) - off, 32768)
    return p / 100.0, temp / 100.0


def load_port_map():
    """config/port_map.txt 를 읽어 (verified, {ch: (역할, [C1..C6])}) 를 돌려준다.

    i2c_driver_node::verify_port_map() 과 같은 파일·같은 형식을 읽는다.
    파일이 없으면 (None, {}) — 교체 직후 지운 상태가 정상적인 경우다.
    """
    home = os.environ.get('HOME')
    path = os.path.join(home, 'ros2_ws/config/port_map.txt') if home else 'config/port_map.txt'
    if not os.path.exists(path):
        return None, {}
    verified, rows = False, {}
    with open(path, encoding='utf-8') as f:
        for line in f:
            # "verified:" 뒤 **첫 토큰만** 본다. 줄 안에 'yes' 가 있는지로 보면
            # 안내문("... 후 yes 로 바꾸십시오")이 자기 자신을 통과시킨다
            # (2026-08-25 에 드라이버에서 실제로 났던 사고).
            vp = line.find('verified:')
            if vp != -1:
                tok = line[vp + 9:].split()
                if tok:
                    verified = (tok[0] == 'yes')
            if not line.strip() or line.lstrip().startswith('#'):
                continue
            f7 = line.split()
            if len(f7) < 8:
                continue
            try:
                ch = int(f7[1])
                coef = [int(x) for x in f7[2:8]]
            except ValueError:
                continue
            if 0 <= ch <= 2:
                rows[ch] = (f7[0], coef)
    return verified, rows


def match_port_map(ch, prom, rows):
    """이 자리의 개체가 파일과 같은지 판정해 사람이 읽을 한 줄로 돌려준다."""
    if not rows:
        return f"C1={prom[1]} (배정 파일 없음 — 확정 전)"
    if ch not in rows:
        return f"C1={prom[1]} **파일에 없던 채널** — 새로 꽂은 자리"
    role, coef = rows[ch]
    if list(prom[1:7]) == coef:
        return f"{role} 지문일치"
    return f"**{role} 자리 개체 교체됨** (파일 C1={coef[0]} -> 실제 C1={prom[1]})"


def probe_adc(fd, C, tries):
    """ADC 변환을 tries 회 시도하고 (상태, 비고) 를 돌려준다."""
    ok, fails, press, temps = 0, {}, [], []
    for _ in range(tries):
        d1, why = read_adc(fd, D1_CMD)
        if d1 is None:
            fails[why] = fails.get(why, 0) + 1
            continue
        d2, why = read_adc(fd, D2_CMD)
        if d2 is None:
            fails[why] = fails.get(why, 0) + 1
            continue
        p, t = convert_02ba(C, d1, d2)
        ok += 1
        press.append(p)
        temps.append(t)

    if ok == 0:
        if fails.get('nack'):
            return 'ADC 사망', (f"PROM은 정상인데 변환명령을 {fails['nack']}/{tries} 거부 "
                              "— 개체 불량, 교체 대상")
        return 'ADC 무응답', f"변환 결과를 못 받음 ({fails}) — 배선/접촉 확인"

    avg = sum(press) / len(press)
    span = max(press) - min(press)
    note = f"{avg:.2f} mbar {sum(temps)/len(temps):.1f}°C (폭 {span:.2f}), ADC {ok}/{tries}"
    if not (300.0 <= avg <= 1200.0):
        # 공기 중 대기압은 어디서든 300~1200mbar 안이다. 벗어나면 센서 고장이거나
        # 환산식이 모델과 안 맞는 것(30BA/02BA 혼동 = 정확히 20배).
        return '값 이상', note + " — 대기압 범위(300~1200) 밖. 센서 모델/환산식 확인"
    if ok < tries:
        return f'ADC 불안정 {ok}/{tries}', note
    return '정상', note


def main():
    knocks = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    fd = os.open(BUS, os.O_RDWR)

    print("=" * 78)
    print(f" 먹스·압력센서 진단  (채널당 {knocks}회)")
    print("=" * 78)

    if talk(fd, MUX, rd=1) is None:
        print(" 먹스(0x70) 무응답 — 버스가 잠겼습니다.")
        print(" -> 전원을 완전히 내렸다 올린 뒤 다시 실행하세요 (재부팅으로는 안 풀립니다).")
        return 1
    print(" 먹스 응답 정상")

    verified, pmap = load_port_map()
    if not pmap:
        print(" 배정 파일 없음 — 지문 대조를 건너뜁니다 (기동하면 지금 상태로 새로 만들어집니다)")
    elif verified:
        print(" 배정 파일: 물리 확인 완료(verified: yes) — 아래 지문이 이 배정의 근거입니다")
    else:
        print(" 배정 파일: **물리 확인 전(verified: no)** — 역할은 추정값입니다. 바람 시험을 하십시오")
    print()

    results = []
    for ch in range(3):
        name = f"ch{ch} (J{ch+5})"
        mask = 1 << ch

        # --- 1) 채널 전기 상태 ---
        if talk(fd, MUX, [mask]) is None:
            results.append((name, "채널 열기 실패", "-", "버스가 이미 죽었을 수 있음"))
            break
        time.sleep(0.05)
        back = talk(fd, MUX, rd=1)
        if back is None:
            results.append((name, "버스 사망", "-",
                            "이 채널을 여는 순간 버스가 죽음 -> 커넥터/기판 단락 의심"))
            print(f" {name}: 버스 사망 — 여기서 중단합니다 (전원 내렸다 올려야 복구)")
            break
        if back[0] != mask:
            results.append((name, f"래치 불일치(0x{back[0]:02X})", "-", "먹스 이상"))
            continue

        # --- 2) 센서 응답 (여러 번 두드려 접촉 불량을 잡는다) ---
        hit = 0
        for _ in range(knocks):
            if talk(fd, SENS, [0x1E]) is not None:
                hit += 1
            time.sleep(0.05)

        if hit == 0:
            results.append((name, "정상", "센서 없음", "미연결이거나 완전 불량"))
        else:
            time.sleep(0.02)
            prom = read_prom(fd)
            if prom is None:
                results.append((name, "정상", f"불안정 {hit}/{knocks}",
                                "주소는 응답하나 PROM 읽기 실패"))
            elif any(w in (0x0000, 0xFFFF) for w in prom):
                results.append((name, "정상", f"불안정 {hit}/{knocks}",
                                "PROM에 0/FFFF 포함 — 통신 불량"))
            elif crc4(prom) != (prom[0] >> 12):
                results.append((name, "정상", f"불안정 {hit}/{knocks}",
                                f"PROM CRC 불일치 (계산 {crc4(prom)}, 저장 {prom[0] >> 12})"))
            else:
                # --- 3) ADC 변환 — 여기까지 와야 "정상"이라고 부를 수 있다 ---
                state, note = probe_adc(fd, prom, knocks)
                if state == '정상' and hit < knocks:
                    state = f"접촉 불안정 {hit}/{knocks}"
                # 대조는 도구가 한다 — 사람이 파일을 열어 숫자를 눈으로 맞추지 않게.
                note = match_port_map(ch, prom, pmap) + ", " + note
                results.append((name, "정상", state, note))

        talk(fd, MUX, [0x00])

    talk(fd, MUX, [0x00])
    os.close(fd)

    print(f" {'채널':<12}{'채널 상태':<16}{'센서 상태':<18}비고")
    print(" " + "-" * 76)
    for r in results:
        print(f" {r[0]:<12}{r[1]:<16}{r[2]:<18}{r[3]}")
    print("=" * 78)

    # 해석 안내
    states = [r[2] for r in results]
    notes = [r[3] for r in results]
    if any("교체됨" in n or "파일에 없던" in n for n in notes):
        print(" **지금 꽂힌 개체가 배정 파일과 다릅니다.** 의도한 교체라면 파일을 지우고")
        print(" 시스템을 한 번 띄우면 지금 상태로 새로 만들어집니다(verified: no 로 적힙니다):")
        print("   rm ~/ros2_ws/config/port_map.txt")
        print(" 그 다음 **바람 시험**으로 앞/좌/우를 다시 확정하고 verified 를 yes 로 바꾸십시오 —")
        print(" 지문이 맞아도 그건 '개체가 그 자리에 있다'는 뜻일 뿐, 튜브가 어디로 가는지는 모릅니다.")
        print()
    if any("ADC" in s or s == "값 이상" for s in states):
        print(" **PROM은 멀쩡한데 ADC가 죽은 채널이 있습니다.** 이 고장은 저널에 흔적을")
        print(" 남기지 않습니다(전 채널이 죽어야 ERROR가 뜹니다) — 압력만 조용히 NaN이 됩니다.")
        print(" 센서와 커넥터 중 어느 쪽인지는 **정상 채널과 서로 바꿔 꽂고** 다시 재면 갈립니다.")
        print("   고장이 커넥터를 따라가면 -> 배선,  개체를 따라가면 -> 센서 폐기.")
    elif any("불안정" in s for s in states):
        print(" 접촉 불안정이 있습니다. `bash ~/press_watch.sh` 를 띄워두고 커넥터를")
        print(" 눌러보거나 케이블을 흔들어 어느 자세에서 붙는지 찾으세요.")
    elif any(r[1] == "버스 사망" for r in results):
        print(" 버스를 죽이는 채널이 있습니다. 전원을 끄고 그 커넥터의 SDA/SCL과 GND")
        print(" 사이 통전을 확인하세요 (삐 소리가 나면 단락).")
    elif all(s == "센서 없음" for s in states):
        print(" 센서가 하나도 응답하지 않습니다. 커넥터의 VCC 전압(3.3V)과")
        print(" SDA/SCL 핀 순서를 먼저 확인하세요.")
    elif any(s == "정상" for s in states):
        print(" 압력까지 확인된 채널이 있습니다. 그대로 측정에 들어가도 됩니다.")
        print(" (수심은 좌·우 두 정압 포트가 **둘 다** 살아야 나옵니다. 노즈만으로는 안 됩니다.)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
