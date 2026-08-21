#!/usr/bin/env python3
# 먹스 채널 상태와 압력 센서 연결 상태를 분리해서 진단한다.
#
#   python3 ~/mux_probe.py            기본 (채널당 5회 두드림)
#   python3 ~/mux_probe.py 20         채널당 20회 (접촉 불량 잡을 때)
#
# 두 가지를 따로 판정한다:
#   1) 채널 전기 상태 — 그 채널을 열었을 때 버스가 살아있는가
#   2) 센서 상태      — 센서가 응답하는가, PROM이 온전한가 (CRC 검증)
#
# ※ 탐지는 반드시 "쓰기"(리셋 0x1E)로 한다. MS5837은 선행 명령 없이 읽으면
#   응답하지 않아, i2cdetect/i2cget 으로는 멀쩡한 센서도 못 찾는다.
#   (2026-08-20에 이걸로 J5를 "무응답"으로 오진했다)
import fcntl, os, sys, time

I2C_SLAVE = 0x0703
BUS, MUX, SENS = '/dev/i2c-1', 0x70, 0x76


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


def main():
    knocks = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    fd = os.open(BUS, os.O_RDWR)

    print("=" * 70)
    print(f" 먹스·압력센서 진단  (채널당 {knocks}회 두드림)")
    print("=" * 70)

    if talk(fd, MUX, rd=1) is None:
        print(" 먹스(0x70) 무응답 — 버스가 잠겼습니다.")
        print(" -> 전원을 완전히 내렸다 올린 뒤 다시 실행하세요 (재부팅으로는 안 풀립니다).")
        return 1
    print(" 먹스 응답 정상\n")

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
                note = "주소는 응답하나 PROM 읽기 실패"
                state = f"불안정 {hit}/{knocks}"
            elif any(w in (0x0000, 0xFFFF) for w in prom):
                note = "PROM에 0/FFFF 포함 — 통신 불량"
                state = f"불안정 {hit}/{knocks}"
            elif crc4(prom) != (prom[0] >> 12):
                note = f"PROM CRC 불일치 (계산 {crc4(prom)}, 저장 {prom[0] >> 12})"
                state = f"불안정 {hit}/{knocks}"
            else:
                ver = (prom[0] >> 5) & 0x7F
                note = f"PROM 정상 (CRC OK, 버전비트 {ver:07b})"
                state = "정상" if hit == knocks else f"접촉 불안정 {hit}/{knocks}"
            results.append((name, "정상", state, note))

        talk(fd, MUX, [0x00])

    talk(fd, MUX, [0x00])
    os.close(fd)

    print(f" {'채널':<12}{'채널 상태':<16}{'센서 상태':<18}비고")
    print(" " + "-" * 68)
    for r in results:
        print(f" {r[0]:<12}{r[1]:<16}{r[2]:<18}{r[3]}")
    print("=" * 70)

    # 해석 안내
    states = [r[2] for r in results]
    if any("불안정" in s for s in states):
        print(" 접촉 불안정이 있습니다. `bash ~/press_watch.sh` 를 띄워두고 커넥터를")
        print(" 눌러보거나 케이블을 흔들어 어느 자세에서 붙는지 찾으세요.")
    elif any(r[1] == "버스 사망" for r in results):
        print(" 버스를 죽이는 채널이 있습니다. 전원을 끄고 그 커넥터의 SDA/SCL과 GND")
        print(" 사이 통전을 확인하세요 (삐 소리가 나면 단락).")
    elif all(s == "센서 없음" for s in states):
        print(" 센서가 하나도 응답하지 않습니다. 커넥터의 VCC 전압(3.3V)과")
        print(" SDA/SCL 핀 순서를 먼저 확인하세요.")
    elif any(s == "정상" for s in states):
        print(" 정상인 채널이 있습니다. 그대로 측정에 들어가도 됩니다.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
