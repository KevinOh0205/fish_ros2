#!/bin/bash
# 압력 센서 접촉 상태 실시간 감시.
#
# 사용법:  bash ~/press_watch.sh          (Ctrl+C 로 종료)
#
# 켜둔 채로 커넥터를 눌러보거나 케이블을 살살 흔들어 보세요.
# 응답이 들어오는 순간 해당 채널이 ● 로 바뀝니다. 그 자세가 접촉되는 자세입니다.
#
# ※ 센서 탐지는 반드시 "쓰기"(리셋 0x1E)로 합니다. MS5837은 선행 명령 없이
#   읽으면 응답하지 않아, i2cdetect/i2cget 으로는 멀쩡한 센서도 못 찾습니다.
set -u
BUS=1; MUX=0x70; SENS=0x76; N=20      # N = 최근 몇 회를 막대로 보여줄지

declare -a hist0 hist1 hist2
declare -a tot=(0 0 0) ok=(0 0 0)
t0=$(date +%s)

cleanup() { timeout 2 i2cset -y $BUS $MUX 0x00 >/dev/null 2>&1; echo; echo "종료."; exit 0; }
trap cleanup INT TERM

# 먹스가 살아있는지 먼저
if ! timeout 3 i2cget -y $BUS $MUX >/dev/null 2>&1; then
    echo "먹스(0x70) 무응답 — 버스가 잠겼습니다. 전원을 완전히 내렸다 올리세요."
    exit 1
fi

echo "=== 압력 센서 접촉 감시 ==="
echo "커넥터를 눌러보거나 케이블을 살살 흔들어 보세요. ● = 응답, · = 무응답"
echo "Ctrl+C 로 종료합니다."
echo

while true; do
    for ch in 0 1 2; do
        timeout 2 i2cset -y $BUS $MUX $((1 << ch)) >/dev/null 2>&1
        if timeout 2 i2cset -y $BUS $SENS 0x1E >/dev/null 2>&1; then
            r="●"; ok[$ch]=$(( ok[ch] + 1 ))
        else
            r="·"
        fi
        tot[$ch]=$(( tot[ch] + 1 ))
        eval "hist$ch+=( \"\$r\" )"
        eval "len=\${#hist$ch[@]}"
        if [ "$len" -gt "$N" ]; then eval "hist$ch=( \"\${hist$ch[@]:1}\" )"; fi
    done
    timeout 2 i2cset -y $BUS $MUX 0x00 >/dev/null 2>&1

    line=""
    for ch in 0 1 2; do
        eval "bar=\$(printf '%s' \"\${hist$ch[@]}\")"
        pct=$(( ok[ch] * 100 / (tot[ch] > 0 ? tot[ch] : 1) ))
        line+=$(printf "  J%d:%-20s%3d%%" "$((ch+5))" "$bar" "$pct")
    done
    printf "\r[%4d초]%s " "$(( $(date +%s) - t0 ))" "$line"
done
