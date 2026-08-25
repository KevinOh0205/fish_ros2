# config/ — 지우면 안 되는 파일

`log_csv/`는 **버려도 되는 로그**가 쌓이는 곳이다. 예전에는 보정 파일도 거기 같이 있었는데,
2026-08-25 로그 정리 때 `mag_calib_params.txt`가 같이 사라져 **EKF가 소스 기본값으로
돌고 있었다**(복각 11.1° — 물리적으로 불가능한 값). 그래서 여기로 분리했다.

| 파일 | 없으면 | git |
|---|---|---|
| `mag_calib_params.txt` | 방위(yaw)가 사실상 무의미해진다. roll/pitch는 정상 | **추적** |
| `port_map.txt` | 어느 압력 센서가 앞/좌/우인지 소프트웨어가 모른다 | **추적** |
| `hydro_zero.txt` | 대기압·q 영점을 다시 잡아야 한다 (수심 180초, 속도는 그전까지 미보정) | 무시 |

**셋 다 조용히 실패한다** — WARN 한 줄 찍고 기본값으로 계속 돈다. 그래서 앞의 둘은
일부러 git으로 추적한다. 실수로 지워도 이렇게 되살릴 수 있다:

```bash
git checkout -- config/mag_calib_params.txt config/port_map.txt
```

`hydro_zero.txt`만 추적하지 않는다 — 영점을 잡을 때마다 바뀌어서 커밋 잡음이 된다.
잃어버려도 다시 잡으면 되고, 그게 정상 절차다.

## 누가 읽고 쓰나

| 파일 | 읽는 곳 | 쓰는 곳 |
|---|---|---|
| `mag_calib_params.txt` | `state_estimation_ekf_node` (기동 시 + 보정 성공 시 자동 리로드) | `magneto_cal.py` **단독** (백업·8 % 품질 게이트 내장) |
| `port_map.txt` | `i2c_driver_node`(지문 대조), `hydro_estimator_node`(역할↔채널) | 사람 |
| `hydro_zero.txt` | `hydro_estimator_node` (기동 시, 하루 이내면 적재) | `hydro_estimator_node` (영점 포착 성공 시) |
