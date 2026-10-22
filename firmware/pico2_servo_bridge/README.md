# Pico 2 서보 브릿지 펌웨어 (MicroPython)

Raspberry Pi 5 는 ROS 2 연산·통신만 맡고, **SN74125N + Pico 2 자작 보드**가 STS3215 버스와 DS3225 서보를 구동합니다.
Pi 와 Pico 는 **W5500** 이더넷(UDP)으로 연결합니다. Pi 쪽은 `servo_bus_node` 의 `backend:=pico` 입니다.

```
Pi 5 (ROS 2) ──UDP 5556 지령──▶ W5500 ─SPI─ Pico 2 ─UART 1 Mbps / SN74125N─▶ STS3215 x6 (ID 1~6)
            ◀─UDP 5555 상태───                      └─PWM 50 Hz─────────────▶ DS3225 x4 (텐던)
```

## 파일

| 파일 | 내용 |
|---|---|
| `config.py` | IP, 핀 번호, 서보 ID, 워치독. **[확인 필요] 값은 보드 회로도에 맞춰 수정** |
| `link.py` | Pi↔Pico 이진 패킷 (ROS 쪽 `pico_link.py` 와 동일, 테스트로 확인) |
| `sts.py` | STS3215 패킷 (토크 40, 목표 위치 42, 현재 위치 56) |
| `main.py` | 100 Hz 루프: 지령 수신 → SYNC WRITE + PWM → 위치·부하 읽기 → 상태 회신 |

## 패킷

- 지령 28 B `"<2sBBI6H4H"`: `'H3'`, 버전 1, 플래그(bit0 토크), seq, STS 목표 raw x6 (0~4095), PWM 펄스 µs x4
- 상태 34 B `"<2sBBI6H6hBB"`: `'H3'`, 버전 1, 플래그(bit0 토크, bit1 워치독), 마지막 seq, STS 현재 raw x6, 부하 x6, 통신 오류 비트(bit k = k번째 서보)

## 안전

- 지령이 100 ms 넘게 없으면 상태 플래그 bit1 을 세우고, `WATCHDOG_ACTION="torque_off"` 이면 STS 토크 해제 + PWM 정지 (기본은 `hold`).
- 메인 루프가 1 s 넘게 멈추면 하드웨어 WDT 가 Pico 를 재부팅합니다.
- 반이중 버스는 송신 중 RX 버퍼를, 수신 중 TX 버퍼를 닫습니다 (SN74125N OE active-low).

## 올리기

1. WIZNET5K 가 포함된 MicroPython 빌드(예: W5500-EVB-Pico2 용)를 Pico 2 에 굽습니다.
2. `config.py` 의 IP·핀을 보드에 맞게 고치고, `mpremote cp config.py link.py sts.py main.py :` 로 복사합니다.
3. Pi 에서 `ros2 launch hand_motion_control hardware_bringup.launch.py` (기본 `servo_backend:=pico`), `hand_params.yaml` 의 `pico.ip` 를 같은 주소로.

이 펌웨어는 실제 보드에서 돌려보지 않았습니다. `link.py`, `sts.py` 의 패킷 형식만 CPython 테스트로 확인했습니다.
