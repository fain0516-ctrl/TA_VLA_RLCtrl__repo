# Pico 2 서보 브릿지 펌웨어 (C, Pico SDK)

MicroPython 판(`../pico2_servo_bridge/`)과 **같은 동작, 같은 패킷**의 C 버전입니다. Pi(또는 노트북)의 `servo_bus_node backend:=pico` 와 그대로 통신합니다.
W5500 은 WIZnet ioLibrary 없이 레지스터를 직접 다루는 작은 드라이버(`src/w5500.c`, 소켓 0 UDP 하나)를 씁니다.

```
노트북/Pi ──UDP 5556 지령──▶ W5500 ─SPI0 10 MHz─ Pico 2 ─UART0 1 Mbps─▶ ADuM1201 ─▶ URT-2 ─▶ STS3215 x5 (ID 1~5, 엄지~새끼 뿌리 관절)
          ◀─UDP 5555 상태───                            └─PWM 50 Hz─▶ SN74HCT125N ──────────▶ DS3225 검지~새끼 (U4~U7)
                                                         └─PWM 50 Hz─▶ (GP6 직결, 가정) ────▶ DS3225 엄지
```

## 바로 올리기 (빌드 불필요)

1. Pico 2 의 **BOOTSEL** 버튼을 누른 채 USB 를 노트북에 꽂습니다 → `RP2350` 드라이브가 뜹니다.
2. `prebuilt/pico2_servo_bridge.uf2` 를 그 드라이브에 끌어다 놓습니다. 자동으로 재부팅되며 실행됩니다.
3. 상태 확인: USB 시리얼 포트(Windows 장치 관리자의 COMx, 속도 값 무관)를 PuTTY·Arduino 시리얼 모니터 등으로 열면 1 초마다 출력됩니다.
   ```
   [sts] ID 1: 응답 ...   (ID 1~5 = 엄지~새끼 뿌리 관절)
   [net] W5500 OK, IP 192.168.10.20, UDP 5556, MAC 02:..
   [run] 지령 100 Hz, 잘못된 패킷 0, 토크 OFF, 워치독 정상, STS 오류 0x00, 위치 2048 ...
   ```
   `[net] W5500 응답 없음 (VERSIONR=0x00 또는 0xFF)` 이면 SPI 배선 또는 MISO(GP16) 넷의 C1 을 확인하세요.

`prebuilt/` 의 파일은 `src/config.h` 기본값(보드 IP 192.168.10.20, 핀은 2026-10-22 회로도 기준)으로 빌드한 것입니다. 값을 바꾸면 아래처럼 다시 빌드하세요.

## 노트북과 연결해 시험

1. 노트북 랜 포트에 고정 IP 를 줍니다. Windows: 설정 → 네트워크 → 이더넷 → IP 할당 "수동" → IPv4 `192.168.10.10`, 서브넷 `255.255.255.0` (게이트웨이 비워 둠).
2. 보드와 랜선으로 직결 → `ping 192.168.10.20` 으로 응답 확인.
3. `tools/pc_test.py` 실행 (Python 3, 추가 설치 없음). 처음 실행 시 Windows 방화벽이 묻으면 **허용** (UDP 5555 수신).
   ```
   python tools/pc_test.py monitor                    # 토크 끔, 위치만 읽기 (먼저 이것으로 통신 확인)
   python tools/pc_test.py hold                       # 현재 위치에서 STS 토크 켜고 유지
   python tools/pc_test.py sweep --servo 1 --amp 100  # STS 순번 1(검지, ID 2) 를 ±100 raw(약 ±8.8°) 사인파
   python tools/pc_test.py pwm --ch 1 --us 1500       # DS3225 채널 1(검지, GP2, U4) 에 1500 µs. 채널 0 = 엄지(GP6)
   ```
   Ctrl+C 로 끝내면 토크 끔 지령을 보냅니다. 텐던 서보는 걸린 상태에서 범위 끝 펄스를 주지 않도록 조심하세요.

## 엄지 텐던 출력 (보드에 없음)

2조 구조로 바꾸면서 DS3225 가 5개(엄지 추가)가 되었지만, 보드의 SN74HCT125N 버퍼는 4개(U4~U7)뿐입니다.
펌웨어는 엄지 텐던을 **GP6 에서 직접** 내보낸다고 가정합니다 (`config.h` `PWM_PINS` 첫 값).
GP6 가 보드에서 비어 있는지, DS3225 가 3.3 V 신호로 동작하는지 확인하고, 다른 핀이면 `PWM_PINS` 를 바꿔 다시 빌드하세요.

## 다시 빌드 (설정을 바꿨을 때)

필요: CMake, Ninja(또는 make), `arm-none-eabi-gcc`, Pico SDK 2.x. 가장 쉬운 방법은 VS Code 의 "Raspberry Pi Pico" 확장으로 이 폴더를 여는 것입니다.
명령줄(Linux/WSL):
```bash
git clone --depth 1 -b 2.1.1 https://github.com/raspberrypi/pico-sdk.git ~/pico-sdk
(cd ~/pico-sdk && git submodule update --init --depth 1 lib/tinyusb)
export PICO_SDK_PATH=~/pico-sdk
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
ninja -C build          # -> build/pico2_servo_bridge.uf2
```

## 파일

| 파일 | 내용 |
|---|---|
| `src/config.h` | IP, 핀, STS ID, PWM 범위, 워치독. **[확인 필요] 값은 보드에 맞춰 수정** |
| `src/w5500.c/.h` | W5500 최소 드라이버 (리셋, IP 설정, UDP 송수신) |
| `src/sts.c/.h` | STS3215 패킷: 토크 40, 목표 위치 42 SYNC WRITE, 현재 위치 56 읽기, 핑. URT-2 가 송신 바이트를 되돌려 보내도 응답만 골라냄 |
| `src/link.c/.h` | Pi↔Pico 이진 패킷 (ROS `pico_link.py` 와 같은지 `ros2_ws` 테스트가 호스트 gcc 로 확인) |
| `src/main.c` | 100 Hz 루프: 지령 수신 → SYNC WRITE + PWM → 위치·부하 읽기 → 상태 회신, USB 시리얼 로그 |
| `tools/pc_test.py` | 노트북용 시험 도구 (ROS 없이) |

## MicroPython 판과 다른 점

- PWM 지령 0 µs 는 그 채널 펄스 끔 (시험 도구가 사용). ROS 쪽은 0 을 보내지 않으므로 동작은 같습니다.
- W5500 재전송을 10 ms x 2 회로 줄여, 상대가 없을 때 루프가 오래 멈추지 않게 했습니다.
- 부팅 시 STS ID 1~6 핑 결과와 W5500 버전 확인 결과를 USB 시리얼로 출력합니다.

## 안전

- 지령이 100 ms 넘게 없으면 상태 플래그 bit1(워치독)을 세웁니다. `WATCHDOG_TORQUE_OFF 1` 이면 STS 토크 해제 + PWM 정지 (기본 0: 마지막 자세 유지).
- 메인 루프가 1 s 넘게 멈추면 하드웨어 WDT 가 재부팅합니다.

이 펌웨어는 Pico SDK 2.1.1 로 빌드까지만 확인했고, 실제 보드에서는 아직 돌려보지 않았습니다.
