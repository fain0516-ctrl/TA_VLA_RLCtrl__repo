# -*- coding: utf-8 -*-
"""Pico 2 자작 보드 설정. [확인 필요] 표시는 보드 회로도에 맞춰 바꿔야 하는 값입니다."""

# ---- 이더넷 (W5500, SPI0) ----------------------------------------------------
PICO_IP = "192.168.10.20"          # [확인 필요] ROS 파라미터 servo_bus_node pico.ip 와 같게
NETMASK = "255.255.255.0"
GATEWAY = "192.168.10.1"
DNS = "192.168.10.1"
CMD_PORT = 5556                    # Pi -> Pico 지령
STATE_PORT = 5555                  # Pico -> Pi 상태 (지령을 보낸 주소로 응답)
W5500_SPI = 0
W5500_SCK, W5500_MOSI, W5500_MISO = 18, 19, 16   # [확인 필요] (WIZnet W5500-EVB-Pico2 배치 기준)
W5500_CS, W5500_RST = 17, 20                     # [확인 필요]
SPI_BAUD = 20_000_000

# ---- STS3215 반이중 버스 (UART + SN74125N) -----------------------------------
STS_UART = 0
STS_TX, STS_RX = 0, 1              # [확인 필요]
STS_BAUD = 1_000_000
# SN74125N 의 OE 는 active-low. TX 버퍼 OE, RX 버퍼 OE 를 각각 GPIO 로 제어.
# RX 버퍼 OE 를 TX 쪽과 반전 회로로 묶었거나 GND 고정이면 RX_EN_N_PIN = None.
TX_EN_N_PIN = 2                    # [확인 필요]
RX_EN_N_PIN = 3                    # [확인 필요]
STS_IDS = (1, 2, 3, 4, 5, 6)       # 엄지 0-0, 0-1, 검지~새끼 MCP (ROS sts.ids 와 같은 순서)
READ_TIMEOUT_MS = 1

# ---- DS3225 텐던 서보 PWM ------------------------------------------------------
PWM_PINS = (6, 7, 8, 9)            # [확인 필요] 검지, 중지, 약지, 새끼
PWM_FREQ = 50
PWM_MIN_US, PWM_MAX_US = 500, 2500 # 범위 밖 지령은 절삭 [추정: DS3225 사양 확인]

# ---- 안전 ---------------------------------------------------------------------
LOOP_HZ = 100
WATCHDOG_MS = 100                  # 지령 미수신 한계 [인터페이스 문서 IF-04]
WATCHDOG_ACTION = "hold"           # "hold": 마지막 자세 유지 / "torque_off": STS 토크 해제 + PWM 정지
HW_WDT_MS = 1000                   # 루프가 멈추면 Pico 재부팅
