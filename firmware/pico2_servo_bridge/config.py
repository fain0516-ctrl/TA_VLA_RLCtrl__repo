# -*- coding: utf-8 -*-
"""Pico 2 자작 보드 설정. [확인 필요] 표시는 보드 회로도에 맞춰 바꿔야 하는 값입니다."""

# ---- 이더넷 (W5500, SPI0) ----------------------------------------------------
PICO_IP = "192.168.10.20"          # [확인 필요] ROS 파라미터 servo_bus_node pico.ip 와 같게
NETMASK = "255.255.255.0"
GATEWAY = "192.168.10.1"
DNS = "192.168.10.1"
CMD_PORT = 5556                    # Pi -> Pico 지령
STATE_PORT = 5555                  # Pico -> Pi 상태 (지령을 보낸 주소로 응답)
# 2026-10-22 회로도 캡처에서 읽은 배선 (J1 "W5500R" 아래 / J2 "W5500L" 위, 각 6핀, 1번 = GND 쪽, 실물은 위아래 반전 결선):
#   J1: 1 GND, 2 GND, 3 GP19, 4 GP18, 5 GP17, 6 GP20 (10 kΩ 풀업)
#   J2: 1 GND, 2 GND, 3 3V3, 4 NC, 5 GP21, 6 GP16 (10 kΩ 풀업 + 100 nF 접지 = 리셋용 RC 형태)
# GP18/19/17 은 RP2350 SPI0 의 SCK/TX(MOSI)/CSn 자리와 맞습니다.
# MISO 와 RSTn 이 GP16/GP21 중 어느 쪽인지는 W5500S 모듈 핀 표기로 확인해야 합니다 [확인 필요].
W5500_SPI = 0
W5500_SCK, W5500_MOSI = 18, 19
W5500_CS = 17
W5500_INT = 20                     # 드라이버는 INT 를 쓰지 않음 (참고용)
W5500_MISO, W5500_RST = 16, 21     # [확인 필요] 반대(MISO=21, RST=16)라면 숫자를 바꾸고 SOFT_SPI = True
# GP21 은 SPI0 RX(MISO) 로 쓸 수 없는 핀이라, MISO 가 GP21 이면 소프트웨어 SPI 로 돌립니다 (느려짐).
SOFT_SPI = False
SPI_BAUD = 20_000_000

# ---- STS3215 버스 (UART) --------------------------------------------------------
# 회로도: GP0/GP1 -> ADuM1201 디지털 아이솔레이터(U9) -> 4핀 헤더 H1 (AGND 쪽) -> URT-2.
STS_UART = 0
STS_TX, STS_RX = 0, 1              # GP0 = UART0 TX, GP1 = UART0 RX
STS_BAUD = 1_000_000               # ADuM1201 B 등급 데이터율(10 Mbps) 이내로 알고 있음 [확인 필요: 데이터시트]
# URT-2 가 반이중 방향 전환을 자동 처리하므로 방향 제어 핀은 쓰지 않습니다 (None).
# SN74125N 자작 반이중 회로를 쓸 때만 OE(active-low) GPIO 번호를 넣으세요.
TX_EN_N_PIN = None
RX_EN_N_PIN = None
STS_IDS = (1, 2, 3, 4, 5, 6)       # 엄지 0-0, 0-1, 검지~새끼 MCP (ROS sts.ids 와 같은 순서)
READ_TIMEOUT_MS = 1

# ---- DS3225 텐던 서보 PWM ------------------------------------------------------
# 회로도: GP2->1A->1Y->U4, GP3->2A->2Y->U5, GP4->3A->3Y->U6, GP5->4A->4Y->U7
# (SN74HCT125N 버퍼, OE# 4개 모두 GND = 항상 출력, 서보 신호를 VCC 레벨로 올림)
PWM_PINS = (2, 3, 4, 5)            # 검지, 중지, 약지, 새끼 순서 [확인 필요: U4~U7 에 어느 손가락을 꽂는지]
PWM_FREQ = 50
PWM_MIN_US, PWM_MAX_US = 500, 2500 # 범위 밖 지령은 절삭 [추정: DS3225 사양 확인]

# ---- 안전 ---------------------------------------------------------------------
LOOP_HZ = 100
WATCHDOG_MS = 100                  # 지령 미수신 한계 [인터페이스 문서 IF-04]
WATCHDOG_ACTION = "hold"           # "hold": 마지막 자세 유지 / "torque_off": STS 토크 해제 + PWM 정지
HW_WDT_MS = 1000                   # 루프가 멈추면 Pico 재부팅
