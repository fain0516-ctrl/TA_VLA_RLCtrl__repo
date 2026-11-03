// Pico 2 자작 보드 설정 (C 펌웨어). MicroPython 판 firmware/pico2_servo_bridge/config.py 와 같은 값입니다.
// [확인 필요] 표시는 보드나 네트워크에 맞춰 바꿔야 하는 값입니다. 바꾼 뒤 다시 빌드하세요.
#pragma once

// ---- 이더넷 (W5500, SPI0) ----------------------------------------------------
#define PICO_IP        {192, 168, 10, 20}   // [확인 필요] ROS 파라미터 servo_bus_node pico.ip 와 같게
#define NETMASK        {255, 255, 255, 0}
#define GATEWAY        {192, 168, 10, 1}
#define CMD_PORT       5556                 // Pi(노트북) -> Pico 지령
#define STATE_PORT     5555                 // Pico -> 지령을 보낸 주소의 이 포트로 상태 회신

// 회로도 (2026-10-22, 사용자 확인): J1 SCK GP18 / MOSI GP19 / CS GP17 / INT GP20, J2-6 MISO GP16, J2-5 RSTn GP21
// GP16(MISO) 넷의 C1 100 nF 는 떼거나 RSTn 넷으로 옮길 것 (그대로면 SPI 가 동작하지 않음).
#define W5500_SPI      spi0
#define W5500_SCK      18
#define W5500_MOSI     19
#define W5500_MISO     16
#define W5500_CS       17
#define W5500_RST      21
#define W5500_INT      20                   // 사용하지 않음 (폴링)
#define W5500_SPI_HZ   (10 * 1000 * 1000)   // 10 MHz. 배선이 길면 낮추세요

// ---- STS3215 버스 (UART0 -> ADuM1201 -> H1 -> URT-2) ----------------------------
#define STS_UART       uart0
#define STS_TX         0
#define STS_RX         1
#define STS_BAUD       1000000
#define STS_N          5
#define STS_IDS        {1, 2, 3, 4, 5}      // 엄지~새끼 뿌리 관절 (ROS sts.ids 와 같은 순서) [확인 필요: 실제 ID]
#define STS_TIMEOUT_US 1500                 // 응답 대기 (첫 바이트), 이후 바이트 간 200 µs

// ---- DS3225 텐던 서보 PWM (엄지~새끼 5개) -----------------------------------------
// 검지~새끼: GP2~5 -> SN74HCT125N -> U4~U7 [확인 필요: U4~U7 손가락 순서]
// 엄지: 보드에 다섯 번째 출력이 없습니다 (SN74HCT125N 버퍼 4개). 빈 GPIO 에서 직접 뽑는다고 가정 [확인 필요: 핀, 레벨 변환]
#define PWM_N          5
#define PWM_PINS       {6, 2, 3, 4, 5}      // 엄지 GP6(가정), 검지, 중지, 약지, 새끼
#define PWM_MIN_US     500
#define PWM_MAX_US     2500                 // 범위 밖 지령은 절삭 [추정: DS3225 사양 확인]

// ---- 안전 ---------------------------------------------------------------------
#define LOOP_HZ        100
#define WATCHDOG_MS    100                  // 지령 미수신 한계 [인터페이스 문서 IF-04]
#define WATCHDOG_TORQUE_OFF 0               // 0: 마지막 자세 유지 / 1: STS 토크 해제 + PWM 정지
#define HW_WDT_MS      1000                 // 루프가 멈추면 Pico 재부팅
