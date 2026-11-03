// W5500 레지스터 주소와 SPI 프레임 형식은 WIZnet W5500 데이터시트 기준
// (주소 16 bit + 제어 바이트 [BSB 5 bit | R/W | OM 2 bit], 가변 길이 모드).
#include "w5500.h"

#include "config.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

// 블록 선택 (소켓 0)
#define BSB_COMMON  0x00
#define BSB_S0_REG  0x01
#define BSB_S0_TX   0x02
#define BSB_S0_RX   0x03

// 공통 레지스터
#define MR          0x0000
#define GAR         0x0001
#define SUBR        0x0005
#define SHAR        0x0009
#define SIPR        0x000F
#define RTR         0x0019
#define RCR         0x001B
#define PHYCFGR     0x002E
#define VERSIONR    0x0039

// 소켓 레지스터
#define Sn_MR       0x0000
#define Sn_CR       0x0001
#define Sn_IR       0x0002
#define Sn_SR       0x0003
#define Sn_PORT     0x0004
#define Sn_DIPR     0x000C
#define Sn_DPORT    0x0010
#define Sn_TX_FSR   0x0020
#define Sn_TX_WR    0x0024
#define Sn_RX_RSR   0x0026
#define Sn_RX_RD    0x0028

#define CR_OPEN     0x01
#define CR_CLOSE    0x10
#define CR_SEND     0x20
#define CR_RECV     0x40
#define IR_TIMEOUT  0x08
#define IR_SENDOK   0x10
#define SR_UDP      0x22
#define MR_UDP      0x02

static inline void cs(bool on) {
    asm volatile("nop \n nop \n nop");
    gpio_put(W5500_CS, !on);
    asm volatile("nop \n nop \n nop");
}

static void wr(uint16_t addr, uint8_t bsb, const uint8_t *d, size_t n) {
    uint8_t h[3] = {addr >> 8, addr & 0xFF, (uint8_t)((bsb << 3) | 0x04)};
    cs(true);
    spi_write_blocking(W5500_SPI, h, 3);
    spi_write_blocking(W5500_SPI, d, n);
    cs(false);
}

static void rd(uint16_t addr, uint8_t bsb, uint8_t *d, size_t n) {
    uint8_t h[3] = {addr >> 8, addr & 0xFF, (uint8_t)(bsb << 3)};
    cs(true);
    spi_write_blocking(W5500_SPI, h, 3);
    spi_read_blocking(W5500_SPI, 0x00, d, n);
    cs(false);
}

static void wr8(uint16_t a, uint8_t bsb, uint8_t v) { wr(a, bsb, &v, 1); }
static uint8_t rd8(uint16_t a, uint8_t bsb) { uint8_t v; rd(a, bsb, &v, 1); return v; }
static void wr16(uint16_t a, uint8_t bsb, uint16_t v) { uint8_t b[2] = {v >> 8, v & 0xFF}; wr(a, bsb, b, 2); }
static uint16_t rd16(uint16_t a, uint8_t bsb) { uint8_t b[2]; rd(a, bsb, b, 2); return (b[0] << 8) | b[1]; }

// 칩이 갱신 중인 16 bit 카운터는 두 번 같은 값이 나올 때까지 읽습니다 (데이터시트 권고).
static uint16_t rd16_stable(uint16_t a, uint8_t bsb) {
    uint16_t v1, v2 = rd16(a, bsb);
    do { v1 = v2; v2 = rd16(a, bsb); } while (v1 != v2);
    return v1;
}

static bool sock_cmd(uint8_t c) {
    wr8(Sn_CR, BSB_S0_REG, c);
    for (int i = 0; i < 1000; i++) {           // 명령 접수되면 Sn_CR 이 0 으로 돌아옴
        if (rd8(Sn_CR, BSB_S0_REG) == 0) return true;
        sleep_us(1);
    }
    return false;
}

uint8_t w5500_version(void) { return rd8(VERSIONR, BSB_COMMON); }
bool w5500_link_up(void) { return rd8(PHYCFGR, BSB_COMMON) & 0x01; }

bool w5500_init(const uint8_t mac[6], const uint8_t ip[4], const uint8_t mask[4], const uint8_t gw[4]) {
    spi_init(W5500_SPI, W5500_SPI_HZ);
    spi_set_format(W5500_SPI, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);   // SPI 모드 0
    gpio_set_function(W5500_SCK, GPIO_FUNC_SPI);
    gpio_set_function(W5500_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(W5500_MISO, GPIO_FUNC_SPI);
    gpio_init(W5500_CS);
    gpio_set_dir(W5500_CS, GPIO_OUT);
    gpio_put(W5500_CS, 1);
    gpio_init(W5500_RST);
    gpio_set_dir(W5500_RST, GPIO_OUT);

    gpio_put(W5500_RST, 0);                    // 하드웨어 리셋 (RSTn 에 RC 가 있어도 충분한 길이)
    sleep_ms(5);
    gpio_put(W5500_RST, 1);
    sleep_ms(100);                             // 내부 PLL 안정 대기

    if (w5500_version() != 0x04) return false;
    wr8(MR, BSB_COMMON, 0x80);                 // 소프트 리셋
    sleep_ms(5);
    wr(GAR, BSB_COMMON, gw, 4);
    wr(SUBR, BSB_COMMON, mask, 4);
    wr(SHAR, BSB_COMMON, mac, 6);
    wr(SIPR, BSB_COMMON, ip, 4);
    // 재전송 간격 10 ms x 2 회: 상대가 없을 때 ARP 대기로 루프가 오래 멈추지 않게 (기본 200 ms x 8)
    wr16(RTR, BSB_COMMON, 100);
    wr8(RCR, BSB_COMMON, 2);
    return true;
}

bool w5500_udp_open(uint16_t port) {
    sock_cmd(CR_CLOSE);
    wr8(Sn_MR, BSB_S0_REG, MR_UDP);
    wr16(Sn_PORT, BSB_S0_REG, port);
    if (!sock_cmd(CR_OPEN)) return false;
    for (int i = 0; i < 100; i++) {
        if (rd8(Sn_SR, BSB_S0_REG) == SR_UDP) return true;
        sleep_us(10);
    }
    return false;
}

int w5500_udp_recv(uint8_t *buf, size_t cap, uint8_t ip[4], uint16_t *port) {
    if (rd16_stable(Sn_RX_RSR, BSB_S0_REG) < 8) return -1;
    uint16_t ptr = rd16(Sn_RX_RD, BSB_S0_REG);
    uint8_t h[8];                              // UDP 헤더: 보낸 IP 4, 포트 2, 길이 2
    rd(ptr, BSB_S0_RX, h, 8);                  // 버퍼 끝 넘김은 칩이 처리 (포인터 그대로 사용)
    uint16_t len = (h[6] << 8) | h[7];
    rd((uint16_t)(ptr + 8), BSB_S0_RX, buf, len < cap ? len : cap);
    wr16(Sn_RX_RD, BSB_S0_REG, (uint16_t)(ptr + 8 + len));
    sock_cmd(CR_RECV);
    for (int i = 0; i < 4; i++) ip[i] = h[i];
    *port = (h[4] << 8) | h[5];
    return len;
}

bool w5500_udp_send(const uint8_t *buf, size_t n, const uint8_t ip[4], uint16_t port) {
    if (rd16_stable(Sn_TX_FSR, BSB_S0_REG) < n) return false;
    wr(Sn_DIPR, BSB_S0_REG, ip, 4);
    wr16(Sn_DPORT, BSB_S0_REG, port);
    uint16_t ptr = rd16(Sn_TX_WR, BSB_S0_REG);
    wr(ptr, BSB_S0_TX, buf, n);
    wr16(Sn_TX_WR, BSB_S0_REG, (uint16_t)(ptr + n));
    if (!sock_cmd(CR_SEND)) return false;
    absolute_time_t until = make_timeout_time_ms(30);   // ARP 포함 최대 약 30 ms
    while (!time_reached(until)) {
        uint8_t ir = rd8(Sn_IR, BSB_S0_REG);
        if (ir & IR_SENDOK) { wr8(Sn_IR, BSB_S0_REG, IR_SENDOK); return true; }
        if (ir & IR_TIMEOUT) { wr8(Sn_IR, BSB_S0_REG, IR_TIMEOUT); return false; }
    }
    return false;
}
