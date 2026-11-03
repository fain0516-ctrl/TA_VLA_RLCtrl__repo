#include "sts.h"

#include "config.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

#define ADDR_TORQUE_ENABLE    40
#define ADDR_GOAL_POSITION    42
#define ADDR_PRESENT_POSITION 56
#define BROADCAST_ID          0xFE
#define INST_PING             0x01
#define INST_READ             0x02
#define INST_WRITE            0x03
#define INST_SYNC_WRITE       0x83

static uint8_t checksum(const uint8_t *b, int n) {
    unsigned s = 0;
    for (int i = 0; i < n; i++) s += b[i];
    return (uint8_t)~s;
}

// FF FF id len instr params... sum
static void send(uint8_t id, uint8_t instr, const uint8_t *prm, int n) {
    uint8_t pkt[64];
    pkt[0] = pkt[1] = 0xFF;
    pkt[2] = id;
    pkt[3] = (uint8_t)(n + 2);
    pkt[4] = instr;
    for (int i = 0; i < n; i++) pkt[5 + i] = prm[i];
    pkt[5 + n] = checksum(pkt + 2, n + 3);
    while (uart_is_readable(STS_UART)) (void)uart_getc(STS_UART);   // 남은 바이트 버림
    uart_write_blocking(STS_UART, pkt, n + 6);
    uart_tx_wait_blocking(STS_UART);
}

// 응답 바이트를 끊길 때까지 받습니다. URT-2 가 송신 바이트를 되돌려 보내더라도(에코)
// 아래 find_reply 가 id·길이로 응답을 골라내므로 상관없습니다.
static int collect(uint8_t *b, int cap) {
    int n = 0;
    uint32_t wait = STS_TIMEOUT_US;
    while (n < cap && uart_is_readable_within_us(STS_UART, wait)) {
        b[n++] = uart_getc(STS_UART);
        wait = 200;
    }
    return n;
}

// FF FF id len err params sum 을 찾습니다. len = 파라미터 수 + 2.
static const uint8_t *find_reply(const uint8_t *b, int n, uint8_t id, int nparam) {
    int total = nparam + 6;
    for (int i = 0; i + total <= n; i++) {
        const uint8_t *p = b + i;
        if (p[0] == 0xFF && p[1] == 0xFF && p[2] == id && p[3] == nparam + 2 &&
            checksum(p + 2, nparam + 3) == p[total - 1])
            return p;
    }
    return NULL;
}

void sts_init(void) {
    uart_init(STS_UART, STS_BAUD);
    gpio_set_function(STS_TX, GPIO_FUNC_UART);
    gpio_set_function(STS_RX, GPIO_FUNC_UART);
    uart_set_format(STS_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(STS_UART, true);
}

void sts_torque(uint8_t id, bool on) {
    uint8_t prm[2] = {ADDR_TORQUE_ENABLE, on ? 1 : 0};
    uint8_t b[32];
    send(id, INST_WRITE, prm, 2);
    collect(b, sizeof b);                      // 상태 응답 버림
}

void sts_sync_goal(const uint8_t *ids, const uint16_t *raw, int n) {
    uint8_t prm[2 + 3 * 16];
    prm[0] = ADDR_GOAL_POSITION;
    prm[1] = 2;
    for (int k = 0; k < n; k++) {
        uint16_t v = raw[k] > 4095 ? 4095 : raw[k];
        prm[2 + 3 * k] = ids[k];
        prm[3 + 3 * k] = v & 0xFF;
        prm[4 + 3 * k] = v >> 8;
    }
    send(BROADCAST_ID, INST_SYNC_WRITE, prm, 2 + 3 * n);   // SYNC WRITE 는 응답 없음
}

bool sts_read_state(uint8_t id, uint16_t *pos, int16_t *load) {
    uint8_t prm[2] = {ADDR_PRESENT_POSITION, 6};
    uint8_t b[40];
    send(id, INST_READ, prm, 2);
    const uint8_t *p = find_reply(b, collect(b, sizeof b), id, 6);
    if (!p) return false;
    *pos = p[5] | (p[6] << 8);
    uint16_t ld = p[9] | (p[10] << 8);         // bit10 = 방향(부호), 하위 10 bit = 크기
    *load = (ld & 0x400) ? -(int16_t)(ld & 0x3FF) : (int16_t)(ld & 0x3FF);
    return true;
}

bool sts_ping(uint8_t id) {
    uint8_t b[32];
    send(id, INST_PING, NULL, 0);
    return find_reply(b, collect(b, sizeof b), id, 0) != NULL;
}
