#include "link.h"

static uint16_t get16(const uint8_t *p) { return p[0] | (p[1] << 8); }
static void put16(uint8_t *p, uint16_t v) { p[0] = v & 0xFF; p[1] = v >> 8; }

bool link_unpack_cmd(const uint8_t *b, size_t n, link_cmd_t *c) {
    if (n != LINK_CMD_SIZE || b[0] != 'H' || b[1] != '3' || b[2] != 1) return false;
    c->torque = b[3] & LINK_FLAG_TORQUE;
    c->seq = b[4] | (b[5] << 8) | (b[6] << 16) | ((uint32_t)b[7] << 24);
    for (int k = 0; k < 6; k++) c->sts_raw[k] = get16(b + 8 + 2 * k);
    for (int k = 0; k < 4; k++) c->pwm_us[k] = get16(b + 20 + 2 * k);
    return true;
}

void link_pack_state(uint8_t o[LINK_STATE_SIZE], uint32_t seq, bool torque, bool watchdog,
                     const uint16_t pos[6], const int16_t load[6], uint8_t err_mask) {
    o[0] = 'H';
    o[1] = '3';
    o[2] = 1;
    o[3] = (torque ? LINK_FLAG_TORQUE : 0) | (watchdog ? LINK_FLAG_WATCHDOG : 0);
    for (int i = 0; i < 4; i++) o[4 + i] = (seq >> (8 * i)) & 0xFF;
    for (int k = 0; k < 6; k++) put16(o + 8 + 2 * k, pos[k]);
    for (int k = 0; k < 6; k++) put16(o + 20 + 2 * k, (uint16_t)load[k]);
    o[32] = err_mask;
    o[33] = 0;
}
