// Pi(노트북) <-> Pico 이진 패킷. ROS 쪽 pico_link.py, MicroPython 쪽 link.py 와 같은 형식 (리틀 엔디언).
// 버전 2 (2026-11-03): STS3215 5개(엄지~새끼 뿌리 관절) + DS3225 5개(엄지~새끼 텐던).
//   CMD   28 B: 'H3', ver 2, flags(bit0 토크), seq u32, STS 목표 raw u16 x5, PWM 펄스 µs u16 x5 (0 = 펄스 끔)
//   STATE 30 B: 'H3', ver 2, flags(bit0 토크, bit1 워치독), seq u32, STS 현재 raw u16 x5, 부하 i16 x5, 오류 비트, 예비
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LINK_VERSION    2
#define LINK_N_STS      5
#define LINK_N_PWM      5
#define LINK_CMD_SIZE   28
#define LINK_STATE_SIZE 30
#define LINK_FLAG_TORQUE   0x01
#define LINK_FLAG_WATCHDOG 0x02

typedef struct {
    bool torque;
    uint32_t seq;
    uint16_t sts_raw[LINK_N_STS];
    uint16_t pwm_us[LINK_N_PWM];
} link_cmd_t;

bool link_unpack_cmd(const uint8_t *b, size_t n, link_cmd_t *c);
void link_pack_state(uint8_t out[LINK_STATE_SIZE], uint32_t seq, bool torque, bool watchdog,
                     const uint16_t pos[LINK_N_STS], const int16_t load[LINK_N_STS], uint8_t err_mask);
