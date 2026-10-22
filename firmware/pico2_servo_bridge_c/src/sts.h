// STS3215 버스 (Feetech STS 프로토콜). 레지스터: 토크 40, 목표 위치 42, 현재 위치 56 (위치·속도·부하 각 2 B).
// ROS 쪽 sts_protocol.py, MicroPython 쪽 sts.py 와 같은 패킷입니다.
#pragma once
#include <stdbool.h>
#include <stdint.h>

void sts_init(void);
void sts_torque(uint8_t id, bool on);
void sts_sync_goal(const uint8_t *ids, const uint16_t *raw, int n);
// 현재 위치(0~4095)와 부하(부호 포함, 0.1 % 단위). 응답이 없거나 깨지면 false.
bool sts_read_state(uint8_t id, uint16_t *pos, int16_t *load);
// 핑 (부팅 시 연결 확인용)
bool sts_ping(uint8_t id);
