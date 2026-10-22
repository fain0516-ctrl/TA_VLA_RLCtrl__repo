// W5500 최소 드라이버: 소켓 0 하나를 UDP 로 씁니다 (WIZnet ioLibrary 없이 레지스터 직접 접근).
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// 리셋·초기화. 칩 버전 레지스터(0x04)를 못 읽으면 false (SPI 배선, C1 확인).
bool w5500_init(const uint8_t mac[6], const uint8_t ip[4], const uint8_t mask[4], const uint8_t gw[4]);
uint8_t w5500_version(void);
bool w5500_link_up(void);
bool w5500_udp_open(uint16_t port);
// 받은 데이터그램 하나를 꺼냅니다. 없으면 -1. buf 보다 길면 잘라서 복사하고 원래 길이를 돌려줍니다.
int w5500_udp_recv(uint8_t *buf, size_t cap, uint8_t ip[4], uint16_t *port);
bool w5500_udp_send(const uint8_t *buf, size_t n, const uint8_t ip[4], uint16_t port);
