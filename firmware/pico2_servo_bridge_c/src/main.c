// Pico 2 자작 보드 펌웨어 (C, Pico SDK): 지령(UDP) -> STS3215 버스 + DS3225 PWM, 상태 회신.
// MicroPython 판(firmware/pico2_servo_bridge/main.py)과 같은 동작·같은 패킷입니다.
//
//   Pi/노트북 --UDP 5556--> W5500 --SPI0--> Pico 2 --UART0 1 Mbps--> ADuM1201 --> URT-2 --> STS3215 x5
//                                                  --PWM 50 Hz--> SN74HCT125N ---------------> DS3225 x5
//   Pi/노트북 <--UDP 5555-- (지령 1개당 상태 1개)
// 상태 메시지는 USB 시리얼(115200 아무 값이나)로 1 초마다 출력합니다.
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/watchdog.h"
#include "link.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "sts.h"
#include "w5500.h"

_Static_assert(STS_N == LINK_N_STS && PWM_N == LINK_N_PWM, "config.h 서보 수가 패킷(link.h)과 다릅니다");

static const uint8_t ids[STS_N] = STS_IDS;
static const uint pwm_pins[PWM_N] = PWM_PINS;

// ---------------------------------------------------------------- DS3225 PWM (50 Hz, 1 µs 분해능)
static void pwm_setup(void) {
    float div = (float)clock_get_hz(clk_sys) / 1e6f;    // 1 tick = 1 µs
    for (int k = 0; k < PWM_N; k++) {
        gpio_set_function(pwm_pins[k], GPIO_FUNC_PWM);
        uint sl = pwm_gpio_to_slice_num(pwm_pins[k]);
        pwm_set_clkdiv(sl, div);
        pwm_set_wrap(sl, 20000 - 1);                    // 20 ms 주기
        pwm_set_gpio_level(pwm_pins[k], 0);             // 펄스 없음 -> 서보 무부하
        pwm_set_enabled(sl, true);
    }
}

// 0 µs 는 그 채널 펄스 끔 (노트북 시험에서 텐던 서보를 쉬게 할 때). 그 외는 범위로 절삭.
static void pwm_set_us(const uint16_t *us) {
    for (int k = 0; k < PWM_N; k++) {
        uint16_t v = us[k] == 0 ? 0 : us[k] < PWM_MIN_US ? PWM_MIN_US : us[k] > PWM_MAX_US ? PWM_MAX_US : us[k];
        pwm_set_gpio_level(pwm_pins[k], v);
    }
}

static void pwm_off(void) {
    for (int k = 0; k < PWM_N; k++) pwm_set_gpio_level(pwm_pins[k], 0);
}

static void set_torque(bool on) {
    for (int k = 0; k < STS_N; k++) sts_torque(ids[k], on);
    if (!on) pwm_off();
}

// ---------------------------------------------------------------- 네트워크
static bool net_up(void) {
    static const uint8_t ip[4] = PICO_IP, mask[4] = NETMASK, gw[4] = GATEWAY;
    pico_unique_board_id_t uid;
    pico_get_unique_board_id(&uid);
    // 로컬 관리 MAC (02:xx...): 보드 고유 ID 에서 만들어 보드마다 다르게
    uint8_t mac[6] = {0x02, uid.id[3], uid.id[4], uid.id[5], uid.id[6], uid.id[7]};
    if (!w5500_init(mac, ip, mask, gw)) {
        printf("[net] W5500 응답 없음 (VERSIONR=0x%02X, 정상 0x04). SPI 배선과 MISO 넷 C1 을 확인하세요\n",
               w5500_version());
        return false;
    }
    if (!w5500_udp_open(CMD_PORT)) {
        printf("[net] UDP 소켓 열기 실패\n");
        return false;
    }
    printf("[net] W5500 OK, IP %d.%d.%d.%d, UDP %d, MAC %02X:%02X:%02X:%02X:%02X:%02X\n",
           ip[0], ip[1], ip[2], ip[3], CMD_PORT, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return true;
}

// ---------------------------------------------------------------- 메인 루프
int main(void) {
    stdio_init_all();
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    sts_init();
    pwm_setup();
    sleep_ms(1500);                                     // USB 시리얼 연결 여유
    if (watchdog_caused_reboot()) printf("[boot] 하드웨어 WDT 재부팅\n");
    printf("[boot] pico2_servo_bridge (C)\n");

    for (int k = 0; k < STS_N; k++)
        printf("[sts] ID %d: %s\n", ids[k], sts_ping(ids[k]) ? "응답" : "응답 없음");

    watchdog_enable(HW_WDT_MS, true);
    bool net = net_up();

    uint16_t pos[STS_N];
    int16_t load[STS_N] = {0};
    for (int k = 0; k < STS_N; k++) pos[k] = 2048;
    bool torque_on = false, have_cmd = false, have_peer = false;
    absolute_time_t last_cmd = get_absolute_time();
    uint32_t seq = 0;
    uint8_t peer[4];
    uint32_t n_cmd = 0, n_bad = 0;
    uint8_t err = 0;
    absolute_time_t next = get_absolute_time(), next_log = make_timeout_time_ms(1000);
    bool was_link = false;

    while (true) {
        watchdog_update();
        next = delayed_by_us(next, 1000000 / LOOP_HZ);

        if (!net) {                                     // W5500 없으면 1 초마다 다시 시도
            sleep_ms(1000);
            watchdog_update();
            net = net_up();
            continue;
        }

        // 1) 쌓인 지령 중 가장 최근 것만 적용
        uint8_t buf[64], ip[4];
        uint16_t port;
        link_cmd_t c, cmd;
        bool got = false;
        int n;
        while ((n = w5500_udp_recv(buf, sizeof buf, ip, &port)) >= 0) {
            if (link_unpack_cmd(buf, (size_t)n, &c)) {
                cmd = c;
                got = true;
                memcpy(peer, ip, 4);
                have_peer = true;
            } else {
                n_bad++;
            }
        }
        if (got) {
            n_cmd++;
            have_cmd = true;
            last_cmd = get_absolute_time();
            seq = cmd.seq;
            if (cmd.torque != torque_on) {
                set_torque(cmd.torque);
                torque_on = cmd.torque;
            }
            if (torque_on) {
                sts_sync_goal(ids, cmd.sts_raw, STS_N);
                pwm_set_us(cmd.pwm_us);
            }
        }

        // 2) 지령 워치독
        bool tripped = !have_cmd || absolute_time_diff_us(last_cmd, get_absolute_time()) > WATCHDOG_MS * 1000;
        if (tripped && torque_on && WATCHDOG_TORQUE_OFF) {
            set_torque(false);
            torque_on = false;
        }

        // 3) 상태 읽기·회신
        err = 0;
        for (int k = 0; k < STS_N; k++)
            if (!sts_read_state(ids[k], &pos[k], &load[k])) err |= 1 << k;
        if (have_peer && got) {
            uint8_t st[LINK_STATE_SIZE];
            link_pack_state(st, seq, torque_on, tripped, pos, load, err);
            w5500_udp_send(st, sizeof st, peer, STATE_PORT);
        }

        // 4) 1 초마다 USB 시리얼 상태 출력, LED 깜빡임
        if (time_reached(next_log)) {
            next_log = make_timeout_time_ms(1000);
            bool link = w5500_link_up();
            if (link != was_link) printf("[net] 랜선 링크 %s\n", link ? "연결" : "끊김");
            was_link = link;
            printf("[run] 지령 %lu Hz, 잘못된 패킷 %lu, 토크 %s, 워치독 %s, STS 오류 0x%02X, 위치",
                   (unsigned long)n_cmd, (unsigned long)n_bad, torque_on ? "ON" : "OFF",
                   tripped ? "동작" : "정상", err);
            for (int k = 0; k < STS_N; k++) printf(" %u", pos[k]);
            printf("\n");
            n_cmd = n_bad = 0;
            gpio_xor_mask(1u << PICO_DEFAULT_LED_PIN);
        }

        sleep_until(next);
    }
}
