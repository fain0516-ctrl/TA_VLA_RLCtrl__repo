# 리버스.ENG 골격 (v0.1.0)

기성 기기를 **읽기 전용·비파괴**로 분석해 프로필(JSON)로 만들고, MQTT로 노트북에 전달하는 골격입니다. 첫 대상은 Raspberry Pi 5 (BCM2712 + RP1).

## 구성

| 경로 | 내용 |
|---|---|
| `src/reverseeng/schema/device-profile.schema.json` | 프로필 JSON 스키마 (v0.1.0, draft 2020-12) |
| `src/reverseeng/source.py` | 읽기 전용 접근 계층: 파일 읽기 + 읽기 전용 명령 허용목록만 |
| `src/reverseeng/collectors/` | 보드·칩셋·부트로더·커널·스토리지·버스·핀아웃·부팅설정·보안진단 수집기 |
| `src/reverseeng/publisher.py` | 파이 측: 프로필(retained) 발행 + 상태/텔레메트리 스트리밍 |
| `src/reverseeng/subscriber.py` | 노트북 측: 구독 + 스키마 검증 + 요약 출력 |

## 사용법

```bash
pip install -e .            # 파이와 노트북 모두
# 파이: 프로필만 뽑기
reverseeng collect -o profile.json
# 파이: 오프라인 모드 (다른 기기에 마운트한 SD 카드 루트)
reverseeng collect --root /mnt/sdroot -o profile.json
# 파이: 브로커로 발행 (프로필 1회 retained, 이후 5초마다 텔레메트리)
reverseeng publish --broker <브로커 주소>
# 노트북: 구독
reverseeng subscribe --broker <브로커 주소>
```

브로커는 mosquitto를 기본으로 가정합니다 (파이 또는 노트북 어느 쪽에 띄워도 됨).

## MQTT 토픽

`reverseeng/v1/<device_id>/` 아래:

- `profile`: retained, QoS 1. 전체 프로필. 노트북이 나중에 붙어도 바로 받음.
- `status`: retained, QoS 1. `{"state":"online"|"offline"}`. 비정상 종료 시 Last Will로 offline.
- `telemetry`: QoS 0. CPU 온도, 부하, 가용 메모리, 업타임, 스로틀 상태.

TCP 소켓은 파이↔피코 제어용으로 예약돼 있어 모니터링에는 쓰지 않습니다.

## 수집 모드

- `running` (기본): 동작 중인 기기에서 `/proc`, `/sys`, 디바이스트리, 허용된 읽기 전용 명령(`pinctrl get`, `vcgencmd bootloader_version`, `rpi-eeprom-config`, `vcgencmd get_throttled`)을 읽음.
- `offline`: 꺼진 기기의 저장장치를 `--root`로 읽음. 커널(설치된 릴리스), 핀 헤더 맵, `config.txt`, 보안 진단만.

수집기 하나가 실패해도 나머지는 계속 진행하고, 실패는 `errors`에 기록됩니다.

## 보안 경계

- 공식 잠금 해제 경로 지원 + 잠금/퓨즈 **위험 진단**까지가 범위입니다.
- 에이전트는 OTP/eFuse를 쓰지 않고, 잠금 상태를 바꾸지 않으며, 하드웨어 보안 보호를 우회하지 않습니다.
- 다음 부팅 때 OTP를 영구적으로 굽는 `config.txt` 옵션(`program_pubkey`, `revoke_devkey`, `program_rpiboot_gpio`, `program_usb_boot_mode`)이 켜져 있으면 `security.lock_risks`에 `critical` + `irreversible: true`로 경고합니다.

## 테스트

```bash
pip install -e '.[test]'
pytest            # mosquitto가 설치돼 있으면 MQTT 종단 테스트까지 실행
```

테스트는 가짜 Pi 5 파일시스템(`tests/conftest.py`)으로 돌기 때문에 실제 하드웨어가 필요 없습니다. 실기기 검증은 아직 안 했습니다.
