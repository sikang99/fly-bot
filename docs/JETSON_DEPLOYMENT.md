# Unitree A2 외장 Jetson 배포 안내

작성/확인: 2026-09-27. 대상 프로젝트: `fly-bot`.

배포 대상 하드웨어: **Unitree A2 Pro + 외장 NVIDIA Jetson Orin NX 16GB** (사용자 확인).
JetPack/L4T, 캐리어 보드, SSH 주소 및 A2 펌웨어는 아직 미확인이다.
모델명 확인은 실제 장비 접속/런타임 호환성 검증을 대신하지 않는다.

전후방 경량 영상 인식·LiDAR 거리·위험 추적의 1단계 구현과 실제 입력 연결 조건은 [A2_PRO_PERCEPTION.md](A2_PRO_PERCEPTION.md)를 참조한다. 새 toolkit 패키지는 `vision.py` 오프라인 영상 어댑터를 함께 포함하며, 실제 탐지기 가중치와 실시간 스트림은 포함하지 않는다.

## 1. 먼저 알아야 할 현재 상태

**현재 A2 화면에서 보행에 성공했다고 해서 학습된 모델 파일이 만들어진 것은 아니다.**

- `src/a2/controller.js`는 보행 위상·관절 목표·PD 제어를 계산하는 규칙 기반 제어기다.
- `navigation.js`와 `a2.worker.js`의 보상은 도착 점수 누적이다. A2 정책의 역전파, optimizer, checkpoint 저장은 없다.
- `applyPlanarVelocityAssist` 및 `step`은 MuJoCo 루트 위치를 보조하고, 전도 복구도 루트 자세를 보간한다. 실제 모터 제어만으로 얻은 보행 결과가 아니다.
- 접힌 로봇팔은 시각 모형이며 실제 질량·관성·무게중심이 보행 모델에 포함되지 않았다.
- 원본 fly 연구의 학습 코드/데이터는 별개이며 A2용 훈련 완료 checkpoint가 아니다.

따라서 지금 배포할 수 있는 것은 **환경 진단·파일 검증·오프라인 추론 도구**다. 훈련 완료 모델은 추가로 만들어야 한다. 아래 도구는 없는 모델을 생성하거나 훈련 완료로 표시하지 않는다.

## 2. 권장 최종 구조

```text
PC: 모델 학습 → ONNX + 입출력 계약 + 기준 입력/출력 저장
                              ↓ SSH 배포
외장 Jetson: 카메라/LiDAR/상태 → 센서 변환 → 정책 → 안전 감독기
                                                    ↓ 고수준 속도 명령
A2: 검증된 A2 SportClient 인터페이스 → 내장 보행기 → 모터
                         ↑
              독립된 조종기·물리 비상 정지
```

첫 실기 이식은 **내장 보행기를 대체하지 않고**, Jetson이 `vx, vy, yaw rate`를 결정하는 고수준 자율주행 방식으로 진행한다. 공식 A2 Python SDK에는 A2 전용 `SportClient`, `Move`, `StopMove`, `StandUp` 등이 있다. Go2용 클래스를 A2용으로 가정해서 쓰지 않는다. [A2 SportClient 소스](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/unitree_sdk2py/a2/sport/sport_client.py), [A2 공식 예제](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/example/a2/sport/a2_sport_client.py).

자체 **저수준 보행 정책**을 Jetson에서 실행하려면 관절 관측/순서/오프셋, 제어 주기, 통신 지연, 토크 제한, 내장 제어권 전환까지 별도로 구현·검증해야 한다. 이번 배포기는 관절/토크 출력을 거부한다. 브라우저의 `data.ctrl`이나 위치 보조를 실기에 전달하지 않는다.

## 3. 이번에 자동화한 범위

| 명령 | 구현한 기능 | 로봇 동작 여부 |
|---|---|---|
| `doctor` | OS/ARM/L4T/JetPack 패키지/Python/라이브러리/인터페이스 진단 | 없음 |
| `setup` | 새 venv에 해시 고정된 로컬 wheel 설치 | 없음 |
| `pack` | 도구 및 선택적 ONNX/계약/기준 샘플 묶음 생성 | 없음 |
| `verify` | 누락·추가 파일·심볼릭 링크·SHA-256 오류 검출 | 없음 |
| `smoke` | 고정 shape/자료형/출력 오차/유한값/추론 지연 확인 | 없음 |
| `deploy` | 기본 명령 미리보기; `--execute` 지정 시 SSH 전송·진단·검증 후 current 전환 | 없음 |
| `activate` | 검증 통과한 보관 버전으로 원자적 current 링크 전환/복귀 | 없음 |

**미구현:** 학습기, A2 실시간 센서 수집·위치추정, 실시간 정책 루프, SDK 명령 송신 bridge, 하드웨어 Manual 중재, 실기 watchdog, 실기 주행 서비스. `smoke`는 녹화/기준 데이터의 오프라인 재생이지 실제 센서를 읽는 shadow 운용이 아니다. 부팅 시 자동주행 서비스도 설치하지 않는다.

## 4. 준비할 정보와 장비

1. Jetson Orin NX 16GB는 확인 완료. 캐리어 보드 모델, OS 및 JetPack/L4T 버전을 추가 확인한다.
2. Jetson의 SSH 계정/주소, A2 쪽 유선 인터페이스 이름, A2 모델/펌웨어/개발 인터페이스 권한.
3. 제조사 권장 전원 공급, 냉각, 케이블 고정 및 장착 질량/위치. 실험실용 임의 전압을 로봇 전원 단자에 연결하지 않는다.
4. 독립된 물리 비상 정지, 운영자 조종기, 비어 있는 시험 구역. 하네스 사용은 제조사 지침을 따른다.

JetPack은 **Jetson 모델과 지원 OS에 맞는 버전**을 설치한다. 버전 미확인 상태에서 최신 CUDA/JetPack을 덮어쓰지 않는다. 이 도구는 flash, 네트워크 설정, sudo, 방화벽 변경을 하지 않는다. [NVIDIA JetPack 설치 안내](https://docs.nvidia.com/jetson/agx-orin-devkit/user-guide/setup_jetpack.html).

SSH용 관리망과 로봇 통신망을 구분한다. A2 IP/서브넷을 이 문서가 임의로 지정하지 않는다. 제조사 문서와 실제 장비 설정을 확인하고, SSH 정상 접속 후 `ip -br addr`로 인터페이스를 확인한다. DDS 탐색은 ping 성공만으로 보장되지 않으며 multicast·라우팅·방화벽과 인터페이스 선택을 확인해야 한다. [Unitree SDK 설치 및 네트워크 지침](https://github.com/unitreerobotics/unitree_sdk2_python#readme).

## 5. 지금 가능한 가장 쉬운 배포: 모델 없이 도구부터

PC에서 저장소 루트로 이동한다. 아래 `operator@JETSON_HOST`, `/home/operator`는 실제 값으로 바꾼다.

```bash
cd /Users/stoney/coding/js/src/github.com/sikang99/fly-bot
npm run jetson:test
npm run jetson:pack -- --out deploy/jetson/bundles/toolkit-001
```

같은 출력 폴더가 이미 있으면 덮어쓰지 않고 실패한다. 다음 배포에는 `toolkit-002`처럼 새 이름을 사용한다. 모델이 없다는 `kind: toolkit-only` 표시가 정상이다.

Jetson에 먼저 직접 SSH 접속해 관리자가 제공한 host key fingerprint를 확인한다. 배포는 비밀번호 대신 설정된 SSH key/agent를 사용하며, 등록되지 않은 host key나 인증 오류에서 중단한다. `StrictHostKeyChecking=no`를 사용하지 않는다. 비표준 포트/키는 `~/.ssh/config`에 설정한다.

```bash
ssh operator@JETSON_HOST
# Jetson에서 확인만 한다.
uname -m
cat /etc/nv_tegra_release
python3 --version
exit
```

먼저 명령 미리보기(네트워크 접근 없음):

```bash
npm run jetson:deploy -- \
  --bundle deploy/jetson/bundles/toolkit-001 \
  --host operator@JETSON_HOST --root /home/operator/ant-a2
```

확인 후 실제 파일 전송:

```bash
npm run jetson:deploy -- \
  --bundle deploy/jetson/bundles/toolkit-001 \
  --host operator@JETSON_HOST --root /home/operator/ant-a2 --execute
```

전송은 `releases/<UTC시간>-<임의ID>`에 하고, ARM Linux/L4T 진단과 오프라인 검증에 성공했을 때만 `current` 링크를 바꾼다. 실패하면 기존 current를 유지하고 실패한 릴리스 디렉터리도 진단용으로 남긴다. 도구 전용 성공 메시지는 `toolkit-verified-no-model`이며 **주행 모델 배포 성공을 뜻하지 않는다.**

```bash
ssh operator@JETSON_HOST \
  'python3 -B /home/operator/ant-a2/current/ant_jetson.py doctor --require-jetson'
```

도구는 Python 3.8 이상을 대상으로 표준 라이브러리만 사용한다. 현재 개발 PC에서 테스트한 버전과 Jetson 실제 버전은 별개다. `/proc/device-tree/model`, `/etc/nv_tegra_release`가 숨겨진 컨테이너에서는 Jetson 판별이 실패할 수 있으므로 처음에는 호스트 OS에서 실행한다.

## 6. 학습 모델을 만들 때 지킬 계약

새 정책 학습 환경은 다음을 갖춰야 한다.

- A2 관측량과 센서 좌표/단위/관측 주기를 고정한다. 세계 좌표 목표는 로봇 몸 좌표로 변환한다.
- 위치/회전 강제 보정 없이 검증한다. 마찰·질량·지연·센서 노이즈·지형 및 실제 팔 장착 영향을 반영한다.
- 장애물 접촉/넘어짐/진행 실패, 경로 오차, 이동 시간 등을 학습 평가에 포함한다. 화면 보상 카운터를 optimizer로 오인하지 않는다.
- 훈련 데이터와 별개의 경로·동적 장애물·좁은 통로에서 검증하고 실험 ID와 checkpoint 해시를 기록한다.
- 고수준 출력은 `[vx_m_s, vy_m_s, yaw_rad_s]`. 좌표는 +X 전방, +Y 왼쪽, +Z 위다. 실제 A2 좌표 규약은 펌웨어/SDK로 다시 확인한다.
- export 모델 안에 정규화/clip/action scaling을 포함한다. 최종 출력은 정규화된 [-1,1]이 아니라 물리 단위다.
- 이번 런타임은 float32 입력 `[1,N]`, 출력 `[1,3]`, stateless 단일 입력/출력만 지원한다. recurrent hidden state, image tensor, 외부 ONNX 가중치 파일은 지원하지 않는다.

`deploy/jetson/contract.example.json`은 **설계 예시**다. 8개 관측량을 기존 센서 어댑터가 이미 생성한다는 뜻이 아니다. 실제 모델의 이름·차원·순서·단위에 맞춰 복사/수정한다. `training_run`의 `REPLACE...`를 그대로 두면 배포 패키지 생성이 거부된다. `simulation_assistance: false`는 학습 실험자가 증거로 확인한 사실이어야 하며, 파일의 선언만으로 물리 타당성이 증명되지 않는다.

훈련 프로젝트에서 생성할 세 파일:

1. `policy.onnx`: 단일 파일, 추론 모드. 훈련 프레임워크의 공식 exporter로 생성.
2. `contract.json`: 위 계약과 실제 실험 ID.
3. `samples.json`: 원래 훈련 모델에서 저장한 관측 및 **기준 action** 배열.

```json
[
  {
    "observation": [0, 0, 0, 0, 0, 5, 5, 5],
    "expected_action": [0, 0, 0]
  }
]
```

위 JSON은 형식 예시일 뿐 기준값을 임의로 작성하면 안 된다. 정지·전후좌우·회전·목표 근접·장애물 접근 등 실제 훈련 모델 결과를 여러 건 기록한다. ONNX와 원래 모델의 차이를 `atol`로 비교한다. 실패할 때 허용오차만 늘리지 말고 정규화·관측 순서·export 연산을 먼저 확인한다.

## 7. Jetson 추론 환경 설치와 가속

첫 확인은 CPU ONNX Runtime으로 하고, 충분한 성능이면 GPU를 필수로 만들지 않는다. CUDA/TensorRT 사용 시 **JetPack의 CUDA/cuDNN/TensorRT, Python ABI, aarch64에 맞는 빌드**가 필요하다. 일반 PC용 `onnxruntime-gpu` wheel을 무조건 설치하지 않는다. PyTorch가 필요한 export/학습 환경도 Jetson 배포 환경과 분리한다. [ONNX Runtime TensorRT 호환성 및 설치](https://onnxruntime.ai/docs/execution-providers/TensorRT-ExecutionProvider.html), [NVIDIA Jetson PyTorch 안내](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html).

이 저장소는 장비 정보 없이 특정 wheel 버전을 추정해 고정하지 않는다. 검증한 wheel 세트와 모든 전이 의존성의 버전·SHA-256이 들어 있는 `requirements.lock`을 준비한다. 해시는 `python3 -m pip hash <wheel파일>`로 구할 수 있다. 예를 들어 lock의 각 항목은 `패키지명==검증버전 --hash=sha256:실제해시` 형식이다. `numpy`, `onnxruntime` 또는 호환 GPU 배포판과 필요한 의존성을 모두 포함한다. 테스트용 `onnx`는 모델 생성/검증용이며 배포 추론에는 필수가 아니다.

Jetson에서 설치 자동화:

```bash
python3 -B /home/operator/ant-a2/current/ant_jetson.py setup \
  --venv /home/operator/ant-a2/envs/runtime-001 \
  --requirements /home/operator/artifacts/requirements.lock \
  --wheelhouse /home/operator/artifacts/wheelhouse
```

새 venv에만 설치하며 기존 환경을 변경하지 않는다. 인터넷 패키지 검색과 소스 빌드는 하지 않고 로컬 binary wheel 및 해시 확인을 강제한다. 실패한 venv는 재사용하지 말고 오류를 확인한 후 새 버전 이름을 사용한다. OS에 `venv` 지원 패키지가 없으면 해당 OS의 관리자 설치가 먼저 필요하다.

## 8. 학습 완료 후 모델 배포

```bash
python3 deploy/jetson/ant_jetson.py pack \
  --out deploy/jetson/bundles/policy-001 \
  --model /absolute/path/policy.onnx \
  --contract /absolute/path/contract.json \
  --samples /absolute/path/samples.json

npm run jetson:deploy -- \
  --bundle deploy/jetson/bundles/policy-001 \
  --host operator@JETSON_HOST --root /home/operator/ant-a2 \
  --python /home/operator/ant-a2/envs/runtime-001/bin/python --execute
```

모델 포함 번들은 원격 Python에 ONNX Runtime/numpy가 없으면 활성화되지 않는다. 검증은 warm-up 뒤 각 샘플을 10회 추론하여 출력 오차·NaN/Inf·shape·자료형·최대 지연을 검사한다. 최초 session 생성/engine build 시간은 steady-state latency에서 제외되며 배포 명령 전체 제한시간은 별도로 적용된다. 작은 golden 샘플 집합의 통과는 장시간 지연/열/주행 안정성 인증이 아니다.

`--provider CUDAExecutionProvider` 또는 `TensorrtExecutionProvider`를 지정할 수 있다. 선택 provider가 없으면 실패한다. session provider 목록을 출력하지만 연산 일부의 CPU 배치까지 없음을 보장하지 않는다. GPU 사용률/연산 배치를 별도로 프로파일링한다. TensorRT engine은 Jetson의 실제 하드웨어/런타임에서 검증하며 Mac/다른 GPU의 engine 파일을 그대로 복사하지 않는다.

결과는 `offline-inference-passed`, 샘플 수, 평균/최대 지연으로 출력된다. **현재 단계에서는 결과를 로봇으로 보내지 않는다.**

## 9. 이전 버전으로 복귀

Jetson에서 `releases` 목록과 배포 로그에 나온 ID를 확인한 다음 해당 버전을 명시한다. 코드는 삭제하지 않는다.

```bash
ls /home/operator/ant-a2/releases
readlink /home/operator/ant-a2/current
/home/operator/ant-a2/envs/runtime-001/bin/python -B \
  /home/operator/ant-a2/current/ant_jetson.py activate \
  --root /home/operator/ant-a2 --release VERIFIED_PREVIOUS_RELEASE_ID
```

복귀 버전도 다시 해시/추론 검증한다. Python 환경은 current에 포함되지 않으므로 **그 모델에서 검증한 venv**를 명시해야 한다. 배포 로그에 릴리스 ID·venv·JetPack·provider를 함께 보관한다. 동시 활성화는 lock으로 거부한다. 비정상 종료로 `.activation-lock`이 남으면 활성화 프로세스가 없는지 먼저 확인한 후 해당 lock 파일만 정리한다.

SHA-256은 전송 손상/파일 변경 탐지이며 서명이 아니다. 신뢰하는 소스에서 만든 번들만 사용하고 SSH host key를 확인한다. 공격자가 manifest와 파일을 함께 바꿀 수 있는 환경의 공급망 보안은 별도 서명/권한 설계가 필요하다.

## 10. 실제 A2 주행을 열기 위한 다음 단계

아래는 **앞으로 구현/현장 검증해야 할 순서**이며 이번 자동화가 완료했다는 뜻이 아니다.

1. 제조사 A2 전용 SDK 버전을 고정하고 상태 읽기만 검증한다. 공식 예제 전체 실행은 동작 명령이 있으므로 설치 확인용으로 자동 실행하지 않는다.
2. 외장 Jetson의 카메라/LiDAR/IMU/속도/위치추정 입력을 계약에 맞게 변환한다. 시간 동기, 센서 age, 차체 및 팔 외곽, 센서 외부 파라미터를 검증한다.
3. 센서 기반 **실시간 shadow**를 구현해 action을 로그만 남긴다. 실제 로봇의 상태와 정책 결과를 비교하며 송신은 금지한다.
4. 독립된 안전 감독기/명령 중재기를 구현한다. 우선순위는 물리 비상 정지 → 안전 정지 → 사람 Manual → Auto. 무장 해제 상태로 시작하고 조종권 전환 시 이전 명령을 지운다.
5. A2 고수준 명령 bridge에 유한값·속도/가속도 제한·센서/조종기 유효시간·통신 watchdog·SDK 응답 오류 처리를 넣는다. 추론 프로세스가 죽어도 감독기가 남고, Jetson 전원/네트워크 전체가 끊겨도 로봇 쪽 timeout이 작동하는지 확인한다. `StopMove` 호출만으로 네트워크 단절 시 정지를 보장할 수 없다.
6. 화면 Passive(0 토크)를 실기의 안전 정지로 그대로 매핑하지 않는다. 실기에서는 제조사 권장 정지/자세 유지/감쇠 모드의 의미를 구분한다. StandUp, RecoveryStand도 사람의 명시적 승인과 공간 확인 없이 자동 호출하지 않는다.
7. 사람 운영자가 제조사 절차에 따라 Manual 상태·정지·매우 낮은 속도 순서로 시험한다. 초기 제한은 별도 합의한 보수적 값으로 시작한다. 시뮬레이션 1m/s가 실기 승인 속도가 아니다.
8. 유선 분리, 추론 hang, 조종기 단절, NaN, 센서 지연, 재부팅 및 Manual 전환 시험을 기록한다. 사람이 아닌 부드러운 시험 장애물로 검증하고 사람 근접 운용은 이후 단계로 둔다.
9. 장시간 온도/전력/지연과 경로·충돌·전도 지표가 통과한 뒤에만 service 설치를 검토한다. 부팅 시에는 관측/대기만 시작하고 자동 무장·자동 Walk는 금지한다. 업데이트와 rollback은 로봇이 정지·무장 해제된 상태에서 한다.

외장 Jetson 정보, 실제 A2 SDK/펌웨어, 센서 인터페이스와 훈련 산출물이 확보되면 1~5의 구현 범위를 구체화할 수 있다. 현재는 실기 주소가 제공되지 않아 원격 배포와 하드웨어 추론/주행을 시험하지 않았다.

## 11. 개발 검증

```bash
npm run jetson:test
npm test
npm run build
```

Jetson 테스트에는 해시 손상, 링크/추가 파일, 잘못된 계약, 전송 실패, 추론 실패 시 활성화 방지, 기본 무접속, 버전 복귀가 포함된다. `numpy`, `onnx`, `onnxruntime`가 설치된 별도 테스트 환경에서는 작은 **시험용** ONNX를 만들어 실제 추론/기준값 불일치도 검사한다. 이 시험용 모델은 학습된 A2 모델이 아니며 배포 산출물에 포함되지 않는다.
