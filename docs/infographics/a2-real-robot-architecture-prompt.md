# 실물 적용 구조 인포그래픽

생성 방식: 내장 image_gen. 작성일: 2026-09-27.

이미지: `a2-real-robot-architecture-20260927.png`

자료: `docs/JETSON_DEPLOYMENT.md`, `docs/A2_PRO_PERCEPTION.md`, `docs/A2_ARCHITECTURE.md` 및 현재 구현 상태. 실기 적용 목표 구조이며 현재 하드웨어 연결이 완료됐다는 의미가 아니다. 로봇 외형은 개념도다.

## 최초 프롬프트

Use case: infographic-diagram.
Create a polished, detailed Korean technical infographic poster, landscape 3:2, high resolution with readable Korean typography. Title "ANT · A2 Pro 실물 적용 동작 구조". Subtitle "외장 Jetson Orin NX 16GB + AgileX PiPER-X | 목표 아키텍처와 현재 구현 범위".
Clean white/very light gray background, navy headings, cyan data paths, orange planned implementation, red safety paths. Precise engineering editorial illustration, not a wall of small text. Generous padding, grid, clear numbered modules and arrowheads. All labels in Korean except product names and short API names.

Top ribbon prominently states: "현재: 브라우저 시뮬레이션 + 오프라인 배포 도구 / 실기 자율주행 연결은 미완료".
Legend: "청록 = 시뮬레이션·오프라인 구현", "주황 점선 = 실기 연결·검증 필요", "빨강 = 독립 안전 경로".
All live hardware flows MUST be orange dashed or marked "실기 구현 필요"; don't imply deployed live.

Main middle section is a left-to-right architecture with a big central Jetson processing box and a robot illustration on right:
1 left operator panel "① 운영자 · 경로 편집기": "경로점 · 순서 · 팔 작업 지정"; "Manual: 전후·옆걸음·회전"; "Auto: 경로 추종"; "상태·영상·이벤트 확인". Browser UI illustrated as compact top-view route with waypoints. Dashed link to Jetson: "임무·조종 명령 / 실기 통신 필요".
2 lower-left sensors panel "② 실제 센서 입력": "전·후방 카메라 → 사람·차량 분류"; "LiDAR → 거리·빈 공간"; "IMU·주행 상태 → 자세·속도"; "위치추정 → 몸 중심의 지도 좌표". Note "드라이버·시간 동기·좌표 보정 필요". Sensor arrow into central.
3 central large box header "③ Jetson Orin NX 16GB · 상위 판단". Three internal stages connected:
"인지·추적" with "경량 영상 인식 + LiDAR 거리", "전후방 상대속도·충돌 예측";
"경로·행동 계획" with "몸 중심 경로 추종", "회피: 좌우 → 짧은 후퇴 → 대기", "안전한 경우만 전방 탈출";
"임무 상태 관리" with "이동 → 도착 → Stand → 팔 작업", "접힘 확인 → 다음 경로점", "도착 점수는 학습이 아님".
Note under central box "현재 인지는 가상 입력·오프라인 검증 / 실시간 센서 융합·위치추정 추가 필요".
4 Between central and robot outputs, visible red-bordered safety gate "④ 안전 감독 · 명령 중재 [실기 구현 필요]". Text "비상정지 > 안전정지 > Manual > Auto", "입력 유효시간 · 속도 제한 · watchdog". All motion commands pass through gate.
5 output upper-right box "⑤ A2 내장 보행기": "A2 전용 SDK bridge [필요]", "vx · vy · yaw rate", "내장 균형·보행 제어 → 12개 다리 관절". Dashed feedback arrow robot-to-Jetson "자세·속도·오류·위치".
6 output lower-right box "⑥ PiPER-X · 6축 로봇팔": "전용 SDK·보정·충돌 검사 [필요]", "정지 확인 → 앞 / 왼쪽 / 오른쪽 작업", "원위치 확인 후 주행 재개". Clearly distinguish from legs branch.
Robot illustration right region showing silver dark quadruped with 4 legs, front-oriented head/nose, compact 6-axis arm mounted above front legs folded over deck, rectangular Jetson box behind arm rear upper body smaller than body width, red/green beacon and red mushroom emergency button atop rear box. Body side lettering "TeamGRIT". This illustration is a conceptual hardware layout not exact manufacturer CAD. Clear small labels "앞: PiPER-X", "뒤: Jetson + 경광등 + 비상정지".

Red independent line from physical e-stop / operator controller icon directly to hardware safety, bypassing Jetson inference. Label "독립 물리 비상정지 · 로봇 측 통신 timeout 필요". Do not depict cloud controlling joints.

Bottom 3 compact horizontal panels:
A "동작 예시" with numbered small sequence "경로 시작 → 장애물 예측·회피 → 몸 중심 도착 → 정지·팔 작업 → 접기 → 다음 지점".
B "표시·기록" show beacon icons and labels "이동: 적녹 교대", "도착·장애물: 적색 점멸", "오류: 적색 고정". Note "현재 화면 표현 / 실물 드라이버 필요".
C "실기 적용 순서" "오프라인 검증 → 센서 읽기 → Shadow(명령 미송신) → 안전 정지·Manual 시험 → 저속 Auto". Note "SSH 배포·해시 검증·버전 복귀 ≠ 실기 주행 승인".

Bottom full-width warning in pale red: "시뮬레이터의 관절 토크·위치 보조·자동기립을 실기로 직접 전송하지 않습니다. 팔 질량·관성·충돌, 실제 Stand 수평 제어는 별도 검증이 필요합니다."
No claims of a trained A2 model, hardware-tested 1m/s, actual SDK bridge implemented, accurate LiDAR/camera specifications, or physically validated self-righting. No decorative brain mythology or fly mapping. Focus on real operational architecture.

## 최종 수정 프롬프트

Change the tiny quadruped label from AgileX A2 Pro to Unitree A2 Pro (개념도). Preserve other content. AgileX manufactures the PiPER-X arm, not the A2 quadruped.

