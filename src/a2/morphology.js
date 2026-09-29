import { LINK_GROUPS } from './connectome-link.js';

export const MORPHOLOGY = [
  ['겹눈 · 시각', '전방·후방 카메라 + LiDAR 거리', '설계 개념 / 센서는 SIM', '카메라는 시각, LiDAR는 거리 보완 역할이다. 좌우 눈을 전후 카메라에 1:1 대응시키는 구조가 아니다. 현재 위험 추적은 가상 객체 정보이며 영상 인식·실물 센서 입력은 미연결이다.'],
  ['더듬이 · 후각 / 먹이 유인', '마우스로 지정한 경로점', '행동 목표의 비유', '냄새를 따라가는 행동을 목표점 추종에 비유한다. 실제 후각 뉴런 입력이나 냄새장 모델은 없으며 기존 항법기가 몸 중심 도착과 보상을 처리한다.'],
  ['평형곤 · 자세 안정 기능', 'IMU에 해당하는 자세·각속도', '기능적 대응 / SIM', '평형곤과 IMU를 동일 센서로 간주하지 않는다. 현재 자세 안정은 MuJoCo 상태와 기존 제어기로 처리하며 평형곤 신경 회로는 연결되지 않았다.'],
  ['다리 감각 · 고유수용 / 접촉', '관절 상태·발 접촉·몸체 이동', '기능적 대응', '관절과 접촉은 물리 엔진에서 계산한다. 커넥톰 입력은 개별 발 감각이 아니라 현재 전후·측방 속도 크기를 축약한 실험적 자극이다.'],
  ['T1 앞다리 · 좌 / 우', 'PiPER-X 단일 로봇팔', '형태 매핑 제안 · 미구현', '두 앞다리의 출력을 하나의 팔로 합치는 것은 향후 설계이다. 현재 팔은 정해진 시각 동작 순서로 움직이며 T1 운동 뉴런 합산·그리퍼 제어는 구현하지 않았다.'],
  ['T2 가운데다리 · 좌 / 우', 'A2 앞다리 · FL / FR', '형태 매핑 제안 · 미구현', '좌→FL, 우→FR의 형태 대응이다. 실제 보행은 별도 사족 제어기가 담당하며 T2 회로가 앞다리를 구동하지 않는다.'],
  ['T3 뒷다리 · 좌 / 우', 'A2 뒷다리 · RL / RR', '형태 매핑 제안 · 미구현', '좌→RL, 우→RR의 형태 대응이다. T1·T2·T3 신경 연결을 보존한 사족 보행 변환이나 도약 제어는 아직 구현하지 않았다.'],
  ['뇌 · 신경계', 'Jetson 실행 환경 / 커넥톰 엔진', '현재 브라우저 실행', 'Jetson은 향후 실행 장치이다. 현재 커넥톰은 브라우저 WASM LIF 엔진에서 계산하며 로봇 제어 출력은 없다.'],
];

export function installMorphologyPanel(before) {
  const button = document.createElement('button');
  button.textContent = '몸·감각 매핑'; button.setAttribute('aria-expanded', 'false'); button.setAttribute('aria-controls', 'morphologyPanel');
  before.before(button);
  const panel = document.createElement('section'); panel.id = 'morphologyPanel'; panel.hidden = true;
  panel.setAttribute('aria-label', '초파리와 로봇의 몸·감각 매핑');
  panel.innerHTML = `<header><h1>초파리 ↔ A2 + PiPER-X</h1><button data-close>닫기</button></header>
    <p>형태·기능 대응 설계도 — 생물학적으로 검증된 뉴런 매핑이 아닙니다.</p>
    <div class="morph-head"><span>초파리 몸 / 감각</span><span>대응 관계</span><span>로봇 몸 / 센서</span></div>
    <div class="morph-rows"></div>
    <p class="morph-detail" aria-live="polite"></p>
    <h2>현재 구현된 신호 방향 · 로봇 상태 → 커넥톰 관찰</h2>
    <div class="morph-live"></div>
    <p class="morph-warning">커넥톰 → 로봇 제어는 미연결입니다. 위 수치는 외부 입력 자극이며 뉴런의 측정 발화율이 아닙니다.</p>`;
  document.body.append(panel);
  const detail = panel.querySelector('.morph-detail');
  MORPHOLOGY.forEach(([fly, robot, state, text], i) => {
    const row = document.createElement('button'); row.className = 'morph-row'; row.setAttribute('aria-pressed', String(i === 0));
    row.innerHTML = `<span>${fly}</span><span class="morph-arrow">⇢<small>${state}</small></span><span>${robot}</span>`;
    row.onclick = () => {
      panel.querySelectorAll('.morph-row').forEach(r => r.setAttribute('aria-pressed', String(r === row)));
      detail.textContent = text;
    };
    panel.querySelector('.morph-rows').append(row);
  });
  detail.textContent = MORPHOLOGY[0][3];
  const labels = { motion: '전후·측방 속도 크기', turn: '회전 명령 크기', obstacle: '회피 활성', arm: '팔 작업 활성' };
  const ranges = { motion: '0–120', turn: '0–100', obstacle: '0 / 100', arm: '0 / 100' };
  for (const [key, group] of Object.entries(LINK_GROUPS)) {
    const row = document.createElement('p'); row.textContent = `${labels[key]} → ${group} · ${ranges[key]} Hz`; panel.querySelector('.morph-live').append(row);
  }
  const toggle = open => { panel.hidden = !open; button.setAttribute('aria-expanded', String(open)); button.classList.toggle('active', open); };
  button.onclick = () => toggle(panel.hidden);
  panel.querySelector('[data-close]').onclick = () => { toggle(false); button.focus(); };
  panel.addEventListener('keydown', event => { if (event.key === 'Escape') { toggle(false); button.focus(); } });
}
