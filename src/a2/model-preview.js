import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFExporter } from 'three/addons/exporters/GLTFExporter.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { PLYExporter } from 'three/addons/exporters/PLYExporter.js';

// Snapshot only: never modify or dispose the simulator's shared resources.
export function robotSnapshot(robot) {
  const copy = robot.clone(true);
  const remove = [];
  copy.traverse(o => { if (o.isCamera || !o.layers.isEnabled(0)) remove.push(o); });
  for (const o of remove) o.removeFromParent();
  copy.updateMatrixWorld(true);
  const bounds = new THREE.Box3().setFromObject(copy);
  const center = bounds.getCenter(new THREE.Vector3());
  copy.position.sub(new THREE.Vector3(center.x, bounds.min.y, center.z));
  copy.name = 'ANT_A2_PIPER_X_visual_snapshot';
  copy.userData = { description: 'Procedural visual snapshot; not CAD, MJCF or a physics model. Units: metres; Y up.' };
  copy.updateMatrixWorld(true);
  return copy;
}

export async function showModelPreview(robot, onOpen = () => {}, onClose = () => {}) {
  const dialog = document.createElement('dialog');
  dialog.id = 'modelPreview';
  dialog.setAttribute('aria-label', '3D 모델 미리보기');
  dialog.style.cssText = 'position:fixed;top:76px;margin:0 auto;z-index:20;width:min(1000px,94vw);max-width:94vw;max-height:calc(100vh - 100px);overflow:auto;background:#101923;color:#eef5ff;border:1px solid #506075;border-radius:12px;padding:16px;';
  dialog.innerHTML = `<h2>ANT : A2 + PIPER-X · GLB 미리보기</h2>
    <p>현재 자세의 정적 외형 · 드래그: 회전 / 휠: 확대 · STEP 하우징·물리 정보 미포함</p>
    <div data-view style="height:55vh;min-height:240px"></div>
    <p data-status role="status">GLB 생성 중…</p>
    <button data-glb disabled>GLB 다운로드</button> <button data-ply disabled>PLY 다운로드 (형상)</button> <button data-close>닫기</button>`;
  // Non-modal so the header toggle and emergency stop remain reachable.
  document.body.append(dialog); dialog.show();
  const status = dialog.querySelector('[data-status]');
  const host = dialog.querySelector('[data-view]');
  let renderer, controls, observer, loaded;
  const urls = [];
  const escape = event => { if (event.key === 'Escape') dialog.close(); };
  document.addEventListener('keydown', escape);
  dialog.querySelector('[data-close]').onclick = () => dialog.close();
  dialog.addEventListener('close', () => {
    document.removeEventListener('keydown', escape);
    renderer?.setAnimationLoop(null); controls?.dispose(); observer?.disconnect(); renderer?.dispose();
    loaded?.traverse(o => {
      o.geometry?.dispose();
      for (const material of o.material ? (Array.isArray(o.material) ? o.material : [o.material]) : []) {
        for (const value of Object.values(material)) if (value?.isTexture) value.dispose();
        material.dispose();
      }
    });
    urls.forEach(url => URL.revokeObjectURL(url)); dialog.remove(); onClose();
  }, { once: true });
  onOpen(() => dialog.close());
  function download(data, type, filename) {
    const url = URL.createObjectURL(new Blob([data], { type })); urls.push(url);
    const link = document.createElement('a'); link.href = url; link.download = filename;
    dialog.append(link); link.click(); link.remove();
  }
  try {
    const snapshot = robotSnapshot(robot);
    const glb = await new GLTFExporter().parseAsync(snapshot, { binary: true, onlyVisible: true });
    if (!dialog.open) return;
    // Display the exported GLB itself, not the source scene.
    loaded = (await new GLTFLoader().parseAsync(glb, '')).scene;
    if (!dialog.open) { loaded.traverse(o => o.geometry?.dispose()); return; }
    const scene = new THREE.Scene(); scene.background = new THREE.Color('#101923'); scene.add(loaded);
    scene.add(new THREE.HemisphereLight(0xeaf4ff, 0x555566, 2));
    const light = new THREE.DirectionalLight(0xffffff, 3); light.position.set(3, 5, 4); scene.add(light);
    const bounds = new THREE.Box3().setFromObject(loaded);
    const center = bounds.getCenter(new THREE.Vector3());
    const size = bounds.getSize(new THREE.Vector3()).length();
    const camera = new THREE.PerspectiveCamera(40, 1, .01, Math.max(100, size * 20));
    camera.position.copy(center).add(new THREE.Vector3(1.2, .8, 1.4).multiplyScalar(size));
    renderer = new THREE.WebGLRenderer({ antialias: true }); renderer.setPixelRatio(Math.min(devicePixelRatio, 2)); host.append(renderer.domElement);
    controls = new OrbitControls(camera, renderer.domElement); controls.target.copy(center); controls.enableDamping = true;
    observer = new ResizeObserver(() => {
      renderer.setSize(host.clientWidth, host.clientHeight); camera.aspect = host.clientWidth / host.clientHeight; camera.updateProjectionMatrix();
    }); observer.observe(host);
    renderer.setAnimationLoop(() => { controls.update(); renderer.render(scene, camera); });
    const glbButton = dialog.querySelector('[data-glb]'), plyButton = dialog.querySelector('[data-ply]');
    glbButton.disabled = plyButton.disabled = false;
    glbButton.onclick = () => download(glb, 'model/gltf-binary', 'ant-a2-piper-x.glb');
    plyButton.onclick = () => {
      new PLYExporter().parse(loaded, data => download(data, 'application/octet-stream', 'ant-a2-piper-x.ply'), { binary: true, littleEndian: true });
    };
    status.textContent = `GLB 재로딩 완료 · ${(glb.byteLength / 1024).toFixed(0)} KB · GLB: 색상·텍스처·부품 구조 / PLY: 정적 형상, 로고 텍스처 제외`;
  } catch (error) { status.textContent = `모델 생성 실패: ${error.message}`; }
}
