/* Doosan M0609 + OnRobot Sander — 관절 리그
 *
 * 자산: assets/models/robot_arm.opt.glb
 *   base_link · link_1 … link_6 · tool_sander 를 각각 별도 노드로 담고 있고,
 *   지오메트리는 전부 "포즈된 상태의 월드 좌표"로 구워져 있다.
 *
 * 피벗과 축은 rmpflow/m0609_isaac_sim.urdf 의 링크 길이
 * (어깨 0.1345 · 상완 0.411 · 전완 0.368 · 손목 0.121)로 거리를 강제하고,
 * 방향은 포즈된 형상에서 유도했다. 유도 과정은 scratchpad/rig.py 참고.
 */
import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { MeshoptDecoder } from '../vendor/meshopt_decoder.mjs';

// 포즈 상태의 월드 피벗(p)과 회전축(a). three.js Y-up, 미터.
export const RIG = {
  j1: { p: [0.000000, 0.134500, 0.000000], a: [0.000000, 1.000000, 0.000000] },
  j2: { p: [0.000000, 0.134500, 0.000000], a: [0.018108, 0.000991, -0.999836] },
  j3: { p: [-0.372969, 0.307046, -0.006584], a: [0.018108, 0.000991, -0.999836] },
  j4: { p: [-0.240990, 0.650555, -0.003853], a: [0.358637, 0.933447, 0.007420] },
  j5: { p: [-0.240990, 0.650555, -0.003853], a: [-0.024697, 0.017434, -0.999543] },
  j6: { p: [-0.120059, 0.653060, -0.007083], a: [0.761661, 0.647932, -0.007518] },
};

/* 샌딩 패드 — j6(플랜지)에서 툴 축을 따라 잰 값.
 * 툴 정점 26만 개를 j6 기준 축방향으로 투영해 실측했다:
 * 툴 전체가 −0.097 ~ +0.049 m 에 걸쳐 있고, 디스크 면은 +0.044 m 부근이다. */
export const TOOL = {
  offset: 0.046,   // j6 → 패드 접촉면 (m)
  radius: 0.055,   // 패드 반지름 (m) — 웹 UI 기본값 패드 지름 110 mm 와 일치
};

const ORDER = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6'];
// 어느 링크 메시가 어느 관절에 매달리는지
const LINK_OF = { j1: 'link_1', j2: 'link_2', j3: 'link_3', j4: 'link_4', j5: 'link_5', j6: 'link_6' };

// URDF joint limit (rad). 실물 가동범위를 넘기면 즉시 가짜로 보인다.
export const LIMITS = [
  [-Math.PI, Math.PI], [-Math.PI, Math.PI], [-2.72, 2.72],
  [-Math.PI, Math.PI], [-2.27, 2.27], [-Math.PI, Math.PI],
];

export async function loadArmGeometry(url) {
  const loader = new GLTFLoader().setMeshoptDecoder(MeshoptDecoder);
  const gltf = await loader.loadAsync(url);
  const parts = {};
  gltf.scene.updateMatrixWorld(true);
  gltf.scene.traverse((o) => {
    if (!o.isMesh) return;
    const g = o.geometry.clone();
    g.applyMatrix4(o.matrixWorld);
    if (!g.attributes.normal) g.computeVertexNormals();
    parts[o.name.replace(/\.\d+$/, '')] = g;
  });
  const missing = ['base_link', ...Object.values(LINK_OF), 'tool_sander'].filter((k) => !parts[k]);
  if (missing.length) throw new Error('링크 누락: ' + missing.join(', '));
  return parts;
}

/** 관절 체인을 세운다. parts 는 loadArmGeometry 결과(공유 가능). */
export function buildArm(parts, materials) {
  const root = new THREE.Group();
  const mk = (geo, mat) => {
    const m = new THREE.Mesh(geo, mat);
    m.castShadow = true; m.receiveShadow = true;
    return m;
  };

  root.add(mk(parts.base_link, materials.dark));

  const joints = [];
  let parent = root;
  let prev = new THREE.Vector3(0, 0, 0);

  for (const key of ORDER) {
    const spec = RIG[key];
    const p = new THREE.Vector3().fromArray(spec.p);
    const g = new THREE.Group();
    g.position.copy(p).sub(prev);
    g.userData.axis = new THREE.Vector3().fromArray(spec.a).normalize();
    g.userData.rest = p.clone();
    parent.add(g);

    // 지오메트리가 월드로 구워져 있으므로 관절 위치만큼 되돌려 붙인다
    const mesh = mk(parts[LINK_OF[key]], materials.arm);
    mesh.position.copy(p).negate();
    g.add(mesh);

    joints.push(g);
    parent = g;
    prev = p;
  }

  // 샌더는 마지막 관절(툴 roll)에 매달린다
  const tool = new THREE.Group();
  const toolMesh = mk(parts.tool_sander, materials.tool);
  toolMesh.position.copy(new THREE.Vector3().fromArray(RIG.j6.p)).negate();
  tool.add(toolMesh);
  parent.add(tool);

  /* 패드 앵커 — 원점이 접촉면, +Z 가 바깥 법선.
     휴지 자세에서는 월드축 == j6 로컬축이므로 그대로 쓴다. */
  const j6p = new THREE.Vector3().fromArray(RIG.j6.p);
  const j6a = new THREE.Vector3().fromArray(RIG.j6.a).normalize();
  const padAnchor = new THREE.Object3D();
  padAnchor.position.copy(j6p).addScaledVector(j6a, TOOL.offset).sub(j6p);
  padAnchor.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), j6a);
  parent.add(padAnchor);

  // 실제 연마 패드. 메시에서 분리할 수 없어 접촉면 바로 위에 얹는다.
  const padDisc = new THREE.Mesh(
    new THREE.CylinderGeometry(TOOL.radius, TOOL.radius, 0.004, 40, 1),
    materials.pad
  );
  padDisc.rotation.x = Math.PI / 2;        // 원통 축(Y) → 앵커의 +Z
  padDisc.position.z = 0.002;
  padDisc.castShadow = false;
  padAnchor.add(padDisc);

  const arm = {
    root, joints, tool, toolMesh, padAnchor, padDisc,

    /** 패드 접촉면의 월드 위치와 바깥 법선 */
    padFrame(outPos = new THREE.Vector3(), outNormal = new THREE.Vector3()) {
      padAnchor.getWorldPosition(outPos);
      outNormal.set(0, 0, 1).applyQuaternion(padAnchor.getWorldQuaternion(new THREE.Quaternion()));
      return { position: outPos, normal: outNormal };
    },

    /** 패드 자전. rpm 과 경과시간(s)으로 회전각을 누적한다. */
    spin(rpm, dt) {
      padDisc.rotation.y += (rpm / 60) * Math.PI * 2 * dt;
    },

    q: new Array(6).fill(0),
    setQ(q) {
      for (let i = 0; i < 6; i++) {
        const v = THREE.MathUtils.clamp(q[i], LIMITS[i][0], LIMITS[i][1]);
        this.q[i] = v;
        joints[i].quaternion.setFromAxisAngle(joints[i].userData.axis, v);
      }
      root.updateMatrixWorld(true);
    },
  };
  arm.setQ(arm.q);
  return arm;
}
