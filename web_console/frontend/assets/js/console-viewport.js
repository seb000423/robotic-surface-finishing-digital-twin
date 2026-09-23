const THREE = await import(window.__resources.three);

// Offline build: three.js and its addons are inlined by the bundler and handed
// to us as blob URLs, so the module graph is rewired against those.
const ADDONS = {
  'loaders/GLTFLoader.js': 'gltfLoader',
  'controls/OrbitControls.js': 'orbitControls',
  'utils/BufferGeometryUtils.js': 'bufferGeometryUtils',
};
const __modCache = new Map();
async function __resolveAddon(path) {
  if (__modCache.has(path)) return __modCache.get(path);
  const p = (async () => {
    const key = ADDONS[path];
    if (!key) throw new Error('addon not bundled: ' + path);
    let src = await (await fetch(window.__resources[key])).text();
    src = src.replace(/(from\s*|import\s*)(['"])([^'"]+)\2/g, (m, kw, q, spec) => {
      if (spec === 'three') return kw + q + window.__resources.three + q;
      const hit = Object.keys(ADDONS).find((k) => spec.endsWith(k.split('/').pop()));
      return hit ? kw + q + '@@' + hit + '@@' + q : m;
    });
    for (const dep of Object.keys(ADDONS)) {
      if (src.includes('@@' + dep + '@@')) src = src.split('@@' + dep + '@@').join(await __resolveAddon(dep));
    }
    return URL.createObjectURL(new Blob([src], { type: 'text/javascript' }));
  })();
  __modCache.set(path, p);
  return p;
}
const loadAddon = async (path) => import(await __resolveAddon(path));

const ACCENT = 0x3e9dbe;  

/* 무광 → 유광. 패드가 지나간 자리를 하이트필드 좌표계의 마스크에 찍고,
   차체 셰이더에서 roughness 를 보간한다. 의 가치 제안 그 자체다.*/
const ROUGH_MATTE = 0.82, ROUGH_GLOSS = 0.14;
/* 기본값을 null 로 두면 안 된다. 경로선 재질은 모델이 로드되기 전에 이미
   렌더되는데, 그때 initPolish 가 아직 안 돌아 three.js 가 null 을 Vector3 로
   업로드하려다 터진다. uPolishOn 이 0 이라 값 자체는 쓰이지 않는다. */
const polishU = {
  uPolish: { value: null },        // 샘플러는 null 이어도 기본 텍스처가 바인딩된다
  uLongSel: { value: new THREE.Vector3(0, 0, 1) },   // 월드 좌표에서 long 축을 뽑는 선택자
  uCrossSel: { value: new THREE.Vector3(1, 0, 0) },
  uFieldMin: { value: new THREE.Vector2(0, 0) },     // (l0, c0)
  uFieldSpan: { value: new THREE.Vector2(1, 1) },    // (l1-l0, c1-c0)
  uPolishOn: { value: 0 },
};

function attachPolish(mat) {
  mat.onBeforeCompile = (shader) => {
    Object.assign(shader.uniforms, polishU);
    shader.vertexShader = shader.vertexShader
      .replace('#include <common>', '#include <common>\nvarying vec3 vPolishPos;')
      .replace('#include <worldpos_vertex>',
        '#include <worldpos_vertex>\n  vPolishPos = (modelMatrix * vec4(transformed, 1.0)).xyz;');
    shader.fragmentShader = shader.fragmentShader
      .replace('#include <common>', [
        '#include <common>',
        'varying vec3 vPolishPos;',
        'uniform sampler2D uPolish;',
        'uniform vec3 uLongSel;',
        'uniform vec3 uCrossSel;',
        'uniform vec2 uFieldMin;',
        'uniform vec2 uFieldSpan;',
        'uniform float uPolishOn;',
      ].join('\n'))
      .replace('#include <roughnessmap_fragment>', [
        '#include <roughnessmap_fragment>',
        'float polished = 0.0;',
        'if (uPolishOn > 0.5) {',
        '  vec2 pUv = vec2(',
        '    (dot(vPolishPos, uLongSel) - uFieldMin.x) / uFieldSpan.x,',
        '    (dot(vPolishPos, uCrossSel) - uFieldMin.y) / uFieldSpan.y);',
        '  if (pUv.x > 0.0 && pUv.x < 1.0 && pUv.y > 0.0 && pUv.y < 1.0) {',
        '    polished = texture2D(uPolish, pUv).r;',
        '  }',
        '  roughnessFactor = mix(0.82, 0.14, polished);',
        '}',
      ].join('\n'))
      // 광택은 클리어코트가 만든다. 여기까지 같이 보간해야 무광->유광이 읽힌다.
      .replace('#include <lights_physical_fragment>', [
        '#include <lights_physical_fragment>',
        '#ifdef USE_CLEARCOAT',
        'if (uPolishOn > 0.5) {',
        '  material.clearcoatRoughness = mix(0.42, 0.03, polished);',
        '}',
        '#endif',
      ].join('\n'));
  };
  mat.needsUpdate = true;
  return mat;
}

/* 경로선도 같은 마스크를 본다. 계획선이 차체를 끝까지 덮고 있으면
   무광->유광이 아무리 잘 돌아도 화면에는 '줄무늬 덮인 차'만 보인다.
   지나간 자리의 선을 걷어야 닦인 도장이 드러난다. */
function attachPathFade(mat) {
  mat.onBeforeCompile = (shader) => {
    Object.assign(shader.uniforms, polishU);
    shader.vertexShader = shader.vertexShader
      .replace('#include <common>', '#include <common>\nvarying vec3 vPolishPos;')
      .replace('#include <begin_vertex>',
        '#include <begin_vertex>\n  vPolishPos = (modelMatrix * vec4(position, 1.0)).xyz;');
    shader.fragmentShader = shader.fragmentShader
      .replace('#include <common>', [
        '#include <common>',
        'varying vec3 vPolishPos;',
        'uniform sampler2D uPolish;',
        'uniform vec3 uLongSel;',
        'uniform vec3 uCrossSel;',
        'uniform vec2 uFieldMin;',
        'uniform vec2 uFieldSpan;',
        'uniform float uPolishOn;',
      ].join('\n'))
      .replace('#include <opaque_fragment>', [
        '  if (uPolishOn > 0.5) {',
        '    vec2 pUv = vec2(',
        '      (dot(vPolishPos, uLongSel) - uFieldMin.x) / uFieldSpan.x,',
        '      (dot(vPolishPos, uCrossSel) - uFieldMin.y) / uFieldSpan.y);',
        '    if (pUv.x > 0.0 && pUv.x < 1.0 && pUv.y > 0.0 && pUv.y < 1.0) {',
        // 완전히 지우지 않는다. 옅게 남겨야 '어디를 지났는지'가 읽힌다
        '      diffuseColor.a *= 1.0 - 0.88 * texture2D(uPolish, pUv).r;',
        '    }',
        '  }',
        '#include <opaque_fragment>',
      ].join('\n'));
  };
  mat.needsUpdate = true;
  return mat;
}

const SKIP = /environment|backdrop|^cube$|^plane$|floor|ground/i;
const CAR_LENGTH = 4.35;
// 경로선을 도장면에서 띄우는 양(m). 선이 도장면과 z-fighting 하지 않게 하려는 것이고,
// 패드는 이 값을 되빼서 도장면 자체를 짚는다.
const PATH_LIFT = 0.013;
const DEMO_SPEEDUP = 26;   // 이송속도 배속 — 실시간이면 한 패스가 2분을 넘는다
const ARM_HEIGHT = 0.86;   // m — an M0609 stands ~0.74 m folded, plus the wrist tool
const PEDESTAL = 0.72;     // m — the pillar each cell is bolted to

async function loadCar(url) {
  const { GLTFLoader } = await loadAddon('loaders/GLTFLoader.js');
  // 차체도 meshopt 로 구운 GLB 다(4.2MB -> 264KB). 디코더 없이는 로더가 거부한다.
  const { MeshoptDecoder } = await import(MESHOPT_URL);
  const gltf = await new GLTFLoader().setMeshoptDecoder(MeshoptDecoder).loadAsync(url);
  const root = gltf.scene;
  root.updateMatrixWorld(true);

  const geos = [], mats = [];
  root.traverse((o) => {
    if (!o.isMesh) return;
    for (let p = o; p; p = p.parent) if (p.name && SKIP.test(p.name)) return;
    // 양자화된 정수 배열에 변환 결과를 도로 써 넣으면 형상이 뭉개진다.
    // dequantize 로 float 으로 풀고 나서 행렬을 먹인다.
    const g = dequantize(o.geometry.clone());
    g.applyMatrix4(o.matrixWorld);
    geos.push(g); mats.push(o.material);
  });
  if (!geos.length) throw new Error('no mesh in car model');

  const bbox = () => {
    const b = new THREE.Box3();
    geos.forEach((g) => { g.computeBoundingBox(); b.union(g.boundingBox); });
    return b;
  };
  const bake = (m) => geos.forEach((g) => g.applyMatrix4(m));
  let b = bbox(), sz = b.getSize(new THREE.Vector3());
  if (sz.x > sz.z) { bake(new THREE.Matrix4().makeRotationY(Math.PI / 2)); b = bbox(); sz = b.getSize(new THREE.Vector3()); }
  bake(new THREE.Matrix4().makeScale(...Array(3).fill(CAR_LENGTH / sz.z)));
  b = bbox();
  const c = b.getCenter(new THREE.Vector3());
  bake(new THREE.Matrix4().makeTranslation(-c.x, -b.min.y, -c.z));

  const group = new THREE.Group();
  geos.forEach((g, i) => {
    if (!g.attributes.normal) g.computeVertexNormals();
    const src = mats[i];
    const lum = src && src.color ? 0.2126 * src.color.r + 0.7152 * src.color.g + 0.0722 * src.color.b : 0;
    const mat = attachPolish(new THREE.MeshPhysicalMaterial({
      color: lum > 0.5 ? 0x2f343b : (src && src.color ? src.color.getHex() : 0x22262c),
      metalness: 0.35, roughness: ROUGH_MATTE,
      clearcoat: 1, clearcoatRoughness: 0.28, envMapIntensity: 1.4,
    }));
    const m = new THREE.Mesh(g, mat);
    m.castShadow = true; m.receiveShadow = true;
    group.add(m);
  });
  group.userData.size = bbox().getSize(new THREE.Vector3());
  return group;
}

/* ══ 표면 필드 ══════════════════════════════════════════════════
   차체는 삼각형이 20만 개인데 가속 구조가 없다. CPU 로 격자를 쏘면
   메인 스레드가 멈춘다. GPU 로 굽는다 — 직교 카메라 한 장으로 표면의
   '깊이축 좌표'를 써 내고 한 번 읽어 온다.

   축은 세 개로 말한다.
     u  레인이 달리는 축      (차체 길이)
     v  레인 사이를 옮기는 축  (위에서 보면 폭, 옆에서 보면 높이)
     d  깊이축                (위에서 보면 Y, 옆에서 보면 폭)
   dir 은 카메라가 서 있는 쪽. +1 이면 d 가 큰 쪽에서 들여다본다.

   위에서 찍은 필드 하나로는 문짝을 낼 수 없다. 한 칸에 값이 하나뿐이라
   수직면을 담을 수 없기 때문이다. 그래서 옆에서도 두 장 굽는다. */

const AX = { x: new THREE.Vector3(1, 0, 0), y: new THREE.Vector3(0, 1, 0), z: new THREE.Vector3(0, 0, 1) };

function buildField(renderer, model, spec) {
  const { u, v, d, dir, nU, nV, u0, u1, v0, v1, keep, skip } = spec;
  const eu = AX[u], ev = AX[v], ed = AX[d];

  const target = new THREE.WebGLRenderTarget(nU, nV, {
    type: THREE.FloatType, format: THREE.RGBAFormat,
    minFilter: THREE.NearestFilter, magFilter: THREE.NearestFilter, depthBuffer: true,
  });

  const mat = new THREE.ShaderMaterial({
    uniforms: { uSel: { value: ed.clone() } },
    vertexShader: `uniform vec3 uSel; varying float vD;
      void main() {
        vec4 wp = modelMatrix * vec4(position, 1.0);
        vD = dot(wp.xyz, uSel);
        gl_Position = projectionMatrix * viewMatrix * wp;
      }`,
    fragmentShader: `varying float vD;
      void main() { gl_FragColor = vec4(vD, 1.0, 0.0, 1.0); }`,
    side: THREE.DoubleSide,
  });

  const b = new THREE.Box3().setFromObject(model);
  const size = b.getSize(new THREE.Vector3());
  const back = size[d] + 4;

  const cam = new THREE.OrthographicCamera(
    (u0 - u1) / 2, (u1 - u0) / 2, (v1 - v0) / 2, (v0 - v1) / 2, 0.01, back
  );
  const mid = new THREE.Vector3();
  mid[u] = (u0 + u1) / 2;
  mid[v] = (v0 + v1) / 2;
  mid[d] = dir > 0 ? b.max[d] + 2 : b.min[d] - 2;
  cam.up.copy(ev);
  cam.position.copy(mid);
  const look = mid.clone();
  look[d] -= dir * 3;
  cam.lookAt(look);
  cam.updateMatrixWorld(true);

  /* 판독 방향은 추측하지 않는다. lookAt 이 만든 카메라 축을 직접 읽어
     u·v 가 뒤집혔는지 본다. 예전 코드는 이걸 손으로 계산하다 틀려
     후드 위에 루프 높이가 얹혔다. */
  const camX = new THREE.Vector3().setFromMatrixColumn(cam.matrixWorld, 0);
  const camY = new THREE.Vector3().setFromMatrixColumn(cam.matrixWorld, 1);
  const flipU = camX.dot(eu) < 0;
  const flipV = camY.dot(ev) < 0;

  const scene = new THREE.Scene();
  const proxy = new THREE.Group();
  model.updateMatrixWorld(true);
  model.traverse((o) => {
    if (!o.isMesh) return;
    if (skip) { for (let p = o; p; p = p.parent) if (p.name && skip.test(p.name)) return; }
    const m = new THREE.Mesh(o.geometry, mat);
    m.matrixAutoUpdate = false;
    m.matrix.copy(o.matrixWorld);
    proxy.add(m);
  });
  scene.add(proxy);

  const prevTarget = renderer.getRenderTarget();
  renderer.setRenderTarget(target);
  renderer.setClearColor(0x000000, 0);
  renderer.clear();
  renderer.render(scene, cam);
  const buf = new Float32Array(nU * nV * 4);
  renderer.readRenderTargetPixels(target, 0, 0, nU, nV, buf);
  renderer.setRenderTarget(prevTarget);
  target.dispose(); mat.dispose();

  // 경로선은 z-fighting 을 피하려고 표면에서 바깥으로 띄운다. 바깥은 dir 쪽이다.
  const h = new Float32Array(nU * nV).fill(NaN);
  for (let j = 0; j < nV; j++) {
    for (let i = 0; i < nU; i++) {
      const pi = flipU ? nU - 1 - i : i;
      const pj = flipV ? nV - 1 - j : j;
      const k = (pj * nU + pi) * 4;
      if (buf[k + 3] > 0.5 && (!keep || keep(buf[k]))) h[j * nU + i] = buf[k] + PATH_LIFT * dir;
    }
  }
  return { h, nU, nV, u0, u1, v0, v1, u, v, d, dir };
}

/** 위에서 내려찍은 필드. 연마 마스크가 이 필드의 좌표계를 쓰므로
    예전 이름(long/cross/l0..c1/nLong/nCross)도 그대로 달아 둔다. */
function buildHeightField(renderer, model, nLong = 256, nCross = 160) {
  const b = new THREE.Box3().setFromObject(model);
  const size = b.getSize(new THREE.Vector3());
  const long = size.x >= size.z ? 'x' : 'z';
  const cross = long === 'x' ? 'z' : 'x';
  const l0 = b.min[long] + size[long] * 0.05, l1 = b.max[long] - size[long] * 0.05;
  const c0 = b.min[cross] + size[cross] * 0.06, c1 = b.max[cross] - size[cross] * 0.06;
  const floor = b.min.y + size.y * 0.26;

  const f = buildField(renderer, model, {
    u: long, v: cross, d: 'y', dir: 1, nU: nLong, nV: nCross,
    u0: l0, u1: l1, v0: c0, v1: c1,
    keep: (y) => y > floor,
  });
  return Object.assign(f, { nLong, nCross, l0, l1, c0, c1, long, cross });
}

/** 옆에서 찍은 필드 두 장. 문짝·펜더처럼 위에서는 담을 수 없는 면을 낸다.
    바퀴는 뺀다 — 광택 대상이 아니고, 넣으면 팔이 타이어를 훑는다. */
const WHEEL = /wheel|tire|tyre|rim|brake|caliper|hub/i;

function buildSideFields(renderer, model, nU = 256, nV = 96) {
  const b = new THREE.Box3().setFromObject(model);
  const size = b.getSize(new THREE.Vector3());
  const long = size.x >= size.z ? 'x' : 'z';
  const cross = long === 'x' ? 'z' : 'x';
  const u0 = b.min[long] + size[long] * 0.05, u1 = b.max[long] - size[long] * 0.05;
  // 아래는 사이드실 아래를 버리고, 위는 위에서 찍은 필드가 이미 맡은 구간을 피한다
  const v0 = b.min.y + size.y * 0.05, v1 = b.max.y - size.y * 0.30;
  const cd = (b.min[cross] + b.max[cross]) / 2;

  return [1, -1].map((dir) => buildField(renderer, model, {
    u: long, v: 'y', d: cross, dir, nU, nV, u0, u1, v0, v1,
    // 창을 통해 반대편 옆면이 보이는 표본은 버린다 — 카메라 쪽 절반만 남긴다
    keep: (c) => c * dir > cd * dir,
    skip: WHEEL,
  }));
}

// serpentine passes read off the height field at the requested lane pitch
// 필드보다 가파른 면은 패드가 법선을 따라 접근할 수 없으므로 경로를 내지 않는다.
const MAX_SLOPE = 62 * Math.PI / 180;

/** 필드 기울기에서 표면 법선을 낸다. 중앙차분, 가장자리는 편차분.
    (u, v, h) 면의 법선은 (-hu, -hv, 1) 이고 바깥쪽은 dir 쪽이다. */
function fieldNormal(field, i, j, out) {
  const { h, nU, nV, u0, u1, v0, v1, u, v, d, dir } = field;
  const du = (u1 - u0) / (nU - 1);
  const dv = (v1 - v0) / (nV - 1);
  const at = (ii, jj) => {
    if (ii < 0 || ii >= nU || jj < 0 || jj >= nV) return NaN;
    return h[jj * nU + ii];
  };
  const pick = (a, bb, c, step) => {
    // 중앙차분이 안 되면 한쪽만으로 낸다
    if (!Number.isNaN(a) && !Number.isNaN(bb)) return (bb - a) / (2 * step);
    if (!Number.isNaN(bb) && !Number.isNaN(c)) return (bb - c) / step;
    if (!Number.isNaN(a) && !Number.isNaN(c)) return (c - a) / step;
    return 0;
  };
  const gu = pick(at(i - 1, j), at(i + 1, j), at(i, j), du);
  const gv = pick(at(i, j - 1), at(i, j + 1), at(i, j), dv);
  out.set(0, 0, 0);
  out[u] = -gu * dir;
  out[v] = -gv * dir;
  out[d] = dir;
  return out.normalize();
}

function tracePath(field, spacing) {
  const { h, nU, nV, u0, u1, v0, v1, u, v, d, dir } = field;
  const span = v1 - v0;
  const du = (u1 - u0) / (nU - 1);
  // 한 스텝에서 허용할 최대 깊이차. 이걸 넘으면 단차이므로 잇지 않는다.
  const maxRise = du * Math.tan(MAX_SLOPE) * 1.6;

  const lanes = [];
  const normals = [];
  let sweep = 1;
  for (let vv = v0; vv <= v1; vv += spacing) {
    const jf = ((vv - v0) / span) * (nV - 1);
    const j = Math.max(0, Math.min(nV - 1, Math.round(jf)));
    const lane = [];
    const lnorm = [];
    const lidx = [];
    for (let i = 0; i < nU; i++) {
      const idx = sweep > 0 ? i : nU - 1 - i;
      const dv2 = h[j * nU + idx];
      if (Number.isNaN(dv2)) continue;
      const n = fieldNormal(field, idx, j, new THREE.Vector3());
      // 필드를 마주보지 않는 면은 이 방향에서 접근할 수 없다
      if (Math.acos(Math.min(1, Math.max(-1, n[d] * dir))) > MAX_SLOPE) continue;
      const p = new THREE.Vector3();
      p[d] = dv2;
      p[u] = u0 + (u1 - u0) * (idx / (nU - 1));
      p[v] = vv;
      lane.push(p);
      lnorm.push(n);
      lidx.push(idx);
    }
    if (lane.length > 2) { lanes.push(lane); normals.push(lnorm); lane.idx = lidx; }
    sweep *= -1;
  }

  const pts = [];
  lanes.forEach((l) => {
    for (let i = 1; i < l.length; i++) {
      // 표본이 인접해 있고 단차가 아닐 때만 잇는다
      const step = Math.abs(l.idx[i] - l.idx[i - 1]);
      if (step > 2) continue;
      if (Math.abs(l[i][d] - l[i - 1][d]) > maxRise * step) continue;
      pts.push(l[i - 1], l[i]);
    }
  });
  return { pts, lanes, normals };
}

function envTexture(renderer) {
  const w = 1024, h = 512;
  const cv = document.createElement('canvas');
  cv.width = w; cv.height = h;
  const g = cv.getContext('2d');
  g.fillStyle = '#0a0d12'; g.fillRect(0, 0, w, h);
  const sky = g.createLinearGradient(0, 0, 0, h * 0.5);
  sky.addColorStop(0, '#2b323d'); sky.addColorStop(1, '#0d1116');
  g.fillStyle = sky; g.fillRect(0, 0, w, h * 0.5);
  const box = (cx, cy, bw, bh, i) => {
    const rg = g.createRadialGradient(cx, cy, 0, cx, cy, Math.max(bw, bh));
    rg.addColorStop(0, `rgba(255,255,255,${i})`);
    rg.addColorStop(0.45, `rgba(220,230,245,${i * 0.45})`);
    rg.addColorStop(1, 'rgba(0,0,0,0)');
    g.fillStyle = rg;
    g.save(); g.translate(cx, cy); g.scale(1, bh / bw);
    g.beginPath(); g.arc(0, 0, bw, 0, Math.PI * 2); g.fill(); g.restore();
  };
  box(w * 0.5, h * 0.12, 340, 150, 1.0);
  box(w * 0.12, h * 0.36, 170, 240, 0.6);
  box(w * 0.88, h * 0.34, 150, 220, 0.5);

  /* 스트립 조명 — 경계가 뚜렷한 광원.

     거칠기가 하는 일은 반사를 뭉개는 것이다. 그런데 반사할 대상이
     위의 부드러운 blob 뿐이면 뭉개나 마나 같아 보여서, 무광(0.82)과
     유광(0.14)의 차이가 화면에 안 나타난다. 실제로 재보니 완전 무광과
     완전 유광의 픽셀차가 평균 1.77/255 였다 — 셰이더는 도는데 안 보였다.

     자동차 스튜디오가 긴 스트립 조명을 쓰는 이유가 이것이다.
     무광에서는 넓고 흐린 띠로, 유광에서는 날카로운 선으로 맺힌다.
     그 대비가 곧 '닦였다'는 신호다. */
  const strip = (x0, y0, sw, sh, i) => {
    // 짧은 축만 아주 좁게 풀어 준다. 여기를 넓히면 다시 blob 이 된다
    const lg = g.createLinearGradient(0, y0, 0, y0 + sh);
    lg.addColorStop(0, 'rgba(255,255,255,0)');
    lg.addColorStop(0.10, `rgba(255,255,255,${i})`);
    lg.addColorStop(0.90, `rgba(255,255,255,${i})`);
    lg.addColorStop(1, 'rgba(255,255,255,0)');
    g.fillStyle = lg;
    g.fillRect(x0, y0, sw, sh);
  };
  /* 위치가 중요하다. 카메라 앙각이 약 11도라, 후드·루프 같은 수평 패널이
     카메라로 되돌려 보내는 반사는 앙각 11도 부근(equirect y≈226)에서 온다.
     천장 꼭대기(y≈50)에 걸면 아무 패널에도 안 잡힌다 — 처음에 그렇게 두고
     효과가 없다고 착각했다. 곡면이 각도를 훑으므로 앙각 6~30도를 덮는다. */
  strip(w * 0.04, h * 0.345, w * 0.34, 20, 0.95);   // 앙각 ~28도
  strip(w * 0.46, h * 0.385, w * 0.30, 17, 0.90);   // ~20도
  strip(w * 0.16, h * 0.425, w * 0.46, 14, 1.00);   // ~13도 — 수평 패널이 이걸 문다
  strip(w * 0.70, h * 0.445, w * 0.26, 12, 0.80);   // ~10도

  g.fillStyle = 'rgba(200,215,235,0.4)';
  g.fillRect(0, h * 0.455, w, 5);
  const tex = new THREE.Texture(cv);
  tex.mapping = THREE.EquirectangularReflectionMapping;
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.needsUpdate = true;
  const pm = new THREE.PMREMGenerator(renderer);
  const env = pm.fromEquirectangular(tex).texture;
  pm.dispose(); tex.dispose();
  return env;
}


/* ══════════════════════════════════════════════════════════════════
   Doosan M0609 관절 리그

   자산: assets/models/robot_arm.opt.glb — base_link·link_1…link_6·
   tool_sander 가 각각 별도 노드이고, 지오메트리는 "포즈된 상태의 월드
   좌표"로 구워져 있다.

   피벗 거리는 rmpflow/m0609_isaac_sim.urdf 의 링크 길이
   (어깨 0.1345 · 상완 0.411 · 전완 0.368 · 손목 0.121)로 강제했고,
   축 방향은 포즈된 형상에서 유도했다. 유도 과정은 rig-check.html 로
   검수할 수 있다.
   ══════════════════════════════════════════════════════════════════ */
// 청크가 블롭 URL 로 실행되므로 상대경로가 으로 풀리지 않는다.
// 자산은 위치를 기준으로 절대화한다.
const asset = (rel) => new URL(rel, location.href).href;
const ARM_URL = asset('assets/models/robot_arm.opt.glb');
/* 레일 폭 배율 — 받침판 폭을 여기서 역산하므로 레일 배치와 반드시 같은 값을 쓴다 */
const RAIL_SX = 0.55;
const LIFT_URL = asset('assets/models/lift.opt.glb');   // 텔레스코픽 컬럼 0.80 x 2.11 x 0.80 m
const RAIL_URL = asset('assets/models/rail.opt.glb');   // 직선 레일, 원본 길이 22.86 m
const MESHOPT_URL = asset('assets/vendor/meshopt_decoder.mjs');
// 차체는 디스크의 압축본을 쓴다. 번들에 박아 두면 HTML 하나가 6MB 가 된다 —
/* 차종 3개. 예전에는 셋이 Z4 하나를 색만 바꿔 쓰고 있었다 — 에
   벤츠·페라리 실제 스캔을 넣었다. 감축 과정은
   tint 는 도장색이다. 세 모델 모두 재질이 하나뿐이라 원본 색이 의미가 없다. */
const CAR_MODELS = {
  z4:    { url: asset('assets/models/car.opt.glb'),     tint: 0x2f343b },
  coupe: { url: asset('assets/models/benz.opt.glb'),    tint: 0x3a3f47 },
  sf90:  { url: asset('assets/models/ferrari.opt.glb'), tint: 0x353a42 },
  sonata:{ url: asset('assets/models/sonata.opt.glb'),  tint: 0x3f444c },
};

const RIG = {
  j1: { p: [0.000000, 0.134500, 0.000000], a: [0.000000, 1.000000, 0.000000] },
  j2: { p: [0.000000, 0.134500, 0.000000], a: [0.018108, 0.000991, -0.999836] },
  j3: { p: [-0.372969, 0.307046, -0.006584], a: [0.018108, 0.000991, -0.999836] },
  j4: { p: [-0.240990, 0.650555, -0.003853], a: [0.358637, 0.933447, 0.007420] },
  j5: { p: [-0.240990, 0.650555, -0.003853], a: [-0.024697, 0.017434, -0.999543] },
  j6: { p: [-0.120059, 0.653060, -0.007083], a: [0.761661, 0.647932, -0.007518] },
};
const RIG_ORDER = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6'];
const RIG_LINK = { j1: 'link_1', j2: 'link_2', j3: 'link_3', j4: 'link_4', j5: 'link_5', j6: 'link_6' };
// URDF joint limit — 실물 가동범위를 넘기면 즉시 가짜로 보인다
const RIG_LIMITS = [[-Math.PI, Math.PI], [-Math.PI, Math.PI], [-2.72, 2.72],
                    [-Math.PI, Math.PI], [-2.27, 2.27], [-Math.PI, Math.PI]];
const PAD_OFFSET = 0.046;   // j6 → 패드 접촉면 (m), 툴 정점 26만 개에서 실측
const REACH = 0.86;         // 어깨에서 패드까지 실용 도달거리 (m)
// 관절 각속도 상한 (rad/s). IK 는 목표가 튀면 해도 튄다 — 실물처럼 한 프레임에
// 꺾이지 않게 여기서 한 번 거른다. M0609 정격 선회속도가 이 언저리다.
const JOINT_RATE = 3.2;
// 레일 대차 속도 (m/s). 지수 추종으로 두면 대차가 목표에 0.2 m 씩 뒤처지고,
// 그만큼 팔이 닿지 못해 패드가 표면에서 뜬다. 속도만 제한하고 따라붙게 한다.
const RAIL_SPEED = 1.8;

/* meshopt/KHR_mesh_quantization 로 구운 GLB 는 위치·법선을 정수로 담고
   노드 스케일로 복원한다. 이 상태에서 geometry.applyMatrix4() 를 쓰면
   변환한 실수값을 정수 배열에 도로 써 넣어 형상이 뭉개진다.
   행렬을 적용하기 전에 float 으로 풀어둔다. */
function dequantize(geo) {
  for (const name of ['position', 'normal']) {
    const a = geo.attributes[name];
    if (!a || (!a.normalized && a.array.BYTES_PER_ELEMENT >= 4)) continue;
    const f = new Float32Array(a.count * a.itemSize);
    for (let i = 0; i < a.count; i++) {
      f[i * a.itemSize] = a.getX(i);
      f[i * a.itemSize + 1] = a.getY(i);
      if (a.itemSize > 2) f[i * a.itemSize + 2] = a.getZ(i);
    }
    geo.setAttribute(name, new THREE.BufferAttribute(f, a.itemSize));
  }
  return geo;
}

async function loadArmParts(url) {
  const { GLTFLoader } = await loadAddon('loaders/GLTFLoader.js');
  // meshopt 로 압축한 GLB 다. 디코더를 붙이지 않으면 로더가 거부한다.
  const { MeshoptDecoder } = await import(MESHOPT_URL);
  const gltf = await new GLTFLoader().setMeshoptDecoder(MeshoptDecoder).loadAsync(url);
  const parts = {};
  gltf.scene.updateMatrixWorld(true);
  gltf.scene.traverse((o) => {
    if (!o.isMesh) return;
    const g = dequantize(o.geometry.clone());
    g.applyMatrix4(o.matrixWorld);
    if (!g.attributes.normal) g.computeVertexNormals();
    parts[o.name.replace(/\.\d+$/, '')] = g;
  });
  const need = ['base_link', 'link_1', 'link_2', 'link_3', 'link_4', 'link_5', 'link_6', 'tool_sander'];
  const missing = need.filter((k) => !parts[k]);
  if (missing.length) throw new Error('링크 누락: ' + missing.join(', '));
  return parts;
}

/** 단일 메시 프롭(리프트·레일)을 지오메트리 하나로 읽는다. */
async function loadProp(url, longAxis) {
  const { GLTFLoader } = await loadAddon('loaders/GLTFLoader.js');
  const { MeshoptDecoder } = await import(MESHOPT_URL);
  const gltf = await new GLTFLoader().setMeshoptDecoder(MeshoptDecoder).loadAsync(url);
  const geos = [];
  gltf.scene.updateMatrixWorld(true);
  gltf.scene.traverse((o) => {
    if (!o.isMesh) return;
    const g = dequantize(o.geometry.clone());
    g.applyMatrix4(o.matrixWorld);
    if (!g.attributes.normal) g.computeVertexNormals();
    geos.push(g);
  });
  if (!geos.length) throw new Error('빈 프롭: ' + url);
  let geo = geos[0];
  if (geos.length > 1) {
    const { mergeGeometries } = await loadAddon('utils/BufferGeometryUtils.js');
    geo = mergeGeometries(geos, false);
  }
  /* 자산이 누워서 구워져 있다 — 실측하면 리프트는 기둥축이 Z(0.80 x 0.80 x 2.11 m),
     레일은 길이가 Y(0.89 x 22.86 x 0.19 m) 다. 배치 코드가 축을 추측하는 대신
     여기서 가장 긴 변을 요청한 축으로 돌려 둔다. 이걸 빼면 리프트가 눕고
     레일이 22.86 m 짜리 벽으로 선다. */
  geo.computeBoundingBox();
  const s0 = geo.boundingBox.getSize(new THREE.Vector3());
  const cur = s0.x >= s0.y && s0.x >= s0.z ? 'x' : (s0.y >= s0.z ? 'y' : 'z');
  if (longAxis && cur !== longAxis) {
    const R = { zy: ['x', -1], yz: ['x', 1], xy: ['z', 1],
                yx: ['z', -1], xz: ['y', -1], zx: ['y', 1] }[cur + longAxis];
    const m = new THREE.Matrix4();
    if (R[0] === 'x') m.makeRotationX(R[1] * Math.PI / 2);
    else if (R[0] === 'y') m.makeRotationY(R[1] * Math.PI / 2);
    else m.makeRotationZ(R[1] * Math.PI / 2);
    geo.applyMatrix4(m);
  }

  // 원점을 "바닥 중앙"으로 통일한다. 배치 코드가 자산의 원점을 추측하지 않게.
  geo.computeBoundingBox();
  const b = geo.boundingBox;
  geo.translate(-(b.min.x + b.max.x) / 2, -b.min.y, -(b.min.z + b.max.z) / 2);
  geo.computeBoundingBox();
  geo.userData.size = geo.boundingBox.getSize(new THREE.Vector3());
  return geo;
}

function robotCell(parts, mats) {
  const g = new THREE.Group();

  // 받침대는 layoutCells 에서 원통 또는 텔레스코픽 리프트로 채운다
  const stand = new THREE.Group();
  g.add(stand);
  g.userData.stand = stand;

  const arm = new THREE.Group();
  g.add(arm);
  g.userData.armRoot = arm;

  const mk = (geo, m) => {
    const o = new THREE.Mesh(geo, m);
    o.castShadow = true; o.receiveShadow = true;
    return o;
  };
  arm.add(mk(parts.base_link, mats.dark));

  const joints = [];
  let parent = arm;
  let prev = new THREE.Vector3();
  for (const key of RIG_ORDER) {
    const p = new THREE.Vector3().fromArray(RIG[key].p);
    const node = new THREE.Group();
    node.position.copy(p).sub(prev);
    node.userData.axis = new THREE.Vector3().fromArray(RIG[key].a).normalize();
    parent.add(node);
    const mesh = mk(parts[RIG_LINK[key]], mats.arm);
    mesh.position.copy(p).negate();          // 월드로 구운 지오메트리를 되돌린다
    node.add(mesh);
    joints.push(node);
    parent = node;
    prev = p;
  }

  const j6p = new THREE.Vector3().fromArray(RIG.j6.p);
  const j6a = new THREE.Vector3().fromArray(RIG.j6.a).normalize();

  const toolMesh = mk(parts.tool_sander, mats.tool);
  toolMesh.position.copy(j6p).negate();
  parent.add(toolMesh);

  // 패드 앵커 — 원점이 접촉면, +Z 가 바깥 법선
  const padAnchor = new THREE.Object3D();
  padAnchor.position.copy(j6a).multiplyScalar(PAD_OFFSET);
  padAnchor.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), j6a);
  parent.add(padAnchor);

  const padDisc = new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.055, 0.005, 36), mats.pad);
  padDisc.rotation.x = Math.PI / 2;
  padDisc.position.z = 0.0025;
  padAnchor.add(padDisc);

  g.userData.joints = joints;
  g.userData.padAnchor = padAnchor;
  g.userData.padDisc = padDisc;
  g.userData.q = new Array(6).fill(0);
  // 툴 축 위의 보조점 — 위치와 자세를 한 번에 맞추기 위한 두 번째 제어점
  g.userData.padBack = new THREE.Object3D();
  g.userData.padBack.position.z = -0.12;
  padAnchor.add(g.userData.padBack);
  return g;
}

function setCellQ(cell, q) {
  const js = cell.userData.joints;
  for (let i = 0; i < 6; i++) {
    const v = THREE.MathUtils.clamp(q[i], RIG_LIMITS[i][0], RIG_LIMITS[i][1]);
    cell.userData.q[i] = v;
    js[i].quaternion.setFromAxisAngle(js[i].userData.axis, v);
  }
}

/* CCD IK — 패드 접촉면과 툴 축 위 보조점, 두 점을 동시에 맞춰
   위치와 자세를 한 번에 푼다. 이전 프레임 해에서 웜스타트한다. */
const _wp = new THREE.Vector3(), _wb = new THREE.Vector3(), _ax = new THREE.Vector3();
const _v1 = new THREE.Vector3(), _v2 = new THREE.Vector3(), _jp = new THREE.Vector3();
const _q = new THREE.Quaternion();
const _ikPad = new THREE.Vector3(), _ikBack = new THREE.Vector3(), _ikBase = new THREE.Vector3();
const _tgtP = new THREE.Vector3(), _tgtN = new THREE.Vector3(), _leadP = new THREE.Vector3();

function solveIK(cell, targetPad, targetBack, iterations = 4, tol = 0.0015) {
  const js = cell.userData.joints;
  const q = cell.userData.q;
  let residual = Infinity;
  for (let it = 0; it < iterations; it++) {
    // 후반 반복에서는 자세보다 위치를 우선한다. 두 목표가 서로 당기면
    // 어느 쪽도 수렴하지 못하고 잔차가 남는다.
    const backW = it < iterations * 0.5 ? 0.6 : 0.15;
    for (let i = 5; i >= 0; i--) {
      const node = js[i];
      node.getWorldPosition(_jp);
      _ax.copy(node.userData.axis).applyQuaternion(node.getWorldQuaternion(_q)).normalize();

      cell.userData.padAnchor.getWorldPosition(_wp);
      cell.userData.padBack.getWorldPosition(_wb);

      // 두 제어점의 오차를 관절축 둘레의 회전각 하나로 근사한다
      let num = 0, den = 0;
      for (const [cur, goal, w] of [[_wp, targetPad, 1.0], [_wb, targetBack, backW]]) {
        _v1.copy(cur).sub(_jp);
        _v1.addScaledVector(_ax, -_v1.dot(_ax));        // 축에 수직인 성분만
        if (_v1.lengthSq() < 1e-8) continue;
        _v2.copy(goal).sub(_jp);
        _v2.addScaledVector(_ax, -_v2.dot(_ax));
        if (_v2.lengthSq() < 1e-8) continue;
        const r = _v1.length();
        _v1.normalize(); _v2.normalize();
        const cross = _v1.clone().cross(_v2).dot(_ax);
        const dot = THREE.MathUtils.clamp(_v1.dot(_v2), -1, 1);
        num += w * r * Math.atan2(cross, dot);
        den += w * r;
      }
      if (den < 1e-8) continue;
      const step = THREE.MathUtils.clamp(num / den, -0.35, 0.35);
      q[i] = THREE.MathUtils.clamp(q[i] + step, RIG_LIMITS[i][0], RIG_LIMITS[i][1]);
      node.quaternion.setFromAxisAngle(node.userData.axis, q[i]);
      node.updateMatrixWorld(true);
    }
    cell.userData.padAnchor.getWorldPosition(_wp);
    residual = _wp.distanceTo(targetPad);
    if (residual < tol) break;
  }
  cell.userData.residual = residual;
  return residual;
}

class PolyTwinViewport extends HTMLElement {
  connectedCallback() {
    if (this._init) {
      cancelAnimationFrame(this._raf);
      this._raf = requestAnimationFrame(this._tick.bind(this));
      addEventListener('resize', this._onResize);
      this._onResize();
      return;
    }
    this._init = true;
    this.style.display = 'block';

    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setPixelRatio(Math.min(devicePixelRatio, 1.5));
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.25;
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.setClearColor(0x000000, 0);
    this.appendChild(renderer.domElement);
    Object.assign(renderer.domElement.style, { display: 'block', width: '100%', height: '100%' });

    const scene = new THREE.Scene();
    scene.environment = envTexture(renderer);
    scene.environmentIntensity = 1.0;

    const camera = new THREE.PerspectiveCamera(34, 1, 0.1, 100);
    camera.position.set(5.6, 2.3, 6.2);

    const key = new THREE.DirectionalLight(0xffffff, 3.0);
    key.position.set(2.4, 6.5, 3.2);
    key.castShadow = true;
    key.shadow.mapSize.set(1024, 1024);
    key.shadow.autoUpdate = false;
    key.shadow.needsUpdate = true;
    key.shadow.camera.left = -6; key.shadow.camera.right = 6;
    key.shadow.camera.top = 6; key.shadow.camera.bottom = -6;
    key.shadow.camera.near = 1; key.shadow.camera.far = 24;
    key.shadow.normalBias = 0.03;
    const rimL = new THREE.DirectionalLight(0xbcd4ff, 1.5);
    rimL.position.set(-7, 3.4, -4.5);
    const rimR = new THREE.DirectionalLight(0xffd9c2, 1.1);
    rimR.position.set(7.5, 2.6, -3.5);
    const hemi = new THREE.HemisphereLight(0x8496ad, 0x080a0d, 0.5);
    scene.add(key, rimL, rimR, hemi);

    const ground = new THREE.Mesh(
      new THREE.PlaneGeometry(60, 60),
      new THREE.MeshPhysicalMaterial({ color: 0x0b0e12, roughness: 0.92, metalness: 0 })
    );
    ground.rotation.x = -Math.PI / 2;
    ground.receiveShadow = true;
    scene.add(ground);

    const grid = new THREE.GridHelper(24, 48, 0x1c2128, 0x141821);
    grid.position.y = 0.002;
    scene.add(grid);

    const cars = new THREE.Group();
    scene.add(cars);

    const raster = new THREE.LineSegments(
      new THREE.BufferGeometry(),
      attachPathFade(new THREE.LineBasicMaterial({ color: ACCENT, transparent: true, opacity: 0.62 }))
    );
    cars.add(raster);

    const head = new THREE.Mesh(
      new THREE.SphereGeometry(0.05, 16, 12),
      new THREE.MeshBasicMaterial({ color: ACCENT, transparent: true, opacity: 0.85 })
    );
    head.visible = false;
    cars.add(head);

    const robots = new THREE.Group();
    scene.add(robots);
    const props = new THREE.Group();      // 리프트·레일·갠트리 등 설비
    scene.add(props);

    // M0609 의 도달거리는 0.9 m 다. 셀은 차체 바운딩 박스에서 계산해
    // 작업면 바로 옆에 세운다 — 고정 좌표로 3 m 밖에 두면 영원히 닿지 않는다.
    const armMats = {
      dark: new THREE.MeshPhysicalMaterial({ color: 0x22262c, metalness: 0.5, roughness: 0.55 }),
      arm: new THREE.MeshPhysicalMaterial({
        color: 0xb9c0c8, metalness: 0.35, roughness: 0.38,
        clearcoat: 0.4, clearcoatRoughness: 0.3, envMapIntensity: 1.1,
      }),
      tool: new THREE.MeshPhysicalMaterial({ color: 0x2A3038, metalness: 0.6, roughness: 0.4 }),
      pad: new THREE.MeshPhysicalMaterial({ color: 0x3E9DBE, metalness: 0.1, roughness: 0.72 }),
      lift: new THREE.MeshPhysicalMaterial({ color: 0x5b626b, metalness: 0.45, roughness: 0.62, envMapIntensity: 0.45 }),
      rail: new THREE.MeshPhysicalMaterial({ color: 0x474d55, metalness: 0.5, roughness: 0.58, envMapIntensity: 0.4 }),
    };

    Object.assign(this, { renderer, scene, camera, cars, raster, head, robots, props, ground, grid, armMats });
    this._cells = [];
    this._models = new Map();
    this._armParts = null;

    Promise.all([
      loadArmParts(ARM_URL),
      loadProp(LIFT_URL, 'y').catch((e) => { console.error('리프트 로드 실패:', e); return null; }),
      loadProp(RAIL_URL, 'z').catch((e) => { console.error('레일 로드 실패:', e); return null; }),
    ]).then(([parts, lift, rail]) => {
      this._armParts = parts;
      this._liftGeo = lift;
      this._railGeo = rail;
      this.layoutCells();
    }).catch((err) => console.error('로봇 팔 로드 실패:', err));
    this._params = { pad: 110, overlap: 40, feed: 0.03, force: 8, rpm: 3000, robotCount: 1,
                     hasRail: false, hasLift: false, carLift: 0, running: false };
    this._t0 = performance.now();
    this._vehicle = null;

    this._onResize = () => {
      const w = this.clientWidth || 800, h = this.clientHeight || 600;
      renderer.setSize(w, h, false);
      camera.aspect = w / h;
      camera.fov = THREE.MathUtils.clamp(34 * (1.5 / camera.aspect), 34, 50);
      camera.updateProjectionMatrix();
    };
    addEventListener('resize', this._onResize);
    this._onResize();

    loadAddon('controls/OrbitControls.js').then(({ OrbitControls }) => {
      const c = new OrbitControls(camera, renderer.domElement);
      c.target.set(0, 0.72, 0);
      c.enableDamping = true;
      c.dampingFactor = 0.08;
      c.minDistance = 3.2;
      c.maxDistance = 14;
      c.maxPolarAngle = Math.PI / 2 - 0.04;
      c.update();
      this._controls = c;
    });

    this.setVehicle(this.getAttribute('vehicle') || 'z4');
    this._raf = requestAnimationFrame(this._tick.bind(this));
  }

  disconnectedCallback() {
    cancelAnimationFrame(this._raf);
    removeEventListener('resize', this._onResize);
  }

  async setVehicle(id) {
    if (this._vehicle === id) return;
    this._vehicle = id;
    if (this._models.has(id)) {
      this._showVehicle(id);
      return;
    }
    /* 차종마다 자기 모델을 받는다. 처음 고른 것만 내려받는다 —
       셋을 미리 다 받으면 첫 화면이 1.5MB 무거워진다. */
    const car = CAR_MODELS[id] || CAR_MODELS.z4;
    // 같은 차종을 연타해도 파스는 한 번만
    this._carLoads = this._carLoads || new Map();
    if (!this._carLoads.has(id)) this._carLoads.set(id, loadCar(car.url));
    let model;
    try {
      model = await this._carLoads.get(id);
    } catch (err) {
      this._carLoads.delete(id);
      console.error('차체 모델을 불러오지 못했다: ' + car.url, err);
      return;
    }
    model.traverse((o) => { if (o.isMesh) o.material.color.setHex(car.tint); });
    this._models.set(id, model);
    // 받는 사이에 다른 차종으로 넘어갔으면 늦게 온 응답이 화면을 덮지 않게 한다
    if (this._vehicle !== id) return;
    this._showVehicle(id);
  }

  /** 연마 마스크를 하이트필드와 같은 격자로 만든다. */
  initPolish() {
    const f = this._field;
    const data = new Uint8Array(f.nLong * f.nCross);
    const tex = new THREE.DataTexture(data, f.nLong, f.nCross, THREE.RedFormat, THREE.UnsignedByteType);
    tex.minFilter = tex.magFilter = THREE.LinearFilter;
    tex.wrapS = tex.wrapT = THREE.ClampToEdgeWrapping;
    tex.needsUpdate = true;
    this._polish = { tex, data };
    const sel = (axis) => new THREE.Vector3(axis === 'x' ? 1 : 0, 0, axis === 'z' ? 1 : 0);
    polishU.uPolish.value = tex;
    polishU.uLongSel.value = sel(f.long);
    polishU.uCrossSel.value = sel(f.cross);
    polishU.uFieldMin.value = new THREE.Vector2(f.l0, f.c0);
    polishU.uFieldSpan.value = new THREE.Vector2(f.l1 - f.l0, f.c1 - f.c0);
    polishU.uPolishOn.value = 1;
  }

  /** 패드가 지나간 자리를 마스크에 찍는다. 반지름은 패드 지름 파라미터 그대로. */
  stampPolish(worldPos, radius) {
    const P = this._polish, f = this._field;
    if (!P) return;
    const u = (worldPos[f.long] - f.l0) / (f.l1 - f.l0);
    const v = (worldPos[f.cross] - f.c0) / (f.c1 - f.c0);
    if (u < 0 || u > 1 || v < 0 || v > 1) return;
    const ci = u * (f.nLong - 1), cj = v * (f.nCross - 1);
    const ri = Math.max(1, radius / ((f.l1 - f.l0) / (f.nLong - 1)));
    const rj = Math.max(1, radius / ((f.c1 - f.c0) / (f.nCross - 1)));
    let touched = false;
    for (let j = Math.floor(cj - rj); j <= Math.ceil(cj + rj); j++) {
      if (j < 0 || j >= f.nCross) continue;
      for (let i = Math.floor(ci - ri); i <= Math.ceil(ci + ri); i++) {
        if (i < 0 || i >= f.nLong) continue;
        const d = Math.hypot((i - ci) / ri, (j - cj) / rj);
        if (d > 1) continue;
        // 가장자리는 덜 먹인다 — 패드 압력 분포와 같은 모양
        const k = j * f.nLong + i;
        const nv = Math.min(255, P.data[k] + 255 * (1 - d * d) * 0.9);
        if (nv !== P.data[k]) { P.data[k] = nv; touched = true; }
      }
    }
    if (touched) P.tex.needsUpdate = true;
  }

  /** 차종을 바꾸거나 공정을 다시 시작할 때 연마 상태를 지운다. */
  resetPolish() {
    if (!this._polish) return;
    this._polish.data.fill(0);
    this._polish.tex.needsUpdate = true;
  }

  /** 차체 바운딩 박스를 기준으로 셀을 배치한다. 대수는 setParams 로 바뀐다. */
  layoutCells() {
    try { this._layoutCells(); }
    catch (err) { console.error('layoutCells 실패:', err); }
  }

  _layoutCells() {
    if (!this._armParts || !this._model) return;
    const p = this._params;
    const want = Math.max(1, Math.min(3, p.robotCount || 1));
    const sig = [want, !!p.hasRail, !!p.hasLift, p.carLift || 0, this._model.uuid].join('|');
    if (sig === this._cellSig) return;
    this._cellSig = sig;

    this._cells.forEach((c) => this.robots.remove(c));
    while (this.props.children.length) this.props.remove(this.props.children[0]);
    this._cells = [];

    this._model.updateMatrixWorld(true);
    const b = new THREE.Box3().setFromObject(this._model);
    const size = b.getSize(new THREE.Vector3());
    const mid = b.getCenter(new THREE.Vector3());
    const surfaceY = b.max.y;

    /* 차체의 길이축과 폭축. 경로 생성기(buildHeightField)와 같은 규칙을 쓴다.
       레일은 길이축을 따라 깔리고 로봇은 폭축 바깥에 선다. 축을 x/z 로 박아 두면
       차종을 90도 돌려 구운 순간 레일과 로봇이 어긋난다. */
    const LONG = (this._field && this._field.long) || (size.x >= size.z ? 'x' : 'z');
    const CROSS = LONG === 'x' ? 'z' : 'x';
    this._axes = { LONG, CROSS };
    const half = size[CROSS] / 2 + 0.46;
    const l0 = mid[LONG] - size[LONG] * 0.42;
    const l1 = mid[LONG] + size[LONG] * 0.42;
    /* 받침대 높이는 차체 리프트를 따라가지 않는다. 같이 올라가면 어깨와
       옆면의 상대 높이가 그대로라 차를 든 의미가 없다 — 실제로 리프트는
       차를 로봇 쪽으로 올리는 장치다. 차를 들면 사이드실이 어깨로 올라오고
       그만큼 루프는 도달 범위 밖으로 나간다. 그 맞바꿈이 이 슬라이더다. */
    const standY = Math.max(0.2, surfaceY - (p.carLift || 0) - 0.52);
    /* 레일은 ㄷ자 채널이다 — 실측하면 양쪽 립 상면이 0.190, 가운데 바닥판은
       0.025 다. 받침대는 립 상면(= 바운딩 박스 상면)에서 시작한다. 이걸 빼면
       레일이 기둥 밑동을 관통하고 받침판이 레일 밖 바닥에 걸쳐 앉는다. */
    const railH = (p.hasRail && this._railGeo) ? this._railGeo.userData.size.y : 0;
    /* 받침판 폭 — 립 위에 걸쳐 앉아야 한다. 좁으면 립 사이 빈 곳 위에
       떠 보이고, 넓으면 레일 밖으로 나간다. 레일 폭에서 역산하고 양옆
       30 mm 씩 물린다. 레일이 없으면 원래 비율(0.62)을 쓴다. */
    const standS = (p.hasRail && this._railGeo && this._liftGeo)
      ? (this._railGeo.userData.size.x * RAIL_SX - 0.06) / this._liftGeo.userData.size.x
      : 0.62;

    /** 받침대 — 리프트를 켜면 텔레스코픽 컬럼, 아니면 원통 기둥.
        리프트 지오메트리는 loadProp 에서 기둥축을 Y 로 세워 두었다. */
    const buildStand = (cell, height) => {
      const stand = cell.userData.stand;
      while (stand.children.length) stand.remove(stand.children[0]);
      let m;
      if (p.hasLift && this._liftGeo) {
        m = new THREE.Mesh(this._liftGeo, this.armMats.lift);
        m.scale.set(standS, height / this._liftGeo.userData.size.y, standS);
      } else {
        m = new THREE.Mesh(new THREE.CylinderGeometry(0.13, 0.17, 1, 20), this.armMats.dark);
        m.scale.y = height;
        m.position.y = height / 2;
      }
      m.castShadow = true; m.receiveShadow = true;
      stand.add(m);
    };

    /* 슬롯 — [폭축 부호, 길이축 위치]. 두 대 이상이면 반드시 반대편에 세우고
       길이 방향으로도 엇갈리게 둔다. 같은 쪽에 나란히 두면 두 팔이
       같은 공간을 지난다. */
    const SLOTS = [
      [[-1, 0]],
      [[-1, -0.19], [1, 0.19]],
      [[-1, -0.26], [1, 0], [-1, 0.26]],
    ][want - 1];

    for (let i = 0; i < want; i++) {
      const side = SLOTS[i][0];
      const cell = robotCell(this._armParts, this.armMats);
      cell.position.set(0, 0, 0);
      cell.position[CROSS] = mid[CROSS] + side * half;
      cell.position[LONG] = mid[LONG] + size[LONG] * SLOTS[i][1];
      /* 레일 위에 앉히고 기둥을 그만큼 줄인다 — 어깨 높이(standY)는 그대로다.
         차체 리프트를 끝까지 올리면 standY 가 하한 0.2 에 걸리므로 기둥에도
         하한을 둔다. 안 그러면 기둥이 0 이 되어 팔이 레일에 박힌다. */
      const columnH = Math.max(0.2, standY - railH);
      cell.position.y = railH;
      buildStand(cell, columnH);
      cell.userData.armRoot.position.y = columnH;

      /* 베이스는 레일과 나란히 둔다. 차체를 향하는 회전은 1축(선회 관절)이
         맡는다 — 셀 전체를 돌리면 받침대 볼트판이 레일과 어긋나 보인다.
         base_link 는 관절 0 의 부모라 이 회전에 따라 돌지 않는다. */
      cell.rotation.y = 0;
      setCellQ(cell, [Math.atan2(mid.x - cell.position.x, mid.z - cell.position.z), 0, 0, 0, 0, 0]);
      /* 공정 전에는 이 자세로 멈춰 선다. 환경을 구성하는 동안 팔이 혼자
         움직이면 이미 공정이 도는 것처럼 보인다 */
      cell.userData.qHome = cell.userData.q.slice();

      cell.userData.side = side;
      cell.userData.onRail = !!p.hasRail;
      cell.userData.span = [l0, l1];
      cell.userData.home = cell.position[LONG];
      this.robots.add(cell);
      this._cells.push(cell);
    }

    /* 차체 리프트 — 4주. 옆면을 닦으려면 차를 들어 올려야 도어가 로봇 어깨
       높이에 온다. 차체는 _applyCarLift 가 이미 올려 뒀으므로 기둥은
       바닥에서 차체 밑면(b.min.y)까지만 세우면 된다. */
    if ((p.carLift || 0) > 0.01) {
      // 텔레스코픽 컬럼 자산을 눌러 쓰면 내부 축이 드러나 앙상해 보인다.
      // 4주 리프트는 각기둥 + 받침판 + 상판이면 충분히 읽힌다.
      const postH = Math.max(0.12, b.min.y);
      for (const su of [-1, 1]) {
        for (const sc of [-1, 1]) {
          const g = new THREE.Group();
          const col = new THREE.Mesh(new THREE.BoxGeometry(0.15, postH, 0.15), this.armMats.lift);
          col.position.y = postH / 2;
          col.castShadow = true; col.receiveShadow = true;
          g.add(col);
          const foot = new THREE.Mesh(new THREE.BoxGeometry(0.36, 0.035, 0.36), this.armMats.dark);
          foot.position.y = 0.017;
          foot.receiveShadow = true;
          g.add(foot);
          const cap = new THREE.Mesh(new THREE.BoxGeometry(0.28, 0.045, 0.28), this.armMats.dark);
          cap.position.y = postH - 0.022;
          cap.castShadow = true;
          g.add(cap);
          g.position.set(0, 0, 0);
          g.position[LONG] = mid[LONG] + su * size[LONG] * 0.30;
          g.position[CROSS] = mid[CROSS] + sc * size[CROSS] * 0.30;
          this.props.add(g);
        }
      }
    }

    // 바닥 레일 — 셀이 올라타 차체 길이 방향으로 이동한다
    if (p.hasRail && this._railGeo) {
      const seen = new Set();
      for (const cell of this._cells) {
        const c = cell.position[CROSS].toFixed(3);
        if (seen.has(c)) continue;
        seen.add(c);
        const r = new THREE.Mesh(this._railGeo, this.armMats.rail);
        r.scale.set(RAIL_SX, 1, (l1 - l0) / this._railGeo.userData.size.z);
        if (LONG === 'x') r.rotation.y = Math.PI / 2;
        r.position.set(0, 0, 0);
        r.position[CROSS] = Number(c);
        r.position[LONG] = (l0 + l1) / 2;
        r.receiveShadow = true;
        this.props.add(r);
      }
    }

    // 천장 갠트리 — 지붕은 옆에서 못 닿는다. 위에서 한 대가 맡는다.
    if (p.hasLift && this._railGeo && this._liftGeo) {
      const beamY = surfaceY + 1.28;
      const beam = new THREE.Mesh(this._railGeo, this.armMats.rail);
      beam.scale.set(RAIL_SX, 1, (l1 - l0) / this._railGeo.userData.size.z);
      beam.rotation.x = Math.PI;                     // 레일 면을 아래로 향하게
      if (LONG === 'x') beam.rotation.y = Math.PI / 2;
      beam.position.set(0, beamY, 0);
      beam.position[CROSS] = mid[CROSS];
      beam.position[LONG] = (l0 + l1) / 2;
      this.props.add(beam);

      // 문형 프레임 — 양쪽에 기둥을 세우고 위를 가로보로 잇는다
      const postC = half + 0.62;
      for (const lEnd of [l0, l1]) {
        for (const sc of [-1, 1]) {
          const post = new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.09, beamY, 12), this.armMats.dark);
          post.position.set(0, beamY / 2, 0);
          post.position[CROSS] = mid[CROSS] + sc * postC;
          post.position[LONG] = lEnd;
          post.castShadow = true;
          this.props.add(post);
        }
        const cross = new THREE.Mesh(new THREE.BoxGeometry(postC * 2, 0.11, 0.11), this.armMats.dark);
        if (LONG === 'x') cross.rotation.y = Math.PI / 2;
        cross.position.set(0, beamY, 0);
        cross.position[CROSS] = mid[CROSS];
        cross.position[LONG] = lEnd;
        this.props.add(cross);
      }

      // 매달린 리프트 + 팔. 어깨가 지붕 위 0.62 m 에 오도록 길이를 잡는다
      const shoulderY = surfaceY + 0.62;
      const drop = beamY - shoulderY;
      const hang = new THREE.Mesh(this._liftGeo, this.armMats.lift);
      hang.scale.set(0.5, drop / this._liftGeo.userData.size.y, 0.5);
      hang.rotation.x = Math.PI;                     // 뒤집어 매단다
      hang.position.set(0, beamY, 0);
      hang.position[CROSS] = mid[CROSS];
      hang.position[LONG] = mid[LONG];
      this.props.add(hang);

      const cell = robotCell(this._armParts, this.armMats);
      cell.position.set(0, shoulderY, 0);
      cell.position[CROSS] = mid[CROSS];
      cell.position[LONG] = mid[LONG];
      cell.rotation.x = Math.PI;                     // 팔을 아래로
      cell.userData.armRoot.position.y = 0;
      cell.userData.ceiling = true;
      cell.userData.side = 0;                        // 지붕 담당 — 옆면 셀과 구역이 겹치지 않는다
      cell.userData.onRail = false;
      cell.userData.span = [l0, l1];
      cell.userData.home = mid[LONG];
      this.robots.add(cell);
      this._cells.push(cell);
    }

    this.assignWork();
  }

  /** 셀마다 겹치지 않는 작업 구역을 준다.

      점 하나를 '닿을 수 있는 셀 중 가장 가까운 셀' 하나에만 준다(보로노이 분할).
      예전처럼 닿기만 하면 전부 나눠 주면 두 팔이 같은 자리를 노려 서로 파고든다.
      그 다음 레인마다 자기 몫의 '가장 긴 연속 구간' 하나만 남긴다 — 끊긴 조각을
      이어 붙이면 팔이 프레임마다 순간이동하고, 그게 버벅임으로 보인다. */
  assignWork() {
    if (!this._lanes || !this._normals || !this._cells.length) return;
    const { LONG, CROSS } = this._axes || { LONG: 'z', CROSS: 'x' };
    this.robots.updateMatrixWorld(true);

    for (const cell of this._cells) {
      const sh = cell.userData.shoulder || (cell.userData.shoulder = new THREE.Vector3());
      cell.userData.joints[0].getWorldPosition(sh);
    }

    // 레일 위 셀은 길이축으로 미끄러지므로 도달 판정에서 그 축을 뺀다.
    // 다만 구역을 가를 때는 약하게(0.3) 살려 둬야 같은 쪽 두 대가 갈린다.
    const gap = (cell, pt, w) => {
      const dc = pt[CROSS] - cell.userData.shoulder[CROSS];
      const dy = pt.y - cell.userData.shoulder.y;
      const dl = (pt[LONG] - cell.userData.shoulder[LONG]) * (cell.userData.onRail ? w : 1);
      return Math.sqrt(dc * dc + dy * dy + dl * dl);
    };

    const owner = this._lanes.map((lane) => lane.map((pt) => {
      let best = -1, bd = Infinity;
      for (let ci = 0; ci < this._cells.length; ci++) {
        const cell = this._cells[ci];
        // 팔을 다 편 자리는 배정하지 않는다. 경계에서는 CCD 가 수렴하지 못해
        // 패드가 표면에서 20 cm 뜬 채로 따라다닌다 — 닿을 수 있는 곳만 맡긴다.
        if (gap(cell, pt, 0) > REACH * 0.90) continue;
        const d = gap(cell, pt, 0.3);
        if (d < bd) { bd = d; best = ci; }
      }
      return best;
    }));

    for (let ci = 0; ci < this._cells.length; ci++) {
      const cell = this._cells[ci];
      const pts = [], nrm = [];
      this._lanes.forEach((lane, li) => {
        const ns = this._normals[li], own = owner[li];
        // 이 레인에서 내 몫의 가장 긴 연속 구간 하나
        let best = null, run = null;
        for (let k = 0; k < lane.length; k++) {
          if (own[k] === ci) { if (run) run[1] = k; else run = [k, k]; }
          else if (run) { if (!best || run[1] - run[0] > best[1] - best[0]) best = run; run = null; }
        }
        if (run && (!best || run[1] - run[0] > best[1] - best[0])) best = run;
        if (!best || best[1] - best[0] < 3) return;
        for (let k = best[0]; k <= best[1]; k++) { pts.push(lane[k]); nrm.push(ns[k]); }
      });

      cell.userData.work = pts;
      cell.userData.workN = nrm;

      /* 누적 호 길이 — 목표를 이송속도로 전진시키기 위한 것.
         레인이 바뀌는 지점은 실제로 건너뛰는 거리이므로 그대로 넣는다.
         예전처럼 0.10 m 로 깎아 두면 2 m 를 한 프레임에 날아간다. */
      const cum = new Float32Array(pts.length);
      let acc = 0;
      for (let i = 1; i < pts.length; i++) {
        acc += pts[i].distanceTo(pts[i - 1]);
        cum[i] = acc;
      }
      cell.userData.cum = cum;
      cell.userData.pathLen = Math.max(0.01, acc);
      cell.userData.s = (cell.userData.s || 0) % cell.userData.pathLen;
      cell.userData.cursor = 0;
    }
  }

  _showVehicle(id) {
    this._models.forEach((m, k) => {
      if (k === id) { if (!m.parent) this.cars.add(m); m.visible = true; }
      else m.visible = false;
    });
    this._model = this._models.get(id);
    this._cellBox = null;
    this._applyCarLift();
    // 차종마다 다시 굽는다. 예전에는 처음 한 번만 구워서 차를 바꾸면
    // 앞차의 표면 위에 경로가 그려졌다.
    if (this._fieldFor !== this._model.uuid) {
      this._fieldFor = this._model.uuid;
      this._buildFields();
      this.initPolish();
    }
    this.rebuildPath();
    this.layoutCells();
  }

  /** 차체를 리프트 높이만큼 올린다. cars 그룹이 아니라 차체 모델만 옮긴다 —
      경로선(raster)이 같은 그룹에 있어 그룹을 올리면 두 번 올라간다. */
  _applyCarLift() {
    const y = Math.max(0, this._params.carLift || 0);
    this._models.forEach((m) => { m.position.y = y; m.updateMatrixWorld(true); });
  }

  /** 위에서 한 장, 옆에서 두 장. 옆면 두 장이 도어와 펜더를 맡는다. */
  _buildFields() {
    if (!this._model) return;
    this._model.updateMatrixWorld(true);
    this._field = buildHeightField(this.renderer, this._model);
    this._sideFields = buildSideFields(this.renderer, this._model);
  }

  setParams(p) {
    const prev = this._params;
    const spacingChanged = p.pad !== prev.pad || p.overlap !== prev.overlap;
    this._params = { ...prev, ...p };
    const f = THREE.MathUtils.clamp((this._params.force - 3) / 9, 0, 1);
    this.raster.material.opacity = 0.34 + f * 0.5;
    this.raster.material.color.setHSL(0.543, 0.40 + f * 0.22, 0.42 + f * 0.16);
    if (p.carLift !== undefined && p.carLift !== prev.carLift) {
      // 차를 올리면 표면 좌표가 통째로 바뀐다 — 필드부터 다시 굽는다
      this._applyCarLift();
      this._buildFields();
      this.rebuildPath();
      this.layoutCells();
    } else if (spacingChanged) {
      this.rebuildPath();
    }
    if (p.running === true && prev.running === false) {
      this.resetPolish();
      /* 대기 중에 진행량이 남아 있으면 시작하자마자 중간부터 튄다.
         셀마다 조금씩 어긋나게 두는 건 세 대가 같은 자리를 훑지 않게 하려는 것이다 */
      this._cells.forEach((cell, ci) => {
        cell.userData.s = ci * (cell.userData.pathLen || 0) * 0.31;
        cell.userData.cursor = 0;
      });
    }
    if (['robotCount', 'hasRail', 'hasLift'].some((k) => p[k] !== undefined && p[k] !== prev[k])) {
      this.layoutCells();
    }
  }

  rebuildPath() {
    // coalesce bursts of slider input into a single rebuild per frame
    if (this._pathQueued) return;
    this._pathQueued = true;
    requestAnimationFrame(() => {
      this._pathQueued = false;
      if (!this._field) return;
      const { pad, overlap } = this._params;
      const spacing = Math.max(0.03, (pad / 1000) * (1 - overlap / 100));

      // 위에서 한 장 + 옆에서 두 장. 옆면 레인이 없으면 도어에는 경로 자체가 없다.
      const pts = [], lanes = [], normals = [];
      for (const f of [this._field].concat(this._sideFields || [])) {
        if (!f) continue;
        const r = tracePath(f, spacing);
        for (let k = 0; k < r.pts.length; k++) pts.push(r.pts[k]);
        for (let k = 0; k < r.lanes.length; k++) { lanes.push(r.lanes[k]); normals.push(r.normals[k]); }
      }
      this.raster.geometry.dispose();
      this.raster.geometry = new THREE.BufferGeometry().setFromPoints(pts);
      this._lanes = lanes;
      this._flat = lanes.flat();
      // 법선은 하이트필드 기울기에서 나온다 — 패드가 곡면에 눕는 각도가 이것이다
      this._normals = normals;
      this.head.visible = this._flat.length > 0;
      this.assignWork();
    });
  }

  /* 공정 모니터링 패널이 읽는 값. 화면에 보이는 것과 같은 수를 돌려준다 —
     패널에만 있는 숫자를 따로 만들면 둘이 어긋난다. */
  getStats() {
    const cells = this._cells.map((c) => {
      const len = c.userData.pathLen || 0;
      return {
        lap: len > 0 ? ((c.userData.s || 0) / len) : 0,   // 담당 구역 진행 비율
        pts: (c.userData.work || []).length,              // 맡은 작업점 수
        onRail: !!c.userData.onRail,
      };
    });
    return {
      cells,
      lanes: (this._lanes || []).length,
      points: (this._flat || []).length,
      /* 팔이 실제로 맡은 점 수. 도달거리 밖은 아무 셀에도 배정되지 않는다 —
         커버리지가 100% 에 못 가는 이유가 여기 있고, 그건 숨길 값이 아니다 */
      assigned: cells.reduce((n, c) => n + c.pts, 0),
      // 실제로 닦인 면적 비율. 시간 진행률로 대신하면 3D 와 숫자가 어긋난다
      polished: this._polishCoverage(),
    };
  }

  /** 폴리싱 마스크에서 실제로 찍힌 비율. 차체 실루엣이 아니라 필드 기준이다. */
  _polishCoverage() {
    const P = this._polish;
    if (!P) return null;
    const d = P.data;
    let n = 0;
    for (let i = 0; i < d.length; i++) if (d[i] > 8) n++;
    return n / d.length;
  }

  setView(v) {
    const P = {
      front: [new THREE.Vector3(0.1, 1.5, 7.4), new THREE.Vector3(0, 0.7, 0)],
      top: [new THREE.Vector3(0.1, 8.2, 0.6), new THREE.Vector3(0, 0.4, 0)],
      side: [new THREE.Vector3(8.4, 1.8, 0.2), new THREE.Vector3(0, 0.75, 0)],
      free: [new THREE.Vector3(5.6, 2.3, 6.2), new THREE.Vector3(0, 0.72, 0)],
    }[v];
    if (!P) return;
    this._camGoal = { pos: P[0], look: P[1] };
  }

  _tick(now) {
    this._raf = requestAnimationFrame(this._tick.bind(this));
    try { this._frame(now); }
    catch (err) {
      if (!this._tickErr) { this._tickErr = 1; console.error('틱 실패:', err); }
      this.renderer.render(this.scene, this.camera);
    }
  }

  _frame(now) {
    const t = (now - this._t0) / 1000;
    const p = this._params;

    const dt = Math.min(0.05, (now - (this._last || now)) / 1000);
    this._last = now;

    if (this._camGoal) {
      const g = this._camGoal;
      // 프레임 수와 무관하게 같은 속도로 붙는다 — 고정 계수는 120 Hz 에서 두 배 빠르다
      const a = 1 - Math.pow(0.91, dt * 60);
      this.camera.position.lerp(g.pos, a);
      if (this._controls) {
        this._controls.target.lerp(g.look, a);
        if (this.camera.position.distanceTo(g.pos) < 0.02) this._camGoal = null;
      }
    }
    if (this._controls) this._controls.update();

    /* 팔마다 자기 레인을 맡아 이송속도로 훑는다. 패드는 표면점에 닿고
       법선 반대로 눌린다 — 위치와 자세를 IK 로 동시에 푼다. */
    /* 공정 시작 전에는 로봇이 멈춰 있어야 한다. 이 화면은 환경을 구성하는
       화면이고, 팔이 도는 것은 'RL 공정 시작' 이후의 일이다.
       대수를 바꾸면 팔이 늘고 줄되, 늘어난 팔도 대기 자세로 서 있는다. */
    if (!p.running) {
      const rate = JOINT_RATE * dt * 0.5;      // 대기 복귀는 공정보다 느리게
      for (const cell of this._cells) {
        const home = cell.userData.qHome;
        if (!home) continue;
        const q = cell.userData.q;
        let moved = false;
        for (let jj = 0; jj < 6; jj++) {
          const d = home[jj] - q[jj];
          if (Math.abs(d) < 1e-4) continue;
          q[jj] += THREE.MathUtils.clamp(d, -rate, rate);
          moved = true;
        }
        if (moved) setCellQ(cell, q);
        if (cell.userData.padDisc) cell.userData.padDisc.rotation.y = 0;
        cell.userData.qPrev = null;
      }
      this.head.visible = false;              // 작업점 표시도 공정 중에만
      this.renderer.render(this.scene, this.camera);
      return;
    }

    if (this._lanes && this._lanes.length && this._cells.length) {
      // 실제 이송속도(m/s)에 데모 배속을 곱한다. 0.03 m/s 그대로면
      // 한 패스에 2분이 넘어 심사에서 아무 일도 안 일어나 보인다.
      const speed = p.feed * DEMO_SPEEDUP;
      const nCell = this._cells.length;
      let lead = null;

      for (let ci = 0; ci < nCell; ci++) {
        const cell = this._cells[ci];
        const work = cell.userData.work;
        if (!work || work.length < 2) continue;

        /* 자기 구역을 이송속도로 훑는다. 진행량은 비율이 아니라 미터다 —
           작업점이 몇 개든 패드가 실제로 움직이는 속도가 같아야 팔이 따라온다. */
        const cum = cell.userData.cum, len = cell.userData.pathLen;
        let s = (cell.userData.s || (ci * len * 0.31)) + speed * dt;
        if (s >= len) s -= len * Math.floor(s / len);
        cell.userData.s = s;
        // 이전 위치에서 이어 찾는다(대부분 한두 칸)
        let k = cell.userData.cursor || 0;
        if (cum[k] > s) k = 0;
        while (k < work.length - 1 && cum[k + 1] <= s) k++;
        cell.userData.cursor = k;
        if (!work[k]) continue;

        /* 표본과 표본 사이를 보간한다. 웨이포인트에 그대로 스냅하면 목표가
           한 칸씩 튀고 팔이 그 튐을 그대로 따라 한다 — 버벅임의 정체다. */
        const k1 = Math.min(k + 1, work.length - 1);
        const seg = cum[k1] - cum[k];
        const u = seg > 1e-6 ? THREE.MathUtils.clamp((s - cum[k]) / seg, 0, 1) : 0;
        const pt = _tgtP.copy(work[k]).lerp(work[k1], u);
        const nrm = _tgtN.copy(cell.userData.workN[k]).lerp(cell.userData.workN[k1], u).normalize();

        /* 레인이 바뀌는 구간(20 cm 이상 건너뜀)은 표면을 긁으며 가지 않는다.
           실제 공정처럼 법선 방향으로 들었다가 내려놓는다. 이걸 안 하면
           패드가 도장면을 뚫고 직선으로 지나가면서 지나지도 않은 자리를 연마한다. */
        const hop = seg > 0.20 ? Math.sin(Math.PI * u) * Math.min(0.20, seg * 0.26) : 0;
        if (hop > 0) pt.addScaledVector(nrm, hop);
        const contact = hop < 0.004;
        if (ci === 0) lead = _leadP.copy(pt);

        // 레일 위 셀은 목표를 따라 길이 방향으로 미끄러진다
        if (cell.userData.onRail) {
          const LONG = (this._axes && this._axes.LONG) || 'z';
          const [a0, a1] = cell.userData.span;
          const goal = THREE.MathUtils.clamp(pt[LONG], a0, a1);
          const step = RAIL_SPEED * dt;
          cell.position[LONG] += THREE.MathUtils.clamp(goal - cell.position[LONG], -step, step);
          cell.updateMatrixWorld(true);
        }

        // 패드 접촉면 = 표면점, 툴 축은 법선 반대 방향.
        // solveIK 는 월드 좌표로 푼다 — 여기서 로컬로 바꾸면 안 된다.
        // 경로선은 z-fighting 을 피하려고 도장면에서 13 mm 띄워 그린다.
        // 패드는 그 선이 아니라 도장면을 짚어야 하므로 되돌려 겨눈다.
        _ikPad.copy(pt).addScaledVector(nrm, -PATH_LIFT);
        _ikBack.copy(_ikPad).addScaledVector(nrm, 0.12);

        // 이번 프레임 시작 자세를 기억해 두고 푼다
        const q0 = cell.userData.qPrev || (cell.userData.qPrev = cell.userData.q.slice());
        for (let jj = 0; jj < 6; jj++) q0[jj] = cell.userData.q[jj];
        solveIK(cell, _ikPad, _ikBack, 12);

        // 관절 각속도 제한 — 해가 튀어도 팔은 튀지 않게
        const maxStep = JOINT_RATE * dt;
        let over = false;
        for (let jj = 0; jj < 6; jj++) {
          const d = cell.userData.q[jj] - q0[jj];
          if (Math.abs(d) > maxStep) { q0[jj] += Math.sign(d) * maxStep; over = true; }
          else q0[jj] = cell.userData.q[jj];
        }
        if (over) setCellQ(cell, q0);

        // 패드 자전 — 공정 중일 때만 RPM 대로 돈다
        const rpm = p.rpm || 3000;
        cell.userData.padDisc.rotation.y += (rpm / 60) * Math.PI * 2 * dt;

        // 지나간 자리를 무광에서 유광으로
        if (contact) {
          cell.userData.padAnchor.getWorldPosition(_ikBase);
          this.stampPolish(_ikBase, (p.pad / 1000) / 2);
        }
      }

      if (lead) this.head.position.copy(lead);
      this.head.visible = true;
      const pulse = 0.55 + Math.sin(t * 14) * 0.3;
      this.head.material.opacity = pulse;
    }

    this.renderer.render(this.scene, this.camera);
  }
}

if (!customElements.get('polytwin-viewport')) customElements.define('polytwin-viewport', PolyTwinViewport);
