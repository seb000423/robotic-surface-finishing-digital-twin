/* ══════════════════════════════════════════════════════════════
   면적 가중 스무스 법선을 GLB 에 굽는다.

   trimesh 가 뽑은 OBJ 에는 vn 이 없다. 이걸 감축 뒤에 돌려야 한다
   감축 전에 법선을 넣으면 감축기가 그 값을 보간해 뭉갠 법선이 남는다.

   외적을 정규화하지 않고 더한다. 그러면 큰 면이 셰이딩을 지배해
   도장면이 매끄럽게 읽힌다.

     npm i --no-save @gltf-transform/core
     node scripts/bake-normals.mjs in.glb out.glb

   meshopt 압축본은 못 읽는다 (EXT_meshopt_compression). meshopt 는
   이 단계 다음이다 —
   ══════════════════════════════════════════════════════════════ */
import { NodeIO } from '@gltf-transform/core';

const [inp, out] = process.argv.slice(2);
if (!inp || !out) {
  console.error('사용: node scripts/bake-normals.mjs <in.glb> <out.glb>');
  process.exit(1);
}

const io = new NodeIO();
const doc = await io.read(inp);

for (const mesh of doc.getRoot().listMeshes()) {
  for (const prim of mesh.listPrimitives()) {
    const pos = prim.getAttribute('POSITION');
    if (!pos) continue;
    const n = pos.getCount();
    const idxAcc = prim.getIndices();
    const idx = idxAcc ? idxAcc.getArray() : Uint32Array.from({ length: n }, (_, i) => i);

    const P = new Float32Array(n * 3);
    for (let i = 0; i < n; i++) {
      const v = pos.getElement(i, [0, 0, 0]);
      P[i * 3] = v[0]; P[i * 3 + 1] = v[1]; P[i * 3 + 2] = v[2];
    }

    const N = new Float32Array(n * 3);
    for (let t = 0; t < idx.length; t += 3) {
      const a = idx[t] * 3, b = idx[t + 1] * 3, c = idx[t + 2] * 3;
      const e1x = P[b] - P[a], e1y = P[b + 1] - P[a + 1], e1z = P[b + 2] - P[a + 2];
      const e2x = P[c] - P[a], e2y = P[c + 1] - P[a + 1], e2z = P[c + 2] - P[a + 2];
      const nx = e1y * e2z - e1z * e2y;
      const ny = e1z * e2x - e1x * e2z;
      const nz = e1x * e2y - e1y * e2x;
      N[a] += nx; N[a + 1] += ny; N[a + 2] += nz;
      N[b] += nx; N[b + 1] += ny; N[b + 2] += nz;
      N[c] += nx; N[c + 1] += ny; N[c + 2] += nz;
    }
    for (let i = 0; i < n; i++) {
      const o = i * 3;
      const l = Math.hypot(N[o], N[o + 1], N[o + 2]) || 1;
      N[o] /= l; N[o + 1] /= l; N[o + 2] /= l;
    }

    prim.setAttribute('NORMAL', doc.createAccessor()
      .setType('VEC3').setArray(N).setBuffer(doc.getRoot().listBuffers()[0]));
  }
}

await io.write(out, doc);
console.log('법선 baked →', out);
