/* ══════════════════════════════════════════════════════════════
   대용량 OBJ → GLB (POSITION + 인덱스만)

   obj2gltf 는 숫자를 전부 JS 배열에 담는다. 553MB 짜리 페라리 원본에서
   힙을 넘긴다. 여기서는 파일을 줄 단위로 흘리면서 타입드 배열에 바로
   쌓는다 — 553MB 를 9.5 초에 읽는다.

   법선·UV·재질은 만들지 않는다. 감축 뒤에 scripts/bake-normals.mjs 로
   굽는 것이 순서다 (감축 전에 넣으면 감축기가 보간해 뭉갠다).

     npm i --no-save @gltf-transform/core
     node --max-old-space-size=14336 scripts/obj2glb.mjs in.obj out.glb

   감축까지의 전체 절차는
   ══════════════════════════════════════════════════════════════ */
import fs from 'node:fs';
import readline from 'node:readline';
import { Document, NodeIO } from '@gltf-transform/core';

const [inp, out] = process.argv.slice(2);
if (!inp || !out) {
  console.error('사용: node scripts/obj2glb.mjs <in.obj> <out.glb>');
  process.exit(1);
}

let posCap = 1 << 22, posN = 0, P = new Float32Array(posCap * 3);
let idxCap = 1 << 23, idxN = 0, I = new Uint32Array(idxCap * 3);
const growP = () => { const t = new Float32Array(posCap * 2 * 3); t.set(P); P = t; posCap *= 2; };
const growI = () => { const t = new Uint32Array(idxCap * 2 * 3); t.set(I); I = t; idxCap *= 2; };

const rl = readline.createInterface({
  input: fs.createReadStream(inp, { highWaterMark: 1 << 22 }),
  crlfDelay: Infinity,
});

const fan = [];
for await (const line of rl) {
  if (line.charCodeAt(0) === 118 /* v */ && line.charCodeAt(1) === 32) {
    if (posN === posCap) growP();
    const t = line.split(/\s+/);
    P[posN * 3] = +t[1]; P[posN * 3 + 1] = +t[2]; P[posN * 3 + 2] = +t[3];
    posN++;
  } else if (line.charCodeAt(0) === 102 /* f */ && line.charCodeAt(1) === 32) {
    fan.length = 0;
    const t = line.split(/\s+/);
    for (let k = 1; k < t.length; k++) {
      if (!t[k]) continue;
      const slash = t[k].indexOf('/');
      const v = +(slash === -1 ? t[k] : t[k].slice(0, slash));
      fan.push(v > 0 ? v - 1 : posN + v);   // OBJ 는 1-based, 음수는 뒤에서부터 센다
    }
    for (let k = 2; k < fan.length; k++) {  // 다각형은 팬으로 자른다
      if (idxN === idxCap) growI();
      I[idxN * 3] = fan[0]; I[idxN * 3 + 1] = fan[k - 1]; I[idxN * 3 + 2] = fan[k];
      idxN++;
    }
  }
}

/* o/g 그룹은 합친다. loadCar 가 어차피 메시를 전부 모아 한 덩이로 쓰고,
   OBJ 의 면 인덱스는 그룹과 무관하게 파일 전체에서 이어진다. */
const pos = P.subarray(0, posN * 3).slice();
const idx = I.subarray(0, idxN * 3).slice();
P = I = null;
console.log(`파싱: 정점 ${posN.toLocaleString()} · 삼각형 ${idxN.toLocaleString()}`);

const doc = new Document();
const buf = doc.createBuffer();
const prim = doc.createPrimitive()
  .setAttribute('POSITION', doc.createAccessor().setType('VEC3').setArray(pos).setBuffer(buf))
  .setIndices(doc.createAccessor().setType('SCALAR').setArray(idx).setBuffer(buf))
  .setMaterial(doc.createMaterial('mat_0').setRoughnessFactor(0.4).setMetallicFactor(0.2));
doc.createScene().addChild(doc.createNode('car').setMesh(doc.createMesh('car').addPrimitive(prim)));
await new NodeIO().write(out, doc);
console.log('→', out);
