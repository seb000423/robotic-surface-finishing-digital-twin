/* ══════════════════════════════════════════════════════════════
   쏘나타 원본에서 외부 셸만 남기고 텍스처·UV 를 버린다.

   BeamNG 계열 GLB 라 48MB 중 44MB 가 텍스처이고 엔진룸·시트·대시보드까지
   들어 있다. ①의 loadCar 도 ②도 재질을 자기 MeshPhysicalMaterial 로 갈아
   끼우므로 텍스처는 한 바이트도 안 쓰인다.

     npm i --no-save @gltf-transform/core
     node scripts/prep-sonata.mjs "2015 Hyundai Sonata.glb" s0.glb

   이 다음이 prune → dedup → flatten → join → weld → simplify →
   bake-normals → meshopt 다. 전체 절차는
   ══════════════════════════════════════════════════════════════ */
import { NodeIO } from '@gltf-transform/core';

/* 실내·엔진. 재질별 바운딩 박스로 골랐다 — 전부 차체 안쪽에 갇혀 있다.
   바깥까지 뻗는 vehicle_dark_kg(범퍼·몰딩)·sonatablack 은 남긴다. */
const DROP = new Set([
  'sunburst_engine.003',              // 엔진룸  z 0.61..1.73
  'intlight',                         // 실내등  x 0.20..0.79 · z 0.05..0.42
  'etk800_seats.skin_interior.beige', // 시트
  'torpedo_leather',                  // 대시보드·리어 선반
  'aluminium',                        // 엔진·서스펜션
  'midsize_mechanical.004',           // 언더바디 기계부
  'sunburst_main',                    // 섀시·플로어
  'etk800_gauges_screen.001',
  'etk800_screen.001',
  'invis.014',
]);

const io = new NodeIO();
const doc = await io.read(process.argv[2]);

let kept = 0, dropped = 0;
for (const mesh of doc.getRoot().listMeshes()) {
  for (const prim of mesh.listPrimitives()) {
    const name = prim.getMaterial()?.getName() || '';
    const pos = prim.getAttribute('POSITION');
    const t = (prim.getIndices()?.getCount() ?? pos.getCount()) / 3;
    if (DROP.has(name)) { mesh.removePrimitive(prim); prim.dispose(); dropped += t; continue; }
    kept += t;
    // UV·탄젠트·정점색은 쓰이지 않는다. 법선은 감축 뒤에 다시 굽는다
    for (const a of ['TEXCOORD_0', 'TEXCOORD_1', 'TANGENT', 'COLOR_0', 'NORMAL']) {
      const acc = prim.getAttribute(a);
      if (acc) { prim.setAttribute(a, null); }
    }
  }
  if (mesh.listPrimitives().length === 0) mesh.dispose();
}

/* 텍스처를 떼면 재질이 전부 같아진다 → dedup 이 하나로 합치고
   join 이 200 개 메시를 한 덩이로 묶는다. 드로우 콜 200 → 1 */
for (const mat of doc.getRoot().listMaterials()) {
  mat.setBaseColorTexture(null).setNormalTexture(null)
     .setMetallicRoughnessTexture(null).setEmissiveTexture(null)
     .setOcclusionTexture(null)
     .setBaseColorFactor([1, 1, 1, 1]).setMetallicFactor(0.2).setRoughnessFactor(0.4);
}

await io.write(process.argv[3], doc);
console.log(`남김 ${Math.round(kept).toLocaleString()} tri · 버림 ${Math.round(dropped).toLocaleString()} tri`);
