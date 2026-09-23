/* ══════════════════════════════════════════════════════════════
   Edge Middleware — 위임 샴(shim)

   Vercel 은 middleware 를 저장소 루트에서만 찾는다. 그래서 파일
   자체는 여기 있어야 하고, 내용은 backend/middleware.js 에 둔다 —
   백엔드 코드가 backend/ 한곳에 모여 있게 하려는 것이다.

   config(matcher) 도 같이 재수출한다. 빠뜨리면 미들웨어가
   모든 요청에 붙어 자산까지 지나간다.
   ══════════════════════════════════════════════════════════════ */
export { default, config } from './backend/middleware.js';
