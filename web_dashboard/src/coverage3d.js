/**
 * Coverage3D — 의존성 없는 캔버스 3D 포인트클라우드 뷰어.
 * 폴리싱 완료 커버리지(점 + 폴리싱여부)를 마우스로 360° 회전하며 확인.
 *   폴리싱됨 = 하늘색 / 안된 부분 = 빨강(눈에 띄게).
 */
export class Coverage3D {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.pts = null;          // Float32Array length 3N (centered)
    this.pol = null;          // Uint8Array length N
    this.n = 0;
    this.azim = -0.9;         // 좌우 회전(rad)
    this.elev = 0.35;         // 상하 기울기(rad)
    this.scale = 1;
    this.center = [0, 0, 0];
    this._raf = null;
    this._dragging = false;
    this._lx = 0; this._ly = 0;
    this._zoom = 1;
    this._bindEvents();
  }

  async loadUrl(url) {
    const res = await fetch(url, { cache: 'no-store' });
    if (!res.ok) throw new Error('coverage fetch failed: ' + res.status);
    this.setData(await res.json());
  }

  setData(data) {
    const p = data.points || [];
    this.n = p.length;
    this.pts = new Float32Array(this.n * 3);
    this.pol = new Uint8Array(this.n);
    const mn = data.min || [0, 0, 0];
    const mx = data.max || [1, 1, 1];
    this.center = [(mn[0] + mx[0]) / 2, (mn[1] + mx[1]) / 2, (mn[2] + mx[2]) / 2];
    let maxr = 1e-6;
    for (let i = 0; i < this.n; i++) {
      const x = p[i][0] - this.center[0];
      const y = p[i][1] - this.center[1];
      const z = p[i][2] - this.center[2];
      this.pts[i * 3] = x; this.pts[i * 3 + 1] = y; this.pts[i * 3 + 2] = z;
      this.pol[i] = (data.polished && data.polished[i]) ? 1 : 0;
      const r = Math.sqrt(x * x + y * y + z * z);
      if (r > maxr) maxr = r;
    }
    this._maxr = maxr;
    this.meta = {
      covered: data.covered || 0,
      total: data.total || this.n,
      elapsed_sec: data.elapsed_sec,
      avg_force: data.avg_force,
      contact_ratio: data.contact_ratio,
    };
    this.resize();
    this.render();
  }

  /**
   * 차량 표면을 3×3 구역으로 나눠 구역별 폴리싱 완료율을 계산.
   * 좌표 규약(프로젝트): 앞=-Y, 뒤=+Y, 왼쪽=+X, 오른쪽=-X.
   * 반환: 완료율 오름차순(가장 덜 된 곳 먼저)의 [{name, pct, tot, pol}].
   */
  analyzeRegions() {
    if (!this.pts || !this.n) return [];
    let xmin = Infinity, xmax = -Infinity, ymin = Infinity, ymax = -Infinity;
    for (let i = 0; i < this.n; i++) {
      const x = this.pts[i * 3], y = this.pts[i * 3 + 1];
      if (x < xmin) xmin = x; if (x > xmax) xmax = x;
      if (y < ymin) ymin = y; if (y > ymax) ymax = y;
    }
    const nx = 3, ny = 3;
    const ex = (xmax - xmin) || 1, ey = (ymax - ymin) || 1;
    const cells = Array.from({ length: nx * ny }, () => ({ tot: 0, pol: 0 }));
    for (let i = 0; i < this.n; i++) {
      const x = this.pts[i * 3], y = this.pts[i * 3 + 1];
      let ix = Math.floor((x - xmin) / ex * nx); ix = Math.max(0, Math.min(nx - 1, ix));
      let iy = Math.floor((y - ymin) / ey * ny); iy = Math.max(0, Math.min(ny - 1, iy));
      const c = cells[iy * nx + ix];
      c.tot++; if (this.pol[i]) c.pol++;
    }
    // ix: 작은 x(=-X)=오른쪽 → 큰 x(=+X)=왼쪽 / iy: 작은 y(=-Y)=앞 → 큰 y(=+Y)=뒤
    const xLabel = ['오른쪽', '중앙', '왼쪽'];
    const yLabel = ['앞', '중앙', '뒤'];
    const minPts = Math.max(5, this.n * 0.01);   // 표본 너무 적은 칸은 제외
    const regions = [];
    for (let iy = 0; iy < ny; iy++) {
      for (let ix = 0; ix < nx; ix++) {
        const c = cells[iy * nx + ix];
        if (c.tot < minPts) continue;
        const name = (yLabel[iy] === '중앙' && xLabel[ix] === '중앙')
          ? '중앙부' : `${yLabel[iy]} ${xLabel[ix]}`;
        regions.push({ name, pct: 100 * c.pol / c.tot, tot: c.tot, pol: c.pol });
      }
    }
    regions.sort((a, b) => a.pct - b.pct);
    return regions;
  }

  resize() {
    const c = this.canvas;
    const w = c.clientWidth || c.width || 640;
    const h = c.clientHeight || c.height || 420;
    c.width = w; c.height = h;
    this.scale = (0.46 * Math.min(w, h) / (this._maxr || 1)) * this._zoom;
  }

  _bindEvents() {
    const c = this.canvas;
    c.style.cursor = 'grab';
    c.addEventListener('mousedown', (e) => {
      this._dragging = true; this._lx = e.clientX; this._ly = e.clientY;
      c.style.cursor = 'grabbing';
    });
    window.addEventListener('mouseup', () => { this._dragging = false; c.style.cursor = 'grab'; });
    window.addEventListener('mousemove', (e) => {
      if (!this._dragging) return;
      this.azim += (e.clientX - this._lx) * 0.01;
      this.elev += (e.clientY - this._ly) * 0.01;
      this.elev = Math.max(-1.5, Math.min(1.5, this.elev));
      this._lx = e.clientX; this._ly = e.clientY;
      this._scheduleRender();
    });
    c.addEventListener('wheel', (e) => {
      e.preventDefault();
      this._zoom *= (e.deltaY < 0 ? 1.1 : 0.9);
      this._zoom = Math.max(0.3, Math.min(4, this._zoom));
      this.resize(); this._scheduleRender();
    }, { passive: false });
  }

  _scheduleRender() {
    if (this._raf) return;
    this._raf = requestAnimationFrame(() => { this._raf = null; this.render(); });
  }

  render() {
    if (!this.pts) return;
    const { ctx, canvas } = this;
    const W = canvas.width, H = canvas.height;
    ctx.fillStyle = '#0d1326';
    ctx.fillRect(0, 0, W, H);
    const ca = Math.cos(this.azim), sa = Math.sin(this.azim);
    const ce = Math.cos(this.elev), se = Math.sin(this.elev);
    const cx = W / 2, cy = H / 2, s = this.scale;

    // 깊이 정렬용 인덱스 배열
    const order = this._order || (this._order = new Int32Array(this.n).map((_, i) => i));
    const depth = this._depth || (this._depth = new Float32Array(this.n));
    const sxv = this._sx || (this._sx = new Float32Array(this.n));
    const syv = this._sy || (this._sy = new Float32Array(this.n));
    for (let i = 0; i < this.n; i++) {
      const x = this.pts[i * 3], y = this.pts[i * 3 + 1], z = this.pts[i * 3 + 2];
      const x1 = x * ca - y * sa;
      const y1 = x * sa + y * ca;
      const y2 = y1 * ce - z * se;
      const z2 = y1 * se + z * ce;
      sxv[i] = cx + s * x1;
      syv[i] = cy - s * z2;
      depth[i] = y2;
    }
    // 먼 점부터(작은 y2가 더 먼 쪽이 되도록) 그리기 — 간단한 painter's
    order.sort((a, b) => depth[a] - depth[b]);
    for (let k = 0; k < this.n; k++) {
      const i = order[k];
      if (this.pol[i]) ctx.fillStyle = '#59bfff';        // 폴리싱됨 = 하늘색
      else ctx.fillStyle = '#ff3b30';                    // 미폴리싱 = 빨강
      ctx.fillRect(sxv[i], syv[i], 2.5, 2.5);
    }
  }
}
