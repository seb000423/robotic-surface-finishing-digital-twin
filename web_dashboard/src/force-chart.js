/**
 * Force Chart Module — Chart.js 실시간 Force 그래프
 * ROS2 토픽 또는 force_log.csv 데모 데이터 사용
 */
import { Chart, registerables } from 'chart.js';
Chart.register(...registerables);

export class ForceChart {
  constructor(canvasId, maxPoints = 1800) {
    this.canvas = document.getElementById(canvasId);
    this.maxPoints = maxPoints;
    this.data = [];          // [{x: 경과초, y: 힘(N)}]
    this.labels = [];
    this.startTime = null;   // 첫 데이터 시각(ms) — x축 0초 기준
    this._lastPlot = 0;      // 플롯 스로틀(고빈도 입력 대비)
    this.chart = null;
    this.demoData = null;
    this.demoIndex = 0;
    this.demoTimer = null;
    this._init();
  }

  _init() {
    const ctx = this.canvas.getContext('2d');
    const gradient = ctx.createLinearGradient(0, 0, 0, 260);
    gradient.addColorStop(0, 'rgba(255, 152, 0, 0.3)');
    gradient.addColorStop(1, 'rgba(255, 152, 0, 0.0)');

    this.chart = new Chart(ctx, {
      type: 'line',
      data: {
        labels: this.labels,
        datasets: [{
          label: 'Force (N)',
          data: this.data,
          borderColor: '#FF9800',
          backgroundColor: gradient,
          borderWidth: 1.5,
          fill: true,
          tension: 0.3,
          pointRadius: 0,
          pointHoverRadius: 3,
        }]
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: { duration: 0 },
        interaction: { mode: 'index', intersect: false },
        scales: {
          x: {
            type: 'linear',
            display: true,
            min: 0,
            grid: { color: 'rgba(0,0,0,0.06)' },
            ticks: {
              color: '#64748b', font: { size: 10, family: "'JetBrains Mono'" },
              stepSize: 1, maxTicksLimit: 12,
              callback: (v) => Math.round(v) + 's',
            },
            title: { display: true, text: '시간 (s)', color: '#64748b', font: { size: 10 } }
          },
          y: {
            display: true,
            min: 0,
            max: 5,
            grid: { color: 'rgba(0,0,0,0.06)' },
            ticks: { color: '#64748b', font: { size: 10, family: "'JetBrains Mono'" }, stepSize: 1 },
            title: { display: true, text: 'Force (N)', color: '#64748b', font: { size: 10 } }
          }
        },
        plugins: {
          legend: { display: false },
          tooltip: {
            backgroundColor: 'rgba(255, 255, 255, 0.95)',
            titleColor: '#0f172a',
            bodyColor: '#F57C00',
            borderColor: 'rgba(245, 124, 0, 0.3)',
            borderWidth: 1,
            bodyFont: { family: "'JetBrains Mono'" },
          }
        }
      }
    });
  }

  addPoint(force) {
    const now = performance.now();
    if (this.startTime === null) this.startTime = now;
    if (now - this._lastPlot < 80) return force;   // ~12Hz 로 제한
    this._lastPlot = now;
    const t = (now - this.startTime) / 1000;       // 경과 초
    this.data.push({ x: t, y: force });
    if (this.data.length > this.maxPoints) this.data.shift();
    this.chart.update('none');
    return force;
  }

  reset() {
    this.startTime = null;
    this._lastPlot = 0;
    this.data.length = 0;
    this.chart.update('none');
  }

  async loadDemoData(csvUrl = '/data/force_log.csv') {
    try {
      const res = await fetch(csvUrl);
      const text = await res.text();
      const lines = text.trim().split('\n');
      this.demoData = [];
      for (let i = 1; i < lines.length; i++) {
        const cols = lines[i].split(',');
        if (cols.length >= 10) {
          this.demoData.push({
            step: parseInt(cols[0]),
            filteredForce: parseFloat(cols[9]),
            virtualForce: parseFloat(cols[7]),
            contact: parseInt(cols[1]),
          });
        }
      }
      console.log(`[ForceChart] Loaded ${this.demoData.length} demo data points`);
    } catch (e) {
      console.warn('[ForceChart] Failed to load demo CSV:', e);
    }
  }

  startDemo(intervalMs = 50) {
    if (!this.demoData || this.demoData.length === 0) return;
    this.demoIndex = 0;
    this.demoTimer = setInterval(() => {
      if (this.demoIndex >= this.demoData.length) {
        this.demoIndex = 0;
        this.reset();
      }
      const d = this.demoData[this.demoIndex];
      this.addPoint(d.filteredForce);
      this.demoIndex++;
    }, intervalMs);
  }

  stopDemo() {
    if (this.demoTimer) { clearInterval(this.demoTimer); this.demoTimer = null; }
  }

  getCurrentValue() {
    return this.data.length > 0 ? this.data[this.data.length - 1].y : 0;
  }

  destroy() {
    this.stopDemo();
    if (this.chart) this.chart.destroy();
  }
}
