/**
 * ROS2 WebSocket Bridge Module
 * roslibjs 패턴으로 rosbridge_server와 WebSocket 통신
 * rosbridge가 없으면 데모 데이터로 자동 폴백
 */

export class RosBridge {
  constructor(url = 'ws://localhost:9090') {
    this.url = url;
    this.ws = null;
    this.connected = false;
    this.subscribers = {};
    this.onConnectionChange = null;
    this.reconnectTimer = null;
    this.reconnectInterval = 3000;
  }

  connect() {
    try {
      this.ws = new WebSocket(this.url);
      this.ws.onopen = () => {
        this.connected = true;
        console.log('[ROS2] Connected to rosbridge:', this.url);
        if (this.onConnectionChange) this.onConnectionChange(true);
        // Re-subscribe all topics
        Object.keys(this.subscribers).forEach(topic => {
          this._sendSubscribe(topic, this.subscribers[topic].msgType);
        });
      };
      this.ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          if (data.op === 'publish' && data.topic && this.subscribers[data.topic]) {
            this.subscribers[data.topic].callbacks.forEach(cb => cb(data.msg));
          }
        } catch (e) { /* ignore parse errors */ }
      };
      this.ws.onclose = () => {
        this.connected = false;
        console.log('[ROS2] Disconnected');
        if (this.onConnectionChange) this.onConnectionChange(false);
        this._scheduleReconnect();
      };
      this.ws.onerror = () => {
        this.connected = false;
        if (this.onConnectionChange) this.onConnectionChange(false);
      };
    } catch (e) {
      console.log('[ROS2] WebSocket not available, using demo mode');
      if (this.onConnectionChange) this.onConnectionChange(false);
    }
  }

  _scheduleReconnect() {
    if (this.reconnectTimer) return;
    this.reconnectTimer = setTimeout(() => {
      this.reconnectTimer = null;
      this.connect();
    }, this.reconnectInterval);
  }

  _sendSubscribe(topic, msgType) {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return;
    this.ws.send(JSON.stringify({
      op: 'subscribe', topic, type: msgType
    }));
  }

  subscribe(topic, msgType, callback) {
    if (!this.subscribers[topic]) {
      this.subscribers[topic] = { msgType, callbacks: [] };
    }
    this.subscribers[topic].callbacks.push(callback);
    if (this.connected) this._sendSubscribe(topic, msgType);
  }

  _sendUnsubscribe(topic) {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return;
    this.ws.send(JSON.stringify({ op: 'unsubscribe', topic }));
  }

  unsubscribe(topic) {
    if (!this.subscribers[topic]) return;
    delete this.subscribers[topic];
    this._sendUnsubscribe(topic);
  }

  publish(topic, msgType, msg) {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return;
    this.ws.send(JSON.stringify({ op: 'publish', topic, msg }));
  }

  callService(service, serviceType, args = {}) {
    return new Promise((resolve) => {
      if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
        resolve(null);
        return;
      }
      const id = 'call_' + Date.now();
      const handler = (event) => {
        try {
          const data = JSON.parse(event.data);
          if (data.op === 'service_response' && data.id === id) {
            this.ws.removeEventListener('message', handler);
            resolve(data.values);
          }
        } catch (e) { /* ignore */ }
      };
      this.ws.addEventListener('message', handler);
      this.ws.send(JSON.stringify({
        op: 'call_service', id, service, type: serviceType, args
      }));
    });
  }

  disconnect() {
    if (this.reconnectTimer) { clearTimeout(this.reconnectTimer); this.reconnectTimer = null; }
    if (this.ws) {
      this.ws.onclose = null;
      this.ws.onerror = null;
      this.ws.onmessage = null;
      this.ws.onopen = null;
      this.ws.close();
      this.ws = null;
    }
    this.connected = false;
    if (this.onConnectionChange) this.onConnectionChange(false);
  }
}

/**
 * 예상 ROS2 토픽 정의
 * Isaac Sim에서 퍼블리시할 토픽들의 이름과 메시지 타입
 */
export const ROS_TOPICS = {
  CAMERA_RGB:       { topic: '/camera/rgb/image_raw/compressed', type: 'sensor_msgs/msg/CompressedImage' },
  // 4코너 + 천장 카메라
  CAM_LEFT_FRONT:   { topic: '/camera/left_front/compressed',  type: 'sensor_msgs/msg/CompressedImage' },
  CAM_LEFT_BACK:    { topic: '/camera/left_back/compressed',   type: 'sensor_msgs/msg/CompressedImage' },
  CAM_RIGHT_FRONT:  { topic: '/camera/right_front/compressed', type: 'sensor_msgs/msg/CompressedImage' },
  CAM_RIGHT_BACK:   { topic: '/camera/right_back/compressed',  type: 'sensor_msgs/msg/CompressedImage' },
  CAM_CEILING:      { topic: '/camera/ceiling/compressed',     type: 'sensor_msgs/msg/CompressedImage' },
  FORCE:            { topic: '/contact_force/data',          type: 'std_msgs/msg/Float64' },
  // 로봇팔 3개 개별 접촉력 (천장 C / 좌측면 SL / 우측면 SR)
  FORCE_C:          { topic: '/polishing/force/C',           type: 'std_msgs/msg/Float64' },
  FORCE_SL:         { topic: '/polishing/force/SL',          type: 'std_msgs/msg/Float64' },
  FORCE_SR:         { topic: '/polishing/force/SR',          type: 'std_msgs/msg/Float64' },
  PROGRESS:         { topic: '/polishing/progress',          type: 'std_msgs/msg/Float64' },
  // 로봇별 진행률 (좌측 SL / 우측 SR / 천장 C)
  PROGRESS_SL:      { topic: '/polishing/progress/SL',       type: 'std_msgs/msg/Float64' },
  PROGRESS_SR:      { topic: '/polishing/progress/SR',       type: 'std_msgs/msg/Float64' },
  PROGRESS_C:       { topic: '/polishing/progress/C',        type: 'std_msgs/msg/Float64' },
  HEATMAP:          { topic: '/polishing/heatmap',           type: 'std_msgs/msg/Float64MultiArray' },
  STATE:            { topic: '/polishing/state',             type: 'std_msgs/msg/String' },
  PAD_COMPRESSION:  { topic: '/polishing/pad_compression',  type: 'std_msgs/msg/Float64' },
  CONTACT_STEPS:    { topic: '/polishing/contact_steps',     type: 'std_msgs/msg/Int32' },
  ELAPSED_TIME:     { topic: '/polishing/elapsed_time',      type: 'std_msgs/msg/Float64' },
};
