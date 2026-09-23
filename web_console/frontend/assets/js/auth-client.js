/* ══════════════════════════════════════════════════════════════
   화면 보호는 서버가 한다. 여기서 하는 일은 표시뿐이다 —
   지금 누가 로그인해 있는지 헤더에 보여주고 로그아웃을 건다.
   ══════════════════════════════════════════════════════════════ */
(function () {
  'use strict';

  var api = function (path, opts) {
    var o = opts || {};
    return fetch(path, {
      method: o.method || 'GET',
      credentials: 'same-origin',
      headers: o.body ? { 'Content-Type': 'application/json' } : undefined,
      body: o.body ? JSON.stringify(o.body) : undefined,
    }).then(function (res) {
      return res.json().catch(function () { return {}; }).then(function (data) {
        if (!res.ok) {
          var err = new Error(data.error || '요청을 처리하지 못했습니다.');
          err.status = res.status;
          throw err;
        }
        return data;
      });
    });
  };

  var PT = {
    user: null,

    me: function () {
      return api('/api/me').then(function (d) { PT.user = d.user; return d.user; });
    },
    login: function (loginId, password) {
      return api('/api/login', { method: 'POST', body: { loginId: loginId, password: password } })
        .then(function (d) { PT.user = d.user; return d.user; });
    },
    signup: function (loginId, name, password) {
      return api('/api/signup', { method: 'POST', body: { loginId: loginId, name: name, password: password } });
    },
    logout: function () {
      return api('/api/logout', { method: 'POST' }).then(function () { PT.user = null; });
    },
    /* 숙련공 데이터 라이브러리 — ① 웹 UI이 쓰고 ④ 라이브러리가 읽는다.
       전에는 localStorage 'polytwin_saved' 였다 */
    library: {
      list: function () { return api('/api/library').then(function (d) { return d.entries; }); },
      save: function (rec) { return api('/api/library', { method: 'POST', body: rec }); },
      remove: function (rowId) { return api('/api/library/' + rowId, { method: 'DELETE' }); },
    },

    admin: {
      users: function () { return api('/api/admin/users').then(function (d) { return d.users; }); },
      audit: function () { return api('/api/admin/audit').then(function (d) { return d.entries; }); },
      update: function (id, patch) { return api('/api/admin/users/' + id, { method: 'PATCH', body: patch }); },
      remove: function (id) { return api('/api/admin/users/' + id, { method: 'DELETE' }); },
    },
  };

  /* ── 헤더 슬롯 렌더 ────────────────────────────────────────
     [data-auth-slot] 이 있는 화면에만 그린다. 없으면 아무것도
     만들지 않는다 — 번들된 화면 위에 떠다니는 조각을 얹지 않는다.
     ──────────────────────────────────────────────────────── */
  function render(slot, user) {
    slot.replaceChildren();

    if (!user) {
      var login = document.createElement('button');
      login.type = 'button';
      login.id = 'loginBtn';
      login.className = 'btn btn--ghost hdr__cta';
      login.setAttribute('aria-haspopup', 'dialog');
      login.textContent = 'LOG IN';
      slot.appendChild(login);
      slot.dispatchEvent(new CustomEvent('pt:render', { bubbles: true, detail: { user: null } }));
      return;
    }

    /* 로그인 ID 는 적지 않는다 — 계정이 'admin' 이면 옆의 ADMIN 배지와
       같은 글자가 두 번 나온다. 누가 무엇을 했는지는 admin.html 의
       접속 기록이 남긴다. 여기서는 권한만 알리면 된다. */
    if (user.role === 'admin') {
      var who = document.createElement('span');
      who.className = 'hdr__who';
      var tag = document.createElement('span');
      tag.className = 'hdr__role';
      tag.textContent = 'ADMIN';
      who.appendChild(tag);
      slot.appendChild(who);
    }

    if (user.role === 'admin') {
      var adminLink = document.createElement('a');
      adminLink.className = 'btn btn--ghost hdr__cta';
      adminLink.href = '/admin.html';
      adminLink.textContent = '관리자';
      slot.appendChild(adminLink);
    }

    var out = document.createElement('button');
    out.type = 'button';
    out.className = 'btn btn--ghost hdr__cta';
    out.textContent = 'LOG OUT';
    out.addEventListener('click', function () {
      out.disabled = true;
      PT.logout().then(function () { window.location.href = '/'; });
    });
    slot.appendChild(out);

    slot.dispatchEvent(new CustomEvent('pt:render', { bubbles: true, detail: { user: user } }));
  }

  PT.paint = function () {
    var slots = document.querySelectorAll('[data-auth-slot]');
    if (!slots.length) return Promise.resolve(null);
    return PT.me().then(function (user) {
      slots.forEach(function (s) { render(s, user); });
      return user;
    }).catch(function () { return null; });
  };

  window.PT = PT;

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', PT.paint);
  } else {
    PT.paint();
  }
})();
