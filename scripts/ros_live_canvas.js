/**
 * Live maze arena for the public /ros dashboard — walls, three waypoints, robot pose.
 */
(function () {
  'use strict';

  var WORLD = { xmin: -5.5, xmax: 2.0, ymin: -3.5, ymax: 2.5 };
  var D_MAX = 3.5;
  var API = location.origin + '/ros/api/lidar_live';

  var DEFAULT_WPS = [
    { x: -2.8, y: -1.8, n: 1 },
    { x: -4.5, y: -0.2, n: 2 },
    { x: -3.2, y: 1.2, n: 3 },
  ];
  var DEFAULT_WALLS = [
    { x: -4.10, y: -1.00, w: 1.80, h: 0.2 },
    { x: -1.10, y: -1.00, w: 2.20, h: 0.2 },
    { x: -4.50, y: 0.50, w: 1.00, h: 0.2 },
    { x: -1.25, y: 0.50, w: 2.50, h: 0.2 },
    { x: -4.25, y: 2.20, w: 1.50, h: 0.2 },
    { x: -1.75, y: 2.20, w: 1.50, h: 0.2 },
    { x: -0.50, y: -0.25, w: 0.2, h: 1.50 },
    { x: -2.00, y: 1.35, w: 0.2, h: 1.70 },
    { x: -4.50, y: -2.60, w: 0.2, h: 1.50 },
    { x: -0.50, y: -2.80, w: 2.00, h: 0.2 },
    { x: -1.25, y: 1.40, w: 0.2, h: 0.90 },
  ];

  function $(id) {
    return document.getElementById(id);
  }

  function init() {
    var canvas = $('arena');
    var vp = $('viewport');
    if (!canvas || !vp) return;
    var ctx = canvas.getContext('2d');
    if (!ctx) return;

    var layout = { ox: 0, oy: 0, s: 0, w: 0, h: 0, dpr: 1 };
    var state = {
      x: -2, y: -2, yaw: 0, scan: null, trail: [],
      walls: DEFAULT_WALLS.slice(),
      waypoints: DEFAULT_WPS.slice(),
      wpIdx: 0,
      envName: 'Maze · 3 waypoints',
    };
    var pulse = 0;

    function syncLayout() {
      var dpr = Math.min(window.devicePixelRatio || 1, 2);
      var w = Math.max(120, Math.floor(vp.clientWidth));
      var h = Math.max(120, Math.floor(vp.clientHeight));
      layout.dpr = dpr;
      layout.w = w;
      layout.h = h;
      canvas.width = Math.floor(w * dpr);
      canvas.height = Math.floor(h * dpr);
      canvas.style.width = w + 'px';
      canvas.style.height = h + 'px';
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      var pad = 32;
      layout.s = Math.max(40, Math.min(w, h) - pad * 2);
      layout.ox = (w - layout.s) / 2;
      layout.oy = (h - layout.s) / 2;
    }

    function wx(x) {
      return layout.ox + ((x - WORLD.xmin) / (WORLD.xmax - WORLD.xmin)) * layout.s;
    }
    function wy(y) {
      return layout.oy + (1 - (y - WORLD.ymin) / (WORLD.ymax - WORLD.ymin)) * layout.s;
    }

    function drawWall(obs) {
      var x1 = wx(obs.x - obs.w / 2);
      var x2 = wx(obs.x + obs.w / 2);
      var yTop = wy(obs.y + obs.h / 2);
      var yBot = wy(obs.y - obs.h / 2);
      var rx = Math.min(x1, x2);
      var ry = Math.min(yTop, yBot);
      var rw = Math.abs(x2 - x1);
      var rh = Math.abs(yBot - yTop);
      ctx.fillStyle = 'rgba(204,68,0,0.45)';
      ctx.fillRect(rx, ry, rw, rh);
      ctx.strokeStyle = '#ff7733';
      ctx.lineWidth = 1.5;
      ctx.strokeRect(rx, ry, rw, rh);
    }

    function drawWaypoint(wp, idx, activeIdx) {
      var px = wx(wp.x);
      var py = wy(wp.y);
      var done = idx < activeIdx;
      var active = idx === activeIdx;
      var r = active ? 11 + Math.sin(pulse) * 2 : 8;
      ctx.fillStyle = done
        ? 'rgba(80,120,80,0.35)'
        : active
          ? 'rgba(0,255,136,0.35)'
          : 'rgba(255,200,0,0.25)';
      ctx.beginPath();
      ctx.arc(px, py, r + 4, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = done ? '#5a8a5a' : active ? '#00ff88' : '#ffcc00';
      ctx.lineWidth = active ? 2.5 : 1.5;
      ctx.beginPath();
      ctx.arc(px, py, r, 0, Math.PI * 2);
      ctx.stroke();
      ctx.fillStyle = '#fff';
      ctx.font = 'bold 10px system-ui,sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(String(wp.n != null ? wp.n : idx + 1), px, py);
    }

    function draw() {
      var L = layout;
      pulse += 0.08;
      ctx.fillStyle = '#050912';
      ctx.fillRect(0, 0, L.w, L.h);
      ctx.strokeStyle = '#1a4a6e';
      ctx.lineWidth = 2;
      ctx.strokeRect(L.ox, L.oy, L.s, L.s);

      state.walls.forEach(drawWall);

      var wps = state.waypoints;
      var wi = Math.max(0, Math.min(state.wpIdx, wps.length - 1));
      for (var i = 0; i < wps.length; i++) {
        drawWaypoint(wps[i], i, wi);
      }

      var rcx = wx(state.x);
      var rcy = wy(state.y);
      var scan = state.scan;
      if (scan && scan.length) {
        for (var j = 0; j < scan.length; j++) {
          var ang = state.yaw + j * 2 * Math.PI / scan.length;
          var dist = scan[j];
          var ex = state.x + Math.cos(ang) * dist;
          var ey = state.y + Math.sin(ang) * dist;
          var alpha = 0.08 + (1 - Math.min(dist, D_MAX) / D_MAX) * 0.45;
          ctx.strokeStyle = 'rgba(0,200,255,' + alpha + ')';
          ctx.lineWidth = 1.5;
          ctx.beginPath();
          ctx.moveTo(rcx, rcy);
          ctx.lineTo(wx(ex), wy(ey));
          ctx.stroke();
        }
      }

      if (state.trail.length > 1) {
        ctx.beginPath();
        ctx.moveTo(wx(state.trail[0].x), wy(state.trail[0].y));
        for (var k = 1; k < state.trail.length; k++) {
          ctx.lineTo(wx(state.trail[k].x), wy(state.trail[k].y));
        }
        ctx.strokeStyle = 'rgba(0,136,200,0.5)';
        ctx.lineWidth = 2;
        ctx.stroke();
      }

      ctx.fillStyle = '#00d4ff';
      ctx.shadowColor = '#00fff2';
      ctx.shadowBlur = 14;
      ctx.beginPath();
      ctx.arc(rcx, rcy, 9, 0, Math.PI * 2);
      ctx.fill();
      ctx.shadowBlur = 0;
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(rcx, rcy);
      ctx.lineTo(rcx + Math.cos(state.yaw) * 22, rcy - Math.sin(state.yaw) * 22);
      ctx.stroke();
    }

    function setLive(on) {
      var pill = $('ws-pill');
      var txt = $('ws-text');
      if (pill) pill.classList.toggle('on', !!on);
      if (txt) txt.textContent = on ? 'LIVE' : 'OFFLINE';
    }

    function applySnap(snap) {
      if (!snap || !snap.ok) {
        setLive(false);
        return;
      }
      var age = Date.now() / 1000 - (snap.updated_at || 0);
      if (age > 30) {
        setLive(false);
        return;
      }
      if (typeof snap.x === 'number' && typeof snap.y === 'number') {
        state.x = snap.x;
        state.y = snap.y;
        state.trail.push({ x: snap.x, y: snap.y });
        if (state.trail.length > 120) state.trail.shift();
      }
      if (typeof snap.yaw === 'number') state.yaw = snap.yaw;
      if (snap.scan && snap.scan.length) state.scan = snap.scan;
      if (snap.waypoints && snap.waypoints.length) state.waypoints = snap.waypoints;
      if (snap.obstacles && snap.obstacles.length) state.walls = snap.obstacles;
      if (snap.wp_idx != null) state.wpIdx = parseInt(snap.wp_idx, 10) || 0;
      if (snap.env_name) state.envName = snap.env_name;

      var envLbl = $('env-label');
      if (envLbl) envLbl.textContent = 'Environment: ' + state.envName;

      setLive(true);
      var ep = snap.ep != null ? snap.ep : 0;
      var step = snap.step != null ? snap.step : 0;
      var en = $('ep-num'), sn = $('step-num');
      if (en) en.textContent = String(ep);
      if (sn) sn.textContent = String(step);
      var mEp = $('m-ep'), mRew = $('m-rew'), mTot = $('m-tot');
      if (mEp) mEp.textContent = String(ep);
      if (mRew) mRew.textContent = Number(snap.rew || 0).toFixed(2);
      if (mTot) mTot.textContent = Number(snap.tot || 0).toFixed(1);
      var adapt = $('adapt-score');
      if (adapt) {
        adapt.textContent = 'WP ' + (state.wpIdx + 1) + '/3 · step ' + step;
        adapt.className = 'ok';
      }
      var adaptSub = $('adapt-sub');
      if (adaptSub) adaptSub.textContent = 'Sequential maze (same as Gazebo training)';

      if (window.__rosApplyLidarSnap) {
        try { window.__rosApplyLidarSnap(snap); } catch (e) { /* optional */ }
      }
    }

    function poll() {
      fetch(API, { cache: 'no-store', credentials: 'same-origin' })
        .then(function (r) { return r.ok ? r.json() : null; })
        .then(function (snap) { applySnap(snap); })
        .catch(function () { setLive(false); });
    }

    syncLayout();
    (function frame() {
      requestAnimationFrame(frame);
      draw();
    })();
    poll();
    setInterval(poll, 280);
    window.addEventListener('resize', syncLayout);
    if (window.ResizeObserver) new ResizeObserver(syncLayout).observe(vp);

    var envLbl = $('env-label');
    if (envLbl) envLbl.textContent = 'Environment: ' + state.envName;

    var tag = $('build-tag');
    if (tag) tag.textContent = 'v12';
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
