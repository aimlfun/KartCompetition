/*
 * The page: place markers, drive them, replay with skid marks and an in-kart view, share the line as a code.
 * The physics and the rules are in physics.js / sim.js. This file is only UI and drawing.
 */
(function () {
  'use strict';

  const D = window.TRACK_DATA;
  if (!D) {
    document.body.insertAdjacentHTML('afterbegin', '<p style="padding:20px;color:#f88">track-data.js is missing. Run: node tools/pack-track.mjs &lt;export-folder&gt;</p>');
    return;
  }

  let MAX_MARKERS = 15; // set from the track's gate count in init()
  const KMH_MIN = 10;
  let KMH_MAX = 70; // set from the vehicle's top speed in init()
  const KMH_STEP = 1;
  let KMH_DEFAULT = 40;
  let topKmh = 0; // what this vehicle reaches flat out on a long straight
  const TRACK_ID = (D.track || 'track').toLowerCase().replace(/[^a-z0-9]+/g, '-').replace(/^-|-$/g, '');

  const $ = (id) => document.getElementById(id);
  const stageBox = $('stageBox');
  const canvas = $('stage');
  const ctx = canvas.getContext('2d');
  const driverCanvas = $('driver');
  const dctx = driverCanvas.getContext('2d');

  // ------------------------------------------------------------------ state

  let track = null; // RLSim.Track
  let trackImg = null;
  let carImg = null; // the car as the app draws it (optional)
  let trackPixels = null; // Uint32Array of the picture, for the kart view
  let markers = []; // { x, y, kmh }
  let sel = -1;
  let mode = 'edit'; // 'edit' | 'replay'
  let result = null; // the lap being replayed
  let ghost = null; // last attempt, shown faintly while editing
  let lineCache = null;
  let dragging = -1;
  let showHit = false;
  let kartView = true;

  const pb = { pos: 0, playing: false, speed: 1, last: 0 };
  let skid = null; // offscreen canvas, track-sized
  let skidCtx = null;
  let skidDone = -1; // last frame whose marks are drawn
  let skidLast = { x: -99, y: -99 };

  let sessionBest = null;
  try { const b = parseFloat(localStorage.getItem('rl-best-' + TRACK_ID)); if (b > 0) sessionBest = b; } catch (e) { /* private mode */ }

  // ------------------------------------------------------------------ setup

  function loadImage(src) {
    return new Promise((res, rej) => {
      const i = new Image();
      i.onload = () => res(i);
      i.onerror = rej;
      i.src = src;
    });
  }

  async function init() {
    const [tImg, mImg] = await Promise.all([loadImage(D.trackImage), loadImage(D.maskImage)]);
    trackImg = tImg;
    if (D.carImage) carImg = await loadImage(D.carImage);

    const off = document.createElement('canvas');
    off.width = D.width;
    off.height = D.height;
    const octx = off.getContext('2d', { willReadFrequently: true });

    octx.drawImage(mImg, 0, 0);
    const grass = RLSim.grassFromRGBA(octx.getImageData(0, 0, D.width, D.height).data, D.width, D.height);
    track = new RLSim.Track(D, grass);

    octx.clearRect(0, 0, D.width, D.height);
    octx.drawImage(tImg, 0, 0);
    trackPixels = new Uint32Array(octx.getImageData(0, 0, D.width, D.height).data.buffer.slice(0));

    // more complex track = more gates = more markers allowed: 40% of the gate count in 5s (the Demo's 52 gates give 20), 10 to 60
    MAX_MARKERS = Math.min(60, Math.max(10, Math.round((D.gates1.length * 0.4) / 5) * 5));

    // top speed of whatever vehicle was exported: flat out on a long straight until it stops gaining
    {
      const probe = new RLPhysics.Car(D.physics, D.dt, 0, 0, 0);
      for (let i = 0; i < 6000; i++) probe.step({ steer: 0, throttle: 1, brake: 0, ebrake: 0 });
      topKmh = probe.speed * 3.6;
      KMH_MAX = Math.max(70, Math.ceil((topKmh * 1.25) / 10) * 10); // you may ask for more than it can do
      KMH_DEFAULT = Math.max(KMH_MIN, Math.round(topKmh * 0.6));
      $('topHint').textContent = `This ${D.vehicle || 'vehicle'} tops out around ${Math.round(topKmh)} km/h flat out. A speed above that is just "as fast as you can", and it takes a long straight to get there.`;
    }

    skid = document.createElement('canvas');
    skid.width = D.width;
    skid.height = D.height;
    skidCtx = skid.getContext('2d');

    $('trackName').textContent = (D.benchmark ? `${D.benchmark.label}: ${D.benchmark.seconds.toFixed(2)} s` : 'Human vs. barely-AI') + '  ·  ' + (D.vehicle || 'Go-kart');

    stageBox.style.maxWidth = `calc(82vh * ${D.width / D.height})`; // the whole track fits on screen without scrolling
    resize();
    new ResizeObserver(() => requestAnimationFrame(resize)).observe(stageBox);
    bind();
    refreshUI();
    fromHash();
    requestAnimationFrame(frame);
  }

  function resize() {
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const cssW = stageBox.clientWidth;
    canvas.width = Math.round(cssW * dpr);
    canvas.height = Math.round((cssW * D.height) / D.width * dpr);
    if (track) draw(); // resizing clears the canvas: repaint straight away
  }

  // ------------------------------------------------------------------ events

  function bind() {
    canvas.addEventListener('pointerdown', onDown);
    canvas.addEventListener('pointermove', onMove);
    canvas.addEventListener('pointerup', onUp);
    canvas.addEventListener('pointercancel', onUp);
    canvas.addEventListener('contextmenu', (e) => e.preventDefault());

    $('btnTry').onclick = tryLine;
    $('btnUndo').onclick = undo;
    $('btnDel').onclick = delSelected;
    $('btnClear').onclick = clearAll;
    holdToRepeat($('spdDown'), -1);
    holdToRepeat($('spdUp'), 1);
    $('btnAll').onclick = () => { if (sel >= 0) { markers.forEach((m) => (m.kmh = markers[sel].kmh)); changed(); } };
    $('btnView').onclick = () => { kartView = !kartView; refreshUI(); };
    $('btnPause').onclick = () => { pb.playing = !pb.playing; pb.last = performance.now(); refreshUI(); };
    $('btnEdit').onclick = enterEdit;
    document.querySelectorAll('.sp').forEach((b) => (b.onclick = () => { pb.speed = +b.dataset.sp; refreshUI(); }));
    $('scrub').oninput = (e) => { pb.playing = false; seek(+e.target.value); refreshUI(); };
    $('btnCopy').onclick = () => copy($('code').value, 'Code copied');
    $('btnLink').onclick = () => copy(linkFor($('code').value), 'Link copied');
    $('btnLoad').onclick = () => loadCode($('codeIn').value.trim());
    $('codeIn').addEventListener('keydown', (e) => { if (e.key === 'Enter') loadCode($('codeIn').value.trim()); });

    window.addEventListener('keydown', (e) => {
      if (/^(INPUT|TEXTAREA)$/.test(e.target.tagName)) return;
      if (e.key === ' ' || e.key === 'Enter') { e.preventDefault(); tryLine(); }
      else if (e.key === 'z' || e.key === 'Z') undo();
      else if (e.key === 'Delete' || e.key === 'Backspace') delSelected();
      else if (e.key === 'ArrowUp') { e.preventDefault(); bumpSpeed(e.shiftKey ? 5 : KMH_STEP); }
      else if (e.key === 'ArrowDown') { e.preventDefault(); bumpSpeed(e.shiftKey ? -5 : -KMH_STEP); }
      else if (e.key === 'h') showHit = !showHit;
      else if (e.key === 'v') { kartView = !kartView; refreshUI(); }
    });
    window.addEventListener('hashchange', fromHash);
  }

  function worldPos(e) {
    const r = canvas.getBoundingClientRect();
    return { x: ((e.clientX - r.left) / r.width) * D.width, y: ((e.clientY - r.top) / r.height) * D.height };
  }

  const cssScale = () => canvas.clientWidth / D.width; // css px per track px

  function markerAt(p) {
    const reach = 18 / cssScale();
    let best = -1;
    let bestD = reach;
    markers.forEach((m, i) => {
      const d = Math.hypot(m.x - p.x, m.y - p.y);
      if (d <= bestD) { bestD = d; best = i; }
    });
    return best;
  }

  function onDown(e) {
    if (mode === 'replay') { enterEdit(); return; }
    e.preventDefault();
    const p = worldPos(e);
    const hit = markerAt(p);

    if (hit >= 0) {
      sel = hit;
      dragging = hit;
      canvas.setPointerCapture(e.pointerId);
      refreshUI();
      return;
    }

    if (markers.length >= MAX_MARKERS) { toast(`That's ${MAX_MARKERS} markers: the most this track allows`, true); return; }
    if (!track.isRoadForMarker(p.x, p.y)) { toast('Keep the markers on the tarmac', true); return; }

    markers.push({ x: Math.round(p.x), y: Math.round(p.y), kmh: markers.length ? markers[markers.length - 1].kmh : KMH_DEFAULT });
    sel = markers.length - 1;
    changed();
  }

  function onMove(e) {
    if (dragging < 0) return;
    const p = worldPos(e);
    if (!track.isRoadForMarker(p.x, p.y)) return; // stays on the last good spot
    markers[dragging].x = Math.round(p.x);
    markers[dragging].y = Math.round(p.y);
    changed(false);
  }

  function onUp() { dragging = -1; }

  function undo() { if (mode === 'replay') enterEdit(); if (markers.length) { markers.pop(); sel = markers.length - 1; changed(); } }
  function delSelected() { if (sel >= 0) { markers.splice(sel, 1); sel = Math.min(sel, markers.length - 1); changed(); } }
  function clearAll() { markers = []; sel = -1; ghost = null; changed(); }

  /** +/- buttons: a tap is 1 km/h, Shift-tap is 5, and holding the button repeats (faster the longer you hold). */
  function holdToRepeat(btn, dir) {
    let timer = 0;
    const stop = () => { clearTimeout(timer); timer = 0; };
    btn.addEventListener('pointerdown', (e) => {
      e.preventDefault();
      let n = 0;
      const tick = () => { bumpSpeed(dir * (e.shiftKey ? 5 : KMH_STEP)); n++; timer = setTimeout(tick, n < 3 ? 380 : n < 12 ? 110 : 45); };
      tick();
    });
    ['pointerup', 'pointerleave', 'pointercancel'].forEach((ev) => btn.addEventListener(ev, stop));
    btn.addEventListener('keydown', (e) => { if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); bumpSpeed(dir * KMH_STEP); } });
  }

  function bumpSpeed(d) {
    if (sel < 0) { toast('Select a marker first', true); return; }
    markers[sel].kmh = Math.max(KMH_MIN, Math.min(KMH_MAX, markers[sel].kmh + d));
    changed();
  }

  function changed(hideResult = true) {
    lineCache = null;
    if (hideResult) $('result').classList.remove('show');
    refreshUI();
  }

  // ------------------------------------------------------------------ codes

  const b64u = (s) => btoa(s).replace(/\+/g, '-').replace(/\//g, '_').replace(/=+$/, '');
  const unb64u = (s) => atob(s.replace(/-/g, '+').replace(/_/g, '/'));

  function encode() {
    if (!markers.length) return '';
    return 'RL1.' + b64u(TRACK_ID + '|' + markers.map((m) => `${m.x},${m.y},${m.kmh}`).join(';'));
  }

  function decode(code) {
    try {
      if (!code.startsWith('RL1.')) return null;
      const [id, body] = unb64u(code.slice(4)).split('|');
      const list = body.split(';').map((s) => s.split(',').map(Number));
      if (!list.length || list.length > MAX_MARKERS) return null;
      if (list.some((a) => a.length !== 3 || a.some((n) => !Number.isFinite(n)))) return null;
      return { id, markers: list.map(([x, y, k]) => ({ x, y, kmh: Math.max(KMH_MIN, Math.min(KMH_MAX, Math.round(k))) })) };
    } catch (e) { return null; }
  }

  function loadCode(code) {
    const d = decode(code.replace(/^.*#/, ''));
    if (!d) { toast('That code does not look right', true); return; }
    if (d.id !== TRACK_ID) toast('That code was made on a different track', true);
    markers = d.markers;
    sel = -1;
    changed();
    tryLine();
    if (/[?&]end/.test(location.search)) { seek(result.steps - 1); updateSkidAll(); } // ?end: jump to the finish (for screenshots / tests)
  }

  function updateSkidAll() { clearSkid(); updateSkid(result.steps - 1); }

  function linkFor(code) { return location.href.split('#')[0] + '#' + code; }

  function fromHash() {
    const h = location.hash.slice(1);
    if (h.startsWith('RL1.')) { $('codeIn').value = h; loadCode(h); }
  }

  function copy(text, msg) {
    if (!text) return;
    const done = () => toast(msg);
    if (navigator.clipboard && window.isSecureContext) navigator.clipboard.writeText(text).then(done, () => fallbackCopy(text, done));
    else fallbackCopy(text, done);
  }

  function fallbackCopy(text, done) {
    const t = document.createElement('textarea');
    t.value = text;
    document.body.appendChild(t);
    t.select();
    try { document.execCommand('copy'); done(); } catch (e) { toast('Select the code and copy it by hand', true); }
    t.remove();
  }

  // ------------------------------------------------------------------ try / replay

  function tryLine() {
    if (!markers.length) { toast('Drop at least one marker on the tarmac first', true); return; }

    const t0 = performance.now();
    result = RLSim.simulate(track, markers.map((m) => ({ x: m.x, y: m.y, v: m.kmh / 3.6 })));
    result.ms = performance.now() - t0;
    ghost = result;

    mode = 'replay';
    pb.pos = 0;
    pb.playing = true;
    pb.last = performance.now();
    $('scrub').max = String(Math.max(1, result.steps - 1));
    $('result').classList.remove('show');
    clearSkid();
    refreshUI();
  }

  function enterEdit() {
    mode = 'edit';
    pb.playing = false;
    refreshUI();
  }

  function seek(i) {
    pb.pos = Math.max(0, Math.min(result.steps - 1, i));
    if (Math.floor(pb.pos) >= result.steps - 1) showResult();
  }

  function showResult() {
    if ($('result').classList.contains('show') || !result) return;
    const big = $('resBig');
    const F = result.FRAME;
    let top = 0;
    for (let i = 0; i < result.steps; i++) top = Math.max(top, result.frames[i * F + 3]);
    const t = result.steps * track.dt;

    if (result.finished) {
      big.className = 'big ok';
      big.textContent = result.lapTime.toFixed(2) + ' s';
      if (!sessionBest || result.lapTime < sessionBest) {
        sessionBest = result.lapTime;
        try { localStorage.setItem('rl-best-' + TRACK_ID, String(sessionBest)); } catch (e) { /* ignore */ }
      }
    } else {
      big.className = 'big dnf';
      big.textContent = 'DNF';
    }

    const why = { grass: `Left the track at ${t.toFixed(1)} s`, wrongway: 'Went the wrong way round', stopped: 'Came to a stop', time: 'Ran out of time', lap: '' }[result.reason];
    $('resSub').textContent = `${why ? why + ' · ' : ''}top speed ${(top * 3.6).toFixed(0)} km/h · ${markers.length} marker${markers.length === 1 ? '' : 's'}` +
      (sessionBest ? ` · your best ${sessionBest.toFixed(2)} s` : '');

    const vs = $('resVs');
    if (D.benchmark) {
      if (result.finished) {
        const diff = result.lapTime - D.benchmark.seconds;
        vs.innerHTML = diff <= 0
          ? `You beat <b>${D.benchmark.label}</b> (${D.benchmark.seconds.toFixed(2)} s) by <b>${(-diff).toFixed(2)} s</b>.`
          : `<b>${D.benchmark.label}</b> did ${D.benchmark.seconds.toFixed(2)} s. You are <b>${diff.toFixed(2)} s</b> behind.`;
      } else vs.innerHTML = `<b>${D.benchmark.label}</b> did ${D.benchmark.seconds.toFixed(2)} s. Finish a lap to compare.`;
    } else vs.textContent = '';

    $('result').classList.add('show');
  }

  // ------------------------------------------------------------------ skid marks (same rule as the app's SkidMarks.cs)

  function clearSkid() { skidCtx.clearRect(0, 0, skid.width, skid.height); skidDone = -1; skidLast = { x: -99, y: -99 }; }

  function layMarks(i) {
    const f = result.frames, F = result.FRAME, o = i * F;
    const x = f[o], y = f[o + 1], ang = (f[o + 2] * Math.PI) / 180, fwd = f[o + 9];
    if (Math.abs(fwd) < 0.5) return;
    if (Math.abs(x - skidLast.x) < 1.5 && Math.abs(y - skidLast.y) < 1.5) return;

    const rear = Math.abs(f[o + 5]), front = Math.abs(f[o + 4]);
    const rearSliding = rear > 0.2, frontSliding = front > 0.2;
    if (!rearSliding && !frontSliding) return;

    const ppm = track.ppm, p = track.phys;
    const halfTrack = (D.widthMetres * ppm) / 2 * 0.85;
    const cos = Math.cos(ang), sin = Math.sin(ang);

    const tyres = (axle, slip) => {
      const s = slip > 0.5 ? 2 : slip > 0.3 ? 1 : 0;
      skidCtx.fillStyle = ['rgba(10,10,10,.30)', 'rgba(10,10,10,.50)', 'rgba(10,10,10,.72)'][s];
      for (const side of [-halfTrack, halfTrack]) skidCtx.fillRect(x + cos * axle - sin * side - 1.1, y + sin * axle + cos * side - 1.1, 2.2, 2.2);
    };
    if (rearSliding) tyres(-p.cogToRearAxle * ppm, rear);
    if (frontSliding) tyres(p.cogToFrontAxle * ppm, front);
    skidLast = { x, y };
  }

  function updateSkid(upTo) {
    if (upTo < skidDone) clearSkid();
    for (let i = skidDone + 1; i <= upTo; i++) layMarks(i);
    skidDone = Math.max(skidDone, upTo);
  }

  // ------------------------------------------------------------------ drawing

  const speedColour = (kmh, a = 1) => {
    const u = Math.max(0, Math.min(1, (kmh - KMH_MIN) / (KMH_MAX - KMH_MIN)));
    return `hsla(${(210 - 210 * u).toFixed(0)}, 90%, 55%, ${a})`;
  };

  function frame(now) {
    requestAnimationFrame(frame);
    if (!track) return;

    if (mode === 'replay' && result) {
      if (pb.playing) {
        const dtMs = Math.min(now - pb.last, 100);
        pb.pos += (dtMs / 1000 / track.dt) * pb.speed;
        if (pb.pos >= result.steps - 1) { pb.pos = result.steps - 1; pb.playing = false; showResult(); refreshUI(); }
      }
      pb.last = now;
      updateSkid(Math.floor(pb.pos));
      $('scrub').value = String(Math.floor(pb.pos));
    }

    draw();
  }

  function drawKart(c, x, y, angleDeg, steerAngle, alpha = 1) {
    if (carImg) {
      // the app's own picture of the car, same size, facing right at 0 degrees
      c.save();
      c.translate(x, y);
      c.rotate((angleDeg * Math.PI) / 180);
      c.globalAlpha = alpha;
      c.drawImage(carImg, -D.sprite.cx, -D.sprite.cy);
      c.restore();
      return;
    }
    const ppm = track.ppm, p = track.phys;
    const L = D.lengthMetres * ppm, W = D.widthMetres * ppm;
    c.save();
    c.translate(x, y);
    c.rotate((angleDeg * Math.PI) / 180);
    c.globalAlpha = alpha;

    // tyres at the axles, the front pair turned with the steering
    c.fillStyle = '#111';
    const tw = W * 0.30, tl = L * 0.26;
    c.fillRect(-p.cogToRearAxle * ppm - tl / 2, -W / 2 - tw * 0.3, tl, tw);
    c.fillRect(-p.cogToRearAxle * ppm - tl / 2, W / 2 - tw * 0.7, tl, tw);
    for (const s of [-1, 1]) {
      c.save();
      c.translate(p.cogToFrontAxle * ppm, s * (W / 2 - tw * 0.2));
      c.rotate(steerAngle || 0);
      c.fillRect(-tl / 2, -tw / 2, tl, tw);
      c.restore();
    }

    // body, nose and driver
    c.fillStyle = '#e63a2e';
    c.beginPath();
    c.roundRect(-L / 2, -W / 2 + 1, L, W - 2, 2);
    c.fill();
    c.fillStyle = '#f4f4f4';
    c.fillRect(L / 2 - 3.2, -W / 2 + 2.5, 2, W - 5);
    c.fillStyle = '#1b1b1b';
    c.beginPath();
    c.arc(-L * 0.12, 0, W * 0.27, 0, Math.PI * 2);
    c.fill();
    c.restore();
  }

  function draw() {
    const s = canvas.width / D.width;
    ctx.setTransform(s, 0, 0, s, 0, 0);
    ctx.imageSmoothingEnabled = true;
    ctx.drawImage(trackImg, 0, 0);

    if (mode === 'replay' && result) drawReplay();
    else drawEdit();

    stageBox.classList.toggle('replay', mode === 'replay');
    stageBox.classList.toggle('dv', kartView);
  }

  function getLine() {
    if (!lineCache && markers.length) {
      lineCache = RLSim.buildLine(track, markers.map((m) => ({ x: m.x, y: m.y, v: m.kmh / 3.6 })), (0.75 * track.phys.brakeForce) / track.phys.mass);
    }
    return lineCache;
  }

  function drawLineOnTrack(line, widthPx) {
    ctx.lineCap = 'round';
    ctx.lineWidth = widthPx;
    const { pts, v } = line;
    for (let i = 0; i + 4 < pts.length; i += 4) {
      ctx.strokeStyle = speedColour(v[i] * 3.6, 0.9);
      ctx.beginPath();
      ctx.moveTo(pts[i].x, pts[i].y);
      for (let k = 1; k <= 4; k++) ctx.lineTo(pts[i + k].x, pts[i + k].y);
      ctx.stroke();
    }
  }

  function drawMarkers(fade) {
    const r = 11 * (D.width / canvas.clientWidth);
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    markers.forEach((m, i) => {
      ctx.globalAlpha = fade;
      ctx.beginPath();
      ctx.arc(m.x, m.y, r, 0, Math.PI * 2);
      ctx.fillStyle = speedColour(m.kmh);
      ctx.fill();
      ctx.lineWidth = (i === sel ? 3 : 1.5) * (D.width / canvas.clientWidth);
      ctx.strokeStyle = i === sel ? '#fff' : 'rgba(0,0,0,.65)';
      ctx.stroke();
      ctx.fillStyle = '#fff';
      ctx.font = `700 ${12 * (D.width / canvas.clientWidth)}px system-ui, sans-serif`;
      ctx.fillText(String(i + 1), m.x, m.y + 0.5);
      ctx.font = `600 ${10 * (D.width / canvas.clientWidth)}px system-ui, sans-serif`;
      ctx.lineWidth = 3 * (D.width / canvas.clientWidth);
      ctx.strokeStyle = 'rgba(0,0,0,.75)';
      ctx.strokeText(String(m.kmh), m.x, m.y + r + 8 * (D.width / canvas.clientWidth));
      ctx.fillStyle = '#fff';
      ctx.fillText(String(m.kmh), m.x, m.y + r + 8 * (D.width / canvas.clientWidth));
    });
    ctx.globalAlpha = 1;
  }

  function drawEdit() {
    // the last attempt, faintly, so you can see what to change
    if (ghost && ghost.frames.length) {
      ctx.strokeStyle = 'rgba(255,255,255,.35)';
      ctx.lineWidth = 1.5 * (D.width / canvas.clientWidth);
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      const f = ghost.frames, F = ghost.FRAME;
      for (let i = 0; i < ghost.steps; i += 2) (i ? ctx.lineTo : ctx.moveTo).call(ctx, f[i * F], f[i * F + 1]);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    const line = getLine();
    if (line) drawLineOnTrack(line, 2.4 * (D.width / canvas.clientWidth));
    drawMarkers(1);

    drawKart(ctx, track.start.x, track.start.y, track.heading, 0, 0.95);

    $('hudSpeed').textContent = '0';
    $('hudTime').textContent = '0.00';
  }

  function drawReplay() {
    ctx.drawImage(skid, 0, 0);

    const i = Math.floor(pb.pos);
    const f = result.frames, F = result.FRAME;

    // the path driven, coloured by actual speed
    ctx.lineCap = 'round';
    ctx.lineWidth = 1.6 * (D.width / canvas.clientWidth);
    const end = pb.playing ? i : result.steps - 1;
    for (let k = 0; k + 3 <= end; k += 3) {
      ctx.strokeStyle = speedColour(f[k * F + 3] * 3.6, 0.55);
      ctx.beginPath();
      ctx.moveTo(f[k * F], f[k * F + 1]);
      ctx.lineTo(f[(k + 3) * F], f[(k + 3) * F + 1]);
      ctx.stroke();
    }

    drawMarkers(0.55);

    const o = i * F;
    drawKart(ctx, f[o], f[o + 1], f[o + 2], f[o + 6]);

    if (showHit) {
      ctx.fillStyle = 'cyan';
      for (const p of track.hitPointsFor(f[o], f[o + 1], f[o + 2])) ctx.fillRect(p.x - 0.5, p.y - 0.5, 1.5, 1.5);
    }

    // where it ended
    if (i >= result.steps - 1 && !result.finished) {
      ctx.strokeStyle = '#ff4d3d';
      ctx.lineWidth = 3 * (D.width / canvas.clientWidth);
      const r = 9 * (D.width / canvas.clientWidth);
      ctx.beginPath();
      ctx.moveTo(f[o] - r, f[o + 1] - r); ctx.lineTo(f[o] + r, f[o + 1] + r);
      ctx.moveTo(f[o] + r, f[o + 1] - r); ctx.lineTo(f[o] - r, f[o + 1] + r);
      ctx.stroke();
    }

    $('hudSpeed').textContent = (f[o + 3] * 3.6).toFixed(0);
    $('hudTime').textContent = ((i + 1) * track.dt).toFixed(2);

    if (kartView) drawDriverView(f[o], f[o + 1], f[o + 2], f[o + 6]);
  }

  // ------------------------------------------------------------------ the in-kart view (a floor projection of the track picture)

  const DW = 320, DH = 180;
  const dImg = dctx.createImageData(DW, DH);
  const d32 = new Uint32Array(dImg.data.buffer);
  const SKY_TOP = [88, 150, 215], SKY_LOW = [200, 225, 245];
  const HORIZON = Math.floor(DH * 0.40);
  const FOCAL = DW * 0.62, CAM_H = 15, CAM_BACK = 8;
  const pack = (r, g, b) => (255 << 24) | (b << 16) | (g << 8) | r;
  const FOG = [200, 225, 245];

  function drawDriverView(x, y, angleDeg, steerAngle) {
    const a = (angleDeg * Math.PI) / 180;
    const fx = Math.cos(a), fy = Math.sin(a), rx = -fy, ry = fx;
    const cx0 = x - fx * CAM_BACK, cy0 = y - fy * CAM_BACK;
    const tw = D.width, th = D.height;

    for (let sy = 0; sy <= HORIZON; sy++) {
      const u = sy / HORIZON;
      const c = pack(SKY_TOP[0] + (SKY_LOW[0] - SKY_TOP[0]) * u, SKY_TOP[1] + (SKY_LOW[1] - SKY_TOP[1]) * u, SKY_TOP[2] + (SKY_LOW[2] - SKY_TOP[2]) * u);
      d32.fill(c, sy * DW, (sy + 1) * DW);
    }

    for (let sy = HORIZON + 1; sy < DH; sy++) {
      const z = (CAM_H * FOCAL) / (sy - HORIZON);
      const fog = Math.min(1, Math.pow(z / 1100, 1.4));
      const px = (z / FOCAL); // world px per screen px across
      const bx = cx0 + fx * z - rx * px * (DW / 2);
      const by = cy0 + fy * z - ry * px * (DW / 2);
      const row = sy * DW;
      for (let sx = 0; sx < DW; sx++) {
        const wx = (bx + rx * px * sx) | 0, wy = (by + ry * px * sx) | 0;
        let c;
        if (wx < 0 || wy < 0 || wx >= tw || wy >= th) c = 0xff2c9181; // off the picture: grass
        else c = trackPixels[wy * tw + wx];
        if (fog > 0.02) {
          const r = (c & 255) + (FOG[0] - (c & 255)) * fog;
          const g = ((c >> 8) & 255) + (FOG[1] - ((c >> 8) & 255)) * fog;
          const b = ((c >> 16) & 255) + (FOG[2] - ((c >> 16) & 255)) * fog;
          c = pack(r, g, b);
        }
        d32[row + sx] = c;
      }
    }
    dctx.putImageData(dImg, 0, 0);

    // the front of the kart and the wheel
    dctx.fillStyle = '#111';
    dctx.fillRect(34, DH - 30, 34, 30);
    dctx.fillRect(DW - 68, DH - 30, 34, 30);
    dctx.fillStyle = '#d6352b';
    dctx.beginPath();
    dctx.moveTo(DW / 2 - 52, DH);
    dctx.lineTo(DW / 2 - 24, DH - 34);
    dctx.lineTo(DW / 2 + 24, DH - 34);
    dctx.lineTo(DW / 2 + 52, DH);
    dctx.fill();
    dctx.save();
    dctx.translate(DW / 2, DH - 6);
    dctx.rotate(steerAngle * 4.5);
    dctx.strokeStyle = '#222';
    dctx.lineWidth = 5;
    dctx.beginPath();
    dctx.arc(0, 0, 21, 0, Math.PI * 2);
    dctx.stroke();
    dctx.beginPath();
    dctx.moveTo(-21, 0); dctx.lineTo(21, 0); dctx.moveTo(0, 0); dctx.lineTo(0, 21);
    dctx.stroke();
    dctx.fillStyle = '#e6c200';
    dctx.fillRect(-3, -23, 6, 7);
    dctx.restore();
  }

  // ------------------------------------------------------------------ UI state

  function refreshUI() {
    $('code').value = encode();
    const hasSel = sel >= 0 && sel < markers.length;
    $('spdVal').textContent = hasSel ? markers[sel].kmh + ' km/h' : '–';
    $('spdDown').disabled = !hasSel;
    $('spdUp').disabled = !hasSel;
    $('btnDel').disabled = !hasSel;
    $('btnAll').disabled = !hasSel || markers.length < 2;
    $('btnUndo').disabled = !markers.length;
    $('btnClear').disabled = !markers.length;
    $('btnView').classList.toggle('on', kartView);
    $('bar').classList.toggle('replay-on', mode === 'replay');
    $('btnPause').textContent = pb.playing ? 'Pause' : (result && pb.pos >= result.steps - 1 ? 'Replay' : 'Play');
    $('btnPause').onclick = () => {
      if (result && pb.pos >= result.steps - 1) { pb.pos = 0; clearSkid(); $('result').classList.remove('show'); }
      pb.playing = !pb.playing; pb.last = performance.now(); refreshUI();
    };
    document.querySelectorAll('.sp').forEach((b) => b.classList.toggle('on', +b.dataset.sp === pb.speed));
  }

  let toastTimer = 0;
  function toast(msg, bad) {
    const t = $('toast');
    t.textContent = msg;
    t.classList.toggle('bad', !!bad);
    t.classList.add('show');
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => t.classList.remove('show'), 2200);
  }

  init().catch((e) => { console.error(e); document.body.insertAdjacentHTML('afterbegin', '<p style="padding:20px;color:#f88">Could not start: ' + e.message + '</p>'); });
})();
