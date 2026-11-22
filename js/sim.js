/*
 * Track, lap rules and the driver that follows your markers.
 *
 * The lap rules (grass test, hit points, gates, lap counting) are the C# app's, so the same lap means the same thing.
 * The DRIVER is new: the AI in the app is a small neural network that sees a few lidar beams. Here the "brain" is you:
 * you place numbered markers with a target speed, a smooth line is drawn through them, and a simple pure-pursuit steering
 * + speed controller drives it. The controller is deliberately plain, so the result is down to your line and speeds.
 *
 * Everything here is deterministic: same markers in, same lap time out.
 */
(function (global) {
  'use strict';

  const { Car, clamp, deg2rad } = global.RLPhysics;

  const roundEven = (v) => {
    const r = Math.round(v);
    return Math.abs(v % 1) === 0.5 && r % 2 !== 0 ? r - 1 : r; // banker's rounding, like C# Math.Round
  };

  // ------------------------------------------------------------------ Track

  class Track {
    /**
     * @param {object} meta   track.json (+ images)
     * @param {Uint8Array} grass   width*height, 1 = grass (mask pixel red == 255), 0 = road
     */
    constructor(meta, grass) {
      this.meta = meta;
      this.w = meta.width;
      this.h = meta.height;
      this.grass = grass;
      this.start = meta.start;
      this.heading = meta.headingDegrees;
      this.g1 = meta.gates1.map((p) => ({ x: p[0], y: p[1] }));
      this.g2 = meta.gates2.map((p) => ({ x: p[0], y: p[1] }));
      this.hit = meta.hitPoints.map((p) => ({ x: p[0], y: p[1] }));
      this.dt = meta.dt;
      this.phys = meta.physics;
      this.ppm = meta.physics.pixelsPerMetre;
    }

    /** Is the pixel under (x, y) grass? Outside the picture counts as grass. (App: (int)(0.5 + v)) */
    isGrassAt(x, y) {
      const ix = Math.trunc(0.5 + x);
      const iy = Math.trunc(0.5 + y);
      if (ix < 0 || iy < 0 || ix >= this.w || iy >= this.h) return true;
      return this.grass[iy * this.w + ix] === 1;
    }

    /** Is a placed marker on the road? (a small ring around it, so you cannot sit on the very edge) */
    isRoadForMarker(x, y) {
      if (this.isGrassAt(x, y)) return false;
      const r = 4;
      return !(this.isGrassAt(x + r, y) || this.isGrassAt(x - r, y) || this.isGrassAt(x, y + r) || this.isGrassAt(x, y - r));
    }

    /** True when any of the car's hit points is on grass: that is how the app decides a car has left the track. */
    collided(car) {
      const ox = roundEven(car.x);
      const oy = roundEven(car.y);
      const a = deg2rad(car.angleDeg);
      const cos = Math.cos(a);
      const sin = Math.sin(a);
      for (const p of this.hit) {
        const x = cos * p.x - sin * p.y + ox;
        const y = sin * p.x + cos * p.y + oy;
        if (this.isGrassAt(x, y)) return true;
      }
      return false;
    }

    /** Where each hit point is for a car at (x, y, angle): for drawing. */
    hitPointsFor(x, y, angleDeg) {
      const ox = roundEven(x);
      const oy = roundEven(y);
      const a = deg2rad(angleDeg);
      const cos = Math.cos(a);
      const sin = Math.sin(a);
      return this.hit.map((p) => ({ x: cos * p.x - sin * p.y + ox, y: sin * p.x + cos * p.y + oy }));
    }
  }

  // ------------------------------------------------------------------ Lap rules (port of LearningAndRaceManager.CarIsAtGate)

  const f32 = Math.fround;

  function distance(a, b) {
    const dx = f32(b.x - a.x);
    const dy = f32(b.y - a.y);
    return f32(Math.sqrt(dx * dx + dy * dy));
  }

  function isPointOnLine(a, b, p) {
    const sum = distance(p, a) + distance(p, b) - distance(a, b);
    return sum >= -0.1 && sum <= 0.1;
  }

  function segmentsIntersect(a, b, c, d) {
    const cross = (p, q, r) => (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
    const d1 = cross(c, d, a);
    const d2 = cross(c, d, b);
    const d3 = cross(a, b, c);
    const d4 = cross(a, b, d);
    return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0));
  }

  class LapCounter {
    constructor(track) {
      this.t = track;
      this.currentGate = 0;
      this.lastGatePassed = 0;
      this.prev = { x: track.start.x, y: track.start.y };
      this.wrongWay = false;
      this.lapped = false;
    }

    /** Call after each move with the car's new position. */
    update(x, y) {
      const t = this.t;
      const now = { x, y };
      const n = t.g1.length;
      const position = this.currentGate % n;
      const last = n - 1;

      for (let gate = 0; gate < n; gate++) {
        if (!isPointOnLine(t.g1[gate], t.g2[gate], now) && !segmentsIntersect(this.prev, now, t.g1[gate], t.g2[gate])) continue;

        let at;
        let lapped = false;
        if (gate === 0 && position === last) {
          at = this.currentGate + 1;
          lapped = true;
        } else {
          at = this.currentGate - position + gate;
        }

        this.currentGate = at;
        if (lapped) this.lapped = true;

        // going backwards, or skipping a long way ahead, is "wrong way" (the app eliminates such cars)
        if (this.lastGatePassed > this.currentGate || this.currentGate - 30 > this.lastGatePassed) this.wrongWay = true;
        else this.lastGatePassed = this.currentGate;
        break;
      }

      this.prev = now;
    }
  }

  // ------------------------------------------------------------------ The line: a smooth path through the markers

  /** Centripetal Catmull-Rom point between p1 and p2 (p0, p3 are the neighbours). t in 0..1. */
  function catmull(p0, p1, p2, p3, t) {
    const a = 0.5;
    const d01 = Math.pow(Math.hypot(p1.x - p0.x, p1.y - p0.y), a) || 1e-6;
    const d12 = Math.pow(Math.hypot(p2.x - p1.x, p2.y - p1.y), a) || 1e-6;
    const d23 = Math.pow(Math.hypot(p3.x - p2.x, p3.y - p2.y), a) || 1e-6;

    const t0 = 0;
    const t1 = t0 + d01;
    const t2 = t1 + d12;
    const t3 = t2 + d23;
    const u = t1 + (t2 - t1) * t;

    const lerp = (pa, pb, ta, tb) => ({
      x: ((tb - u) / (tb - ta)) * pa.x + ((u - ta) / (tb - ta)) * pb.x,
      y: ((tb - u) / (tb - ta)) * pa.y + ((u - ta) / (tb - ta)) * pb.y,
    });

    const A1 = lerp(p0, p1, t0, t1);
    const A2 = lerp(p1, p2, t1, t2);
    const A3 = lerp(p2, p3, t2, t3);
    const B1 = lerp(A1, A2, t0, t2);
    const B2 = lerp(A2, A3, t1, t3);
    return lerp(B1, B2, t1, t2);
  }

  /**
   * Builds the line. Starts at the start point, goes through each marker in order, then through the start line again.
   * markers: [{x, y, v}] with v in m/s.
   * Returns { pts: [{x,y}], v: [m/s target at each pt], ds: px between points }.
   */
  function buildLine(track, markers, brakeDecel) {
    const S = track.start;
    const a = deg2rad(track.heading);
    const d = { x: Math.cos(a), y: Math.sin(a) };
    const at = (k) => ({ x: S.x + d.x * k, y: S.y + d.y * k });

    const vFirst = markers.length ? markers[0].v : 10;
    const vLast = markers.length ? markers[markers.length - 1].v : 10;

    // control points: a phantom behind the start gives the first tangent, the end ones take the car through the line
    // The straight run-in before the line (50 px) only helps when the last marker is well back round the lap. If the last marker
    // is already close to the line, or beyond the run-in point, forcing the path through it would make a kink, so it is dropped
    // and the path simply heads from your last marker through the line.
    const lastM = markers.length ? markers[markers.length - 1] : null;
    const behind = lastM ? (lastM.x - S.x) * d.x + (lastM.y - S.y) * d.y : -999; // along the heading: negative = before the line
    const runIn = !lastM || (Math.hypot(lastM.x - S.x, lastM.y - S.y) > 110 && behind < -60);

    const ctl = [
      { ...at(-30), v: vFirst },
      { ...S, v: vFirst },
      ...markers.map((m) => ({ x: m.x, y: m.y, v: m.v })),
      ...(runIn ? [{ ...at(-50), v: vLast }] : []),
      { ...at(70), v: vLast },
      { ...at(130), v: vLast },
    ];

    // sample finely, remembering the arc length of each control point
    const fine = [];
    const ctlArc = [0];
    for (let i = 1; i < ctl.length - 2; i++) {
      for (let s = 0; s < 40; s++) fine.push(catmull(ctl[i - 1], ctl[i], ctl[i + 1], ctl[i + 2], s / 40));
      ctlArc.push(fine.length);
    }
    fine.push({ x: ctl[ctl.length - 2].x, y: ctl[ctl.length - 2].y });

    // arc length of the fine points
    const arc = [0];
    for (let i = 1; i < fine.length; i++) arc.push(arc[i - 1] + Math.hypot(fine[i].x - fine[i - 1].x, fine[i].y - fine[i - 1].y));
    const total = arc[arc.length - 1];

    // resample at an even spacing (px), and set each point's target speed between its neighbouring controls
    const ds = 2;
    const n = Math.floor(total / ds) + 1;
    const pts = [];
    const v = [];
    const ctlS = ctlArc.map((idx) => arc[Math.min(idx, arc.length - 1)]); // arc length at each control (from ctl[1])

    let j = 0;
    for (let i = 0; i < n; i++) {
      const s = i * ds;
      while (j < arc.length - 2 && arc[j + 1] < s) j++;
      const f = (s - arc[j]) / Math.max(arc[j + 1] - arc[j], 1e-9);
      pts.push({ x: fine[j].x + (fine[j + 1].x - fine[j].x) * f, y: fine[j].y + (fine[j + 1].y - fine[j].y) * f });

      let k = 0;
      while (k < ctlS.length - 2 && ctlS[k + 1] <= s) k++;
      const s0 = ctlS[k];
      const s1 = ctlS[k + 1];
      const va = ctl[k + 1].v;
      const vb = ctl[k + 2].v;
      const u = s1 > s0 ? clamp((s - s0) / (s1 - s0), 0, 1) : 0;
      v.push(va + (vb - va) * u);
    }

    // you cannot brake instantly: look backwards along the line and lower any speed that is too fast to slow in time
    const dsMetres = ds / track.ppm;
    for (let i = n - 2; i >= 0; i--) v[i] = Math.min(v[i], Math.sqrt(v[i + 1] * v[i + 1] + 2 * brakeDecel * dsMetres));

    return { pts, v, ds, ctlS };
  }

  // ------------------------------------------------------------------ The driver: pure pursuit + speed control

  const wrapPi = (a) => {
    while (a > Math.PI) a -= 2 * Math.PI;
    while (a < -Math.PI) a += 2 * Math.PI;
    return a;
  };

  class Driver {
    constructor(track, line) {
      this.t = track;
      this.line = line;
      this.idx = 0;
      this.c = track.phys;
      this.wheelBase = this.c.cogToFrontAxle + this.c.cogToRearAxle;
    }

    /** What the pedals and wheel do this step. */
    control(car) {
      const { pts, v, ds } = this.line;
      const ppm = this.t.ppm;
      const c = this.c;
      const last = pts.length - 1;

      // where on the line are we? (only ever moves forward, within a window)
      let best = this.idx;
      let bestD = Infinity;
      for (let i = this.idx; i <= Math.min(last, this.idx + 120); i++) {
        const dd = (pts[i].x - car.x) ** 2 + (pts[i].y - car.y) ** 2;
        if (dd < bestD) {
          bestD = dd;
          best = i;
        }
      }
      this.idx = best;

      const speed = Math.max(car.fwdSpeed, 0);

      // steering: aim at a point a little way along the line (further away the faster we go)
      const lookM = clamp(0.7 + 0.30 * speed, 1.3, 7.0);
      const ti = Math.min(last, best + Math.round((lookM * ppm) / ds));
      const tx = pts[ti].x;
      const ty = pts[ti].y;
      const alpha = wrapPi(Math.atan2(ty - car.y, tx - car.x) - deg2rad(car.angleDeg));
      const delta = Math.atan2(2 * this.wheelBase * Math.sin(alpha), lookM); // wheel angle that would arc onto the point

      const safe = 1.0 - 0.5 * Math.min(car.speed / c.safeSteerHalfLockSpeed, 1.0);
      const desired = clamp(delta / (c.maxSteer * safe), -1, 1);

      // the wheel moves at a RATE; ask for the rate that gets it to "desired" in one step
      let steer = clamp((desired - car.steerAmount) / (7.0 * car.dt), -1, 1);
      if (Math.abs(steer) < 0.002) steer = steer < 0 ? -0.002 : 0.002; // below 0.001 the wheel would self-centre instead of holding

      // speed: the target at this point on the line, and a little ahead of it (the line already allows for braking distance)
      const ahead = Math.min(last, best + Math.round((speed * 0.25 * ppm) / ds));
      const vt = Math.min(v[best], v[ahead]);
      const err = vt - speed;

      let throttle = 0;
      let brake = 0;
      if (err > -0.15) {
        const full = speed > 1e-6 ? Math.min(c.engineForce, c.enginePowerWatts / speed) : c.engineForce;
        const hold = (vt * (c.rollResist + c.airResist * vt)) / full; // throttle that just balances drag at the target speed
        throttle = clamp(hold + 0.8 * err, 0, 1);
      } else {
        brake = clamp((-err - 0.15) * 1.5, 0, 1);
      }

      return { steer, throttle, brake, ebrake: 0, target: { x: tx, y: ty }, vt };
    }
  }

  // ------------------------------------------------------------------ A whole lap

  const MAX_SECONDS = 120;

  /**
   * Drives the markers and returns everything needed to replay and judge it.
   *   markers: [{x, y, v}]  v in m/s
   * result: { finished, reason, steps, lapTime, frames (Float32Array, FRAME floats each), line, endIndex }
   */
  function simulate(track, markers) {
    const phys = track.phys;
    const brakeDecel = (0.75 * phys.brakeForce) / phys.mass;
    const line = buildLine(track, markers, brakeDecel);

    const car = new Car(phys, track.dt, track.start.x, track.start.y, track.heading);
    const laps = new LapCounter(track);
    const driver = new Driver(track, line);

    const maxSteps = Math.round(MAX_SECONDS / track.dt);
    const FRAME = 10;
    const frames = new Float32Array(maxSteps * FRAME);

    let steps = 0;
    let reason = 'time';
    let finished = false;
    let stuck = 0;

    while (steps < maxSteps) {
      const input = driver.control(car);
      car.step(input);

      const o = steps * FRAME;
      frames[o] = car.x;
      frames[o + 1] = car.y;
      frames[o + 2] = car.angleDeg;
      frames[o + 3] = car.speed;
      frames[o + 4] = car.slipFront;
      frames[o + 5] = car.slipRear;
      frames[o + 6] = car.steerAngle;
      frames[o + 7] = input.throttle;
      frames[o + 8] = input.brake;
      frames[o + 9] = car.fwdSpeed;
      steps++;

      // off the road: out (the app removes such a car)
      if (track.collided(car)) {
        reason = 'grass';
        break;
      }

      laps.update(car.x, car.y);
      if (laps.wrongWay) {
        reason = 'wrongway';
        break;
      }
      if (laps.lapped) {
        finished = true;
        reason = 'lap';
        break;
      }

      stuck = car.speed < 0.3 && steps > 40 ? stuck + 1 : 0;
      if (stuck > 100) {
        reason = 'stopped';
        break;
      }
    }

    return {
      finished,
      reason,
      steps,
      lapTime: finished ? steps * track.dt : null,
      frames: frames.subarray(0, steps * FRAME),
      FRAME,
      line,
    };
  }

  /** Build the grass lookup from the mask picture's RGBA pixels (red == 255 means grass, as in the app). */
  function grassFromRGBA(data, w, h) {
    const grass = new Uint8Array(w * h);
    for (let i = 0; i < w * h; i++) grass[i] = data[i * 4] === 255 ? 1 : 0;
    return grass;
  }

  global.RLSim = { Track, LapCounter, buildLine, Driver, simulate, grassFromRGBA };
})(typeof window !== 'undefined' ? window : globalThis);
