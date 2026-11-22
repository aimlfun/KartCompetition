/*
 * Car physics, ported line-for-line from World/Car/CarUsingRealPhysics.cs in the C# "cars-n-tanks" app, so a human lap here
 * and an AI lap there are driven by the same car. All physics constants come from track.json (written by the app's
 * Track > Export for web), nothing is tuned here.
 *
 * Original physics: https://github.com/spacejack/carphysics2d and
 * https://asawicki.info/Mirror/Car%20Physics%20for%20Games/Car%20Physics%20for%20Games.html
 * .. with improved accuracy using LLMs.
 *
 * The C# stores some values as 32-bit floats, so Math.fround is used at the same places to keep the two in step.
 *
 * Units: physics is in metres / newtons / seconds. Position is in track pixels (pixelsPerMetre converts).
 * Angles: heading is in degrees on the way in and out (0 = right, 90 = down the screen, -90 = up), radians inside.
 */
(function (global) {
  'use strict';

  const f32 = Math.fround;
  const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);
  const deg2rad = (d) => (Math.PI * d) / 180;
  const rad2deg = (r) => (r * 180) / Math.PI;

  class Car {
    /**
     * @param {object} cfg  the "physics" block from track.json
     * @param {number} dt   seconds per step (0.03)
     */
    constructor(cfg, dt, x, y, headingDegrees) {
      this.c = cfg;
      this.dt = dt;
      this.x = x;
      this.y = y;
      this.angleDeg = headingDegrees;

      this.vx = 0; // world velocity, m/s
      this.vy = 0;
      this.speed = 0; // |v|
      this.angVel = 0; // rad/s, +ve = clockwise on screen
      this.steerAmount = 0; // -1..1 steering position
      this.steerAngle = 0; // front wheel angle, radians

      this.accLocalF = 0; // m/s2 along the car
      this.accLocalS = 0; // m/s2 sideways
      this.accWorldX = 0;
      this.accWorldY = 0;

      this.slipFront = 0;
      this.slipRear = 0;
      this.fwdSpeed = 0; // speed along the way the car points (m/s), valid after step()
      this.sideSpeed = 0;

      this.totalThrottleForce = 0;
      this.brakeApplied = 0;
      this.frontWeight = 0;
      this.rearWeight = 0;
      this.frictionFront = 0;
      this.frictionRear = 0;

      this.wheelBase = cfg.cogToFrontAxle + cfg.cogToRearAxle;
    }

    /**
     * One physics step. inputs = { steer, throttle, brake, ebrake }
     *   steer    -1..1: a steering RATE (the wheel moves toward it), exactly as the AI's steering output is used
     *   throttle 0..1.5 (the engine cannot give more than 1)
     *   brake    0..1
     *   ebrake   0..1
     */
    step(inputs) {
      const c = this.c;
      const dt = this.dt;
      let heading = deg2rad(this.angleDeg);

      const steerIn = inputs.steer;
      const throttleIn = clamp(inputs.throttle, 0, 1.5);
      const brakeIn = clamp(inputs.brake, 0, 1);
      const ebrakeIn = clamp(inputs.ebrake, 0, 1);

      // --- steering: smoothed (a rate, not a position) and reduced with speed ("safe steer") ---
      this.steerAmount = this._smoothSteer(steerIn, dt);
      const safe = 1.0 - 0.5 * Math.min(this.speed / c.safeSteerHalfLockSpeed, 1.0);
      this.steerAngle = this.steerAmount * c.maxSteer * safe;

      // --- DoPhysics ---
      const sin = Math.sin(heading);
      const cos = Math.cos(heading);

      const wb = this.wheelBase;
      const ratioFront = c.cogToRearAxle / wb;
      const ratioRear = c.cogToFrontAxle / wb;

      // velocity in the car's own axes
      const vf = f32(cos * this.vx + sin * this.vy); // forwards
      const vs = f32(cos * this.vy - sin * this.vx); // sideways

      // weight on each axle (static split, weight transfer under accel/brake, aero downforce)
      const speedSq = this.vx * this.vx + this.vy * this.vy;
      const downforce = c.downforceCoefficient * speedSq;
      this.frontWeight = Math.max(0, c.mass * (ratioFront * c.gravity - c.weightTransfer * this.accLocalF * c.cogHeight / wb)) + downforce * c.aeroBalanceFront;
      this.rearWeight = Math.max(0, c.mass * (ratioRear * c.gravity + c.weightTransfer * this.accLocalF * c.cogHeight / wb)) + downforce * (1 - c.aeroBalanceFront);

      // yaw: speed of each axle due to the car rotating
      const yawFront = c.cogToFrontAxle * this.angVel;
      const yawRear = -c.cogToRearAxle * this.angVel;

      // slip angles
      const signVf = Math.sign(vf);
      this.slipFront = f32(Math.atan2(vs + yawFront, Math.abs(vf))) - signVf * this.steerAngle;
      this.slipRear = f32(Math.atan2(vs + yawRear, Math.abs(vf)));

      // traction (before friction: the tyres' grip is shared, see the friction circle)
      let brake = brakeIn * c.brakeForce + ebrakeIn * c.eBrakeForce;
      brake = Math.min(brake, c.brakeForce);

      const throttle = Math.min(throttleIn, 1.0);
      const speedAlong = Math.abs(vf);
      const fullThrottle = speedAlong > 1e-6 ? Math.min(c.engineForce, c.enginePowerWatts / speedAlong) : c.engineForce;
      this.totalThrottleForce = throttle * fullThrottle;

      if (c.frictionCircle > 0) {
        const driveAxleGrip = c.driveFrontAxle ? c.gripFront * this.frontWeight : c.gripRear * this.rearWeight;
        this.totalThrottleForce = Math.min(this.totalThrottleForce, driveAxleGrip);
      }
      this.brakeApplied = brake;
      const traction = this.totalThrottleForce - brake * signVf;

      // tyre friction (Pacejka "magic formula")
      const gripFront = c.gripFront;
      const gripRear = c.gripRear * (1.0 - ebrakeIn * (1.0 - c.gripWhenWheelIsLocked / 100));
      const pacejka = (slip, peak, stiff) => peak * Math.sin(c.tyreShape * Math.atan(stiff * slip));
      let muFront = pacejka(this.slipFront, gripFront, c.cornerStiffnessFront);
      let muRear = pacejka(this.slipRear, gripRear, c.cornerStiffnessRear);

      if (c.frictionCircle > 0) {
        const left = (fx, maxGrip) => {
          if (maxGrip < 1e-6) return 0;
          const used = Math.min(Math.abs(fx) / maxGrip, 1);
          const l = Math.sqrt(1 - used * used);
          return 1 - c.frictionCircle * (1 - l);
        };
        const fxFront = (c.driveFrontAxle ? this.totalThrottleForce : 0) + brake * c.brakeBiasFront;
        const fxRear = (c.driveFrontAxle ? 0 : this.totalThrottleForce) + brake * (1 - c.brakeBiasFront);
        muFront *= left(fxFront, gripFront * this.frontWeight);
        muRear *= left(fxRear, gripRear * this.rearWeight);
      }
      this.frictionFront = -muFront * this.frontWeight;
      this.frictionRear = -muRear * this.rearWeight;

      // drag (air + rolling), in the car's axes
      const spd = Math.sqrt(vf * vf + vs * vs);
      const dragLong = vf * (c.rollResist + c.airResist * spd);
      const dragLat = vs * (c.rollResist + c.airResist * spd);

      // total forces
      const cosSteer = Math.cos(this.steerAngle);
      const totalLat = cosSteer * this.frictionFront + this.frictionRear - dragLat;
      const totalLong = traction - dragLong - Math.sin(this.steerAngle) * this.frictionFront;

      // acceleration (F = m a), then into world axes
      this.accLocalF = f32(totalLong / c.mass);
      this.accLocalS = f32(totalLat / c.mass);
      this.accWorldX = f32(cos * this.accLocalF - sin * this.accLocalS);
      this.accWorldY = f32(sin * this.accLocalF + cos * this.accLocalS);

      // velocity
      this.vx = f32(this.vx + f32(this.accWorldX * dt));
      this.vy = f32(this.vy + f32(this.accWorldY * dt));
      this.speed = f32(Math.sqrt(this.vx * this.vx + this.vy * this.vy));

      // torque about the vertical axis
      let torque = cosSteer * this.frictionFront * c.cogToFrontAxle - this.frictionRear * c.cogToRearAxle;

      // very slow and no throttle: just stop (the model is unstable down there)
      if (Math.abs(this.speed) < 0.5 && this.totalThrottleForce === 0) {
        this.vx = 0;
        this.vy = 0;
        this.speed = 0;
        torque = 0;
        this.angVel = 0;
      }

      // rotation
      const angAccel = torque / c.yawInertia;
      this.angVel += angAccel * dt;
      heading += this.angVel * dt;
      if (heading > 2 * Math.PI) heading -= 2 * Math.PI;
      else if (heading < 0) heading += 2 * Math.PI;

      // move (metres -> pixels)
      this.x = f32(this.x + f32(this.vx * dt * c.pixelsPerMetre));
      this.y = f32(this.y + f32(this.vy * dt * c.pixelsPerMetre));

      this.angleDeg = rad2deg(heading);

      // handy for the driver and the HUD: speed along / across the car as it is now
      const h = heading;
      this.fwdSpeed = Math.cos(h) * this.vx + Math.sin(h) * this.vy;
      this.sideSpeed = Math.cos(h) * this.vy - Math.sin(h) * this.vx;
    }

    /** Steering input is a rate. With no input, the wheel self-centres at 1/s. (Same as the C# ApplySmoothSteer.) */
    _smoothSteer(input, dt) {
      if (Math.abs(input) > 0.001) return clamp(this.steerAmount + input * dt * 7.0, -1.0, 1.0);
      if (this.steerAmount > 0) return Math.max(this.steerAmount - dt * 1.0, 0);
      if (this.steerAmount < 0) return Math.min(this.steerAmount + dt * 1.0, 0);
      return 0;
    }
  }

  global.RLPhysics = { Car, clamp, deg2rad, rad2deg, f32 };
})(typeof window !== 'undefined' ? window : globalThis);
