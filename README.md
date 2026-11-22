# Racing Line Challenge

Place numbered markers on a track, give each one a target speed, and watch a physically simulated go-kart drive your line.
Finish a lap, get a time, and share your line as a short code. 

## Can you beat an AI that sees through five lidar beams?

No install, no build step, no server. Download the folder and open `index.html` in a browser.

## Play

1. Tap or click the tarmac to drop markers (the limit grows with the track: 40% of its gate count, rounded to 5, so 20 on the Demo). They are driven in order, from the chequered line, heading up.
2. Select a marker and change its speed (`+` / `-`, or the Up / Down arrows). Drag a marker to move it.
3. Press **Try** (or Space). Finish a lap for a time. Leave the tarmac and it is a DNF.
4. Copy your **code** and post it. Anyone can paste it into "Replay it" and get exactly your lap time.

Keys: `Space`/`Enter` try, `Z` undo, `Delete` remove marker, `Up`/`Down` speed, `V` kart view, `H` show the collision points.

## How honest is it?

- `js/physics.js` is a line-by-line port of the car in the C# app (`World/Car/CarUsingRealPhysics.cs`): a full
  dynamic model (one tyre per axle, as the source puts it) with slip-angle tyres (Pacejka), weight transfer under braking
  and acceleration, aero downforce, a friction circle, air and rolling drag, a power-limited engine with a traction cap, 
  and steering lock that eases off with speed. The same 30 ms time step. Compared against the C# source over 1,500 steps 
  of hard driving (throttle, brake, e-brake, slides) the two agree to about 0.02 px.
- Every constant (mass, grip, power, car size, the nine collision points) is read from `track.json`, which the app writes.
  Nothing is tuned in the browser.
- Lap rules (grass test, gates, "wrong way" detection, lap time = steps x 0.03 s, standing start) are the app's.
- The driver (`js/sim.js`) is deliberately plain: a smooth line through your markers, pure-pursuit steering, and a speed
  controller that brakes early enough to hit the speed you asked for. It will not rescue a bad line.
- Same markers in, same lap out. One caveat: browsers can differ in the last digit of `sin`/`atan`, so a marginal
  line could in theory differ between Chrome and Firefox. 
  
Winners should clarify their browser.

## Putting a new track in

The page runs on whatever `track-data.js` holds. To use a track from the C# app: (soon to be uploaded to GitHub, after the challenge)

1. In the app: **Track > Load...**, pick the track, then **Track > Export for web...** (it needs the real physics and a vehicle,
   see Configure > Vehicle). It writes `track.png`, `mask.png` and `track.json` to a folder on the Desktop.
2. Pack it, adding the AI's lap time so the page can compare:

   ```
   node tools/pack-track.mjs "C:/Users/<you>/Desktop/web-challenge-export-Demo" 12.18 "Label of track"
   ```

   This writes `track-data.js`: the images are embedded as data URLs, because browsers will not let a page read the pixels
   of a file next to it when opened from disk.

`dev-export/` is a stand-in built from a plain picture (`tools/make-dev-track.py`) so the page could be developed without
the app. The real export replaces it.

## Files

| File | What it is |
| --- | --- |
| `index.html` | The page (layout and styling) |
| `js/physics.js` | The car. Port of the C# physics |
| `js/sim.js` | Track, lap rules, the line through your markers, the driver, a whole simulated lap |
| `js/app.js` | Markers, replay, skid marks, kart view, codes |
| `track-data.js` | The track: picture, mask, gates, physics. Generated |
| `tools/pack-track.mjs` | Turns an export folder into `track-data.js` |
| `tools/test-sim.mjs` | Headless check: drives the gate mid-points at a few speeds |
| `tools/optimise.mjs` | Crude hill-climb: how fast can a line be? (sanity check, not shipped to players) |
