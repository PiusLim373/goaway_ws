# goaway_ui

Dashboard for the goaway Go2 stack. Same design as tmms_ui: React + Vite + Tailwind, talking
to ROS through rosbridge (`ws://<page host>:9090`, started by `goaway_master operation.launch.py`).

Dashboard: Lichtblick on the left (60%, drag the divider), quadruped control on the right.
- **Lichtblick:** `goaway_lichtblick` on `:8080` (see `app/goaway_lichtblick`), auto-connected to
  rosbridge, with `dashboard_layout.json`. The robot model comes from `go2_visual.urdf`.
- **Top:** gear shift (SIT, STAND LOCK, WALK), the `UNITREE | HTX` switch (only in SIT), and in
  HTX the skill cards: RECOVERY (always first), DONKEY KICK, QUICK BUCK, and SIDE FLIP (one card,
  `◀ LEFT` | `RIGHT ▶` = the robot's own side; its clip alternates the two flips).
- **Middle:** the SPEED slider (LOW, MID, HIGH; `<` / `>` keys) and driving: joystick, rotation
  knob, or the keyboard (arrows, ⇧+←→ to turn); the SAFETY switch on its own row below them.
- **Bottom:** joint temps.
- **Header:** battery, ping, clock, light/dark. **Footer:** the E-stop (damp).

Speed limits (all inside HTX perceptive_v2's trained ranges):

| Level | forward/back (m/s) | strafe (m/s) | turn (rad/s) |
|---|---|---|---|
| LOW | 0.3 | 0.3 | 0.5 |
| MID | 0.7 | 0.4 | 0.7 |
| HIGH | 1.0 | 0.5 | 1.0 |

The keyboard only drives while the control panel has focus: after clicking inside Lichtblick,
click the panel again.

Skill clips (`public/skills/*.mp4`) are recorded offscreen in MuJoCo (mjlab) from the deployed
checkpoints: muted H.264 loops, so they also play on iPad/Safari.

Everything goes through `goaway_quadruped_controller`:
- `/goaway_quadruped_controller/quadruped_cmd` (goaway_msgs/StringTrigger)
- `quadruped_main_status`
- `/quadruped_cmd_vel_ui` (SAFETY off, and always while upright)
- `/safety_quadruped_cmd_vel_ui` (SAFETY on): nav2's assisted_teleop checks it against the local
  costmap first; `/teleop_obstacle` (Bool) shows "OBSTACLE AHEAD" and holds driving until every
  key / stick is released. `/goaway_quadruped_controller/safety_ready` false = nav2 isn't up, the
  amber banner says so (turn SAFETY off to drive). SAFETY starts ON on every load and unpause.

## Mapping page (footer `🗺️ Mapping`)

Lichtblick with the live map (`mapping_layout.json`) | the MAPPING card above the quadruped
widget (drag the bar between them). The map store is tmms's, `~/.htxgrrt/maps` (C2 reads `png/`).
- **Session (top, as tmms's 3D maps):** `＋ NEW MAP` opens the name box (today's `YYMMDD_`) and
  `● START`; while mapping, `✎ EDIT & SAVE` (`/mapping_manager/start_mapping` / `stop_mapping`).
  Stopping saves `png/<name>.png` + yaml (and an untouched copy + the pose graph in `slam/`), then
  opens the map editor on it.
- **Maps (always shown):** every `png/` map with ✎ Edit, ⬇ Download (zip of the pair), ✕ Delete
  (also its `slam/` files), and ⬆ Upload of a `.png` + `.yaml` pair.
- **Map editor** (from tmms): pencil, eraser, line, rect, circle, crop, undo; Save overwrites
  `png/<name>`, Save as writes a new name.

ui_backend serves the files (`/api/maps2d`, tmms's routes), `GOAWAY_MAPS_DIR` (default
`~/.htxgrrt/maps`); the goaway_ui compose mounts the store read-write. Without the mount every
map route answers 503 "map store … not found (not mounted?)" and the card shows it.

## Navplan tester (`/navplan_tester`, footer `🧭 Navplan`)

tmms's standalone page (its own Vite entry, `navplan_tester.html`), without POIs and 3D maps. The
map canvas shows a `png/` map with the live `/utlidar/scan` and the robot (AMCL pose).
- **Map + RELOCALIZE:** pick a map, place the pose pin, drag its heading ring, then Send pose:
  `/relocalize` (localization_manager) loads the map if needed and converges AMCL.
- **Waypoints** (sidebar): click the map to add, click a pin for its type (simple, home, map
  switch). A map switch is a pair, one pin per map: where it stops, then where it relocalizes.
  Kept in this browser. **Send navplan** shows the plan, then `/navigation_plan`
  (navplan_processor) checks it and goaway_yasmin drives it, segment by segment.
- **Robot card:** state, map, localization, plan + segment; RELOCALIZE, ▶ UNPAUSE / ⏸ PAUSE,
  ✕ CANCEL (`/cancel_goal`).

Unpause (nav drives) is refused unless the robot already walks (WALK in HTX, balance stand in
Unitree), never upright or while mapping; the controller pauses it again if it stops walking
(a fall, the E-stop). While nav drives, the dashboard and Mapping page show "Teleop disabled"
with ⏸ Pause Robot.

Lichtblick's pose estimate and goal go through the same nodes (`/lichtblick_initialpose`,
`/lichtblick_navgoal` in both layouts).

## Run
```bash
npm install
npm run backend    # ui_backend.js on :3001 (/api/health, /api/maps2d)
npm run dev        # Vite on :5173, proxies /api to the backend
```
Open `http://<this machine>:5173`.

**Sim2Sim:** the simulator publishes `/lowstate`, not `/lf/lowstate`. Open
`http://<this machine>:5173/?lowstate=/lowstate` to see the joint temps.

## Build
```bash
npm run lint
npm run build                              # -> dist/
NODE_ENV=production node ui_backend.js     # serves dist/ and /api on :3001
```
