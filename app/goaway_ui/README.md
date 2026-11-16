# goaway_ui

Dashboard for the goaway Go2 stack. Same design as tmms_ui: React + Vite + Tailwind, talking
to ROS through rosbridge (`ws://<page host>:9090`, started by `goaway_master operation.launch.py`).

One page: Lichtblick on the left (60%, drag the divider), quadruped control on the right.
- **Lichtblick:** `goaway_lichtblick` on `:8080` (see `app/goaway_lichtblick`), auto-connected to
  rosbridge, with `dashboard_layout.json`. The robot model comes from `go2_visual.urdf`.
- **Top:** gear shift (SIT, STAND LOCK, WALK), the `UNITREE | HTX` switch (only in SIT), and in
  HTX the skill cards: RECOVERY (always first), DONKEY KICK, QUICK BUCK, and SIDE FLIP (one card,
  `◀ LEFT` | `RIGHT ▶` = the robot's own side; its clip alternates the two flips).
- **Middle:** the SPEED slider (LOW, MID, HIGH; `<` / `>` keys) and driving: joystick, rotation
  knob, or the keyboard (arrows, ⇧+←→ to turn).
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
- `/quadruped_cmd_vel_ui`

## Run
```bash
npm install
npm run backend    # ui_backend.js on :3001 (/api/health for the header's ping)
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
