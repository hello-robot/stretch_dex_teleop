# Data Recorder — Status & Checklist

**Legend**: `[x]` done, `[ ]` not started. `(robot)` = touches robot-side code, `(dev)` = dev-machine-side.

---

## Required Fixes

- [x] (robot) **`goal_from_teleop.py` broken tuple-unpack** — `process_next_frame()` now returns `(markers, color_image)`; the one caller that wasn't updated now is. Verified on real hardware.
- [ ] (robot) **`base_x_joint`/`base_y_joint` are copies of measured state, not real commands** — the robot never gets base-translation commands, so these columns are just echoed odometry, not real actions. **Deliberately not fixed yet** — revisit before training on this data. Plan: drop them from the ML feature set in the converter, keep raw odometry (`base_x`/`base_y`) in the CSV for diagnostics.
- [x] (dev) **Converter crashed with a raw traceback on re-run** — `LeRobotDataset.create()` raised `FileExistsError` against an existing `--repo-id`. Now caught, prints a clear message + next steps.

---

## Recommended Improvements

- [x] (dev) **fps/timestamp handling** — LeRobot only uses the `fps` passed to `create()`, ignores real per-frame timestamps. Hardcoded `fps=30` was ~82% wrong on real data. Now measures actual fps from `trajectory.csv` timestamps and uses that instead. Still one averaged value per dataset (a LeRobot format limit, not fixable here).
- [x] (dev) **Multi-episode support** — added `--append`, using `LeRobotDataset.resume()`. A dataset's `fps` is locked at first creation; appended episodes are timestamped using that original fps regardless of their own measured rate (LeRobot limitation, script prints a note when this happens).
- [ ] (dev) **`robot_type` left as `null`** — deliberate, not an oversight. Only matters for LeRobot's own aggregation tooling and multi-embodiment policy training, neither of which this pipeline uses yet. Revisit if combining Stretch 2/3 data or training a policy that should condition on robot identity.
- [x] (robot) **Image write validation** — `cv2.imwrite()`'s return value (and exceptions) are now checked; a failed write drops that frame from `trajectory.csv` instead of leaving a dangling reference or silently crashing the writer thread.
- [x] **Repo hygiene** — removed a stray committed `trajectory.csv` from tracking (not a full history rewrite — still recoverable from old commits if ever needed).

---

## Success/Failure Labeling

Added after comparing against `stretch_ai`'s recorder. Decision: worth having for real (filtering bad demos before training), but never auto-delete failed episodes — keep everything on disk, filter only at conversion time.

- [x] (robot) **Recording-side**: `y`/`n` prompt after stopping a non-empty recording, writes `success.txt` (`"Success"`/`"Failure"`). `--skip-success` disables it. Works identically whether you stop via `r` or quit via `q` mid-recording. Verified on real hardware across multiple episodes.
- [x] (dev) **Conversion-side**: `check_success_label()` blocks converting a `"Failure"`-labeled episode by default (`--allow-failed` to override); unlabeled episodes proceed normally. Verified against real labeled hardware episodes, including the override path.
