# Data Recorder — Project Status & Checklist

Living status document for the `feature/data-recorder` branch (lightweight recorder + `convert_to_lerobot.py`). Originates from a full read-only technical review of this branch vs. `main`; updated as items are fixed and verified.

**How to read this file**: each item is `[x]` done, `[~]` in progress / partially done, or `[ ]` not started. Every done item lists what was changed and, importantly, *how it was verified* — since some fixes touch robot-side code that can't be exercised on this (local, no-`stretch_body`) machine.

**Testing legend**:
- 🤖 **Robot-side** — touches `dex_teleop.py`, `recorder.py`, `gripper_to_goal.py`, `goal_from_teleop.py`, `webcam_teleop_interface.py`. Can only be statically/logically verified here (no camera/robot hardware on this machine); needs a real smoke test on the robot before being fully trusted.
- 💻 **Local-machine** — touches `convert_to_lerobot.py` / `data_recording.md`. Can be tested for real here against the installed `lerobot==0.6.0`.

---

## Required Fixes

Things that were actively broken or silently produced misleading data.

- [x] **🤖 Fix broken tuple-unpack in `goal_from_teleop.py:240`** — ✅ fully verified, including on real hardware
  `webcam_teleop_interface.py`'s `process_next_frame()` now returns `(markers, color_image)` instead of just `markers`, but `goal_from_teleop.py`'s own `__main__` block wasn't updated to match, and would raise on unpack.
  **Change**: `markers = ...` → `markers, color_image = ...`
  **Verified (static, this machine)**: `py_compile` passes; grepped all 3 callers of `process_next_frame()` in the repo, all now consistently unpack `(markers, image)`; isolated stub test confirms the unpack no longer raises `ValueError`.
  **Verified (real hardware, `stretch-3012`, 2026-07-15)**: you applied the identical one-line change directly on the robot and ran `python3 goal_from_teleop.py` — 11 loop iterations completed cleanly with no `ValueError`, only a `KeyboardInterrupt` from the intentional `Ctrl+C` shutdown. Confirmed fixed on real hardware.
  **Side-finding from this test**: this loop (webcam + ArUco detection only, no IK/robot commands/recording) averaged **~14 Hz**, well under the 30 Hz assumed by the webcam config and hardcoded in `convert_to_lerobot.py`'s `fps=30`. The full `dex_teleop.py` recording loop does strictly more per-iteration work (IK, robot commands, `get_status()`, image queuing, display), so it's unlikely to hold 30 Hz either — this is real supporting evidence for Recommended #4 below, not just a theoretical concern.
  **Outstanding housekeeping**: the fix currently exists as two independent, unsynced manual edits — one in this machine's uncommitted working tree, one hand-edited directly on `stretch-3012`. These need to be reconciled via a real commit/push/pull rather than staying as two copies that happen to match.

- [ ] **🤖 `base_x_joint` / `base_y_joint` are copies of measured state, not real commands** — **decided: not fixing right now**
  The robot never receives a base-translation command (only rotation, via `joint_mobile_base_rotate_by`), so `recorder.py:106-107` currently just echoes `measured_state['base']['x'/'y']` into the "commanded action" columns — and duplicates values already present in `base_x`/`base_y` two columns over, so they carry zero unique information (unlike `base_theta_joint`, which is a genuine derived target and stays). Confirmed bit-identical in the recorded test CSV.
  **Agreed plan for when we revisit**: keep `base_x`/`base_y` (real odometry telemetry, cheap, diagnostic value e.g. detecting base slippage during a forceful grasp) and `base_theta_joint` (genuinely derived) in `recorder.py`'s raw CSV; remove `base_x_joint`/`base_y_joint` from `recorder.py` entirely (pure duplicates, misleading name); make both `observation.state` and `action` **7-dim** in `convert_to_lerobot.py` (`base_theta, lift, arm, wrist_roll, wrist_pitch, wrist_yaw, gripper_width_m`), dropping `base_x`/`base_y` from the ML feature set even though they remain in the raw CSV.
  **Explicitly deferred by you** — revisit before recording data you intend to actually train on.

- [x] **💻 `convert_to_lerobot.py` crashes with raw traceback on re-run**
  `LeRobotDataset.create()` internally does `root.mkdir(..., exist_ok=False)`, so re-running the converter against a `--repo-id` that already has a dataset raised an unhandled `FileExistsError`.
  **Change**: wrapped `LeRobotDataset.create(...)` in `try/except FileExistsError`, printing the resolved dataset path and a clear next-step message, then `sys.exit(1)` — no more raw traceback. (Chose the **minimal** fix — clear error message only, not auto-resume/append. Multi-episode support via `.resume()` is still open, see Recommended #5 below.)
  **Verified for real** against the installed `lerobot==0.6.0` and your actual test episode:
  - Re-running against the existing `local/stretch-dex-teleop-test` repo-id → clean 4-line message, exit code 1, no traceback.
  - Regression check: converting to a brand-new repo-id still works end-to-end (140 frames, 1 episode, loaded back and verified via `LeRobotDataset(...)`). Scratch dataset created for this test was deleted afterward.

---

## Recommended Improvements

Not broken today, but real gaps worth closing before this becomes your standard recording pipeline.

- [ ] **💻 fps/timestamp handling** — LeRobot 0.6.0 assigns `timestamp = frame_index / fps` using the hardcoded `fps=30` in `convert_to_lerobot.py:47`; your recorder's real per-frame wall-clock `timestamp` column is never read by the converter (LeRobot forbids passing a custom timestamp into `add_frame` at all). If the teleop loop doesn't hold a true 30 Hz, dataset timing is quietly wrong. Proposed: compute actual mean fps from `trajectory.csv` timestamps and pass that to `LeRobotDataset.create(fps=...)`, warn if it deviates from nominal.
- [ ] **💻 Multi-episode support via `LeRobotDataset.resume()`** — currently every conversion run creates a brand-new dataset (`.create()` only); LeRobot 0.6.0 exposes `.resume()` specifically for appending episodes, unused today. Needed if you want one dataset built from several recorded episodes.
- [ ] **💻 Set `robot_type` explicitly** — currently `null` in `info.json` since `convert_to_lerobot.py` never passes it to `LeRobotDataset.create()`.
- [ ] **🤖 Remove (or wire up) the dead `fps` parameter in `EpisodeRecorder.__init__`** (`recorder.py:11`) — accepted but never read anywhere in the class; misleading as-is.
- [ ] **💻 Pin the LeRobot version** in `data_recording.md` (currently says `pip install lerobot` with no version) and/or add a small `requirements-lerobot.txt` documenting the local-machine env (`lerobot==0.6.0`, `torch`, `PIL`). The import path already broke once (commit `64ed5db`) due to an unpinned version drift.
- [ ] **🤖 Validate image writes** — `cv2.imwrite()`'s return value is discarded in the writer thread (`recorder.py:130`); a silent write failure (disk full, bad path) only surfaces later as a crash in `convert_to_lerobot.py` on a different machine. Proposed: check the return value and/or verify `images/` file count matches `len(self.records)` at `stop_episode()`.
- [ ] **Repo hygiene: remove committed `data/episode_2026-07-13--14-22-05/trajectory.csv` from git history** — tracked before `.gitignore`'s `data/` rule was added; the images it references were never committed, so it's already orphaned for anyone cloning fresh. This is a decision for you (may involve history rewrite) — not touched yet.

---

## Optional Future Work

Polish items, lower priority.

- [ ] **💻 Dynamically infer image shape** in `convert_to_lerobot.py` from the first frame instead of hardcoding `(3, 1080, 1920)`.
- [ ] **🤖 Replace plain-`bool` `is_recording`/`_stop_writer` flags with `threading.Event`** in `recorder.py` — not a bug today (correct under CPython's GIL), just more explicit.
- [ ] **💻 Expose `--fps` as a converter CLI argument** once the fps/timestamp item above is addressed.
- [ ] **🤖 Bound the image write queue** (`recorder.py:23`, currently unbounded `queue.Queue()`) — only matters for long episodes where disk-write throughput can't keep up with capture rate. No evidence this has happened yet — precautionary.

---

## Notes on Working Session

- Both `[x]` fixes above are now **committed locally, not yet pushed** to `origin/feature/data-recorder`:
  - `55db7d8` — Fix broken markers unpack in `goal_from_teleop.py`
  - `7e445e8` — Handle existing dataset directory in `convert_to_lerobot.py`
- The `stretch-3012` hand-edit of `goal_from_teleop.py` (used to verify commit `55db7d8` on real hardware) is still a standalone manual edit there, not connected to git. Once `55db7d8` is pushed, that robot should `git pull` (or otherwise sync) to replace the manual edit with the tracked version.
- Full original review (executive summary, file-by-file walkthrough, git diff analysis, data-flow diagram, threading analysis) is preserved separately as an artifact: https://claude.ai/code/artifact/11897b5d-5a69-4e82-b7f9-c0d81a63de27
