# stretch_ai vs. stretch_dex_teleop Data Pipeline — Comparison & Recommendations



**Context**: this project's whole premise (see `data_recorder_status.md` / original review) was replacing `stretch_ai`'s data collection because it had teleoperation reliability problems (wrist oscillations, unreliable recording) on the exact same hardware that worked fine with standalone `stretch_dex_teleop`. The decision to move to standalone `stretch_dex_teleop` + a lightweight custom recorder was about **teleoperation reliability**, not about the data/sensing architecture. This comparison is checking whether anything about `stretch_ai`'s data design  is still worth carrying over.


---

## 1. Where our implementation is already better — do not regress toward theirs

| Area | `stretch_ai` | Ours |
|---|---|---|
| Frame timing | Fixed fps=6 assumed, never measured | Real fps measured from timestamps |
| Gripper units | Raw Dynamixel ticks | Converted to meters via calibration |
| Image write robustness | No equivalent | Failed writes detected, frame dropped cleanly |
| Re-running the converter | Not compared | Existing-dataset handling, `--append`, `success.txt` check all built and tested |

---

## 2. Ideas from `stretch_ai` worth adopting

### 2a. Task/user/environment metadata at record time
They capture `user_name`/`task_name`/`env_name`/git info per episode at record time. We only have a `--task` string typed in at conversion, days later, nothing to cross-check it against. Real gap — worth closing before scaling up real task collection.

### 2b. Git commit provenance
Cheap to capture (`git rev-parse HEAD` at `start_episode()`). Bundle with 2a.

### 2c. Episode-level provenance in the LeRobot dataset
Once 2a/2b exist, folding that into the converted dataset's own metadata is a small follow-on. Not urgent standalone.

---

## 3. Ideas from `stretch_ai` explicitly NOT recommended

### 3a. Synthetic `progress` field in `action`
Fabricated (linearly interpolated, never measured), no concrete consumer on our side. Same reasoning that killed our own `--fps` override idea — don't add a fake signal without a real consumer.

### 3b. Fixed-fps timing assumption
Already covered in §1 — do not adopt.

### 3c. Raw Dynamixel gripper units
Already covered in §1 — do not adopt.

### 3d. Their documented converter bug
Their depth columns get marked as video features when they're not. Not something to copy — just a reminder reference code isn't automatically correct code.

### 3e. Waypoint labeling
Considered and deferred separately — nothing in our pipeline would consume it yet.

---

## 4. Camera and sensing architecture

`stretch_ai` captures onboard head + wrist cameras, both RGB-D, plus computed 3D poses. We now capture three feeds too — the external webcam, wrist (D405), and head (D435i) — but the mix is different in two ways:

- The external webcam only sees the operator's tongs, not the task or scene. It's useful for debugging (checking markers tracked cleanly, diagnosing dropped frames) but not useful for training — a deployed policy has no such camera and no tongs to look at.
- Our new implementation has no depth data at all — color only, across all three cameras. `stretch_ai` records RGB-D on both of its cameras.

**Resolved**: wrist camera and head camera, keep it lightweight, no ROS2. External webcam keeps running regardless (drives teleop control); kept saving it too, for the debugging value above, not as a training observation.



