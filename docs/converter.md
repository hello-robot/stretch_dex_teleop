# Converting to LeRobot Dataset Format

Detail doc for `convert_to_lerobot.py`. 

To train imitation learning models using Hugging Face's [LeRobot](https://github.com/huggingface/lerobot) framework, the raw trajectory CSV and images must be converted into a `LeRobotDataset` (images become compressed MP4, states become Parquet tables).

**Run this on your development machine, not the robot** — it's the only piece of this pipeline that imports LeRobot/PyTorch.

## Prerequisites

Developed and validated against `lerobot==0.6.0`. LeRobot's dataset API has changed across versions before (its import path itself changed during this project), so an unpinned `lerobot` may not work without changes to this script:
```bash
pip install lerobot==0.6.0
```
*(To push directly to the Hugging Face Hub, run `hf auth login` first.)*

## Running the Conversion

```bash
python3 convert_to_lerobot.py --episode-dir data/episode_2026-07-13--14-22-05 --repo-id "<repo-id>"
```

By default this creates a brand-new local dataset at `~/.cache/huggingface/lerobot/<repo-id>`. Running it again against the same `--repo-id` without `--append` fails with a clear error rather than overwriting or corrupting anything — see **Multi-Episode Datasets** below.

> **Note:** you may notice an empty `images/` folder alongside `videos/` in the converted dataset. It's harmless internal scaffolding from LeRobot's own pipeline (a staging area it uses for non-streaming video encoding), not something our script creates or needs. All actual frame data lives in `videos/`. Safe to ignore or delete.

Set the task description recorded with each frame using `--task` (defaults to `"teleoperation_task"`):
```bash
python convert_to_lerobot.py --episode-dir data/episode_2026-07-13--14-22-05 --repo-id "<repo-id>" --task "pick_up_the_mug"
```

## Success/Failure Safety Check

Before converting, the script checks `{episode-dir}/success.txt` (see [recorder.md](recorder.md#successfailure-labeling)) — this is a safety check, not a batch filter:
- No file → episode is unlabeled (e.g. recorded with `--skip-success`, or from before this feature existed) — proceeds normally.
- Contains `"Success"` → proceeds normally.
- Contains `"Failure"` → **blocked by default**, clear error, exit code 1 — protects against accidentally training on a known-bad episode. Pass `--allow-failed` to convert it anyway (e.g. debugging, or using it as a negative example).
- Anything else → treated as an error, not silently guessed.

This is per-episode, not automatic batch filtering — there's no glob/multi-directory mode, so you're always converting one episode at a time.

## Wrist and Head Camera Support

If the episode's `trajectory.csv` has wrist and/or head camera images (`image_wrist_cam`/`image_head_cam` non-empty), the converter automatically adds the corresponding video feature(s) — `observation.images.wrist_cam` (640×480) and/or `observation.images.head_cam` (640×480, rotated) — alongside the existing `observation.image` (1920×1080, the external webcam), following LeRobot's own multi-camera naming convention (`observation.images.<camera_name>`). Episodes recorded before a given camera existed still convert exactly as before, with no corresponding feature. Nothing to pass for this — detected automatically per episode, per camera.

## Automatic Frame Rate Measurement

LeRobot requires a single `fps` value per dataset, used to timestamp every frame as `frame_index / fps`. Rather than assuming the webcam's nominal 30 fps target, the converter measures the *actual* recording rate from `trajectory.csv`'s `timestamp` column (`(num_frames - 1) / (last_timestamp - first_timestamp)`, rounded to the nearest integer). It prints the measured rate every run, and warns if it deviates more than 15% from the nominal 30 fps — in practice this deviation is usually large and explained by dropped frames (see [recorder.md](recorder.md#architecture)), not the camera/control loop actually running slowly.

Automatic, no flag needed — there's currently no way to override the measured value from the command line.

## Multi-Episode Datasets

Pass `--append` to add an episode to a dataset that already exists at `--repo-id`:
```bash
python convert_to_lerobot.py --episode-dir data/episode_2026-07-15--13-47-54 --repo-id "<repo-id>" --append

```
- Without `--append`, converting into an existing `--repo-id` always fails with a clear error (protects against mixing episodes into the wrong dataset via a typo'd or reused repo-id).
- With `--append` against a `--repo-id` that does **not** exist yet, it also fails with a clear error — drop `--append` to create it first.
- **A dataset's `fps` is fixed forever by whichever episode created it.** Every appended episode's frames are timestamped using that original fps, not its own freshly measured rate. If an appended episode's measured rate differs meaningfully, the script prints a note — but still proceeds, since there's no other option within LeRobot's dataset format.

## Pushing to Hugging Face Hub

Requires a Hugging Face account and a write-access token — run `hf auth login` once beforehand.

```bash
python3 convert_to_lerobot.py --episode-dir data/episode_2026-07-13--14-22-05 --repo-id "<your-hf-username/stretch_dex_teleop>" --push
```
Works the same whether you just created the dataset or appended to it. `--repo-id` must start with your actual HF username (or an org you belong to), or the push fails with a permission error.

**By default this pushes the dataset as public** — visible and downloadable by anyone. Pass `--private` to push it as a private dataset instead:
```bash
python3 convert_to_lerobot.py --episode-dir data/episode_2026-07-13--14-22-05 --repo-id "your-hf-username/stretch_dex_teleop" --push --private
```
Worth deciding deliberately rather than by default, especially since recorded episodes include real camera footage of your workspace.

**`dataset.finalize()` is always called before pushing** — without it, `meta/episodes/*.parquet` never gets written to disk (it's only flushed on `finalize()`, or as a fallback safety net when the Python process eventually exits), and a dataset pushed without it can't be loaded back. See [data_recording.md](data_recording.md#3-pushing-to-hugging-face-hub) for how to verify a push actually worked, and how to visualize a pushed (or local-only) dataset.

