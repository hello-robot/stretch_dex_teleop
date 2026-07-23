# Stretch Dex Teleop Data Collection

A lightweight data recorder for `stretch_dex_teleop`. It captures synchronized robot telemetry, commanded teleoperation actions, and camera imagery (external webcam, wrist and head cameras), and converts it into a [LeRobot](https://github.com/huggingface/lerobot) dataset for imitation learning.

This pipeline spans **two separate machines**:

| Machine | Role | Script | Depends on LeRobot/PyTorch? |
|---|---|---|---|
| **Stretch robot** | Records raw episodes | `dex_teleop.py` | No |
| **Your dev machine** | Converts raw episodes to LeRobot format | `convert_to_lerobot.py` | Yes |


---

## 1. Recording an episode (Stretch Robot)

Run everything below **on the robot itself**, over SSH or on its own desktop.

### 1.1 One-time setup

Follow **[Setting Up Dex Teleop](../README.md#setting-up-dex-teleop)** in the main README — installing dependencies, generating specialized URDFs, and the physical camera/ring light/stand/tongs setup are all covered there, not repeated here.

### 1.2 Every time you record

```bash
cd ~/stretch_dex_teleop
python3 dex_teleop.py
```

This will open the **Dex Teleop** GUI:
| Key | Action |
|---|---|
| `r` | Start / stop recording an episode |
| `y` / `n` | Answer "Was the episode successful?" (only asked after stopping a non-empty recording, unless `--skip-success`) |
| `q` | Quit (safely stops/labels an in-progress recording first) |

Episodes land in `data/episode_YYYY-MM-DD--HH-MM-SS/` on the robot. Copy that directory
to your dev machine (`scp -r`, `rsync`, USB drive, etc.) before converting.

---

## 2. Converting to LeRobot format (Dev machine side)

Run everything below **on your development machine**, not the robot.

### 2.1 One-time setup

Use a **dedicated virtualenv** for this — `lerobot`'s dataset API has changed across
versions before, so keeping it isolated and pinned matters.

```bash
python3 -m venv ~/.venv-lerobot
source ~/.venv-lerobot/bin/activate
pip install lerobot==0.6.0   
```

If you want to push datasets to the Hugging Face Hub:
```bash
hf auth login
```

You only need `convert_to_lerobot.py` from this repo on the dev machine (plus the copied
episode directories) — nothing else here has any LeRobot/robot-hardware dependency.

### 2.2 Every time you convert


```bash
# Activate the venv first
source ~/.venv-lerobot/bin/activate

# Create a brand-new dataset from the first episode:
python convert_to_lerobot.py \
  --episode-dir data/episode_2026-07-21--11-06-36 \
  --repo-id "<repo-id>" \
  --task "pick_up_the_mug"

# Add more episodes to that same dataset:
python convert_to_lerobot.py \
  --episode-dir data/episode_2026-07-21--11-30-02 \
  --repo-id "<repo-id>" \
  --append

# Push to the Hugging Face Hub once you're done:
python convert_to_lerobot.py \
  --episode-dir data/episode_2026-07-21--11-06-36 \
  --repo-id "<repo-id>" \
  --push
```

`<repo-id>` is a user-defined name. It is used as the output folder name for the converted dataset.

By default this writes to `~/.cache/huggingface/lerobot/<repo-id>`.
An episode marked `success.txt: Failure` is blocked by default — pass `--allow-failed`
to convert it anyway. Full flag reference and behavior notes are in
[converter.md](converter.md).

---

## 3. Pushing to Hugging Face Hub

Pushing is optional — everything above already gives you a fully working local dataset. Push only if you want to back it up, share it, or use Hub-hosted training tools.

### 3.1 One-time setup

```bash
source ~/.venv-lerobot/bin/activate
hf auth login
```
Paste a Hugging Face access token with **write** permission (generate one at `huggingface.co/settings/tokens`). This stays cached, so you only need to do it once per machine.

### 3.2 Pushing

Add `--push` to a conversion command (works whether you're creating a new dataset or appending to one):
```bash
python convert_to_lerobot.py --episode-dir data/episode_2026-07-21--11-06-36 --repo-id "<your-hf-username/stretch_dex_teleop>" --push
```

**By default this pushes as public** — visible and downloadable by anyone. Pass `--private` to keep it private instead:
```bash
python convert_to_lerobot.py --episode-dir data/episode_2026-07-21--11-06-36 --repo-id "your-hf-username/stretch_dex_teleop" --push --private
```
Worth deciding deliberately rather than by default — recorded episodes include real camera footage of your workspace. `--repo-id` must start with your actual HF username (or an org you belong to), or the push fails with a permission error.

### 3.3 Verifying a push worked

Quick check: visit `https://huggingface.co/datasets/<repo-id>` in your browser (log in first if private).

More thorough check — list the exact files that made it to the Hub:
```bash
python3 -c "
from huggingface_hub import HfApi
api = HfApi()
for f in sorted(api.list_repo_files('<repo-id>', repo_type='dataset')):
    print(f)
"
```
You should see `meta/info.json`, `meta/stats.json`, `meta/tasks.parquet`, `meta/episodes/chunk-000/file-000.parquet`, `data/chunk-000/file-000.parquet`, and one `videos/<camera>/chunk-000/file-000.mp4` per camera. If `meta/episodes/...` is missing, the dataset can't be loaded back — this exact failure mode is why `convert_to_lerobot.py` calls `dataset.finalize()` after every conversion (see [converter.md](converter.md)).

Most thorough check — actually load it back, fresh from the Hub (not your local cache):
```bash
python3 -c "
from lerobot.datasets.lerobot_dataset import LeRobotDataset
ds = LeRobotDataset(repo_id='<repo-id>', root='/tmp/hub_verify_test')
print('num frames:', len(ds))
print('num episodes:', ds.num_episodes)
"
rm -rf /tmp/hub_verify_test
```

### 3.4 Visualizing a dataset

The Hub's own "Visualize this dataset" button (on the dataset's webpage) uses HF's hosted dataset viewer, which is **PRO-gated for private datasets** — you'd need a paid HF PRO subscription to use it on a private dataset.

For a free alternative that works regardless of visibility, use LeRobot's local visualizer instead — it downloads/reads the dataset directly and renders it in a local viewer window, never going through the Hub's website:
```bash
pip install "lerobot[viz]"   # one-time; installs rerun-sdk + foxglove-sdk
lerobot-dataset-viz --repo-id <repo-id> --episode-index 0
```
This also works on datasets that were never pushed at all — pass `--root <local-path>` to point it at a local-only dataset instead of `--repo-id`.


<div align="center">
  <img src="../images/visualize_dataset_rerun.png" alt="base" width="400"/>
</div>

---

## 4. Filesystem layout

### Before conversion — one raw episode, as written by `recorder.py` (on the robot)

```
data/episode_2026-07-21--11-06-36/
├── trajectory.csv        # 64 rows: timestamp, measured state, commanded actions,
│                         # image_teleop_webcam + image_wrist_cam + image_head_cam paths
├── success.txt           # "Success" or "Failure" (omitted if --skip-success was used)
├── images/               # external ArUco webcam, one JPEG per recorded frame
│   ├── teleop_webcam_000000.jpg
│   ├── teleop_webcam_000001.jpg
│   ├── ...
│   └── teleop_webcam_000063.jpg
├── wrist_images/         # D405 wrist camera, only present if the camera was detected
│   ├── wrist_cam_000000.jpg
│   ├── wrist_cam_000001.jpg
│   ├── ...
│   └── wrist_cam_000063.jpg
└── head_images/          # D435i head camera, only present if the camera was detected
    ├── head_cam_000000.jpg
    ├── head_cam_000001.jpg
    ├── ...
    └── head_cam_000063.jpg
```


### After conversion — a LeRobotDataset, as written by `convert_to_lerobot.py` (dev machine)

```
~/.cache/huggingface/lerobot/<repo-id>/
├── meta/
│   ├── info.json                    # fps, features schema (dtypes/shapes/names), robot_type, etc.
│   ├── stats.json                   # per-feature dataset-wide statistics
│   ├── tasks.parquet                # task-index -> task-description text mapping
│   └── episodes/
│       └── chunk-000/
│           └── file-000.parquet     # per-episode metadata (length, task, etc.)
├── data/
│   └── chunk-000/
│       └── file-000.parquet         # observation.state / action / timestamp / indices
│                                     # (one row per frame, across all appended episodes)
└── videos/
    ├── observation.image/            # external webcam, AV1-encoded
    │   └── chunk-000/
    │       └── file-000.mp4          # 1920x1080
    ├── observation.images.wrist_cam/ # wrist camera, AV1-encoded (only if any
    │   └── chunk-000/                # converted episode had wrist camera data)
    │       └── file-000.mp4          # 640x480
    └── observation.images.head_cam/  # head camera, AV1-encoded (only if any
        └── chunk-000/                # converted episode had head camera data)
            └── file-000.mp4          # 640x480 (rotated, portrait orientation)
```


## 5. Learn More

- **[recorder.md](recorder.md)** — recorder architecture and the full `trajectory.csv` column reference.
- **[converter.md](converter.md)** — LeRobot conversion details: camera features, fps handling, multi-episode datasets, Hub push.
