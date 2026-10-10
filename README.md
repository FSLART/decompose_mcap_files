# MCAP to CSV — Windows

Convert a ROS MCAP recording into **one CSV file**. Requires Python 3.10 or newer;
you do not need to install ROS. Supports ROS 1 `ros1msg` and ROS 2 `ros2msg` recordings.

## Setup

Install Python, then open PowerShell in the folder containing `decompose_mcap.py`
and `requirements.txt`. Run these commands once:

```powershell
py -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

If `py` is not recognized, use `python` in the first command instead.

## Convert a recording

```powershell
.\.venv\Scripts\python.exe decompose_mcap.py "recording.mcap"
```

Replace `recording.mcap` with your file name or full path. Keep the quotes for
paths containing spaces. The CSV is saved beside the recording as `recording.csv`.
Each row contains one message; all topics share the same file.

## Optional settings

Choose an output file:

```powershell
.\.venv\Scripts\python.exe decompose_mcap.py "recording.mcap" -o "output.csv"
```

Add these options to either command when needed:

- `--topic /imu/data` exports only that topic. Repeat `--topic` for more topics.
- `--overwrite` replaces an existing output CSV.

Large recordings, especially images or point clouds, can produce very large CSVs.
