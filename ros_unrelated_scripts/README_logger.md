# Scrub Nurse Logger

Keyboard-based CLI logger for robotic scrub nurse handover tests.

## Run

```bash
cd "/Users/daniel/Library/CloudStorage/OneDrive-HochschuleLuzern/Master/5. Semester (Thesis)/Code/robotic_scrub_nurse/ros_unrelated_scripts"
python3 scrub_nurse_logger.py
```

Optional custom data directory override:

```bash
python3 scrub_nurse_logger.py --data-dir ./data
```

Default output directory (without `--data-dir`) is script-relative:

- `ros_unrelated_scripts/data`

## Key Mapping

- `0`: Select success
- `1`: Select gesture detection failure
- `2`: Select premature release
- `3`: Select non-release
- `4`: Select collision
- `5`: Select other anomaly
- `6`: Select timeout/no response
- `Enter`: Confirm selected event (`0-6`)
- `c`: Cancel selected event
- `q`: Quit and save session

## Output

A CSV file is created per session.
Filename format:

- `YYYYMMDD_<participant_slug>_s<session_number>.csv`
- If filename exists already, logger appends `_v2`, `_v3`, ...

Startup prompts:

1. `Participant name`
2. `Session number` (required positive integer)

Logging flow:

1. Press `0-6` to select event type.
2. Press `Enter` to confirm (or `c` to cancel).
3. If `1-6`, add optional free-text failure comment.

For `0` (success), no extra comment prompt appears.

Columns:

1. `timestamp_iso`
2. `session_id`
3. `participant_name`
4. `session_number`
5. `event_index`
6. `outcome`
7. `failure_code`
8. `failure_label`
9. `failure_comment`

## Notes

- Single-key input requires a TTY terminal.
- Output uses ANSI colors by default (if terminal supports it).
- Disable colors via: `NO_COLOR=1 python3 scrub_nurse_logger.py`
- Use `Ctrl+C` to stop safely; the script still prints a summary.
