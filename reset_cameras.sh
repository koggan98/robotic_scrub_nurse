#!/bin/bash
# Reset Intel RealSense cameras after a failed launch (Ctrl+C).
#
# Phase 1: Hub reset (sudo, NOPASSWD via /etc/sudoers.d/realsense-reset)
#   Resets USB root hubs usb2 and usb4 so vanished cameras re-enumerate.
# Phase 2: ioctl reset for cameras visible in lsusb
#   Sends USBDEVFS_RESET to each RealSense device node.
#
# One-time setup (run once after a fresh install):
#   sudo bash -c 'cat > /usr/local/bin/realsense-hub-reset << "EOF"
#   #!/bin/bash
#   for hub in usb2 usb4; do
#       auth="/sys/bus/usb/devices/$hub/authorized"
#       [ -f "$auth" ] || continue
#       echo 0 > "$auth"; sleep 0.4; echo 1 > "$auth"; echo "Hub $hub reset"
#   done
#   sleep 2
#   EOF
#   chmod +x /usr/local/bin/realsense-hub-reset'
#   sudo bash -c 'echo "daniel ALL=(root) NOPASSWD: /usr/local/bin/realsense-hub-reset" > /etc/sudoers.d/realsense-reset && chmod 440 /etc/sudoers.d/realsense-reset'
#   sudo usermod -aG plugdev daniel && newgrp plugdev

set -e

# ── Phase 1: Hub reset ─────────────────────────────────────────────────────
if command -v /usr/local/bin/realsense-hub-reset &>/dev/null; then
    echo "Phase 1: resetting USB hubs..."
    sudo /usr/local/bin/realsense-hub-reset
else
    echo "Phase 1: skipped (hub-reset not installed, see comments above)"
fi

# ── Phase 2: ioctl reset for each camera visible in lsusb ─────────────────
echo "Phase 2: ioctl reset for visible cameras..."
python3 - <<'PYEOF'
import fcntl, os, subprocess, re, time, sys

USBDEVFS_RESET = 0x5514

result = subprocess.run(['lsusb'], capture_output=True, text=True)
cameras = []
for line in result.stdout.split('\n'):
    if '8086:0b5c' in line:
        m = re.search(r'Bus (\d+) Device (\d+)', line)
        if m:
            cameras.append((m.group(1), m.group(2)))

if not cameras:
    print("  No RealSense cameras visible in lsusb after hub reset.")
    print("  Try unplugging and replugging the cameras.")
    sys.exit(0)

for bus, dev in cameras:
    dev_path = f'/dev/bus/usb/{bus.zfill(3)}/{dev.zfill(3)}'
    try:
        fd = os.open(dev_path, os.O_WRONLY)
        fcntl.ioctl(fd, USBDEVFS_RESET, 0)
        os.close(fd)
        print(f"  Reset OK: {dev_path}")
    except PermissionError:
        print(f"  Permission denied: {dev_path}")
        print("  Run: sudo usermod -aG plugdev daniel && newgrp plugdev")
        sys.exit(1)
    except OSError as e:
        print(f"  Reset error ({dev_path}): {e}")

print(f"  Waiting 3s for firmware re-init...")
time.sleep(3)
PYEOF

echo "Done. Run: rs-enumerate-devices 2>/dev/null | grep 'Serial Number'"
