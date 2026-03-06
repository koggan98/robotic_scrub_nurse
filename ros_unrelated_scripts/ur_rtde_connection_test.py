#!/usr/bin/env python3
"""
Simple UR RTDE connectivity and slow-motion test.

Usage examples:
  1) Fast check common IPs (no motion):
     python3 ros_unrelated_scripts/ur_rtde_connection_test.py --scan-common

  2) Scan subnet for RTDE-capable robot (no motion):
     python3 ros_unrelated_scripts/ur_rtde_connection_test.py --subnet 192.168.12.0/24

  3) Scan subnet, then do a very small slow motion test:
     python3 ros_unrelated_scripts/ur_rtde_connection_test.py --subnet 192.168.12.0/24 --move

  4) Directly test known IP and move:
     python3 ros_unrelated_scripts/ur_rtde_connection_test.py --ip 192.168.12.10 --move
"""

import argparse
import ipaddress
import socket
import sys
import time
from typing import List, Optional, Tuple

try:
    from rtde_control import RTDEControlInterface as RTDEControl
    from rtde_receive import RTDEReceiveInterface as RTDEReceive
except ImportError:
    RTDEControl = None
    RTDEReceive = None


COMMON_UR_IPS = [
    "192.168.12.10",
    "192.168.1.168",
    "192.168.56.101",
]


def is_port_open(ip: str, port: int, timeout_s: float) -> bool:
    try:
        with socket.create_connection((ip, port), timeout=timeout_s):
            return True
    except OSError:
        return False


def query_dashboard_polyscope(ip: str, timeout_s: float) -> Optional[str]:
    """Returns PolyScope version string if dashboard is reachable, else None."""
    try:
        with socket.create_connection((ip, 29999), timeout=timeout_s) as sock:
            sock.settimeout(timeout_s)
            # Read greeting if present.
            try:
                _ = sock.recv(256)
            except OSError:
                pass

            sock.sendall(b"PolyscopeVersion\n")
            reply = sock.recv(512).decode("utf-8", errors="ignore").strip()
            if reply:
                return reply
            return None
    except OSError:
        return None


def rtde_receive_check(ip: str) -> Tuple[bool, str]:
    """Try connecting to RTDE receive and reading actual joints."""
    if RTDEReceive is None:
        return False, "ur_rtde not installed"

    rtde_r = None
    try:
        rtde_r = RTDEReceive(ip)
        q = rtde_r.getActualQ()
        if not isinstance(q, list) or len(q) != 6:
            return False, "RTDE connected but getActualQ returned invalid data"
        return True, "RTDE receive ok"
    except Exception as exc:
        return False, f"RTDE receive failed: {exc}"
    finally:
        if rtde_r is not None:
            disconnect = getattr(rtde_r, "disconnect", None)
            if disconnect is not None:
                try:
                    disconnect()
                except Exception:
                    pass


def candidate_ips_from_subnet(subnet: str, start_host: int, end_host: int) -> List[str]:
    network = ipaddress.ip_network(subnet, strict=False)
    candidates = []

    for host in network.hosts():
        last_octet = int(str(host).split(".")[-1])
        if start_host <= last_octet <= end_host:
            candidates.append(str(host))

    return candidates


def find_robot_ips(
    ips: List[str], timeout_s: float, print_prefix: str = ""
) -> List[str]:
    found = []
    for ip in ips:
        p30004 = is_port_open(ip, 30004, timeout_s)
        p29999 = is_port_open(ip, 29999, timeout_s)

        if not p30004 and not p29999:
            continue

        dashboard = query_dashboard_polyscope(ip, timeout_s)
        rtde_ok, rtde_msg = rtde_receive_check(ip)

        print(
            f"{print_prefix}{ip}: ports(30004={p30004},29999={p29999}), "
            f"dashboard='{dashboard if dashboard else 'n/a'}', {rtde_msg}"
        )

        if rtde_ok:
            found.append(ip)

    return found


def choose_target_joint(original_q: List[float], joint_index: int, delta_rad: float) -> List[float]:
    q_target = list(original_q)
    q_target[joint_index] += delta_rad
    return q_target


def try_safe_joint_target(control: RTDEControl, original_q: List[float], joint_index: int, delta_rad: float) -> Optional[List[float]]:
    """Try +delta first, then -delta if safety check rejects +delta."""
    safety_fn = getattr(control, "isJointsWithinSafetyLimits", None)

    plus_q = choose_target_joint(original_q, joint_index, delta_rad)
    if safety_fn is None:
        return plus_q

    try:
        if bool(safety_fn(plus_q)):
            return plus_q
    except Exception:
        pass

    minus_q = choose_target_joint(original_q, joint_index, -delta_rad)
    try:
        if bool(safety_fn(minus_q)):
            return minus_q
    except Exception:
        pass

    return None


def run_slow_motion_test(
    ip: str,
    speed: float,
    acceleration: float,
    delta_deg: float,
    joint_index: int,
    dwell_s: float,
    go_home: bool,
    home_joints: List[float],
) -> int:
    if RTDEControl is None or RTDEReceive is None:
        print("ERROR: ur_rtde is not installed. Run: python3 -m pip install --user ur_rtde")
        return 2

    if joint_index < 0 or joint_index > 5:
        print("ERROR: --joint-index must be between 0 and 5")
        return 2

    delta_rad = delta_deg * 3.141592653589793 / 180.0
    if delta_rad <= 0.0:
        print("ERROR: --delta-deg must be > 0")
        return 2

    rtde_c = None
    rtde_r = None
    try:
        print(f"Connecting RTDE to {ip} ...")
        rtde_c = RTDEControl(ip)
        rtde_r = RTDEReceive(ip)

        q0 = rtde_r.getActualQ()
        if not isinstance(q0, list) or len(q0) != 6:
            print("ERROR: Could not read valid current joints from robot.")
            return 1

        print(f"Current joints: {[round(v, 4) for v in q0]}")

        if go_home:
            if len(home_joints) != 6:
                print("ERROR: --go-home requested but --home-joints does not contain 6 values.")
                return 2
            print(f"Moving to configured home joints first: {[round(v, 4) for v in home_joints]}")
            ok_home = bool(rtde_c.moveJ(home_joints, speed, acceleration))
            if not ok_home:
                print("ERROR: moveJ to home joints failed.")
                return 1
            q0 = rtde_r.getActualQ()
            if not isinstance(q0, list) or len(q0) != 6:
                print("ERROR: Could not read valid joints after home move.")
                return 1

        q_target = try_safe_joint_target(rtde_c, q0, joint_index, delta_rad)
        if q_target is None:
            print("ERROR: No safety-valid target for the requested mini motion.")
            return 1

        print(
            "Running slow mini-motion: "
            f"joint[{joint_index}] shift, speed={speed}, acc={acceleration}, dwell={dwell_s}s"
        )

        ok1 = bool(rtde_c.moveJ(q_target, speed, acceleration))
        if not ok1:
            print("ERROR: moveJ to target failed.")
            return 1

        time.sleep(max(0.0, dwell_s))

        ok2 = bool(rtde_c.moveJ(q0, speed, acceleration))
        if not ok2:
            print("ERROR: moveJ back to start failed.")
            return 1

        print("Motion test finished successfully.")
        return 0

    except Exception as exc:
        print(f"ERROR: motion test failed: {exc}")
        print("Hint: robot likely not in Remote Control / not running / network mismatch.")
        return 1
    finally:
        if rtde_c is not None:
            stop_script = getattr(rtde_c, "stopScript", None)
            if stop_script is not None:
                try:
                    stop_script()
                except Exception:
                    pass
        if rtde_r is not None:
            disconnect = getattr(rtde_r, "disconnect", None)
            if disconnect is not None:
                try:
                    disconnect()
                except Exception:
                    pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="UR RTDE connection + slow motion test")
    parser.add_argument("--ip", default="", help="Direct robot IP (skip subnet scan)")
    parser.add_argument(
        "--subnet",
        default="192.168.12.0/24",
        help="Subnet for scan when --ip is not set (default: 192.168.12.0/24)",
    )
    parser.add_argument("--start-host", type=int, default=1, help="Start host index for subnet scan")
    parser.add_argument("--end-host", type=int, default=254, help="End host index for subnet scan")
    parser.add_argument(
        "--scan-common",
        action="store_true",
        help="Try common UR IPs first (192.168.12.10, 192.168.1.168, 192.168.56.101)",
    )
    parser.add_argument(
        "--scan-timeout",
        type=float,
        default=0.20,
        help="TCP timeout per probe in seconds (default: 0.20)",
    )

    parser.add_argument(
        "--move",
        action="store_true",
        help="Perform slow mini motion after connection check",
    )
    parser.add_argument("--speed", type=float, default=0.08, help="Joint speed for test move")
    parser.add_argument("--acc", type=float, default=0.08, help="Joint acceleration for test move")
    parser.add_argument(
        "--delta-deg",
        type=float,
        default=2.0,
        help="Mini motion amplitude in degrees on one joint",
    )
    parser.add_argument(
        "--joint-index",
        type=int,
        default=5,
        help="Joint index for mini move (0..5), default 5 (wrist_3)",
    )
    parser.add_argument(
        "--dwell",
        type=float,
        default=0.5,
        help="Hold time at target in seconds",
    )
    parser.add_argument(
        "--go-home",
        action="store_true",
        help="Move to home joints before mini motion test",
    )
    parser.add_argument(
        "--home-joints",
        type=float,
        nargs=6,
        default=[0.0, -2.486, 1.227, -1.294, -1.5707963267948966, 0.0],
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="Home joint target (rad), default matches tracking config",
    )

    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if RTDEControl is None or RTDEReceive is None:
        print("ERROR: ur_rtde module not found.")
        print("Install with: python3 -m pip install --user ur_rtde")
        return 2

    found_ips: List[str] = []

    if args.ip:
        print(f"Checking provided IP: {args.ip}")
        found_ips = find_robot_ips([args.ip], args.scan_timeout)
    else:
        if args.scan_common:
            print("Checking common UR IPs first...")
            found_ips.extend(find_robot_ips(COMMON_UR_IPS, args.scan_timeout, print_prefix="  "))

        if not found_ips:
            print(
                f"Scanning subnet {args.subnet} hosts {args.start_host}-{args.end_host} "
                f"(timeout {args.scan_timeout}s)..."
            )
            candidates = candidate_ips_from_subnet(args.subnet, args.start_host, args.end_host)
            found_ips = find_robot_ips(candidates, args.scan_timeout, print_prefix="  ")

    if not found_ips:
        print("No RTDE-reachable UR found.")
        print("Check network cable, PC subnet, and Teach Pendant mode (Remote Control).")
        return 1

    print("\nRTDE-reachable candidates:")
    for ip in found_ips:
        print(f"  - {ip}")

    selected_ip = found_ips[0]
    print(f"\nSelected IP for next step: {selected_ip}")

    if not args.move:
        print("Connection check done. Re-run with --move for slow motion test.")
        return 0

    return run_slow_motion_test(
        ip=selected_ip,
        speed=args.speed,
        acceleration=args.acc,
        delta_deg=args.delta_deg,
        joint_index=args.joint_index,
        dwell_s=args.dwell,
        go_home=args.go_home,
        home_joints=args.home_joints,
    )


if __name__ == "__main__":
    sys.exit(main())
