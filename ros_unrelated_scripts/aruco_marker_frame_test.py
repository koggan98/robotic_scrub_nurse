#!/usr/bin/env python3
"""Minimal RealSense + ArUco marker frame overlay test.

Usage:
    python3 ros_unrelated_scripts/aruco_marker_frame_test.py
"""

from __future__ import annotations

import sys
from typing import Optional

try:
    import cv2
except ImportError:
    cv2 = None

try:
    import numpy as np
except ImportError:
    np = None

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None


WINDOW_NAME = "ArUco Marker Frame Test"

# Change this if you want to test another OpenCV ArUco dictionary.
ARUCO_DICT_ID = cv2.aruco.DICT_ARUCO_ORIGINAL if cv2 is not None else None

# IMPORTANT: Replace this with the real physical marker size in meters.
MARKER_SIZE_METERS = 0.10


def check_dependencies() -> int:
    missing = []
    if cv2 is None:
        missing.append("opencv-python")
    if np is None:
        missing.append("numpy")
    if rs is None:
        missing.append("pyrealsense2")

    if missing:
        print("ERROR: Missing required Python package(s): " + ", ".join(missing))
        print(
            "Install example: python3 -m pip install --user opencv-python pyrealsense2 numpy"
        )
        return 2

    if not hasattr(cv2, "aruco"):
        print("ERROR: OpenCV ArUco module is unavailable. Install opencv-contrib-python.")
        print("Install example: python3 -m pip install --user opencv-contrib-python")
        return 2

    return 0


def create_pipeline() -> tuple[Optional["rs.pipeline"], Optional["rs.pipeline_profile"], int]:
    context = rs.context()
    if len(context.devices) == 0:
        print("ERROR: No Intel RealSense camera detected.")
        return None, None, 1

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    try:
        profile = pipeline.start(config)
    except Exception as exc:
        print(f"ERROR: Failed to start RealSense pipeline: {exc}")
        return None, None, 1

    return pipeline, profile, 0


def intrinsics_to_cv(
    profile: "rs.pipeline_profile",
) -> tuple[Optional["np.ndarray"], Optional["np.ndarray"]]:
    try:
        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intrinsics = color_profile.get_intrinsics()
    except Exception as exc:
        print(f"ERROR: Failed to read RealSense intrinsics: {exc}")
        return None, None

    camera_matrix = np.array(
        [
            [intrinsics.fx, 0.0, intrinsics.ppx],
            [0.0, intrinsics.fy, intrinsics.ppy],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    dist_coeffs = np.array(intrinsics.coeffs, dtype=np.float64)
    return camera_matrix, dist_coeffs


def create_marker_object_points(marker_size_m: float) -> "np.ndarray":
    half = marker_size_m / 2.0
    return np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float32,
    )


def main() -> int:
    dep_status = check_dependencies()
    if dep_status != 0:
        return dep_status

    pipeline, profile, pipeline_status = create_pipeline()
    if pipeline_status != 0:
        return pipeline_status

    camera_matrix, dist_coeffs = intrinsics_to_cv(profile)
    if camera_matrix is None or dist_coeffs is None:
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception:
                pass
        return 1

    marker_object_points = create_marker_object_points(MARKER_SIZE_METERS)

    device_name = "Unknown RealSense"
    if profile is not None:
        try:
            device_name = profile.get_device().get_info(rs.camera_info.name)
        except Exception:
            pass

    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
    detector_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)

    print(f"Using camera: {device_name}")
    print(f"Using marker size: {MARKER_SIZE_METERS} m")
    print("Showing detected original ArUco marker IDs and axes. Press 'q' to quit.")

    try:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        while True:
            try:
                frames = pipeline.wait_for_frames()
            except Exception as exc:
                print(f"ERROR: Failed while reading RealSense frames: {exc}")
                return 1

            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            image = np.asanyarray(color_frame.get_data())
            corners, ids, _ = detector.detectMarkers(image)

            annotated = image.copy()
            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
                ids_flat = ids.flatten().tolist()

                for marker_corners, marker_id in zip(corners, ids_flat):
                    corner_points = marker_corners.reshape(-1, 2).astype(int)
                    top_left = tuple(corner_points[0])
                    cv2.putText(
                        annotated,
                        f"ID {marker_id}",
                        (top_left[0], max(20, top_left[1] - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )

                    success, rvec, tvec = cv2.solvePnP(
                        marker_object_points,
                        marker_corners.reshape(4, 2),
                        camera_matrix,
                        dist_coeffs,
                    )
                    if success:
                        cv2.drawFrameAxes(
                            annotated,
                            camera_matrix,
                            dist_coeffs,
                            rvec,
                            tvec,
                            MARKER_SIZE_METERS * 0.75,
                            2,
                        )

            cv2.imshow(WINDOW_NAME, annotated)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
    finally:
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception:
                pass
        if cv2 is not None:
            cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    sys.exit(main())
