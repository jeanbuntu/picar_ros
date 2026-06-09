"""
charuco_calibrate.py -- Camera calibration using the CharUco board.

Board parameters (from charuco_calibration_sheet.pdf):
    Squares  : 7 x 5
    Square   : 30 mm
    Marker   : 22 mm
    Dict     : DICT_4X4_50

Controls:
    SPACE   capture current frame (board must be visible)
    c       run calibration with frames captured so far (min 15)
    q       quit without calibrating

Output:
    camera_calibration.yaml  (camera_matrix + dist_coeffs)
"""

import cv2
import numpy as np
import yaml

SQUARES_X     = 7
SQUARES_Y     = 5
SQUARE_LEN    = 0.030   # metres
MARKER_LEN    = 0.022   # metres
MIN_FRAMES    = 15
TARGET_FRAMES = 20
OUTPUT_FILE   = "camera_calibration.yaml"

FONT = cv2.FONT_HERSHEY_SIMPLEX


def _put_outlined(img, text, xy, color, scale=0.65, thickness=2):
    cv2.putText(img, text, xy, FONT, scale, (0, 0, 0), thickness + 2, cv2.LINE_AA)
    cv2.putText(img, text, xy, FONT, scale, color,     thickness,     cv2.LINE_AA)


def _flash_green(img):
    """Briefly tint the display green to confirm a capture."""
    overlay = np.full_like(img, (0, 200, 80))
    cv2.addWeighted(img, 0.65, overlay, 0.35, 0, img)


def main():
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board = cv2.aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y),
        SQUARE_LEN,
        MARKER_LEN,
        aruco_dict,
    )
    detector = cv2.aruco.CharucoDetector(board)

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera 0")

    all_corners = []
    all_ids     = []
    img_size    = None

    print(f"Hold the CharUco board at varied angles and distances.")
    print(f"Target: {TARGET_FRAMES} frames (minimum {MIN_FRAMES}).")
    print("SPACE = capture  |  c = calibrate  |  q = quit\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Frame grab failed.")
            break

        charuco_corners, charuco_ids, _, _ = detector.detectBoard(frame)
        detected = charuco_ids is not None and len(charuco_ids) >= 6

        vis = frame.copy()

        if detected:
            cv2.aruco.drawDetectedCornersCharuco(
                vis, charuco_corners, charuco_ids, (0, 220, 80))
            status      = f"Board detected ({len(charuco_ids)} corners)"
            status_clr  = (0, 220, 80)
        else:
            status      = "No board detected -- keep board fully in frame"
            status_clr  = (0, 100, 255)

        h, w = vis.shape[:2]
        n = len(all_corners)
        bar_pct = min(n / TARGET_FRAMES, 1.0)
        cv2.rectangle(vis, (10, h - 30), (w - 10, h - 14), (60, 60, 60), -1)
        cv2.rectangle(vis, (10, h - 30), (10 + int((w - 20) * bar_pct), h - 14),
                      (0, 200, 80), -1)

        _put_outlined(vis, status,                  (10, 28),      status_clr,    scale=0.65)
        _put_outlined(vis, f"Captured: {n}/{TARGET_FRAMES}",
                      (10, 58), (255, 200, 0), scale=0.65)
        _put_outlined(vis, "[SPACE] capture  [c] calibrate  [q] quit",
                      (10, h - 38), (180, 180, 180), scale=0.45, thickness=1)

        cv2.imshow("CharUco Calibration", vis)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break

        elif key == ord(' '):
            if not detected:
                print("  Board not clearly visible -- skipped.")
            else:
                all_corners.append(charuco_corners)
                all_ids.append(charuco_ids)
                if img_size is None:
                    img_size = (frame.shape[1], frame.shape[0])
                print(f"  Frame {len(all_corners)} captured ({len(charuco_ids)} corners)")
                _flash_green(vis)
                cv2.imshow("CharUco Calibration", vis)
                cv2.waitKey(250)

        elif key == ord('c'):
            if len(all_corners) < MIN_FRAMES:
                print(f"  Need at least {MIN_FRAMES} frames (have {len(all_corners)}) -- keep capturing.")
            else:
                break

    cap.release()
    cv2.destroyAllWindows()

    if len(all_corners) < MIN_FRAMES:
        print(f"\nOnly {len(all_corners)} frames captured -- calibration aborted.")
        return

    # ── Calibration ──────────────────────────────────────────────────────────
    print(f"\nCalibrating with {len(all_corners)} frames ...")

    obj_pts_list = []
    img_pts_list = []
    for corners, ids in zip(all_corners, all_ids):
        obj_pts, img_pts = board.matchImagePoints(corners, ids)
        if obj_pts is not None and len(obj_pts) >= 4:
            obj_pts_list.append(obj_pts)
            img_pts_list.append(img_pts)

    rms, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(
        obj_pts_list, img_pts_list, img_size, None, None
    )

    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]

    print(f"\nRMS reprojection error : {rms:.4f} px  (good < 1.0)")
    print(f"fx={fx:.2f}  fy={fy:.2f}  cx={cx:.2f}  cy={cy:.2f}")
    print(f"Distortion : {dist_coeffs.ravel()}")

    if rms > 1.0:
        print("\nWARNING: RMS > 1.0 px -- try recapturing with more varied tilt/distance.")

    # ── Save ─────────────────────────────────────────────────────────────────
    data = {
        "image_width":  img_size[0],
        "image_height": img_size[1],
        "rms_reprojection_error": float(rms),
        "camera_matrix": {
            "rows": 3, "cols": 3,
            "data": camera_matrix.ravel().tolist(),
        },
        "distortion_coefficients": {
            "rows": 1,
            "cols": int(dist_coeffs.size),
            "data": dist_coeffs.ravel().tolist(),
        },
    }

    with open(OUTPUT_FILE, "w") as f:
        yaml.dump(data, f, default_flow_style=False)

    print(f"\nSaved to {OUTPUT_FILE}")


if __name__ == "__main__":
    main()
