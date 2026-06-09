"""
charuco_calibrate_pi.py -- CharUco camera calibration for Raspberry Pi.

Board parameters (from charuco_calibration_sheet.pdf):
    Squares  : 7 x 5
    Square   : 30 mm
    Marker   : 22 mm
    Dict     : DICT_4X4_50

No display: view the live stream in a browser at http://<pi-ip>:8080/

Terminal commands (press Enter after each):
    c   capture current frame (board must be detected)
    r   run calibration with frames captured so far (min 15)
    Ctrl+C  quit without calibrating

Output:
    camera_calibration_pi.yaml
"""

import collections
import http.server
import queue
import socketserver
import threading
import time

import cv2
import numpy as np
import yaml
from picamera2 import Picamera2

# ── Board / calibration constants ─────────────────────────────────────────────
SQUARES_X     = 7
SQUARES_Y     = 5
SQUARE_LEN    = 0.030
MARKER_LEN    = 0.022
MIN_FRAMES    = 15
TARGET_FRAMES = 20
OUTPUT_FILE   = "camera_calibration_pi.yaml"

CAM_W, CAM_H = 640, 480
JPEG_QUALITY = 80
FONT = cv2.FONT_HERSHEY_SIMPLEX

# ── MJPEG server ──────────────────────────────────────────────────────────────
_frame_lock  = threading.Lock()
_frame_bytes = b''


def _set_frame(data: bytes) -> None:
    global _frame_bytes
    with _frame_lock:
        _frame_bytes = data


class _MJPEGHandler(http.server.BaseHTTPRequestHandler):
    def log_message(self, *args): pass

    def do_GET(self):
        if self.path != '/':
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Content-Type',
                         'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        try:
            while True:
                with _frame_lock:
                    data = _frame_bytes
                if data:
                    self.wfile.write(
                        b'--frame\r\n'
                        b'Content-Type: image/jpeg\r\n'
                        b'Content-Length: ' + str(len(data)).encode() + b'\r\n'
                        b'\r\n' + data + b'\r\n'
                    )
                time.sleep(0.033)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass


class _ReuseAddrServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True


def _start_mjpeg_server(port: int = 8080):
    server = _ReuseAddrServer(('', port), _MJPEGHandler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"MJPEG stream: http://<pi-ip>:{port}/  (open in browser)")


# ── Drawing helpers ───────────────────────────────────────────────────────────
def _put_outlined(img, text, xy, color, scale=0.65, thickness=2):
    cv2.putText(img, text, xy, FONT, scale, (0, 0, 0), thickness + 2, cv2.LINE_AA)
    cv2.putText(img, text, xy, FONT, scale, color,     thickness,     cv2.LINE_AA)


def _flash_green(img):
    overlay = np.full_like(img, (0, 200, 80))
    cv2.addWeighted(img, 0.65, overlay, 0.35, 0, img)


# ── Input thread ──────────────────────────────────────────────────────────────
_cmd_queue: queue.Queue = queue.Queue()


def _input_loop():
    print("Terminal commands (press Enter after each):")
    print("  c = capture frame | r = run calibration | Ctrl+C = quit\n")
    while True:
        try:
            line = input().strip().lower()
            if line in ('c', 'r'):
                _cmd_queue.put(line)
            elif line:
                print(f"  Unknown command '{line}' -- use 'c' or 'r'")
        except EOFError:
            break


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board = cv2.aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y), SQUARE_LEN, MARKER_LEN, aruco_dict)
    detector = cv2.aruco.CharucoDetector(board)

    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration(
        main={"format": "RGB888", "size": (CAM_W, CAM_H)}))
    picam2.start()
    time.sleep(0.5)

    _start_mjpeg_server()
    threading.Thread(target=_input_loop, daemon=True).start()

    all_corners  = []
    all_ids      = []
    img_size     = None
    do_calibrate = False

    print(f"Hold the CharUco board at varied angles and distances.")
    print(f"Target: {TARGET_FRAMES} frames (minimum {MIN_FRAMES}).\n")

    try:
        while True:
            raw   = picam2.capture_array()
            frame = cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)

            charuco_corners, charuco_ids, _, _ = detector.detectBoard(frame)
            detected = charuco_ids is not None and len(charuco_ids) >= 6

            vis = frame.copy()

            if detected:
                cv2.aruco.drawDetectedCornersCharuco(
                    vis, charuco_corners, charuco_ids, (0, 220, 80))
                status     = f"Board detected ({len(charuco_ids)} corners)"
                status_clr = (0, 220, 80)
            else:
                status     = "No board detected -- keep board fully in frame"
                status_clr = (0, 100, 255)

            h, w = vis.shape[:2]
            n       = len(all_corners)
            bar_pct = min(n / TARGET_FRAMES, 1.0)
            cv2.rectangle(vis, (10, h - 30), (w - 10, h - 14), (60, 60, 60), -1)
            cv2.rectangle(vis, (10, h - 30),
                          (10 + int((w - 20) * bar_pct), h - 14), (0, 200, 80), -1)

            _put_outlined(vis, status,                           (10, 28), status_clr,    scale=0.65)
            _put_outlined(vis, f"Captured: {n}/{TARGET_FRAMES}", (10, 58), (255, 200, 0), scale=0.65)
            _put_outlined(vis, "[c] capture  [r] calibrate  (terminal)",
                          (10, h - 38), (180, 180, 180), scale=0.45, thickness=1)

            try:
                cmd = _cmd_queue.get_nowait()
            except queue.Empty:
                cmd = None

            if cmd == 'c':
                if not detected:
                    print("  Board not clearly visible -- skipped.")
                else:
                    all_corners.append(charuco_corners)
                    all_ids.append(charuco_ids)
                    if img_size is None:
                        img_size = (frame.shape[1], frame.shape[0])
                    print(f"  Frame {len(all_corners)} captured ({len(charuco_ids)} corners)")
                    _flash_green(vis)
                    ok, buf = cv2.imencode('.jpg', vis,
                                          [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                    if ok:
                        _set_frame(buf.tobytes())
                    time.sleep(0.25)
                    continue

            elif cmd == 'r':
                if len(all_corners) < MIN_FRAMES:
                    print(f"  Need at least {MIN_FRAMES} frames "
                          f"(have {len(all_corners)}) -- keep capturing.")
                else:
                    do_calibrate = True
                    break

            ok, buf = cv2.imencode('.jpg', vis, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            if ok:
                _set_frame(buf.tobytes())

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        picam2.stop()

    if not do_calibrate or len(all_corners) < MIN_FRAMES:
        print(f"\nOnly {len(all_corners)} frames captured -- calibration aborted.")
        return

    print(f"\nCalibrating with {len(all_corners)} frames ...")

    obj_pts_list = []
    img_pts_list = []
    for corners, ids in zip(all_corners, all_ids):
        obj_pts, img_pts = board.matchImagePoints(corners, ids)
        if obj_pts is not None and len(obj_pts) >= 4:
            obj_pts_list.append(obj_pts)
            img_pts_list.append(img_pts)

    rms, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(
        obj_pts_list, img_pts_list, img_size, None, None)

    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]

    print(f"\nRMS reprojection error : {rms:.4f} px  (good < 1.0)")
    print(f"fx={fx:.2f}  fy={fy:.2f}  cx={cx:.2f}  cy={cy:.2f}")
    print(f"Distortion : {dist_coeffs.ravel()}")

    if rms > 1.0:
        print("\nWARNING: RMS > 1.0 px -- recapture with more varied tilt/distance.")

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
