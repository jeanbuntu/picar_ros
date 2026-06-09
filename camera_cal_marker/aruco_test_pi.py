"""
aruco_test_pi.py -- Live ArUco detection with 6DOF pose on Raspberry Pi.

Dictionary : DICT_4X4_50
Calib      : camera_calibration_pi.yaml
TAG_SIZE   : 0.05 m (5 cm)

Stream: http://<pi-ip>:8080/
Stop  : Ctrl+C
"""

import collections
import http.server
import socketserver
import threading
import time

import cv2
import numpy as np
import yaml
from picamera2 import Picamera2

TAG_SIZE     = 0.05
CALIB_FILE   = "camera_calibration_pi.yaml"
CAM_W, CAM_H = 640, 480
JPEG_QUALITY = 80

FONT        = cv2.FONT_HERSHEY_SIMPLEX
BORDER_CLR  = (0, 220, 80)
ID_CLR      = (0, 200, 255)
FPS_CLR     = (255, 200, 0)
BLACK       = (0, 0, 0)
FPS_BUF_LEN = 30

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
    print(f"MJPEG stream: http://<pi-ip>:{port}/")


# ── Helpers ───────────────────────────────────────────────────────────────────
def _load_calibration(path):
    with open(path) as f:
        d = yaml.safe_load(f)
    K = np.array(d["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    D = np.array(d["distortion_coefficients"]["data"], dtype=np.float64)
    return K, D


def _put_outlined(img, text, xy, color, scale=0.65, thickness=2):
    cv2.putText(img, text, xy, FONT, scale, BLACK, thickness + 2, cv2.LINE_AA)
    cv2.putText(img, text, xy, FONT, scale, color, thickness,     cv2.LINE_AA)


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    K, D = _load_calibration(CALIB_FILE)
    print(f"Calibration loaded: fx={K[0,0]:.1f} fy={K[1,1]:.1f} "
          f"cx={K[0,2]:.1f} cy={K[1,2]:.1f}")

    aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    detector     = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    half = TAG_SIZE / 2.0
    obj_pts = np.array([
        [-half,  half, 0],
        [ half,  half, 0],
        [ half, -half, 0],
        [-half, -half, 0],
    ], dtype=np.float64)

    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration(
        main={"format": "RGB888", "size": (CAM_W, CAM_H)}))
    picam2.start()
    time.sleep(0.5)

    _start_mjpeg_server()
    print(f"ArUco live detector (DICT_4X4_50, tag_size={TAG_SIZE*100:.0f} cm)"
          " -- Ctrl+C to quit")

    fps_buf = collections.deque(maxlen=FPS_BUF_LEN)
    t_last  = time.perf_counter()

    try:
        while True:
            raw   = picam2.capture_array()
            frame = cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)

            t_now = time.perf_counter()
            fps_buf.append(1.0 / max(t_now - t_last, 1e-9))
            t_last = t_now
            fps = sum(fps_buf) / len(fps_buf)

            corners, ids, _ = detector.detectMarkers(frame)
            vis = frame.copy()

            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(vis, corners, ids, borderColor=BORDER_CLR)

                for i, corner_set in enumerate(corners):
                    pts = corner_set[0]

                    c0 = (int(pts[0, 0]), int(pts[0, 1]))
                    cv2.circle(vis, c0, 7, (0, 60, 255), -1)
                    cv2.circle(vis, c0, 7, BLACK, 1)

                    img_pts = pts.reshape(4, 1, 2).astype(np.float64)
                    ok, rvec, tvec = cv2.solvePnP(
                        obj_pts, img_pts, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)

                    cx_i = int(pts[:, 0].mean())
                    cy_i = int(pts[:, 1].mean())

                    if ok:
                        cv2.drawFrameAxes(vis, K, D, rvec, tvec, TAG_SIZE * 0.5)
                        dist_m     = float(np.linalg.norm(tvec))
                        dist_label = f"{dist_m * 100:.1f} cm"
                        tw, _      = cv2.getTextSize(dist_label, FONT, 0.6, 2)[0]
                        _put_outlined(vis, dist_label,
                                      (cx_i - tw // 2, cy_i + 22),
                                      (200, 200, 255), scale=0.6, thickness=2)

                    _put_outlined(vis, f"ID:{int(ids[i][0])}",
                                  (cx_i - 28, cy_i - 18),
                                  ID_CLR, scale=0.75, thickness=2)

            h, w = vis.shape[:2]
            n = 0 if ids is None else len(ids)
            _put_outlined(vis, f"Markers: {n}",  (10, 28),      FPS_CLR, scale=0.7)
            _put_outlined(vis, f"{fps:.1f} FPS", (w - 110, 28), FPS_CLR, scale=0.7)
            _put_outlined(vis, f"DICT_4X4_50  tag_size={TAG_SIZE*100:.0f} cm",
                          (10, h - 12), (180, 180, 180), scale=0.5, thickness=1)

            ok2, buf = cv2.imencode('.jpg', vis, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            if ok2:
                _set_frame(buf.tobytes())

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        picam2.stop()


if __name__ == "__main__":
    main()
