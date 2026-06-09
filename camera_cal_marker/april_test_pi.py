"""
april_test_pi.py -- Live AprilTag detection with 6DOF pose on Raspberry Pi.

Family   : tag36h11
Calib    : camera_calibration_pi.yaml
TAG_SIZE : 0.05 m (5 cm)

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
import pupil_apriltags as apriltag
import yaml
from picamera2 import Picamera2

TAG_SIZE     = 0.05
CALIB_FILE   = "camera_calibration_pi.yaml"
CAM_W, CAM_H = 640, 480
JPEG_QUALITY = 80

FONT        = cv2.FONT_HERSHEY_SIMPLEX
FILL_CLR    = (0, 170, 255)
BORDER_CLR  = (0, 220, 60)
CORNER_CLRS = [
    (0,   60, 255),
    (255, 80,   0),
    (0,  220,  60),
    (255,200,   0),
]
CENTER_CLR  = (255, 255, 255)
ID_CLR      = (255, 230, 0)
FPS_CLR     = (255, 200, 0)
BLACK       = (0, 0, 0)
FILL_ALPHA  = 0.30
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


def _draw_tag(vis, tag, K, D):
    pts = tag.corners.astype(np.int32)

    overlay = vis.copy()
    cv2.fillPoly(overlay, [pts], FILL_CLR)
    cv2.addWeighted(overlay, FILL_ALPHA, vis, 1.0 - FILL_ALPHA, 0, vis)

    cv2.polylines(vis, [pts], isClosed=True,
                  color=BORDER_CLR, thickness=2, lineType=cv2.LINE_AA)

    for idx, pt in enumerate(pts):
        c = tuple(pt)
        cv2.circle(vis, c, 7, BLACK, -1)
        cv2.circle(vis, c, 6, CORNER_CLRS[idx], -1)

    cx, cy = int(tag.center[0]), int(tag.center[1])
    cv2.circle(vis, (cx, cy), 5, BLACK, -1)
    cv2.circle(vis, (cx, cy), 4, CENTER_CLR, -1)

    if tag.pose_R is not None and tag.pose_t is not None:
        rvec, _ = cv2.Rodrigues(tag.pose_R)
        tvec    = tag.pose_t
        cv2.drawFrameAxes(vis, K, D, rvec, tvec, TAG_SIZE * 0.5)
        dist_m     = float(np.linalg.norm(tvec))
        dist_label = f"{dist_m * 100:.1f} cm"
        tw, _ = cv2.getTextSize(dist_label, FONT, 0.6, 2)[0]
        _put_outlined(vis, dist_label, (cx - tw // 2, cy + 22),
                      (200, 200, 255), scale=0.6, thickness=2)

    label = f"ID:{tag.tag_id}"
    tw, _ = cv2.getTextSize(label, FONT, 0.8, 2)[0]
    _put_outlined(vis, label, (cx - tw // 2, cy - 18), ID_CLR, scale=0.8, thickness=2)


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    K, D = _load_calibration(CALIB_FILE)
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    print(f"Calibration loaded: fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}")

    detector = apriltag.Detector(
        families="tag36h11",
        nthreads=1,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
    )

    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration(
        main={"format": "RGB888", "size": (CAM_W, CAM_H)}))
    picam2.start()
    time.sleep(0.5)

    _start_mjpeg_server()
    print(f"AprilTag live detector (tag36h11, tag_size={TAG_SIZE*100:.0f} cm)"
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

            gray       = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=[fx, fy, cx, cy],
                tag_size=TAG_SIZE,
            )

            vis = frame.copy()
            for tag in detections:
                _draw_tag(vis, tag, K, D)

            h, w = vis.shape[:2]
            _put_outlined(vis, f"Tags: {len(detections)}", (10, 28),      FPS_CLR, scale=0.7)
            _put_outlined(vis, f"{fps:.1f} FPS",           (w - 110, 28), FPS_CLR, scale=0.7)
            _put_outlined(vis, f"tag36h11  tag_size={TAG_SIZE*100:.0f} cm",
                          (10, h - 12), (180, 180, 180), scale=0.5, thickness=1)

            ok, buf = cv2.imencode('.jpg', vis, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            if ok:
                _set_frame(buf.tobytes())

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        picam2.stop()


if __name__ == "__main__":
    main()
