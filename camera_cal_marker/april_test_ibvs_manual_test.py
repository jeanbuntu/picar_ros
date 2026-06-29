# pip install pupil-apriltags
"""
april_test_ibvs_manual_test.py -- Offline batch AprilTag processor.

Reads every JPEG from april_ibvs_debug/raw/, runs the detector N_RUNS times
per image (printing xyzrpyq + confidence each run), then saves annotated images
to april_ibvs_debug/postprocessed/ and writes report.md.

Annotation per image:
  - Image center: gray crosshair
  - Tag center:   cyan crosshair + error-vector line back to frame center
  - Bounding box, corner dots, 3D axes, distance label (existing)
  - TAG DETECTED / NOT DETECTED label bottom-right

Run from anywhere:
    python3 /path/to/april_test_ibvs_manual_test.py
"""

import os
import glob

import cv2
import numpy as np
import pupil_apriltags as apriltag
import yaml

# ── Config ────────────────────────────────────────────────────────────────────
_HERE        = os.path.dirname(os.path.abspath(__file__))
_CONFIG_PATH = os.path.join(_HERE, "config.yaml")

with open(_CONFIG_PATH) as _f:
    _cfg = yaml.safe_load(_f)

_det_cfg   = _cfg.get('detector', {})
_cam_cfg   = _cfg.get('camera', {})
_batch_cfg = _cfg.get('batch', {})

TAG_SIZE   = float(_cam_cfg.get('tag_size_m', 0.05))
CALIB_FILE = os.path.join(_HERE, _cam_cfg.get('calibration_file', 'camera_calibration.yaml'))
INPUT_DIR  = os.path.join(_HERE, _batch_cfg.get('input_dir',  'april_ibvs_debug/raw'))
OUTPUT_DIR = os.path.join(_HERE, _batch_cfg.get('output_dir', 'april_ibvs_debug/postprocessed'))
N_RUNS     = int(_batch_cfg.get('n_runs', 5))

# ── Drawing constants ─────────────────────────────────────────────────────────
FONT        = cv2.FONT_HERSHEY_SIMPLEX
FILL_CLR    = (0, 170, 255)
BORDER_CLR  = (0, 220, 60)
CORNER_CLRS = [
    (0,   60, 255),   # 0 bottom-left  -- red
    (255, 80,   0),   # 1 bottom-right -- blue
    (0,  220,  60),   # 2 top-right    -- green
    (255,200,   0),   # 3 top-left     -- cyan
]
CENTER_CLR      = (255, 255, 255)
TAG_CENTER_CLR  = (0, 255, 255)   # cyan crosshair on tag center
FRAME_CENTER_CLR = (180, 180, 180) # gray crosshair on image center
ERROR_VEC_CLR   = (0, 80, 255)    # red-ish line: frame center → tag center
ID_CLR          = (255, 230, 0)
BLACK           = (0, 0, 0)
FILL_ALPHA      = 0.30


# ── Calibration ───────────────────────────────────────────────────────────────

def _load_calibration(path):
    with open(path) as f:
        d = yaml.safe_load(f)
    K = np.array(d["camera_matrix"]["data"],
                 dtype=np.float64).reshape(3, 3)
    D = np.array(d["distortion_coefficients"]["data"],
                 dtype=np.float64)
    return K, D


# ── Drawing helpers ───────────────────────────────────────────────────────────

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

    tcx, tcy = int(tag.center[0]), int(tag.center[1])
    cv2.circle(vis, (tcx, tcy), 5, BLACK, -1)
    cv2.circle(vis, (tcx, tcy), 4, CENTER_CLR, -1)

    if tag.pose_R is not None and tag.pose_t is not None:
        rvec, _ = cv2.Rodrigues(tag.pose_R)
        tvec    = tag.pose_t
        cv2.drawFrameAxes(vis, K, D, rvec, tvec, TAG_SIZE * 0.5)
        dist_m = float(np.linalg.norm(tvec))
        dist_label = f"{dist_m * 100:.1f} cm"
        tw, _ = cv2.getTextSize(dist_label, FONT, 0.6, 2)[0]
        _put_outlined(vis, dist_label, (tcx - tw // 2, tcy + 22),
                      (200, 200, 255), scale=0.6, thickness=2)

    label = f"ID:{tag.tag_id}"
    tw, _ = cv2.getTextSize(label, FONT, 0.8, 2)[0]
    _put_outlined(vis, label, (tcx - tw // 2, tcy - 18), ID_CLR, scale=0.8, thickness=2)


def _draw_frame_center(vis):
    """Gray crosshair at the geometric centre of the image."""
    h, w = vis.shape[:2]
    mx, my = w // 2, h // 2
    cv2.line(vis, (mx - 25, my), (mx + 25, my), FRAME_CENTER_CLR, 1, cv2.LINE_AA)
    cv2.line(vis, (mx, my - 25), (mx, my + 25), FRAME_CENTER_CLR, 1, cv2.LINE_AA)
    cv2.circle(vis, (mx, my), 3, FRAME_CENTER_CLR, -1)


def _draw_tag_center_and_error(vis, tag):
    """Cyan crosshair on tag center + red error-vector line to frame center."""
    h, w = vis.shape[:2]
    mx, my = w // 2, h // 2
    tcx, tcy = int(tag.center[0]), int(tag.center[1])

    # Error-vector line: frame center → tag center
    cv2.line(vis, (mx, my), (tcx, tcy), ERROR_VEC_CLR, 1, cv2.LINE_AA)

    # Cyan crosshair on tag center
    cv2.line(vis, (tcx - 15, tcy), (tcx + 15, tcy), TAG_CENTER_CLR, 1, cv2.LINE_AA)
    cv2.line(vis, (tcx, tcy - 15), (tcx, tcy + 15), TAG_CENTER_CLR, 1, cv2.LINE_AA)
    cv2.circle(vis, (tcx, tcy), 4, TAG_CENTER_CLR, 1)


# ── Rotation conversion helpers ───────────────────────────────────────────────

def _rotation_to_euler_deg(R):
    """3x3 rotation matrix → [roll, pitch, yaw] in degrees (ZYX convention)."""
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        roll  = np.arctan2( R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw   = np.arctan2( R[1, 0], R[0, 0])
    else:
        roll  = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw   = 0.0
    return np.degrees([roll, pitch, yaw])


def _rotation_to_quaternion(R):
    """3x3 rotation matrix → [qw, qx, qy, qz] via Shepperd method."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s  = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s  = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s  = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s  = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return [qw, qx, qy, qz]


# ── Markdown report writer ────────────────────────────────────────────────────

def _write_report(rows, path, n_images):
    import csv
    n_detected = sum(1 for r in rows if r['status'] == 'DETECTED')
    fieldnames = [
        'image', 'run', 'status', 'tag_id', 'conf',
        'x_m', 'y_m', 'z_m',
        'roll_deg', 'pitch_deg', 'yaw_deg',
        'qw', 'qx', 'qy', 'qz',
        'eu_px', 'ev_px',
    ]
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in rows:
            w.writerow({
                'image':     r['image'],
                'run':       r['run'],
                'status':    r['status'],
                'tag_id':    r['tag_id'],
                'conf':      r['margin'],
                'x_m':       r['x'],
                'y_m':       r['y'],
                'z_m':       r['z'],
                'roll_deg':  r['roll'],
                'pitch_deg': r['pitch'],
                'yaw_deg':   r['yaw'],
                'qw':        r['qw'],
                'qx':        r['qx'],
                'qy':        r['qy'],
                'qz':        r['qz'],
                'eu_px':     r['eu_px'],
                'ev_px':     r['ev_px'],
            })
    print(f'\nReport written → {path}  ({n_detected}/{len(rows)} detections)')


# ── Main batch processor ──────────────────────────────────────────────────────

def process_images():
    K, D = _load_calibration(CALIB_FILE)
    fx, fy = K[0, 0], K[1, 1]
    cal_cx, cal_cy = K[0, 2], K[1, 2]
    print(f"Calibration loaded: fx={fx:.1f} fy={fy:.1f} cx={cal_cx:.1f} cy={cal_cy:.1f}")

    detector = apriltag.Detector(
        families="tag36h11",
        nthreads=int(_det_cfg.get('nthreads', 1)),
        quad_decimate=float(_det_cfg.get('quad_decimate', 1.0)),
        quad_sigma=float(_det_cfg.get('quad_sigma', 0.0)),
        refine_edges=int(_det_cfg.get('refine_edges', 1)),
        decode_sharpening=float(_det_cfg.get('decode_sharpening', 0.25)),
    )

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    paths = sorted(glob.glob(os.path.join(INPUT_DIR, "*.jpg")))
    if not paths:
        print(f"No JPEGs found in {INPUT_DIR}")
        return

    print(f"Processing {len(paths)} images → {OUTPUT_DIR}\n")

    report_rows = []

    for path in paths:
        fname = os.path.basename(path)
        print(f"── {fname} ──")

        frame = cv2.imread(path)
        if frame is None:
            print(f"  [SKIP] Could not read {path}")
            continue

        img_h, img_w = frame.shape[:2]
        img_cx, img_cy = img_w // 2, img_h // 2

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detections = []
        for run in range(N_RUNS):
            detections = detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=[fx, fy, cal_cx, cal_cy],
                tag_size=TAG_SIZE,
            )

            if not detections:
                print(f"  [run {run+1}/{N_RUNS}] No tags detected")
                report_rows.append({
                    'image': fname, 'run': run + 1,
                    'status': 'NOT DETECTED', 'tag_id': '', 'margin': '',
                    'x': '', 'y': '', 'z': '',
                    'roll': '', 'pitch': '', 'yaw': '',
                    'qw': '', 'qx': '', 'qy': '', 'qz': '',
                    'eu_px': '', 'ev_px': '',
                })
            else:
                for tag in detections:
                    xyz    = tag.pose_t.flatten()
                    rpy    = _rotation_to_euler_deg(tag.pose_R)
                    q      = _rotation_to_quaternion(tag.pose_R)
                    margin = tag.decision_margin
                    eu_px  = round(tag.center[0] - img_cx, 1)
                    ev_px  = round(tag.center[1] - img_cy, 1)
                    print(
                        f"  [run {run+1}/{N_RUNS}] Tag ID={tag.tag_id}"
                        f"  conf={margin:.1f}"
                        f"  xyz=[{xyz[0]:.4f}, {xyz[1]:.4f}, {xyz[2]:.4f}] m"
                        f"  rpy=[{rpy[0]:.2f}, {rpy[1]:.2f}, {rpy[2]:.2f}] deg"
                        f"  q=[{q[0]:.4f}, {q[1]:.4f}, {q[2]:.4f}, {q[3]:.4f}]"
                        f"  eu={eu_px:+.1f}px  ev={ev_px:+.1f}px"
                    )
                    report_rows.append({
                        'image': fname, 'run': run + 1,
                        'status': 'DETECTED', 'tag_id': tag.tag_id,
                        'margin': f'{margin:.1f}',
                        'x':     f'{xyz[0]:.4f}', 'y':     f'{xyz[1]:.4f}', 'z':     f'{xyz[2]:.4f}',
                        'roll':  f'{rpy[0]:.2f}',  'pitch': f'{rpy[1]:.2f}',  'yaw':   f'{rpy[2]:.2f}',
                        'qw':    f'{q[0]:.4f}',    'qx':    f'{q[1]:.4f}',
                        'qy':    f'{q[2]:.4f}',    'qz':    f'{q[3]:.4f}',
                        'eu_px': f'{eu_px:+.1f}',  'ev_px': f'{ev_px:+.1f}',
                    })

        # ── Annotate using last run's detections ──────────────────────────────
        vis = frame.copy()

        # Image centre crosshair (drawn first so tag overlays sit on top)
        _draw_frame_center(vis)

        for tag in detections:
            _draw_tag(vis, tag, K, D)

            # Axis-aligned bounding box
            pts = tag.corners.astype(np.int32)
            x1, y1 = pts[:, 0].min(), pts[:, 1].min()
            x2, y2 = pts[:, 0].max(), pts[:, 1].max()
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 255), 2)

            # Tag centre crosshair + error-vector line
            _draw_tag_center_and_error(vis, tag)

        # Bottom-right status label
        if detections:
            status_label, status_color = "TAG DETECTED",     (0, 220,  60)
        else:
            status_label, status_color = "TAG NOT DETECTED", (0,   0, 220)
        tw, _ = cv2.getTextSize(status_label, FONT, 0.65, 2)[0]
        _put_outlined(vis, status_label, (img_w - tw - 10, img_h - 10), status_color, scale=0.65)

        out_path = os.path.join(OUTPUT_DIR, fname)
        cv2.imwrite(out_path, vis)
        print(f"  Saved → {out_path}  ({len(detections)} tag(s))\n")

    _write_report(report_rows, os.path.join(OUTPUT_DIR, "report.csv"), len(paths))
    print(f"Done. {len(paths)} images processed.")


if __name__ == "__main__":
    process_images()
