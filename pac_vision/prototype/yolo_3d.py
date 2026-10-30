from ultralytics import YOLO
from openni import openni2
import numpy as np
import cv2

REDIST = "/home/user/ros2_astra_camera/astra_camera/openni2_redist/x64"
RGB_INDEX = 2

# ── 보정 상수 (measure_3d.py와 일치시킴) ──
FLIP_DEPTH = True
DX, DY     = 25, 0
Z_OFFSET   = -0.021      # measure_3d.py와 동일 (재측정 확정값)

FX = FY = 570.0
CX, CY  = 320.0, 240.0
X_SIGN  = +1.0

# ── 카메라 → base 변환 ──
try:
    _d = np.load("/home/user/vision_test/cam2robot.npz")
    R_c2r, t_c2r = _d["R"], _d["t"]
    HAVE_TF = True
    print("cam2robot.npz 로드됨 — base 좌표 출력 활성화")
except Exception as e:
    HAVE_TF = False
    print("cam2robot.npz 없음 — 카메라 좌표만:", e)

# 사람만 보려면 [0], 전부 보려면 None
CLASSES = None

model = YOLO("yolo26n.pt")
openni2.initialize(REDIST)
dev = openni2.Device.open_any()
ds = dev.create_depth_stream()
ds.start()
cap = cv2.VideoCapture(RGB_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

def get_depth():
    f = ds.read_frame()
    d = np.frombuffer(f.get_buffer_as_uint16(), dtype=np.uint16
                      ).reshape(f.height, f.width)
    return cv2.flip(d, 1) if FLIP_DEPTH else d

def median_depth(depth, u, v, half):
    h, w = depth.shape
    x0, x1 = max(0, u - half), min(w, u + half + 1)
    y0, y1 = max(0, v - half), min(h, v + half + 1)
    if x0 >= x1 or y0 >= y1:
        return 0.0
    vals = depth[y0:y1, x0:x1].ravel()
    vals = vals[vals > 0]
    return float(np.median(vals)) if vals.size else 0.0

print("q=종료.  물체를 0.6m 밖에 두고 확인하세요.")
while True:
    ok, rgb = cap.read()
    if not ok:
        break
    depth = get_depth()
    dh, dw = depth.shape
    rh, rw = rgb.shape[:2]
    sx, sy = dw / rw, dh / rh

    res = model.track(rgb, persist=True, imgsz=640, conf=0.25, classes=CLASSES, verbose=False)
    vis_d = np.clip(depth, 0, 4000).astype(np.float32) / 4000.0
    vis_d = cv2.applyColorMap((vis_d * 255).astype(np.uint8), cv2.COLORMAP_JET)

    for box in res[0].boxes:
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        name = model.names[int(box.cls[0])]
        tid  = int(box.id[0]) if box.id is not None else -1
        u_rgb, v_rgb = (x1 + x2) / 2, (y1 + y2) / 2
        u = int(u_rgb * sx) + DX
        v = int(v_rgb * sy) + DY
        if not (0 <= u < dw and 0 <= v < dh):
            continue
        half = int(min((x2 - x1) * sx, (y2 - y1) * sy) * 0.15)
        half = max(3, min(half, 25))
        z_mm = median_depth(depth, u, v, half)
        if z_mm == 0:
            print(f"{name} id={tid}  거리 측정 실패")
            continue
        Z = z_mm / 1000.0 - Z_OFFSET
        X = X_SIGN * (u - CX) * Z / FX
        Y = (v - CY) * Z / FY

        msg = f"{name} id={tid}  cam(X={X:+.2f} Y={Y:+.2f} Z={Z:.2f})"
        if HAVE_TF:
            P = R_c2r @ np.array([X, Y, Z]) + t_c2r
            msg += f"  base(X={P[0]:+.2f} Y={P[1]:+.2f} Z={P[2]:+.2f})"
            # base 기준 수평거리 (안전 판정용)
            dist = float(np.hypot(P[0], P[1]))
            msg += f"  수평거리={dist:.2f}m"
        print(msg)

        cv2.rectangle(vis_d, (u-half, v-half), (u+half, v+half), (255,255,255), 2)
        cv2.putText(vis_d, f"{Z:.2f}m", (u-20, v-half-8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)

    cv2.imshow("rgb", res[0].plot())
    cv2.imshow("depth", vis_d)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release(); ds.stop(); cv2.destroyAllWindows(); openni2.unload()
