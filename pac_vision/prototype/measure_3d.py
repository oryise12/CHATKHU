from openni import openni2
import numpy as np
import cv2

REDIST = "/home/user/ros2_astra_camera/astra_camera/openni2_redist/x64"
# --- intrinsics (FOV에서 계산됨) ---
FX, FY = 570.0, 570.0
CX, CY = 320.0, 240.0

# --- 카메라 → 로봇 base 변환 (cam2robot.npz에서 불러옴) ---
try:
    _d = np.load("/home/user/vision_test/cam2robot.npz")
    R_c2r, t_c2r = _d["R"], _d["t"]
    HAVE_TF = True
    print("cam2robot.npz 로드됨 — base 좌표 출력 활성화")
except Exception as e:
    HAVE_TF = False
    print("cam2robot.npz 없음 — 카메라 좌표만 출력:", e)

openni2.initialize(REDIST)
dev = openni2.Device.open_any()
ds = dev.create_depth_stream()
ds.start()

def get_depth():
    frame = ds.read_frame()
    data = np.frombuffer(frame.get_buffer_as_uint16(), dtype=np.uint16)
    return data.reshape(frame.height, frame.width)

def patch_depth_mm(depth, u, v, k=2):
    """(u,v) 주변 5x5에서 0(구멍) 제외한 중앙값"""
    h, w = depth.shape
    y0, y1 = max(0, v-k), min(h, v+k+1)
    x0, x1 = max(0, u-k), min(w, u+k+1)
    vals = depth[y0:y1, x0:x1].flatten()
    vals = vals[vals > 0]
    if vals.size == 0:
        return 0
    return float(np.median(vals))

def on_click(event, u, v, flags, param):
    if event != cv2.EVENT_LBUTTONDOWN:
        return
    import time
    samples = []
    for _ in range(30):
        d = get_depth()
        z = patch_depth_mm(d, u, v)
        if z > 0:
            samples.append(z)
        time.sleep(0.04)
    if not samples:
        print(f"({u},{v}) → 측정 실패")
        return
    z_mm = float(np.median(samples))
    valid = len(samples)
    Z_OFFSET = -0.021   # m, 카메라 앞면→타깃 앞면 기준
    Z = z_mm / 1000.0 - Z_OFFSET
    X = -(u - CX) * Z / FX   # 미러 보정
    Y = (v - CY) * Z / FY
    print(f"픽셀({u},{v})  카메라: X={X:+.3f} Y={Y:+.3f} Z={Z:.3f} m  "
          f"(유효 {valid}/30, σ={np.std(samples):.1f}mm)")
    if HAVE_TF:
        P = R_c2r @ np.array([X, Y, Z]) + t_c2r
        print(f"            base:   X={P[0]:+.3f} Y={P[1]:+.3f} Z={P[2]:+.3f} m")

cv2.namedWindow("depth")
state = {'depth': None}
cv2.setMouseCallback("depth", on_click, state)
print("물체를 클릭하면 카메라 좌표 + base 좌표 출력. q로 종료.")
while True:
    depth = get_depth()
    state['depth'] = depth
    vis = np.clip(depth, 0, 4000).astype(np.float32) / 4000.0
    vis = (vis * 255).astype(np.uint8)
    vis = cv2.applyColorMap(vis, cv2.COLORMAP_JET)
    cv2.imshow("depth", vis)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
ds.stop()
cv2.destroyAllWindows()
openni2.unload()
