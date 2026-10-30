from ultralytics import YOLO
from openni import openni2
import numpy as np
import cv2, time

REDIST = "/home/user/ros2_astra_camera/astra_camera/openni2_redist/x64"
RGB_INDEX = 2
FLIP_DEPTH = True
DX, DY     = 25, 0
Z_OFFSET   = -0.021
FX = FY = 570.0
CX, CY  = 320.0, 240.0
X_SIGN  = +1.0
CLASSES = [0]            # 사람만

_d = np.load("/home/user/vision_test/cam2robot.npz")
R_c2r, t_c2r = _d["R"], _d["t"]
print("cam2robot.npz 로드됨")

logf = open("/home/user/vision_test/verify_log.csv", "w")
logf.write("time,cam_X,cam_Y,cam_Z,base_X,base_Y,base_Z,dist_xy\n")

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
    x0, x1 = max(0, u-half), min(w, u+half+1)
    y0, y1 = max(0, v-half), min(h, v+half+1)
    if x0>=x1 or y0>=y1: return 0.0
    vals = depth[y0:y1, x0:x1].ravel()
    vals = vals[vals>0]
    return float(np.median(vals)) if vals.size else 0.0

print("사람만 검출. 아는 자리에 서서 몇 초 유지 → verify_log.csv 에 기록됨. q=종료")
while True:
    ok, rgb = cap.read()
    if not ok: break
    depth = get_depth()
    dh, dw = depth.shape
    rh, rw = rgb.shape[:2]
    sx, sy = dw/rw, dh/rh

    res = model.track(rgb, persist=True, imgsz=640, conf=0.25,
                      classes=CLASSES, verbose=False)
    vis_d = np.clip(depth,0,4000).astype(np.float32)/4000.0
    vis_d = cv2.applyColorMap((vis_d*255).astype(np.uint8), cv2.COLORMAP_JET)

    for box in res[0].boxes:
        x1,y1,x2,y2 = box.xyxy[0].tolist()
        u_rgb, v_rgb = (x1+x2)/2, (y1+y2)/2
        u = int(u_rgb*sx) + DX
        v = int(v_rgb*sy) + DY
        if not (0<=u<dw and 0<=v<dh): continue
        half = int(min((x2-x1)*sx, (y2-y1)*sy)*0.15)
        half = max(3, min(half,25))
        z_mm = median_depth(depth,u,v,half)
        if z_mm==0: continue
        Z = z_mm/1000.0 - Z_OFFSET
        X = X_SIGN*(u-CX)*Z/FX
        Y = (v-CY)*Z/FY
        P = R_c2r @ np.array([X,Y,Z]) + t_c2r
        dist = float(np.hypot(P[0], P[1]))
        print(f"cam(X={X:+.2f} Y={Y:+.2f} Z={Z:.2f})  "
              f"base(X={P[0]:+.2f} Y={P[1]:+.2f} Z={P[2]:+.2f})  수평거리={dist:.2f}m")
        logf.write(f"{time.time():.1f},{X:.3f},{Y:.3f},{Z:.3f},"
                   f"{P[0]:.3f},{P[1]:.3f},{P[2]:.3f},{dist:.3f}\n")
        logf.flush()
        cv2.rectangle(vis_d,(u-half,v-half),(u+half,v+half),(255,255,255),2)

    cv2.imshow("rgb", res[0].plot())
    cv2.imshow("depth", vis_d)
    if cv2.waitKey(1) & 0xFF == ord('q'): break

logf.close()
cap.release(); ds.stop(); cv2.destroyAllWindows(); openni2.unload()
