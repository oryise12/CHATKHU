from openni import openni2
import numpy as np
import cv2

REDIST = "/home/user/ros2_astra_camera/astra_camera/openni2_redist/x64"
FLIP_DEPTH = True      # f 키로 실시간 전환됨

openni2.initialize(REDIST)
dev = openni2.Device.open_any()
ds = dev.create_depth_stream()
ds.start()
cap = cv2.VideoCapture(2)      # Astra RGB

pts = {"rgb": None, "depth": None}

def click_rgb(e, u, v, f, p):
    if e == cv2.EVENT_LBUTTONDOWN:
        pts["rgb"] = (u, v); print(f"RGB 클릭   ({u},{v})")
        report()

def click_depth(e, u, v, f, p):
    if e == cv2.EVENT_LBUTTONDOWN:
        pts["depth"] = (u, v); print(f"depth 클릭 ({u},{v})")
        report()

def report():
    if pts["rgb"] and pts["depth"]:
        dx = pts["depth"][0] - pts["rgb"][0]
        dy = pts["depth"][1] - pts["rgb"][1]
        print(f"  → 오프셋 dx={dx:+d}, dy={dy:+d}   (FLIP={FLIP_DEPTH})\n")

cv2.namedWindow("rgb");   cv2.setMouseCallback("rgb", click_rgb)
cv2.namedWindow("depth"); cv2.setMouseCallback("depth", click_depth)
print("같은 지점을 RGB와 depth에서 각각 클릭. f=좌우반전 전환, q=종료")

while True:
    ok, rgb = cap.read()
    frame = ds.read_frame()
    depth = np.frombuffer(frame.get_buffer_as_uint16(), dtype=np.uint16
                          ).reshape(frame.height, frame.width)
    if FLIP_DEPTH:
        depth = cv2.flip(depth, 1)

    vis = np.clip(depth, 0, 4000).astype(np.float32) / 4000.0
    vis = cv2.applyColorMap((vis * 255).astype(np.uint8), cv2.COLORMAP_JET)

    if ok:
        cv2.imshow("rgb", cv2.resize(rgb, (640, 480)))
    cv2.imshow("depth", vis)

    k = cv2.waitKey(1) & 0xFF
    if k == ord('q'):
        break
    if k == ord('f'):
        FLIP_DEPTH = not FLIP_DEPTH
        print(f"FLIP_DEPTH = {FLIP_DEPTH}")

cap.release(); ds.stop(); cv2.destroyAllWindows(); openni2.unload()
