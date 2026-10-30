import numpy as np

# 점6, 8 제외한 7개 대응점 (RMS 2.26cm)
P_robot = np.array([
    [0.98, -0.485, 0.805],[-0.34, 0.46, 0.50],[-0.628, 0.00, 0.00],
    [0.0, 0.361, 0.00],[0.40, -0.764, 0.78],[0.57, 0.584, 0.00],
    [-0.312, -0.422, 0.10],
])
P_cam = np.array([
    [0.230, -0.750, 2.623],[-0.639, -0.092, 1.375],[-0.215, 0.470, 1.306],
    [-0.627, 0.241, 1.860],[0.506, -0.540, 2.121],[-0.903, 0.033, 2.360],
    [0.194, 0.295, 1.599],
])

def rigid(A, B):
    cA, cB = A.mean(0), B.mean(0)
    H = (A - cA).T @ (B - cB)
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    return R, cB - R @ cA

R, t = rigid(P_cam, P_robot)
np.savez('/home/user/vision_test/cam2robot.npz', R=R, t=t)

pred = (R @ P_cam.T).T + t
err = np.linalg.norm(pred - P_robot, axis=1)
print("R =\n", np.round(R, 5))
print("t =", np.round(t, 5))
print(f"RMS 오차: {np.sqrt((err**2).mean())*100:.2f} cm")
print("cam2robot.npz 저장 완료")
