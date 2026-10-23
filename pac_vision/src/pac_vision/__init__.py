"""PAC 26-1 vision: Astra depth -> base_ref coordinates.

핵심 계산(cam2base, objects, session)은 numpy와 PyYAML만 쓴다.
카메라(openni)는 astra_depth.py, ROS 2(rclpy)는 ros_pub.py에서만 쓴다.
"""

__version__ = "0.1.0"

from .cam2base import Cam2Base, CameraModel, solve_calibration  # noqa: E402,F401


def __getattr__(name):
    # AstraDepth는 처음 쓸 때 불러온다 (openni 없이도 import pac_vision 가능)
    if name == "AstraDepth":
        from .astra_depth import AstraDepth
        return AstraDepth
    raise AttributeError(name)
