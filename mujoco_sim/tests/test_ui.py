"""side_cam monitor UI: stage from the real Session, 16:10 layout, bubbles/notices never cover the path."""
import time

import mujoco
import numpy as np

from pinch_sim.camera_model import CameraModel
from pinch_sim.session import Session
from pinch_sim.overlay import build_overlay
from pinch_sim.ui import (STAGES, Layout, MonitorUI, Notices, auto_layout, notice_text, progress_stage,
                          protect_points)

RM = 0.275                                                # inside the target band 0.25-0.30 m


def _session(scene, cfg, data, threaded=False):
    m, idx, sha = scene
    s = Session(m, idx, sha, cfg, data, threaded=threaded)
    s.cmd_fixed_target(RM, 10.0, data)
    return s


def stage(s):
    return progress_stage(s)[0]


def test_stage_follows_session_through_buttons(scene, cfg, data):
    s = _session(scene, cfg, data)
    assert stage(s) == 0                                    # initial path
    s.handle_key("S", data)
    assert progress_stage(s) == (0, "경로 선택됨")        # unedited path chosen: still 초기 경로 ...
    s.handle_key("V", data)
    assert stage(s) == 2 and s.state == "PASSED"           # ... then straight to 검증 (수정 shown passed)
    s.handle_key("Y", data)
    assert stage(s) == 3 and progress_stage(s)[1] == "이동 중"
    while s.state == "EXECUTING":
        s.physics_step(data)
    assert progress_stage(s) == (3, "완료")
    s.handle_key("H", data)
    assert progress_stage(s) == (0, "홈 복귀 경로")       # return path starts over
    s.handle_key("X", data)
    assert stage(s) == 0
    s.handle_key("N", data)
    assert stage(s) == 0 and s.edit_mode is False          # new target: reset


def test_stage_edit_fail_and_back(scene, cfg, data):
    s = _session(scene, cfg, data)
    k = s.path.waypoint_index[1] - 3
    p0 = s.path.points_m[k].copy()
    assert s.edit_begin(k, p0)
    assert progress_stage(s) == (1, "드래그 중")
    s.edit_update(p0 + [0, 0, -0.15])                      # below the desk top
    s.edit_end()
    assert stage(s) == 1
    s.handle_key("S", data)
    s.handle_key("V", data)
    assert s.state == "FAILED" and stage(s) == 1           # failed validation: back to 수정, 검증/이동 white
    s.handle_key("O", data)
    assert s.state == "GENERATED" and stage(s) == 1        # still in edit mode until a new path/validation


def test_x_during_validation_returns_to_selection(scene, cfg, data):
    s = _session(scene, cfg, data, threaded=True)
    s.handle_key("S", data)
    s.handle_key("V", data)
    assert stage(s) == 2 and s.state == "VALIDATING"
    s.handle_key("X", data)
    assert s.state == "SELECTED" and stage(s) == 0
    s.close()


def test_hold_has_no_stage(scene, cfg, data):
    m, idx, sha = scene
    s = Session(m, idx, sha, cfg, data, threaded=False)
    assert progress_stage(s)[0] == -1 and len(STAGES) == 4


def test_layout_is_16_10_and_about_twice_the_old_area(cfg):
    W, H = cfg["cameras"]["width_px"], cfg["cameras"]["height_px"]
    assert W * 10 == H * 16
    lay = auto_layout(W, H, "auto", (0, 0, 1920, 1080))
    assert abs(lay.img_w / lay.img_h - 1.6) < 0.01
    assert 1.8 < lay.img_w * lay.img_h / (544 * 408) < 2.2           # old window image: 640x480 x 0.85
    assert lay.height <= 1080 - 120 and lay.width <= 0.6 * 1920
    small = auto_layout(W, H, "auto", (0, 0, 1280, 720))
    assert small.height <= 720 - 120 + 1 and small.width <= 0.56 * 1280   # shrinks on a small monitor
    assert lay.region(10, 5) == "top" and lay.region(10, lay.top_h + 5) == "image"
    assert lay.region(10, lay.height - 3) == "bar" and lay.to_image(7, lay.top_h + 9) == (7, 9)


CHAT = [(-3, "user", "쿠옹아, 경로 수정할래"), (-2, "kuong", "네. 경로 수정 모드로 진입할게요. 화면에서 경로를 수정해주세요."),
        (-1, "user", "오키 좋다. 이 경로로 할게"), (0, "kuong", "검증할게요.")]


def _chat():
    now = time.monotonic()
    return [(now + dt, role, text) for dt, role, text in CHAT]


def _covered(rects, P):
    return [(r, p) for r in rects for p in P if r[0] <= p[0] <= r[2] and r[1] <= p[1] <= r[3]]


def test_bubbles_never_cover_the_path_or_targets(scene, cfg, data):
    m, idx, _ = scene
    W, H = cfg["cameras"]["width_px"], cfg["cameras"]["height_px"]
    cam = CameraModel.from_mujoco(m, data, "side_cam", W, H)
    lay = Layout(W, H, 1.1)
    ui = MonitorUI(lay)
    frame = np.zeros((H, W, 3), np.uint8)
    for r, th in [(RM, 10.0), (0.30, -35.0), (0.30, -25.0), (0.30, 0.0), (0.25, 25.0), (0.30, 35.0)]:
        s = _session(scene, cfg, data)
        s.cmd_fixed_target(r, th, data)
        P = protect_points(build_overlay(s, data), cam, lay.scale)
        assert len(P) > 50
        out = ui.render(frame, s, [("N", "new", True)], ["검증 실패: 작업 공간 밖"], _chat(), "대답 기다리는 중 12초", P)
        assert out.shape[:2] == (lay.height, lay.width)
        assert not _covered(ui.ui_rects, P), (r, th)
        assert len(ui.ui_rects) >= 3, (r, th)                 # notice + hint + at least the newest bubble
        assert all(x0 > lay.img_w * 0.6 for x0, *_ in ui.ui_rects[1:])     # bubbles stay in the right column
    x0, y0, x1, y1 = ui.ui_rects[-1]
    assert ui.hit_ui((x0 + x1) // 2, (y0 + y1) // 2)          # a bubble click is a UI click ...
    u, v = cam.project(s.path.points_m[20])[0][0]
    assert not ui.hit_ui(int(u * lay.scale), int(v * lay.scale))   # ... a path point is not


def test_bubbles_and_notices_move_away_from_protected_points(scene, cfg, data):
    W, H = cfg["cameras"]["width_px"], cfg["cameras"]["height_px"]
    lay = Layout(W, H, 1.1)
    ui = MonitorUI(lay)
    s = _session(scene, cfg, data)
    frame = np.zeros((H, W, 3), np.uint8)
    ui.render(frame, s, [("N", "new", True)], ["알림"], _chat(), "대답 기다리는 중", None)
    assert len(ui.ui_rects) == 6                              # nothing to avoid: notice + 4 bubbles + hint
    assert ui.ui_rects[0][1] < 20                             # notice top-left ...
    assert max(r[3] for r in ui.ui_rects[1:]) > lay.img_h - 20     # ... bubbles stacked from the bottom
    ys = np.arange(lay.img_h * 0.55, lay.img_h, 4.0)
    P = np.r_[np.c_[np.full_like(ys, lay.img_w - 60), ys],     # something in the lower right column ...
              [[40, 20]]]                                      # ... and under the top-left notice
    ui.render(frame, s, [("N", "new", True)], ["알림"], _chat(), "대답 기다리는 중", P)
    assert not _covered(ui.ui_rects, P)
    assert ui.ui_rects[0][3] > lay.img_h - 20                 # notice moved to the bottom-left
    assert min(r[1] for r in ui.ui_rects[1:]) < 20            # bubbles stacked from the top instead
    P = np.c_[np.full(60, lay.img_w - 60), np.linspace(0, lay.img_h, 60)]     # the whole right column
    ui.render(frame, s, [("N", "new", True)], [], _chat(), "대답 기다리는 중", P)
    assert ui.ui_rects == []                                   # nothing fits: show nothing rather than cover


def test_notices_explain_refusals_and_failures(scene, cfg, data):
    s = _session(scene, cfg, data)
    notes = Notices()
    s.listeners.append(lambda ev: notes.push(notice_text(ev, s)))
    s.handle_key("Y", data)
    assert notes.current()[-1].startswith("검증을 통과한 경로가 없어요")
    s.handle_key("V", data)
    assert notes.current()[-1] == "먼저 경로를 선택(S)하세요"
    k = s.path.waypoint_index[1] - 3
    p0 = s.path.points_m[k].copy()
    s.edit_begin(k, p0)
    s.edit_update(p0 + [0, 0, -0.15])
    s.edit_end()
    s.handle_key("S", data)
    s.handle_key("V", data)
    assert notes.current()[-1].startswith("검증 실패:") and "작업 공간" in notes.current()[-1]
