"""Browser monitor (webui.py): what the screen says for each Session state, the alerts, browser input and the server's
token / file rules. No browser: the page itself only draws these fields."""
import http.client
import json
import time
from types import SimpleNamespace

import numpy as np
import pytest

from pinch_sim.session import Session
from pinch_sim.ui import notice_text
from pinch_sim.voice_gateway import Dispatcher
from pinch_sim.webui import (STATIC_DIR, Alerts, WebUI, answer_chips, apply_action, failure_dialog, heights,
                             menu_state, parse_input)

from test_voice_gateway import RM


def _session(scene, cfg, data):
    m, idx, sha = scene
    s = Session(m, idx, sha, cfg, data, threaded=False)
    s.cmd_fixed_target(RM, 10.0, data)
    return s


def test_menu_follows_the_session_from_new_path_to_done(scene, cfg, data):
    s = _session(scene, cfg, data)
    m = menu_state(s)
    assert (m["step"], m["chip"]["text"], m["error"]) == (1, "수정 대기", False)   # route set: next is editing
    s.toggle_edit_mode()
    assert menu_state(s)["chip"] == {"text": "경로 수정 중", "icon": "loading"}
    s.toggle_edit_mode()
    s.cmd_select()
    assert menu_state(s)["chip"]["text"] == "검증 대기" and menu_state(s)["step"] == 2
    s.cmd_validate(data)
    assert s.state == "PASSED" and menu_state(s)["chip"]["text"] == "이동 대기" and menu_state(s)["step"] == 3
    assert s.cmd_approve(data)
    assert menu_state(s)["chip"] == {"text": "로봇팔 이동 중", "icon": "loading"}
    s.cmd_cancel(data)
    assert s.state == "ABORTED" and menu_state(s)["chip"] == {"text": "이동 일시정지", "icon": "pause"}


def test_menu_states_without_a_path_failure_and_done():
    base = dict(edit=None, edit_mode=False, phase="outbound", real_arm=None)
    assert menu_state(SimpleNamespace(state="HOLD", path=None, **base))["chip"]["text"] == "경로 설정 전"
    path = SimpleNamespace(source="edited")
    m = menu_state(SimpleNamespace(state="FAILED", path=path, **base))
    assert m["error"] and m["step"] == 2 and m["chip"]["icon"] == "warning"
    m = menu_state(SimpleNamespace(state="DONE", path=path, **dict(base, phase="return")))
    assert m["step"] == 4 and m["chip"]["text"] == "이동 완료"                   # all four steps done
    m = menu_state(SimpleNamespace(state="GENERATED", path=path, **dict(base, phase="return")))
    assert m["chip"]["text"] == "돌아오는 길 · 경로 수정 완료"
    lost = dict(base, real_arm=SimpleNamespace(up=False))
    m = menu_state(SimpleNamespace(state="EXECUTING", path=path, **lost))
    assert m["error"] and m["chip"]["text"] == "로봇 연결 끊김"


def test_answer_chips_only_under_the_apps_questions():
    chat = [(1.0, "user", "수정 완료."), (2.0, "kuong", "검증 후 이동할까요?")]
    chips = answer_chips(chat, 12)
    assert chips == [{"label": "검증 후 이동", "command": "use_current_path", "fields": {"auto_run": True}, "rev": 12}]
    two = answer_chips([(1.0, "kuong", "멈췄어요."), (2.0, "kuong", "계속 갈까요, 돌아갈까요?")], 3)
    assert [c["fields"]["direction"] for c in two] == ["forward", "back"]
    assert answer_chips([(1.0, "kuong", "가운데를 위로 2센티 올렸어요.")], 3) == []
    assert answer_chips([(1.0, "kuong", "검증 후 이동할까요?"), (2.0, "user", "어")], 3) != []   # the last 쿠옹이 line


def test_failure_dialog_names_the_problem():
    assert failure_dialog(["collision", "torque"])["title"] == "충돌 위험 감지"
    assert failure_dialog(["joint_limits"])["title"] == "관절 범위 초과"
    assert failure_dialog(["continuous_ik"])["title"] == "작업 범위 초과"
    d = failure_dialog(["sim_rollout"])
    assert d["title"] == "경로 검증 실패" and d["action"] == "edit" and d["ok"] == "경로 수정"


def test_alerts_dialog_on_failure_toasts_for_refusals(scene, cfg, data):
    s = _session(scene, cfg, data)
    al = Alerts(notice_text)
    s.listeners.append(lambda ev: al.on_event(ev, s))
    s._emit("hand refused: not_edit_mode", "hand_refused", reason="not_edit_mode")
    s._emit("hand refused: not_edit_mode", "hand_refused", reason="not_edit_mode")     # same again: one card
    cards = al.current()
    assert len(cards) == 1 and cards[0]["warn"] and cards[0]["title"] == "손 입력 확인"
    s._emit("real arm link lost: no state for 0.5 s", "arm_link")
    assert al.dialog["title"] == "로봇 연결 끊김"
    s.state = "FAILED"
    s._emit("validation failed", "validation")
    assert al.dialog["title"] == "경로 검증 실패" and al.dialog["id"] > 1


def test_heights_follow_the_path_in_cm(scene, cfg, data):
    s = _session(scene, cfg, data)
    z = np.asarray(s.path.points_m)[:, 2] * 100
    h = heights(s, s.path.points_m[0])
    assert len(h["series"]) <= 120 and abs(h["max"] - z.max()) < 0.6 and abs(h["min"] - z.min()) < 0.6
    assert h["now_at"] == 0.0 and abs(h["now"] - z[0]) < 0.1 and h["route"] == "가는 길"
    assert heights(SimpleNamespace(path=None), None) is None


def test_browser_input_is_checked():
    assert parse_input({"type": "key", "key": "v"}) == {"type": "key", "key": "V"}
    assert parse_input({"type": "key", "key": "Z"}) is None
    assert parse_input({"type": "mouse", "kind": "press", "u": 0.5, "v": 0.2})["kind"] == "press"
    assert parse_input({"type": "mouse", "kind": "press", "u": 3, "v": 0.2}) is None
    assert parse_input({"type": "mouse", "kind": "teleport", "u": 0.5, "v": 0.2}) is None
    a = parse_input({"type": "action", "name": "answer", "command": "resume", "rev": 4,
                     "fields": {"direction": "back", "evil": 1}})
    assert a == {"type": "action", "name": "answer", "command": "resume", "rev": 4, "fields": {"direction": "back"}}
    assert parse_input({"type": "action", "name": "rm -rf"}) is None
    assert parse_input("V") is None


def test_actions_use_the_key_path_or_the_voice_commands(scene, cfg, data):
    s = _session(scene, cfg, data)
    disp = Dispatcher()
    assert apply_action(s, data, {"name": "step", "index": 2}, disp) == (["S", "V"], "")
    assert apply_action(s, data, {"name": "stop"}, disp) == (["X"], "")
    keys, why = apply_action(s, data, {"name": "confirm_run", "rev": s.rev - 1}, disp)
    assert keys == [] and "바뀌었어요" in why                                 # decided on an older path: refused
    keys, why = apply_action(s, data, {"name": "confirm_run", "rev": s.rev}, disp)
    assert why == "" and (s.auto_run is not None or s.state in ("PASSED", "EXECUTING"))
    assert apply_action(s, data, {"name": "answer", "command": "quit_app"}, disp)[1]


@pytest.fixture
def server():
    ui = WebUI().start()
    yield ui
    ui.close()


def _req(ui, method, path, body=None, headers=None):
    c = http.client.HTTPConnection("127.0.0.1", ui.port, timeout=5)
    c.request(method, path, body=body, headers=headers or {})
    r = c.getresponse()
    return r.status, r


def test_server_token_and_files(server):
    st, r = _req(server, "GET", "/")
    assert st == 200 and b"Chat KHU" in r.read()
    assert _req(server, "GET", "/static/app.js")[0] == 200
    assert _req(server, "GET", "/static/../webui.py")[0] == 404                      # nothing outside web/
    assert _req(server, "GET", "/events")[0] == 403 and _req(server, "GET", "/stream?t=wrong")[0] == 403
    body = json.dumps({"type": "key", "key": "N"})
    assert _req(server, "POST", "/api/input", body, {"Content-Type": "application/json"})[0] == 403
    tok = {"X-Token": server.token, "Content-Type": "application/json"}
    assert _req(server, "POST", "/api/input", body, tok)[0] == 204
    assert _req(server, "POST", "/api/input", json.dumps({"type": "key", "key": "!"}), tok)[0] == 400
    assert server.drain() == [{"type": "key", "key": "N"}]
    assert (STATIC_DIR / "fonts" / "LICENSE.txt").exists()


def test_server_streams_frames_and_state(server):
    server.push_state({"state": "HOLD"})
    st, r = _req(server, "GET", f"/events?t={server.token}")
    assert st == 200 and r.getheader("Content-Type").startswith("text/event-stream")
    assert json.loads(r.readline().decode()[len("data: "):]) == {"state": "HOLD"}
    server.push_frame(np.full((48, 64, 3), 128, np.uint8))
    st, r = _req(server, "GET", f"/stream?t={server.token}")
    assert st == 200 and "multipart/x-mixed-replace" in r.getheader("Content-Type")
    assert r.readline() == b"--frame\r\n" and r.readline() == b"Content-Type: image/jpeg\r\n"
    n = int(r.readline().decode().split(":")[1])
    r.readline()
    assert r.read(n)[:2] == b"\xff\xd8"                                          # a JPEG
