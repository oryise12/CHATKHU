"""Demo stations (stations.py, 2026-10-29): HOME -> S by itself, then S -> B / C chosen by voice on straight paths,
edited and run, back to S, the remaining one, and the end at S. Voice side = voice/sim_link.SimEditor over the real
gateway (no AI): what the app asks when the user calls 쿠옹아 at S, and what it announces on each arrival."""
import os
import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np
import pytest

from pinch_sim import stations as st
from pinch_sim.session import Session
from pinch_sim.voice_gateway import VoiceGateway

from test_voice_flow import AMOUNT, MainLoop, _hand_drag, _said, _synced

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))          # the repository root: the voice package
from voice import sim_link  # noqa: E402
from voice.sim_link import SimEditor  # noqa: E402

# world frame (robot base at the origin, front = +Y): S in front, B left, C right, all about 0.25-0.27 m away
WORLD = {"S": (0.0, 0.27), "B": (-0.12, 0.23), "C": (0.12, 0.23)}
DIP = 0.03                                                   # tap at the bowls and back at the basket


def _stations():
    out = {}
    for name, label, marker, h in (("S", "설거지 바구니", 6, 0.12), ("B", "국그릇", 7, 0.12), ("C", "밥그릇", 8, 0.12)):
        x, y = WORLD[name]
        out[name] = st.Station(name, label, marker, h, floor_m=np.array([x, y, 0.0]), base_m=np.array([x, y, 0.0]))
    return out


@pytest.fixture
def demo(scene, cfg, data):
    short = Path(tempfile.mkdtemp(prefix="ps", dir=os.environ.get("XDG_RUNTIME_DIR") or "/tmp"))  # AF_UNIX length
    m, idx, sha = scene
    s = Session(m, idx, sha, cfg, data)
    gw = VoiceGateway(str(short / "p" / "voice.sock")).start(s)
    loop = MainLoop(s, data, gw)
    ed = SimEditor(AMOUNT, gw.address)
    yield s, data, gw, loop, ed
    ed.close()
    loop.close()
    gw.close()
    s.close()
    shutil.rmtree(short, ignore_errors=True)


def _to_base(s, d, loop, ed=None):
    """What the app does once the markers are read and the arm is armed: HOME -> S is drawn, 쿠옹이 asks
    "설거지 바구니로 이동할까요?" and the yes (validate_and_move) validates and runs it."""
    def start():
        s.set_stations(_stations(), dip_m=DIP)
        ok, code = s.cmd_station("S", d)
        assert ok, code
        assert s.move_kind == "to_base" and not s.edit_mode and s.path.meta["shape"] == "straight"
        assert "dip_m" not in s.path.meta                                   # no tap on the first move HOME -> S
        s._emit("ready to move HOME -> S", "stations_ready")
    loop.call(start)
    if ed is None:
        loop.call(lambda: s.cmd_select() and s.cmd_validate(d, auto_run_request="stations-base"))
    else:
        said = _said(ed, 1.0)
        assert "설거지 바구니로 이동할까요?" in said, said
        assert ed.question_functions("설거지 바구니로 이동할까요?") == frozenset({"validate_and_move"})
        assert loop.call(lambda: s.state) == "GENERATED"                     # nothing moves before the yes
        _synced(loop, s, ed)
        assert ed.handle("validate_and_move", {})["status"] in ("validating", "moving")   # "응"
    assert loop.wait(lambda: s.state == "DONE", 90), loop.call(lambda: s.message)


def _go(ed, loop, s, name):
    _synced(loop, s, ed)
    r = ed.handle("go_to_station", {"station": name})
    assert r["status"] == "applied", r
    assert r["say"].startswith(f"{name} ") and r["say"].endswith(sim_link.EDIT_GUIDE)
    assert loop.call(lambda: s.edit_mode and s.path.path_id == "to_target")
    goal = loop.call(lambda: s.stations[name].tcp_goal_m)
    P = loop.call(lambda: s.path.points_m.copy())
    assert np.allclose(P[-1], goal - [0, 0, DIP])                             # ends down at the bowl and stays
    assert np.allclose(P[len(P) // 3: 2 * len(P) // 3, 2], goal[2], atol=1e-6)   # straight at the target height
    assert np.allclose(loop.call(lambda: s.goal.p_m), P[-1])


def _run_edited(ed, loop, s):
    loop.call(_hand_drag(s, 0.03))                                            # a pinch edit
    _synced(loop, s, ed)
    assert ed.handle("finish_edit", {}) == {"status": "asking"}
    assert ed.handle("validate_and_move", {})["status"] in ("validating", "moving")
    assert loop.wait(lambda: s.state == "DONE", 90), loop.call(lambda: s.message)


def test_full_demo_two_bowls_four_edits_ends_at_basket(demo):
    s, d, gw, loop, ed = demo
    _to_base(s, d, loop, ed)
    said = _said(ed, 1.5)
    assert "설거지 바구니에 도착했어요. 어디로 갈까요?" in said, said              # asked on arrival
    assert ed.question_functions(said[-1]) == frozenset({"go_to_station"})
    _synced(loop, s, ed)
    assert ed.wake_question() == sim_link.ASK_WHERE                          # 쿠옹아 at S -> 어디로 갈까요?
    assert ed.question_functions(sim_link.ASK_WHERE) == frozenset({"go_to_station"})

    _go(ed, loop, s, "B")                                                    # "B지점" / "국그릇"
    _run_edited(ed, loop, s)                                                 # edit 1: S -> B
    said = _said(ed, 1.5)
    assert any(t.startswith("B 국그릇에 도착했어요.") and t.endswith(sim_link.ASK_RETURN) for t in said), said
    assert loop.call(lambda: s.visited) == ["B"]
    _synced(loop, s, ed)
    assert ed.wake_question() is None                                        # not at S: no station question

    assert ed.handle("edit_return_path", {})["status"] == "applied"         # 돌아오는 길 수정할래
    P = loop.call(lambda: s.path.points_m.copy())
    zb, zs = loop.call(lambda: (s.stations["B"].tcp_goal_m[2], s.stations["S"].tcp_goal_m[2]))
    assert abs(P[0, 2] - (zb - DIP)) < 1e-6 and abs(P[-1, 2] - (zs - DIP)) < 1e-6   # down at B -> down at S
    assert P[len(P) // 6: -len(P) // 6, 2].min() > zs - 1e-3                  # rises first (the edited way, reversed)
    _run_edited(ed, loop, s)                                                 # edit 2: B -> S
    assert "설거지 바구니로 돌아왔어요. 남은 C 밥그릇으로 갈까요?" in _said(ed, 1.5)
    _synced(loop, s, ed)
    ask = ed.wake_question()
    assert ask == "남은 C 밥그릇으로 갈까요?"
    assert ed.question_functions(ask) == frozenset({"go_to_station"})

    _go(ed, loop, s, "C")
    _run_edited(ed, loop, s)                                                 # edit 3: S -> C
    assert loop.call(lambda: s.visited) == ["B", "C"]
    _said(ed, 1.0)
    _synced(loop, s, ed)
    assert ed.handle("edit_return_path", {})["status"] == "applied"
    _run_edited(ed, loop, s)                                                 # edit 4: C -> S
    assert sim_link.ASK_AGAIN in _said(ed, 1.5)                              # 두 곳 모두 다녀왔어요. 다시 이동할까요?
    assert ed.question_functions(sim_link.ASK_AGAIN) == frozenset({"go_to_station"})
    _synced(loop, s, ed)
    assert ed.wake_question() == sim_link.ASK_AGAIN                          # 쿠옹아 again at S: the same question
    assert loop.call(lambda: s.at_base(d))
    _go(ed, loop, s, "C")                                                    # "응, 밥그릇으로 가자": a new round
    assert loop.call(lambda: s.visited) == []
    _run_edited(ed, loop, s)
    assert loop.call(lambda: s.visited) == ["C"]


def test_reverse_return_and_choice_refused_away_from_basket(demo):
    s, d, gw, loop, ed = demo
    _to_base(s, d, loop)
    _go(ed, loop, s, "C")
    _run_edited(ed, loop, s)
    _synced(loop, s, ed)
    r = ed.handle("go_to_station", {"station": "B"})                         # at C, not at S
    assert r == {"status": "failed", "reason": "not_at_base"}
    assert ed.handle("go_back", {})["status"] == "validating"                # 반대로 돌아가 (no edit)
    assert loop.wait(lambda: s.state == "DONE" and s.at_base(d), 90)
    _synced(loop, s, ed)
    assert ed.wake_question() == "남은 B 국그릇으로 갈까요?"


def test_answer_chips_and_folded_chat_suggestions(scene, cfg, data):
    from pinch_sim.webui import answer_chips, suggestions
    m, idx, sha = scene
    s = Session(m, idx, sha, cfg, data)
    s.set_stations(_stations(), dip_m=DIP)
    ask = [(0.0, "kuong", "설거지 바구니에 도착했어요. 어디로 갈까요?")]
    chips = answer_chips(ask, 7, s)
    assert [(c["label"], c["command"], c["fields"]) for c in chips] == [
        ("B 국그릇", "station_target", {"station": "B"}), ("C 밥그릇", "station_target", {"station": "C"})]
    assert suggestions(s, ask) == ["국그릇으로 가자", "밥그릇으로 가자"]
    s.visited = ["B"]
    rest = [(0.0, "kuong", "설거지 바구니로 돌아왔어요. 남은 C 밥그릇으로 갈까요?")]
    assert [c["label"] for c in answer_chips(rest, 7, s)] == ["C 밥그릇"]
    move = [(0.0, "kuong", "설거지 바구니로 이동할까요?")]
    assert answer_chips(move, 7, s)[0]["command"] == "use_current_path"
    assert suggestions(s, move) == ["응, 이동해", "아니"]
    assert suggestions(s, [(0.0, "kuong", "검증 후 이동할까요?")]) == ["응", "아니, 더 고칠래"]
    s.close()


def test_reader_medians_marker_centres_onto_the_desk():
    class Conv:                                   # pixel -> desk: x = u / 1000, y = v / 1000
        def pixel_to_desk(self, u, v, rgb_mirrored=False, rgb_size=None, plane=None):
            return np.array([[u / 1000.0, v / 1000.0, 0.0]]), None, None

        def desk_plane(self):
            class P:
                normal, d = np.array([0.0, 0.0, 1.0]), 0.0
            return P()

    class Cam:
        conv, mirrored = Conv(), False

    cfg = st.StationConfig("DICT_4X4_50", 3, {n: s for n, s in _stations().items()})
    for s in cfg.stations.values():
        s.floor_m = s.base_m = None
    reader = st.StationReader(cfg, Cam(), lambda p: np.asarray(p, float) + [1.0, 0.0, 0.0])
    import pac_vision.markers as pm
    frames = [{6: (100, 200), 7: (300, 50), 8: (500, 60)}, {6: (102, 200), 7: (300, 52)},
              {6: (104, 200), 7: (300, 54), 8: (500, 62)}, {8: (500, 64)}]
    orig = pm.detect
    try:
        done = []
        for f in frames:
            pm.detect = lambda _img, _dict, f=f: {k: np.array(v, float) for k, v in f.items()}
            done.append(reader.update(np.zeros((4, 4, 3), np.uint8), 0.0))
    finally:
        pm.detect = orig
    assert done == [False, False, False, True]                              # C was seen in its 3rd frame last
    assert np.allclose(cfg.stations["S"].base_m, [0.102, 0.2, 0.0])          # median of the pixel centres
    assert np.allclose(cfg.stations["S"].floor_m, [1.102, 0.2, 0.0])         # then calibration -> world
    assert np.allclose(cfg.stations["C"].tcp_goal_m, [1.5, 0.062, 0.12])


def test_config_file_and_reach_note(tmp_path):
    f = tmp_path / "stations.yaml"
    f.write_text("stations:\n  S: {marker: 6, label: 설거지 바구니, height_m: 0.2, base_m: [0.0, 0.27]}\n"
                 "  B: {marker: 7, label: 국그릇, base_m: [-0.1, 0.25]}\n"
                 "  C: {marker: 8, label: 밥그릇, base_m: [0.5, 0.0]}\n", encoding="utf-8")
    cfg = st.load_config(f)
    assert cfg.stations["S"].height_m == 0.2 and cfg.stations["B"].height_m == 0.12
    assert st.from_base(cfg, lambda p: np.asarray(p, float))
    lines = st.describe(cfg)
    assert "far from the usual reach" in lines[2] and "far" not in lines[0]
