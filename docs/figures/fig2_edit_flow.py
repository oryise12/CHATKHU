"""그림 2. 경로 수정 흐름도. 실행: python3 docs/figures/fig2_edit_flow.py  (graphviz dot 필요)

근거: voice/prompts/system_v8.md(수정에 필요한 정보, 되묻기, 직전 수정 이어 말하기, 결과 문장),
voice/tools.py(함수·거리 단계 2/5/8 cm, 최대 15 cm), voice/sim_link.py(adjust_amount, 손 수정 알림, too_far),
mujoco_sim/src/pinch_sim/hand_input.py(검지 0.4초 = 점 고르기, 핀치 = 3차원 옮기기, 놓으면 반영),
mujoco_sim/src/pinch_sim/session.py(edit_nudge: 말한 거리만큼 못 옮기면 바꾸지 않음), voice/config.yaml(호출어 없음).
"""
from _style import FONT, GRAY, dot_header, dot_png, pill, style

SRC = f"""digraph G {{
{dot_header("TB", nodesep=0.40, ranksep=0.42)}
  labelloc=t; label=<<FONT POINT-SIZE="30"><B>그림 2. 경로 수정 흐름도</B></FONT><BR/><FONT POINT-SIZE="15" COLOR="{GRAY}">쿠옹이는 호출어 없이 계속 듣습니다 · 손 수정과 말 수정은 같은 기록에 남아 &quot;이전으로&quot;로 함께 되돌립니다</FONT><BR/> >;

  start [label="수정 모드\\n그릇(B·C)을 고르면 직선 경로가 그려지며 켜짐\\n(또는 &quot;경로 수정할래&quot;)", {style("algo")}];
  how   [label="고치는 방법", {style("dec", dashed=True)}];
  p_pinch [label="핀치로", {pill()}];   p_point [label="검지로 가리키고 말로", {pill()}];

  pinch1 [label=<엄지·검지로 경로를 집어<BR/>3차원으로 옮김 (손을 놓는 순간 반영)<BR/><FONT POINT-SIZE="13">카메라 정합이 확인되고, 손이 카메라에서 60 cm 넘게 떨어져 있을 때</FONT>>, {style("user")}];
  pinch2 [label="쿠옹이: 경로를 수정했어요.\\n(수정 전 경로는 회색으로 겹쳐 보임)", {style("algo")}];

  point1 [label="경로 위 점을 검지로 0.4초 가리킴\\n→ 노란 점으로 표시", {style("user")}];
  say    [label="점을 보고 말함\\n예: &quot;여기 위로 조금&quot;, &quot;앞쪽으로 2센티&quot;", {style("user")}];
  judge  [label="쿠옹이(ChatKHU 음성 AI)가 말을 함수로 바꿈\\n어디: 가리킨 점 · 시작 · 가운데 · 끝\\n방향: 위 · 아래 · 왼쪽 · 오른쪽 · 앞 · 뒤\\n얼마나: 조금 2 · 보통 5 · 많이 8 cm (말 안 하면 조금, 숫자는 15 cm까지)", {style("ai")}];
  enough [label="정보가 충분한가?", {style("dec", dashed=True)}];
  ask    [label="두 가지 중 고르게 되물음\\n예: 위로 넘어갈까요, 옆으로 돌아갈까요?", {style("ai")}];
  can    [label="말한 거리만큼 옮길 수 있나?", {style("dec", dashed=True)}];
  far    [label="너무 많이 이동해서\\n더 수정할 수 없어요. (경로는 그대로)", {style("warn")}];
  moved  [label="여기를 위로 2센티 올렸어요.\\n(수정 전 경로는 회색으로 겹쳐 보임)", {style("algo")}];
  p_short [label="부족", {pill()}]; p_ok [label="충분", {pill()}]; p_no [label="불가능", {pill()}]; p_yes [label="가능", {pill()}];

  next [label="다음 말이나 손 입력\\n(올 때까지 기다림)", {style("dec", dashed=True)}];
  s_more [label="&quot;조금만 더&quot; · &quot;아직 모자라&quot;", {pill("user")}];
  s_less [label="&quot;너무 갔어&quot; · &quot;좀 덜&quot;", {pill("user")}];
  s_undo [label="&quot;이전으로&quot; · &quot;되돌려&quot;", {pill("user")}];
  s_reset [label="&quot;처음으로&quot; · &quot;초기화&quot;", {pill("user")}];
  s_keep [label="&quot;그대로 둬&quot; · &quot;하지 마&quot;", {pill("user")}];
  s_other [label="다른 점을 가리키거나 핀치", {pill("user")}];
  s_done [label="&quot;수정 완료&quot; · &quot;이 경로로 할게&quot; · &quot;가보자&quot;", {pill("user")}];
  r_more [label="직전 수정을 한 단계 크게 다시\\n2 → 5 → 8 cm (8에서는: 이미 가장 많이 옮겼어요)", {style("algo")}];
  r_less [label="직전 수정을 한 단계 작게 다시\\n8 → 5 → 2 cm (2에서는: 이미 가장 조금 옮겼어요)", {style("algo")}];
  r_undo [label="직전 수정을 되돌렸어요.\\n(손으로 고친 것도)", {style("algo")}];
  r_reset [label="처음 경로로 되돌렸어요.\\n(지금까지의 수정을 모두 취소)", {style("algo")}];
  r_keep [label="네, 그대로 둘게요.\\n(아무것도 움직이지 않음)", {style("algo")}];
  done   [label="쿠옹이: 검증 후 이동할까요?\\n→ 그림 3 전체 흐름으로", {style("algo")}, penwidth=3];

  start -> how;
  how -> p_pinch [arrowhead=none]; how -> p_point [arrowhead=none];
  p_pinch -> pinch1 -> pinch2;
  p_point -> point1 -> say -> judge -> enough;
  enough -> p_short [arrowhead=none]; p_short -> ask; ask -> say [style=dashed, constraint=false];
  enough -> p_ok [arrowhead=none]; p_ok -> can;
  can -> p_no [arrowhead=none]; p_no -> far;
  can -> p_yes [arrowhead=none]; p_yes -> moved;
  pinch2 -> next; moved -> next; far -> next [style=dashed];
  next -> s_more [arrowhead=none]; s_more -> r_more;
  next -> s_less [arrowhead=none]; s_less -> r_less;
  next -> s_undo [arrowhead=none]; s_undo -> r_undo;
  next -> s_reset [arrowhead=none]; s_reset -> r_reset;
  next -> s_keep [arrowhead=none]; s_keep -> r_keep;
  next -> s_other [arrowhead=none]; s_other -> how [style=dashed, constraint=false];
  next -> s_done [arrowhead=none]; s_done -> done;
  {{r_more r_less r_undo r_reset r_keep}} -> next [style=dashed, constraint=false, color="{GRAY}"];

  {{rank=same; p_pinch; p_point}}
  {{rank=same; pinch1; point1}}
  {{rank=same; p_short; p_ok}}
  {{rank=same; ask; can}}
  {{rank=same; s_more; s_less; s_undo; s_reset; s_keep; s_other; s_done}}
  {{rank=same; r_more; r_less; r_undo; r_reset; r_keep; done}}
}}
"""

if __name__ == "__main__":
    print(dot_png(SRC, "fig2_edit_flow"))
