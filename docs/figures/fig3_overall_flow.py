"""그림 3. 전체 흐름도. 실행: python3 docs/figures/fig3_overall_flow.py  (graphviz dot 필요)

근거: voice/sim_link.py(앱 질문·알림 문장: 설거지 바구니로 이동할까요? / 어디로 갈까요? / 검증 후 이동할까요? /
돌아오는 길을 수정할까요… / 남은 ~으로 갈까요? / 다시 이동할까요? / 계속 갈까요, 돌아갈까요? / 시작한 자리로 돌아왔어요…),
voice/prompts/system_v8.md(아니라는 답의 문장), mujoco_sim/src/pinch_sim/session.py(검증 실패 → 수정 모드, 멈춘 뒤 계속·돌아가기,
돌아가기는 그 이동을 시작한 자리로), app.py(--stations: 무장 뒤 HOME→S 질문), stations.py·stations.yaml(도착 후 5 cm 내려가 유지),
manipulator_control(무장 y: 터미널 또는 무장 확인 창, 받은 궤적 재검사).
"""
from _style import GRAY, COL, dot_header, dot_png, pill, style

O = COL["robot"][1]
SRC = f"""digraph G {{
{dot_header("TB", nodesep=0.38, ranksep=0.40)}
  graph [splines=ortho];
  labelloc=t; label=<<FONT POINT-SIZE="30"><B>그림 3. 전체 흐름도</B></FONT><BR/><FONT POINT-SIZE="15" COLOR="{GRAY}">S 설거지 바구니(6번) · B 국그릇(7번) · C 밥그릇(8번) · 검증 = 12개 항목 확인 + 시뮬레이션으로 미리 따라가 보기 · 실물은 받은 궤적을 한 번 더 검사한 뒤 움직임</FONT><BR/> >;

  s0  [label="시작: 카메라가 6·7·8번 마커 위치를 읽음", {style("algo")}];
  s1  [label="실물 팔 HOME(A) → 무장\\n(터미널 또는 작은 &quot;실물 무장&quot; 창에서 y)", {style("robot")}];
  s2  [label="쿠옹이: 설거지 바구니로 이동할까요?", {style("algo")}];
  q0  [label="사용자 답", {style("dec", dashed=True)}];
  w0  [label="네, 준비되면 말씀해 주세요. (대기)\\n나중에 &quot;이동해&quot;라고 하면 출발", {style("algo")}];
  m0  [label="검증 → A에서 S로 이동", {style("robot")}];
  atS [label="쿠옹이: 설거지 바구니에 도착했어요. 어디로 갈까요?", {style("algo")}];
  p00 [label="&quot;응&quot; · &quot;이동해&quot;", {pill("user")}]; p01 [label="&quot;아니&quot;", {pill("user")}];
  pB  [label="&quot;B&quot; · &quot;국그릇으로 가자&quot;", {pill("user")}]; pC [label="&quot;C&quot; · &quot;밥그릇으로 가자&quot;", {pill("user")}];

  e1  [label="① 가는 길 수정 (S → 그릇)\\n직선 경로를 핀치·말로 고침 (그림 2)", {style("user")}, penwidth=2.6];
  a1  [label="&quot;수정 완료&quot; → 쿠옹이: 검증 후 이동할까요?", {style("algo")}];
  q1  [label="사용자 답", {style("dec", dashed=True)}];
  n1  [label="네, 더 고친 뒤\\n수정 완료라고 말씀해 주세요.", {style("algo")}];
  v1  [label="검증 결과", {style("dec", dashed=True)}];
  f1  [label="검증을 통과하지 못했어요 + 이유\\n실패한 곳을 빨간 점으로 → 수정 모드", {style("warn")}];
  mv1 [label="검증을 통과했어요. 이동을 시작할게요.\\n그릇으로 이동 → 도착하면 5 cm 내려가 유지", {style("robot")}];
  arr [label="쿠옹이: B 국그릇에 도착했어요.\\n돌아오는 길을 수정할까요, 가던 길 그대로 반대로 돌아갈까요?", {style("algo")}];
  q2  [label="사용자 답", {style("dec", dashed=True)}];
  w2  [label="네, 여기 있을게요. (대기)", {style("algo")}];
  p10 [label="&quot;응&quot;", {pill("user")}]; p11 [label="&quot;아니&quot;", {pill("user")}];
  pf1 [label="실패", {pill()}]; pp1 [label="통과", {pill()}];
  p20 [label="&quot;수정할래&quot;", {pill("user")}]; p21 [label="&quot;반대로 돌아가&quot;", {pill("user")}]; p22 [label="&quot;아니&quot;", {pill("user")}];

  e2  [label="② 돌아오는 길 수정 (그릇 → S)\\n간 길을 거꾸로 보여 주고 고침 (그림 2)", {style("user")}, penwidth=2.6];
  a2  [label="&quot;수정 완료&quot; → 검증 후 이동할까요? → &quot;응&quot;", {style("algo")}];
  v2  [label="검증 결과", {style("dec", dashed=True)}];
  f2  [label="검증을 통과하지 못했어요 + 이유\\n빨간 점 → 수정 모드", {style("warn")}];
  rv  [label="가던 길을 거꾸로 따라 돌아옴\\n(검증 → 이동)", {style("robot")}];
  mv2 [label="S로 이동 → 5 cm 내려가 유지", {style("robot")}];
  pf2 [label="실패", {pill()}]; pp2 [label="통과", {pill()}];
  hm  [label="설거지 바구니로 돌아왔어요.", {style("algo")}];
  rest[label="남은 그릇이 있나?", {style("dec", dashed=True)}];
  qn  [label="쿠옹이: 남은 C 밥그릇으로 갈까요?", {style("algo")}];
  qa  [label="쿠옹이: 두 곳 모두 다녀왔어요. 다시 이동할까요?", {style("algo")}];
  w3  [label="네, 여기 있을게요. (S에서 대기)", {style("algo")}];
  pr0 [label="있음", {pill()}]; pr1 [label="없음", {pill()}];
  pn0 [label="&quot;응, 가자&quot;", {pill("user")}]; pa0 [label="&quot;응, 국그릇으로 가자&quot; (새 한 바퀴)", {pill("user")}];
  pno [label="&quot;아니&quot;", {pill("user")}];

  st  [label=<<B>이동 중 언제든 &quot;멈춰&quot;</B><BR/><FONT POINT-SIZE="14">(화면 ■ · 실물 제어 창 s · 12 V 스위치도 됨)</FONT><BR/>즉시 정지 · 쿠옹이: 멈췄어요.<BR/>5초 뒤: 계속 갈까요, 돌아갈까요?>, {style("stop")}];
  sc  [label=<<B>&quot;계속 가&quot;</B>: 멈춘 자리에서 원래 가던 곳으로 (검증 → 이동)<BR/><FONT POINT-SIZE="14">그릇으로 가던 중 → 그릇 도착 질문 · S로 오던 중 → 바구니로 돌아왔어요</FONT>>, {style("stop")}];
  sb  [label=<<B>&quot;돌아가&quot;</B>: 그 이동을 시작한 자리로 (검증 → 이동)<BR/><FONT POINT-SIZE="14">가는 길이었으면 → ① 가는 길 수정 · 고친 돌아오는 길이었으면 → ② 돌아오는 길 수정<BR/>거꾸로 오던 길이었으면 → 그릇 도착 질문 다시<BR/>(쿠옹이: 시작한 자리로 돌아왔어요. 경로를 더 고치거나 수정 완료라고 말씀해 주세요.)</FONT>>, {style("stop")}];

  s0 -> s1 -> s2 -> q0;
  q0 -> p00 [arrowhead=none]; p00 -> m0 -> atS;
  q0 -> p01 [arrowhead=none]; p01 -> w0; w0 -> q0 [style=dashed, constraint=false];
  atS -> pB [arrowhead=none]; atS -> pC [arrowhead=none]; pB -> e1; pC -> e1;
  e1 -> a1 -> q1;
  q1 -> p10 [arrowhead=none]; p10 -> v1;
  q1 -> p11 [arrowhead=none]; p11 -> n1; n1 -> e1 [style=dashed, constraint=false];
  v1 -> pf1 [arrowhead=none]; pf1 -> f1; f1 -> e1 [style=dashed, constraint=false];
  v1 -> pp1 [arrowhead=none]; pp1 -> mv1 -> arr -> q2;
  q2 -> p20 [arrowhead=none]; p20 -> e2 -> a2 -> v2;
  v2 -> pf2 [arrowhead=none]; pf2 -> f2; f2 -> e2 [style=dashed, constraint=false];
  v2 -> pp2 [arrowhead=none]; pp2 -> mv2 -> hm;
  q2 -> p21 [arrowhead=none]; p21 -> rv -> hm;
  q2 -> p22 [arrowhead=none]; p22 -> w2;
  hm -> rest;
  rest -> pr0 [arrowhead=none]; pr0 -> qn; qn -> pn0 [arrowhead=none]; pn0 -> e1 [style=dashed, constraint=false];
  rest -> pr1 [arrowhead=none]; pr1 -> qa; qa -> pa0 [arrowhead=none]; pa0 -> e1 [style=dashed, constraint=false];
  qn -> pno [arrowhead=none]; qa -> pno [arrowhead=none]; pno -> w3;

  edge [color="{O}", fontcolor="{O}"];
  {{m0 mv1 mv2 rv}} -> st [style=dashed, constraint=false];
  st -> sc; st -> sb;
  sc -> arr [constraint=false]; sc -> hm [constraint=false];
  sb -> e1 [constraint=false]; sb -> e2 [constraint=false]; sb -> arr [constraint=false];

  {{rank=same; st; e1}}
  {{rank=same; sc; v1}}
  {{rank=same; sb; mv1}}
  {{rank=same; p00; p01}}
  {{rank=same; m0; w0}}
  {{rank=same; pB; pC}}
  {{rank=same; p10; p11}}
  {{rank=same; v1; n1}}
  {{rank=same; pf1; pp1}}
  {{rank=same; p20; p21; p22}}
  {{rank=same; e2; rv; w2}}
  {{rank=same; pf2; pp2}}
  {{rank=same; pr0; pr1}}
  {{rank=same; qn; qa}}
}}
"""

if __name__ == "__main__":
    print(dot_png(SRC, "fig3_overall_flow"))
