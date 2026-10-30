"""그림 5. 시스템 구성도. 실행: python3 docs/figures/fig5_system.py  (graphviz dot 필요)

근거(모듈 이름 그대로): voice/(audio_io.py, session.py, gateway.py, tools.py, sim_link.py, prompts/system_v8.md),
mujoco_sim/src/pinch_sim/(hand_input.py, hand_depth.py, stations.py, voice_gateway.py, session.py, editing.py, paths.py,
validation.py, webui.py, sim_view.py, arm_link.py), pac_vision/(astra_depth.py, markers.py, cam2base.py), path 편집/pac_path_editor,
manipulator_control_ws/src/manipulator_control/(pinch_traj.py, supervisor.py, panel.py), config/robot.yaml.
AI = 학습된 모델이 판단하는 부분(MediaPipe Hands, ChatKHU 음성 모델). 나머지는 정해진 알고리즘.
"""
from _style import COL, FONT, GRAY, dot_header, dot_png, style


def box(name, title, module, kind, **extra):
    fill, line = COL[kind]
    attrs = "".join(f', {k}="{v}"' for k, v in extra.items())
    return (f'  {name} [label=<<B>{title}</B><BR/><FONT POINT-SIZE="13" COLOR="{GRAY}">{module}</FONT>>, '
            f'style="rounded,filled", fillcolor="{fill}", color="{line}"{attrs}];\n')


DEV = ("#F5F5F5", "#757575")
SRC = f"""digraph G {{
{dot_header("TB", nodesep=0.5, ranksep=0.45)}
  graph [splines=spline];
  labelloc=t; label=<<FONT POINT-SIZE="30"><B>그림 5. 시스템 구성도</B></FONT><BR/> >;
  node [fontname="{FONT}", fontsize=17];

  user [label=<<B>사용자</B><BR/>손 · 말>, shape=box, style="rounded,filled", fillcolor="{COL['user'][0]}", color="{COL['user'][1]}"];
  cam  [label=<<B>뎁스 카메라</B><BR/><FONT POINT-SIZE="13" COLOR="{GRAY}">Orbbec Astra Pro (컬러 + 깊이)<BR/>pac_vision · astra_depth.py</FONT>>, fillcolor="{DEV[0]}", color="{DEV[1]}"];
  mic  [label=<<B>마이크 · 스피커</B><BR/><FONT POINT-SIZE="13" COLOR="{GRAY}">voice/audio_io.py</FONT>>, fillcolor="{DEV[0]}", color="{DEV[1]}"];

{box("hand", "손 인식 (손가락 21개 점)", "MediaPipe Hands · pinch_sim/hand_input.py", "ai")}
{box("hand3d", "손가락 3차원 위치 · 핀치 판정", "깊이 + 카메라 보정 · hand_depth.py · cam2base.py", "algo")}
{box("mark", "마커 읽기", "6·7·8번 = S·B·C 지점 · stations.py<BR/>5번 = 카메라 밀림 감시 · pac_vision/markers.py", "algo")}
{box("llm", "쿠옹이: ChatKHU 실시간 음성 모델", "gemini-3.8-live · voice/session.py · gateway.py<BR/>지시문 prompts/system_v8.md", "ai")}
{box("tools", "말 → 함수 16개", "어디 · 방향 · 거리(2/5/8 cm) · 되묻기<BR/>voice/tools.py · sim_link.py", "algo")}
{box("gw", "음성 ↔ 시뮬 연결", "pinch_sim/voice_gateway.py", "algo")}
{box("path", "경로 만들기 · 수정", "직선 경로 · 도착 후 5 cm 내려가기 · paths.py<BR/>보정장 변형(양 끝 고정) · editing.py · pac_path_editor", "algo")}
{box("valid", "검증 12개 항목 + 시뮬레이션", "도달 · 관절 범위 · 특이점 · 충돌 · 속도 · 토크 …<BR/>MuJoCo · validation.py", "algo")}
{box("ok", "사용자 승인", "&quot;검증 후 이동할까요?&quot; → &quot;응&quot;", "user")}
{box("arm_ok", "무장 확인 (처음 한 번)", "터미널 y 또는 작은 &quot;실물 무장&quot; 창 · supervisor.py · panel.py", "user")}
{box("recheck", "로봇 쪽 재검사", "가동 범위 · 제동점 · 속도 · 가속 · 모델 충돌<BR/>manipulator_control/pinch_traj.py", "algo")}
{box("arm", "실물 로봇팔 이동", "같은 궤적 · 같은 시간표 · XM430 × 6, U2D2<BR/>supervisor.py · arm_link.py", "robot")}
{box("ui", "웹 화면", "카메라 영상 위 경로 · 3차원 패널 · 대화 · 추천 말<BR/>webui.py · sim_view.py", "algo")}

  user -> cam [label=" 손"]; user -> mic [label=" 말"];
  cam -> hand -> hand3d -> path;
  cam -> mark -> path;
  mic -> llm -> tools -> gw -> path;
  path -> valid -> ok -> recheck -> arm;
  arm_ok -> recheck [style=dashed];
  llm -> mic [style=dashed, constraint=false, label=" 음성 답"];
  {{path valid arm}} -> ui [style=dashed, color="{GRAY}"];
  ui -> user [style=dashed, color="{GRAY}", constraint=false, label=" 화면"];

  subgraph cluster_legend {{
    label=<<B>색</B>>; fontsize=16; style="rounded"; color="#BDBDBD";
    l1 [label="AI (학습된 모델)", {style("ai")}, fontsize=14];
    l2 [label="알고리즘 (정해진 규칙)", {style("algo")}, fontsize=14];
    l3 [label="사람의 확인", {style("user")}, fontsize=14];
    l4 [label="실물 로봇", {style("robot")}, fontsize=14];
    {{rank=same; l1; l2; l3; l4}}
  }}
  {{rank=same; hand; mark; llm}}
  {{rank=same; ok; arm_ok}}
  {{rank=same; arm; ui}}
}}
"""

if __name__ == "__main__":
    print(dot_png(SRC, "fig5_system"))
