"""Manual client for the voice gateway WITHOUT Gemini (stdlib only): type command names.

  python -m pinch_sim.app --target-r 0.275 --target-deg 10 --voice-gateway     # terminal 1
  python scripts/voice_text_client.py                                           # terminal 2

Commands: start_edit continue_edit undo_last_edit reset_to_original use_current_path new_target
          return_home stop status   ('q' quits). expected_rev is the latest rev this client has seen,
so a command typed after a button press elsewhere may be refused as STALE_REVISION - that is the guard.
Events from the simulator are printed as they arrive.
"""
from __future__ import annotations

import argparse
import itertools
import json
import sys
import threading
import time

from pinch_sim.voice_gateway import COMMANDS, connect


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--socket")
    ap.add_argument("--script", help="comma separated 'delay_s:command' items instead of stdin")
    a = ap.parse_args()
    conn = connect(a.socket)
    state = {"rev": None}
    lock = threading.Lock()

    def reader():
        try:
            while True:
                msg = json.loads(conn.recv_bytes())
                st = msg.get("status") or {}
                with lock:
                    state["rev"] = st.get("rev", state["rev"])
                if msg["type"] == "result":
                    print(f"<- result {msg['command']}: accepted={msg['accepted']} {msg['reason_code']} "
                          f"{msg['previous_state']}->{msg['state']} rev {msg['rev_before']}->{msg['rev']} "
                          f"{msg.get('detail') or ''}", flush=True)
                elif msg["type"] == "event":
                    print(f"<- event {msg['kind']} [{msg['actor']}] {msg['state']} rev {msg['rev']}: "
                          f"{msg['message'][:100]}", flush=True)
                else:
                    print(f"<- {msg['type']} state={st.get('state')} rev={st.get('rev')}", flush=True)
        except (EOFError, OSError):
            print("<- connection closed", flush=True)

    threading.Thread(target=reader, daemon=True).start()
    ids = itertools.count(1)
    items = [tuple(x.split(":", 1)) for x in a.script.split(",")] if a.script else None
    while True:
        if items is not None:
            if not items:
                time.sleep(2.0)
                break
            delay, line = items.pop(0)
            time.sleep(float(delay))
            print(f"-> {line}", flush=True)
        else:
            line = sys.stdin.readline()
            if not line:
                break
            line = line.strip()
        if line in ("q", "quit"):
            break
        if line not in COMMANDS:
            print(f"unknown; one of {', '.join(COMMANDS)}", flush=True)
            continue
        with lock:
            rev = state["rev"]
        conn.send_bytes(json.dumps({"v": 1, "type": "command", "command": line, "request_id": f"txt-{next(ids)}",
                                    "expected_rev": rev}).encode())
    conn.close()


if __name__ == "__main__":
    main()
