"""ChatKHU(BAZE 게이트웨이) 실시간 세션 연결.

1) POST /realtime/sessions/ 로 1회용 토큰(60초)을 받고
2) wss://.../{url}?token= 에 WebSocket으로 붙는다.
세션 요청과 WebSocket 모두 User-Agent를 curl/8.5.0으로 보내야 403을 피한다.
토큰은 한 번만 쓸 수 있으므로 연결할 때마다 새로 받는다.
"""

import json
import os
import urllib.error
import urllib.request

import websockets

from voice import console

BASE = 'https://factchat-cloud.mindlogic.ai'
SESSIONS_PATH = '/v1/gateway/realtime/sessions/'
USER_AGENT = 'curl/8.5.0'


class GatewayError(RuntimeError):
    pass


def api_key() -> str:
    key = os.environ.get('BAZE_API_KEY', '').strip()
    if not key:
        raise GatewayError('환경변수 BAZE_API_KEY가 비어 있습니다.')
    console.register_secret(key)
    return key


def new_session(model: str) -> dict:
    """{"token", "url"}을 돌려준다. 동기 함수이므로 asyncio.to_thread로 부른다."""
    req = urllib.request.Request(
        BASE + SESSIONS_PATH,
        data=json.dumps({'model': model}).encode(),
        headers={'Authorization': 'Bearer ' + api_key(),
                 'Content-Type': 'application/json', 'User-Agent': USER_AGENT},
        method='POST',
    )
    try:
        with urllib.request.urlopen(req, timeout=15) as resp:
            session = json.load(resp)
    except urllib.error.HTTPError as exc:
        body = exc.read().decode(errors='replace')[:300]
        raise GatewayError(console.redact(f'세션 발급 실패 HTTP {exc.code}: {body}')) from None
    except (urllib.error.URLError, TimeoutError) as exc:
        raise GatewayError(console.redact(f'세션 발급 실패: {exc}')) from None
    if not session.get('token') or not session.get('url'):
        raise GatewayError(f'세션 응답에 token/url이 없습니다: {list(session)}')
    return session


async def open_socket(session: dict):
    url = BASE.replace('https', 'wss', 1) + session['url'] + '?token=' + session['token']
    return await websockets.connect(url, max_size=None, user_agent_header=USER_AGENT)
