"""화면 공용 _ui.magnifier: 커서가 영상 밖(문구 패널 위)에 있어도 오류 없이 그대로 돌려준다."""
import sys
from pathlib import Path

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import _ui  # noqa: E402


@pytest.mark.parametrize("cursor", [(100, 100), (639, 479), (650, 300), (300, 500), (800, 520), (-5, 10)])
def test_magnifier_cursor_anywhere(cursor):
    img = np.zeros((480, 640, 3), np.uint8)
    out = _ui.magnifier(img.copy(), cursor)
    assert out.shape == img.shape
    inside = 0 <= cursor[0] < 640 and 0 <= cursor[1] < 480
    assert (out.any() if inside else not out.any())    # 안쪽이면 확대경을 그리고, 밖이면 그대로
