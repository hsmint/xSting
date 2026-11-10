"""과제 '주사위 기억 로봇'(v5)의 고정 값과 신호·바코드 해석.

할당(그룹 배정, 퍽 선택, 하차 순서)은 LLM이 판단한다. 여기에는 관측값을 상태에 적을 때 필요한 것만 둔다.
"""

COLORS = ("R", "Y", "G")  # 신호 색
PUCK_COLORS = ("R", "G")  # 실제 퍽 색 (노란 퍽은 없다)
SHAPES = ("cylinder", "box")
HOME = "HOME"
START_BOXES = tuple(f"S{i}" for i in range(1, 7))
LOCATIONS = ("L1", "L2", "L3")
PLACES = (HOME,) + START_BOXES + LOCATIONS
NUM_PUCKS = 12
NUM_SIGNALS = 12
STORAGE_SLOTS = 12  # 적재함 4x3
NUM_CELLS = 9  # 하차 박스 3x3

# 바코드 → 정렬 방향
BARCODE_ORDER = {"P2-01": "asc", "P2-03": "asc", "P2-02": "desc", "P2-04": "desc"}

# cell 번호. 지도 위쪽이 +Y이고 모든 하차 박스에서 같은 방향이다.
#   0(-60,+60) 1(0,+60) 2(+60,+60)
#   3(-60, 0 ) 4(0, 0 ) 5(+60, 0 )
#   6(-60,-60) 7(0,-60) 8(+60,-60)
def cell_offset_mm(cell):
    return ((cell % 3 - 1) * 60, (1 - cell // 3) * 60)


def count_signal(sequence):
    return {c: sequence.count(c) for c in COLORS}


def signal_errors(sequence, end_seen):
    """관측한 신호가 규칙에 맞는지 검사한다. 빈 리스트면 정상."""
    errors = []
    if len(sequence) != NUM_SIGNALS:
        errors.append(f"SIGNAL_COUNT_{len(sequence)}")
    unknown = [i for i, c in enumerate(sequence) if c not in COLORS]
    if unknown:
        errors.append(f"UNKNOWN_COLOR_AT_{unknown}")
    if not end_seen:
        errors.append("END_DARK_NOT_SEEN")
    if not errors:
        for c, n in count_signal(sequence).items():
            if not 1 <= n <= 6:
                errors.append(f"COUNT_OUT_OF_RANGE_{c}_{n}")
    return errors
