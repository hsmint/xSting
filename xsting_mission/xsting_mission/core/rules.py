"""과제 '주사위 기억 로봇'(v5)의 고정 값과 신호·바코드 해석(할당은 LLM이 한다)."""

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
BARCODE_ORDER = {"P2-01": "asc", "P2-03": "asc",
                 "P2-02": "desc", "P2-04": "desc"}

# cell 번호(지도 위쪽이 +Y, 모든 하차 박스에서 같은 방향, 간격 60mm):
#   0 1 2
#   3 4 5
#   6 7 8
# 주사위 눈 개수 → 쓰는 cell (프롬프트 2절과 같다)
DICE_CELLS = {1: [4], 2: [0, 8], 3: [0, 4, 8], 4: [0, 2, 6, 8],
              5: [0, 2, 4, 6, 8], 6: [0, 3, 6, 2, 5, 8]}


def count_signal(sequence):
    return {color: sequence.count(color) for color in COLORS}


def signal_errors(sequence, end_seen):
    """관측한 신호가 규칙에 맞는지 검사한다. 빈 리스트면 정상."""
    errors = []
    if len(sequence) != NUM_SIGNALS:
        errors.append(f"SIGNAL_COUNT_{len(sequence)}")
    unknown = [index for index, color in enumerate(sequence)
               if color not in COLORS]
    if unknown:
        errors.append(f"UNKNOWN_COLOR_AT_{unknown}")
    if not end_seen:
        errors.append("END_DARK_NOT_SEEN")
    if not errors:
        for color, count in count_signal(sequence).items():
            if not 1 <= count <= 6:
                errors.append(f"COUNT_OUT_OF_RANGE_{color}_{count}")
    return errors
