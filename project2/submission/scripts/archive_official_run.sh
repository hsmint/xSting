#!/usr/bin/env bash
# ==============================================================================
# xSting MK2 — 공식 경기 회차별(1회차 / 2회차) 로그 분리 아카이빙 및 비밀정보 검사 스크립트
# 사용법:
#   bash submission/scripts/archive_official_run.sh <1|2> <~/xsting_logs/run_YYYYMMDD_HHMMSS>
# 특징:
#   - 1회차(round_1)와 2회차(round_2) 폴더를 엄격히 분리해 저장합니다.
#   - 대용량 영상(.avi/.mp4)은 저장소 비대화 방지를 위해 제외하고 외부 링크 기입을 안내합니다.
#   - 복사된 JSON/MD/JSONL 내 API 토큰(sk- 등) 포함 여부를 자동 검사합니다.
# ==============================================================================
set -euo pipefail

if [ "$#" -ne 2 ]; then
  echo "사용법: bash $0 <회차번호: 1 또는 2> <원본로그폴더: ~/xsting_logs/run_YYYYMMDD_HHMMSS>"
  exit 1
fi

ROUND="$1"
SRC_RUN="$2"

if [ "$ROUND" != "1" ] && [ "$ROUND" != "2" ]; then
  echo "[오류] 회차 번호는 반드시 1 또는 2여야 합니다 (입력값: ${ROUND}). 두 회차의 결과를 섞지 마세요."
  exit 1
fi

if [ ! -d "$SRC_RUN" ]; then
  echo "[오류] 원본 로그 디렉토리를 찾을 수 없습니다: ${SRC_RUN}"
  exit 1
fi

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DST_DIR="${WS_DIR}/submission/official_logs/round_${ROUND}"
mkdir -p "${DST_DIR}/images"

# 핵심 판정·검증 로그 복사 (대용량 연속 녹화 .avi/.mp4 제외)
for f in signal.json allocation.json verification.json final_state.json meta.json problem.json report.md mission.jsonl events.jsonl; do
  if [ -f "${SRC_RUN}/${f}" ]; then
    cp -f "${SRC_RUN}/${f}" "${DST_DIR}/${f}"
  fi
done

if [ -f "${SRC_RUN}/media/media_index.json" ]; then
  cp -f "${SRC_RUN}/media/media_index.json" "${DST_DIR}/media_index.json"
fi

if [ -d "${SRC_RUN}/media/images" ]; then
  cp -f "${SRC_RUN}/media/images/"*.jpg "${DST_DIR}/images/" 2>/dev/null || true
fi

# 원본 실행 폴더 이름 및 복사 시각 메타 기록
cat > "${DST_DIR}/archive_source.txt" <<EOF
official_round: ${ROUND}
source_run_basename: $(basename "${SRC_RUN}")
archived_at_kst: $(TZ=Asia/Seoul date '+%Y-%m-%d %H:%M:%S %Z')
note: 대용량 전체 경기 영상(.mp4/.avi)은 외부 공개 링크로 업로드 후 submission/official_match_results.md 의 ${ROUND}회차 항목에 기입하세요.
EOF

# 비밀키(OpenAI API 토큰 등) 혼입 자동 검사
if grep -rnE "sk-[A-Za-z0-9_-]{20,}" "${DST_DIR}" >/dev/null 2>&1; then
  echo "[경고] 복사된 로그에서 API 키 패턴(sk-...)이 감지되었습니다! 제출 전 즉시 마스킹하세요."
  exit 2
fi

echo "[완료] 공식 ${ROUND}회차 로그가 안전하게 분리 저장되었습니다 -> ${DST_DIR}"
ls -la "${DST_DIR}"
