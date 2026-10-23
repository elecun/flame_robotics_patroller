#!/usr/bin/env bash
# API 시험 GUI 설치 버전 관리(설치 폴더 루트에 있는 스크립트)
#
#   manage_releases.sh list              설치된 버전 목록(* = 활성)
#   manage_releases.sh current           활성 버전 id
#   manage_releases.sh activate <id>     활성 버전 전환
#   manage_releases.sh rollback          직전 활성 버전으로 복구
#   manage_releases.sh remove <id>       활성이 아닌 버전 삭제
#
# 전환은 current 링크만 바꾼다. 이미 실행 중인 GUI는 종료·재시작하지 않으며 다음 실행부터 적용된다.
set -euo pipefail

DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd -P)"
HISTORY="$DIR/install-history.log"

die() { printf '[버전 관리 오류] %s\n' "$*" >&2; exit 1; }
say() { printf '[버전 관리] %s\n' "$*"; }
usage() { sed -n '2,10p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }
log_history() { printf '%s\t%s\n' "$(date -Iseconds)" "$*" >> "$HISTORY"; }
active_id() { [ -L "$DIR/current" ] && basename "$(readlink "$DIR/current")" || true; }

grep -qx "component=api-gui" "$DIR/.patrol-robot-install" 2>/dev/null \
  || die "API 시험 GUI 설치 폴더가 아닙니다: $DIR"

valid_id() {
  [[ "$1" =~ ^[0-9A-Za-z][0-9A-Za-z._-]{0,63}$ ]] || die "release id 형식이 잘못됐습니다: $1"
}
lock() {
  [ -n "${PATROL_API_GUI_LOCK_HELD:-}" ] && return 0   # install.sh가 이미 잠금을 잡고 호출한 경우
  exec 9>"$DIR/.install.lock"
  flock -n 9 || die "다른 설치·전환 작업이 진행 중입니다."
}

activate() {
  local id="$1" how="${2:-activate}"
  valid_id "$id"
  [ -f "$DIR/releases/$id/.install-complete" ] || die "설치가 끝난 버전이 아닙니다: $id ('list'로 확인)"
  local prev; prev="$(active_id)"
  if [ "$prev" = "$id" ]; then say "이미 활성 버전입니다: $id"; return 0; fi
  ln -sfn "releases/$id" "$DIR/.current.tmp.$$"
  mv -Tf "$DIR/.current.tmp.$$" "$DIR/current"
  if ! log_history "$how	$id	previous=${prev:--}"; then
    # 이력이 없으면 rollback 경로가 끊기므로 전환을 되돌린다(이전 활성이 없었으면 링크 제거).
    local restored=0
    if [ -n "$prev" ]; then
      ln -sfn "releases/$prev" "$DIR/.current.tmp.$$" && mv -Tf "$DIR/.current.tmp.$$" "$DIR/current" && restored=1
    else
      rm -f -- "$DIR/current" && restored=1
    fi
    rm -f -- "$DIR/.current.tmp.$$"
    [ "$restored" -eq 1 ] || die "이력을 기록하지 못했고 활성 버전도 되돌리지 못했습니다. 현재 활성: $(active_id) (이전: ${prev:-없음}). 이력 파일 쓰기 권한을 확인한 뒤 activate로 맞추세요: $HISTORY"
    die "이력을 기록하지 못해 전환을 취소했습니다(활성 버전: ${prev:-없음} 유지). 이력 파일 쓰기 권한을 확인하세요: $HISTORY"
  fi
  say "활성 버전: $id (이전: ${prev:-없음}). 실행 중인 GUI는 그대로이며 다음 실행부터 적용됩니다."
}

cmd="${1:-}"
case "$cmd" in
  list)
    cur="$(active_id)"
    found=0
    for d in "$DIR"/releases/*/; do
      [ -d "$d" ] || continue
      found=1
      id="$(basename "$d")"
      if [ -f "$d/.install-complete" ]; then state="설치됨"; else state="미완료"; fi
      commit="$(sed -n 's/.*"source_commit": "\([0-9a-f]\{12\}\).*/\1/p' "$d/manifest.json" 2>/dev/null | head -n 1)"
      printf '%s %-40s %-6s commit %s\n' "$([ "$id" = "$cur" ] && echo '*' || echo ' ')" "$id" "$state" "${commit:--}"
    done
    [ "$found" -eq 1 ] || say "설치된 버전이 없습니다."
    ;;
  current)
    cur="$(active_id)"
    [ -n "$cur" ] || die "활성 버전이 없습니다."
    echo "$cur"
    ;;
  activate)
    [ "$#" -eq 2 ] || die "사용법: manage_releases.sh activate <id>"
    lock
    activate "$2"
    ;;
  rollback)
    [ "$#" -eq 1 ] || die "사용법: manage_releases.sh rollback"
    lock
    cur="$(active_id)"
    [ -n "$cur" ] || die "활성 버전이 없습니다."
    # 현재 버전으로 전환한 마지막 기록의 previous가 복구 대상이다.
    prev="$(awk -F'\t' -v cur="$cur" '($2=="activate"||$2=="rollback") && $3==cur {p=$4} END {sub(/^previous=/,"",p); print p}' "$HISTORY" 2>/dev/null || true)"
    [ -n "$prev" ] && [ "$prev" != "-" ] || die "복구할 이전 활성 버전 기록이 없습니다. 'list'로 확인 후 activate <id>를 사용하세요."
    activate "$prev" rollback
    ;;
  remove)
    [ "$#" -eq 2 ] || die "사용법: manage_releases.sh remove <id>"
    lock
    valid_id "$2"
    [ "$2" != "$(active_id)" ] || die "활성 버전은 지울 수 없습니다: $2"
    rel="$DIR/releases/$2"
    [ -d "$rel" ] || die "없는 버전입니다: $2"
    [ -f "$rel/.install-complete" ] || [ -f "$rel/.install-incomplete" ] || die "관리 표식이 없는 폴더라 지우지 않습니다: $rel"
    rm -rf -- "$rel"
    log_history "remove	$2"
    say "삭제했습니다: $2"
    ;;
  -h|--help|"") usage; [ -n "$cmd" ] || exit 2 ;;
  *) die "알 수 없는 명령: $cmd (--help 참고)" ;;
esac
