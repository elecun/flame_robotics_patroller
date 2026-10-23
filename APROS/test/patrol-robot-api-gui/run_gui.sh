#!/usr/bin/env bash
# 설치된 API 시험 GUI 실행(설치 폴더 루트에 있는 스크립트, 어느 폴더에서 실행해도 된다)
#
#   run_gui.sh [--endpoint tcp://127.0.0.1:5590]     인자는 GUI에 그대로 전달한다.
#
# 서비스를 시작하지 않고, 자동 연결·제어권 획득·모드 변경·이동 명령을 하지 않는다.
# 연결은 창의 '연결' 버튼을 눌렀을 때만 한다. 실제 장비에 연결된 서비스라면 버튼이 장치를 움직인다.
#
# 종료 코드: GUI의 종료 코드, 또는 3 = 설치·화면 문제로 실행하지 못함
set -euo pipefail

DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd -P)"
fail() { printf '[API GUI 실행 오류] %s\n' "$*" >&2; exit 3; }

[ -L "$DIR/current" ] || fail "활성 버전이 없습니다. \"$DIR/manage_releases.sh\" list 로 확인하고 activate <id> 하세요."
REL="$(readlink -f "$DIR/current")"   # 실행 중 버전 전환이 있어도 이 실행은 같은 버전을 쓴다
VPY="$REL/venv/bin/python"
[ -f "$REL/.install-complete" ] && [ -x "$VPY" ] || fail "활성 버전 설치가 손상됐습니다: $REL (install.sh로 다시 설치하거나 rollback 하세요)"
export PYTHONPATH="$REL/app/src${PYTHONPATH:+:$PYTHONPATH}"

help_only=0
for a in "$@"; do case "$a" in -h|--help) help_only=1 ;; esac; done

if [ "$help_only" -eq 0 ]; then
  # SSH처럼 화면 정보가 없는 세션: 이 PC에 로그인한 사용자의 X 화면을 찾아 쓴다.
  if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
    for s in /tmp/.X11-unix/X*; do
      [ -S "$s" ] && [ -O "$s" ] && { export DISPLAY=":${s##*/X}"; break; }
    done
    if [ -n "${DISPLAY:-}" ] && [ -z "${XAUTHORITY:-}" ]; then
      for f in "/run/user/$(id -u)/gdm/Xauthority" /run/user/"$(id -u)"/.mutter-Xwaylandauth.* "$HOME/.Xauthority"; do
        [ -f "$f" ] && { export XAUTHORITY="$f"; break; }
      done
    fi
    [ -n "${DISPLAY:-}" ] && printf '[API GUI] 화면 정보가 없어 이 PC의 화면 DISPLAY=%s 에 띄웁니다.\n' "$DISPLAY" >&2
  fi
  # 창을 만들기 전에 화면 연결만 확인한다(창은 표시하지 않음).
  if ! err="$("$VPY" -c 'import tkinter; r = tkinter.Tk(); r.withdraw(); r.destroy()' 2>&1)"; then
    fail "화면(DISPLAY=${DISPLAY:-없음})에 연결하지 못했습니다: $(printf '%s' "$err" | tail -n 1)
  - 이 PC 데스크톱에 로그인한 상태에서 앱 메뉴나 터미널로 실행하세요.
  - SSH로 실행할 때는 그 PC 화면에 로그인한 같은 사용자여야 하며, 필요하면 DISPLAY=:0 처럼 지정하세요.
  - 내 PC 화면으로 띄우려면 X 서버를 켜고 ssh -X 로 접속하세요."
  fi
fi

cd "$REL/app"
exec "$VPY" "$REL/app/tools/api_test_gui.py" "$@"
