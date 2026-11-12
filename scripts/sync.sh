#!/bin/bash
# 公開本体リポジトリと非公開 results リポジトリを「まとめて」扱う。
#
# 属PC化の実害は片方を push し忘れることで起きる（実例: 2026-09-17〜11-11 の
# 図 31 点が results 側未コミットで1台にしか無かった）。必ず両方を見ること。
#
#   scripts/sync.sh status           … 2つの差分と ahead/behind を表示
#   scripts/sync.sh pull             … 2つを pull
#   scripts/sync.sh push "メッセージ" … results の未コミットを commit し、2つを push
#
# 本体側は自動 commit しない（コミットメッセージを機械が決めるべきでないため）。
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RES="$ROOT/results"

has_results() { [ -d "$RES/.git" ]; }

show() {
    local dir="$1" name="$2"
    echo "── $name ($dir)"
    git -C "$dir" status --short | head -20
    local n; n=$(git -C "$dir" status --porcelain | wc -l)
    [ "$n" -gt 20 ] && echo "   …ほか $((n - 20)) 件"
    if git -C "$dir" rev-parse --abbrev-ref '@{upstream}' >/dev/null 2>&1; then
        local ab; ab=$(git -C "$dir" rev-list --left-right --count '@{upstream}...HEAD')
        echo "   behind/ahead: $ab"
    else
        echo "   upstream 未設定"
    fi
}

case "${1:-status}" in
status)
    show "$ROOT" "本体（public）"
    if has_results; then show "$RES" "results（private）"
    else echo "── results（private）が無い。docs/環境構築.md の手順で clone すること"; fi
    ;;
pull)
    git -C "$ROOT" pull --ff-only || exit 1
    has_results && { git -C "$RES" pull --ff-only || exit 1; }
    ;;
push)
    msg="${2:-}"
    if has_results && [ -n "$(git -C "$RES" status --porcelain)" ]; then
        if [ -z "$msg" ]; then
            echo "results に未コミットの変更がある。メッセージを渡してください:" >&2
            echo "  scripts/sync.sh push \"図を再生成\"" >&2
            exit 1
        fi
        git -C "$RES" add -A && git -C "$RES" commit -q -m "$msg"
    fi
    git -C "$ROOT" push || exit 1
    has_results && { git -C "$RES" push || exit 1; }
    ;;
*)
    sed -n '2,12p' "$0"
    exit 1
    ;;
esac
