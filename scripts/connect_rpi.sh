#!/bin/bash
# RPi へ SSH する。接続先は scripts/hosts.env（未作成なら .example）から読む。
#   使い方: scripts/connect_rpi.sh          … SSH_HOSTS を上から順に試す
#           scripts/connect_rpi.sh univ     … UNIV_HOST（大学経由）へ
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
ENV_FILE="$HERE/hosts.env"
[ -f "$ENV_FILE" ] || ENV_FILE="$HERE/hosts.env.example"
# shellcheck disable=SC1090
. "$ENV_FILE"

if [ "${1:-}" = "univ" ]; then
    if [ -z "${UNIV_HOST:-}" ]; then
        echo "UNIV_HOST が未設定。scripts/hosts.env に書いてください。" >&2
        exit 1
    fi
    exec ssh "${RPI_USER}@${UNIV_HOST}"
fi

IFS=',' read -r -a hosts <<< "${SSH_HOSTS}"
for h in "${hosts[@]}"; do
    if ssh -o ConnectTimeout=5 -o BatchMode=yes "${RPI_USER}@${h}" true 2>/dev/null; then
        echo "接続: ${RPI_USER}@${h}"
        exec ssh "${RPI_USER}@${h}"
    fi
    echo "応答なし: ${h}" >&2
done
echo "どのホストにも繋がりません。RPi の電源と IP を確認してください。" >&2
exit 1
