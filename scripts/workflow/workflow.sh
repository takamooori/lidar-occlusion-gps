#!/usr/bin/env bash
# workflow.sh
# LiDAR遮蔽率研究 データ取得ワークフロー支援スクリプト
#
# 配置: ~/ros2_ws/src/lidar_occlusion_gps/scripts/workflow/workflow.sh
# 使い方: ./workflow.sh save <location>
#
# 例:
#   ./workflow.sh save nakaniwa
#   -> /tmp/dump の中身を ~/ros2_ws/dump/nakaniwa_<MMDD_HHMM>/ に移動

set -euo pipefail

# ===== 設定 =====
DUMP_SRC="/tmp/dump"
DUMP_DST_BASE="${HOME}/ros2_ws/dump"

# ===== 共通関数 =====
usage() {
    cat <<EOF
Usage: $(basename "$0") <command> [args]

Commands:
  save <location>    GLIM dump を ~/ros2_ws/dump/<location>_<MMDD_HHMM>/ へ移動
                     移動後 /tmp/dump は空ディレクトリとして再作成される
  help               このメッセージを表示

Examples:
  $(basename "$0") save nakaniwa
    -> /tmp/dump -> ~/ros2_ws/dump/nakaniwa_$(date +%m%d_%H%M)/

Notes:
  - GLIM を停止してから実行すること（書き込み中だと破損リスク）
  - 同じ分内に二重実行するとエラーになります
EOF
}

cmd_save() {
    if [ $# -lt 1 ]; then
        echo "[ERROR] location 引数が必要です" >&2
        echo "  例: $(basename "$0") save nakaniwa" >&2
        exit 1
    fi
    local location="$1"

    # /tmp/dump 存在チェック
    if [ ! -d "$DUMP_SRC" ]; then
        echo "[ERROR] $DUMP_SRC が存在しません" >&2
        echo "        GLIM が dump を出力していたか確認してください" >&2
        exit 1
    fi

    # /tmp/dump が空でないかチェック
    if [ -z "$(ls -A "$DUMP_SRC" 2>/dev/null)" ]; then
        echo "[ERROR] $DUMP_SRC が空です。保存対象がありません。" >&2
        exit 1
    fi

    # タイムスタンプ生成 (MMDD_HHMM)
    local timestamp
    timestamp=$(date +"%m%d_%H%M")
    local dataset_name="${location}_${timestamp}"
    local dst="${DUMP_DST_BASE}/${dataset_name}"

    # 既存ディレクトリチェック（同分内の二重実行防止）
    if [ -e "$dst" ]; then
        echo "[ERROR] 既に存在: $dst" >&2
        echo "        1分待ってから再実行するか、別名でリネームしてください" >&2
        exit 1
    fi

    # 親ディレクトリ作成
    mkdir -p "$DUMP_DST_BASE"

    # 移動実行
    echo "[INFO] 移動中: $DUMP_SRC -> $dst"
    mv "$DUMP_SRC" "$dst"

    # /tmp/dump を空ディレクトリとして再作成
    mkdir -p "$DUMP_SRC"

    # 結果サマリ
    echo "[OK] 保存完了"
    echo "     データセット名: $dataset_name"
    echo "     保存先:        $dst"
    echo ""
    echo "次の作業:"
    echo "  ls $dst"
}

# ===== メイン =====
if [ $# -lt 1 ]; then
    usage
    exit 1
fi

subcommand="$1"
shift

case "$subcommand" in
    save)
        cmd_save "$@"
        ;;
    help|-h|--help)
        usage
        ;;
    *)
        echo "[ERROR] 未知のコマンド: $subcommand" >&2
        echo "" >&2
        usage >&2
        exit 1
        ;;
esac
