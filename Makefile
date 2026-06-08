# ============================================================================
# lidar-occlusion-gps  Makefile
# ----------------------------------------------------------------------------
# 使い方:
#   make help                    # ターゲット一覧
#   make all                     # デフォルトdataset(DATASET)で全実行
#   make analyze D=shibuya_0610  # 略称Dでdataset指定（推奨）
#   make all DATASET=shibuya_0610 # フル名指定でも可
#
# 新datasetを追加するときは、命名規則 <location>_<MMDD> を守れば
# 設定変更ゼロで実行できる（例: shibuya_0610）。
#   - bag  : ~/ros2_ws/bag/<MMDD>/<dataset>_bag/*.db3
#   - dump : ~/ros2_ws/dump/<dataset>/
# 命名規則から外れる場合は BAG_DIR=... を引数で上書き可。
# ============================================================================

# ---- 略称(D)とフル名(DATASET)の統合 ----------------------------------------
ifdef D
DATASET := $(D)
endif
DATASET ?= nakaniwa_0522

# ---- パス自動推定 ----------------------------------------------------------
# DATASET="nakaniwa_0522" → MMDD="0522"
MMDD     := $(lastword $(subst _, ,$(DATASET)))
ROOT     := $(HOME)/ros2_ws
DUMP     := $(ROOT)/dump/$(DATASET)
BAG_DIR  ?= $(ROOT)/bag/$(MMDD)/$(DATASET)_bag
BAG      := $(firstword $(wildcard $(BAG_DIR)/*.db3))
OUT      := $(DUMP)/analysis

PY       := python3
SCRIPTS  := scripts
NOTEBOOK := notebooks/plot_notebook.ipynb

# ---- 共通オプション --------------------------------------------------------
PIPELINE_ARGS := --dump $(DUMP) --bag $(BAG) --out $(OUT)

# ============================================================================
# Targets
# ============================================================================
.PHONY: help all analyze compare notebook occ-only show clean check

.DEFAULT_GOAL := help

help:  ## このヘルプを表示
	@echo ""
	@echo "  lidar-occlusion-gps  -  解析パイプライン"
	@echo "  ============================================"
	@echo "  Current DATASET : $(DATASET)"
	@echo ""
	@echo "  Targets:"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
	  | awk 'BEGIN {FS = ":.*?## "}; {printf "    \033[36m%-12s\033[0m %s\n", $$1, $$2}'
	@echo ""
	@echo "  使用例:"
	@echo "    make all                     # デフォルトdatasetで全実行"
	@echo "    make analyze D=shibuya_0610  # 略称Dでdataset切替"
	@echo "    make show                    # 現在の設定を確認"
	@echo ""

all: analyze compare  ## analyze → compare を連続実行

analyze: check  ## ① analysis_pipeline.py 実行（遮蔽率 + GPS誤差 + アライメント）
	@echo "[make] analyze: DATASET=$(DATASET)"
	@mkdir -p $(OUT)
	$(PY) $(SCRIPTS)/analysis_pipeline.py $(PIPELINE_ARGS)

compare: check  ## ② compare_trajectory.py 実行（軌跡PNG + CSV）
	@echo "[make] compare: DATASET=$(DATASET)"
	@mkdir -p $(OUT)
	$(PY) $(SCRIPTS)/compare_trajectory.py --dump $(DUMP) --bag $(BAG) --out $(OUT)

notebook:  ## ③ plot_notebook.ipynb を開く（VS Code）
	@echo "[make] notebook: $(NOTEBOOK)"
	@if command -v code >/dev/null 2>&1; then \
	    code $(NOTEBOOK); \
	else \
	    jupyter notebook $(NOTEBOOK); \
	fi

occ-only: check-dump  ## 遮蔽率のみ計算（GPS/bagなし）
	@echo "[make] occ-only: DATASET=$(DATASET)"
	@mkdir -p $(OUT)
	$(PY) $(SCRIPTS)/analysis_pipeline.py --dump $(DUMP) --out $(OUT) --no-gps

show:  ## 現在のDATASETとパス設定を表示
	@echo ""
	@echo "  DATASET  : $(DATASET)"
	@echo "  MMDD     : $(MMDD)"
	@echo "  DUMP     : $(DUMP)"
	@echo "  BAG_DIR  : $(BAG_DIR)"
	@echo "  BAG      : $(BAG)"
	@echo "  OUT      : $(OUT)"
	@echo ""

clean:  ## analysis/ 配下のCSV・PNGを削除（dataset単位）
	@echo "[make] clean: $(OUT)"
	@if [ -d "$(OUT)" ]; then \
	    rm -fv $(OUT)/*.csv $(OUT)/*.png 2>/dev/null || true; \
	else \
	    echo "  (nothing to clean: $(OUT) does not exist)"; \
	fi

# ---- 内部チェック ----------------------------------------------------------
check-dump:
	@if [ ! -d "$(DUMP)" ]; then \
	    echo "ERROR: dump directory not found: $(DUMP)"; \
	    echo "  → DATASET=$(DATASET) の dump が存在しません。"; \
	    exit 1; \
	fi

check: check-dump
	@if [ -z "$(BAG)" ]; then \
	    echo "ERROR: bag file not found in: $(BAG_DIR)"; \
	    echo "  → 命名規則から外れる場合は BAG_DIR=... で上書きしてください。"; \
	    echo "    例: make analyze D=$(DATASET) BAG_DIR=/path/to/bag_dir"; \
	    exit 1; \
	fi
