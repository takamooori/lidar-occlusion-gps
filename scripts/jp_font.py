#!/usr/bin/env python3
"""
jp_font.py - matplotlib 日本語フォント自動設定
================================================
環境に日本語フォントがあれば使う。無ければ自動で
japanize-matplotlib を試し、それも無ければ英語フォールバック。

使い方:
  from jp_font import setup_japanese_font, L
  setup_japanese_font()
  ax.set_xlabel(L('遮蔽率', 'Occlusion rate'))

L() はフォントが無いときに第2引数(英語)へ切り替えるヘルパ。
日本語が使える環境では第1引数(日本語)を返す。
"""

import warnings
import matplotlib
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt

_JP_AVAILABLE = False


def setup_japanese_font():
    """日本語フォントを探して設定。成功可否を返す。"""
    global _JP_AVAILABLE

    # 候補フォント（Ubuntu/Mac/Win でよくあるもの）
    candidates = [
        'IPAexGothic', 'IPAGothic', 'Noto Sans CJK JP', 'Noto Sans JP',
        'TakaoGothic', 'VL Gothic', 'Hiragino Sans', 'Yu Gothic',
        'MS Gothic', 'Meiryo',
    ]
    available = {f.name for f in fm.fontManager.ttflist}
    for cand in candidates:
        if cand in available:
            plt.rcParams['font.family'] = cand
            plt.rcParams['axes.unicode_minus'] = False
            _JP_AVAILABLE = True
            return True

    # japanize-matplotlib があれば使う
    try:
        import japanize_matplotlib  # noqa
        _JP_AVAILABLE = True
        return True
    except ImportError:
        pass

    # フォールバック: 英語のみ。警告は抑制。
    warnings.filterwarnings('ignore', category=UserWarning, module='matplotlib')
    _JP_AVAILABLE = False
    return False


def jp_available():
    return _JP_AVAILABLE


def L(jp, en):
    """日本語が使えれば jp、ダメなら en を返すラベルヘルパ。"""
    return jp if _JP_AVAILABLE else en
