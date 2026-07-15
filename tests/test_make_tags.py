# -*- coding: utf-8 -*-
"""make_tags の生成画像がサイズ正しく・検出可能であること。"""
import cv2
import numpy as np

from make_tags import make_tag_image
from truth_core import detect_tags, make_detector


def test_tag_image_size_and_detectable():
    img = make_tag_image(10, 0.12, dpi=300)
    marker_px = int(round(0.12 / 0.0254 * 300))     # 約1417px
    assert img.shape[0] >= marker_px  # 余白・ラベル込みで本体以上
    # 印刷→撮影を模擬: 1/10に縮小しても検出できる
    small = cv2.resize(img, None, fx=0.1, fy=0.1,
                       interpolation=cv2.INTER_AREA)
    det = detect_tags(small, make_detector())
    assert 10 in det
