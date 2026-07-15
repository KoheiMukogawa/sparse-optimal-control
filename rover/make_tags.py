# -*- coding: utf-8 -*-
"""印刷用 AprilTag(36h11) 画像を生成するCLI。

使い方: uv run python rover/make_tags.py [--outdir docs/tags]
床基準タグ id0-3（黒枠15cm目安）とロボット上面タグ id10（12cm目安）を
300dpi相当のPNGで出力する。印刷サイズの精度は計測精度に影響しない
（truth_core はタグ中心と光線交点のみ使用）ので、目安サイズで印刷し、
床タグは「中心位置」を実測して configs/camera_truth.yaml に記入する。
"""
import argparse
from pathlib import Path

import cv2
import numpy as np

from truth_core import TAG_DICT_ID

DICT = cv2.aruco.getPredefinedDictionary(TAG_DICT_ID)
FLOOR_IDS = (0, 1, 2, 3)
FLOOR_SIZE_M = 0.15
ROBOT_ID = 10
ROBOT_SIZE_M = 0.12


def make_tag_image(tag_id, size_m, dpi=300):
    """黒枠 size_m 角のマーカー＋静穏域＋ラベル文字のuint8画像。"""
    px = int(round(size_m / 0.0254 * dpi))
    pad = px // 8  # 静穏域1モジュール
    label_h = pad
    img = np.full((px + 2 * pad + label_h, px + 2 * pad), 255, np.uint8)
    img[pad:pad + px, pad:pad + px] = \
        cv2.aruco.generateImageMarker(DICT, tag_id, px)
    cv2.putText(img, f'tag36h11 id={tag_id} black={size_m * 100:.0f}cm',
                (pad, px + 2 * pad + label_h // 2),
                cv2.FONT_HERSHEY_SIMPLEX, px / 700, 0, 2)
    return img


def main():
    ap = argparse.ArgumentParser(description='印刷用AprilTag生成')
    ap.add_argument('--outdir', default='docs/tags')
    args = ap.parse_args()
    out = Path(args.outdir)
    out.mkdir(parents=True, exist_ok=True)
    for tid in FLOOR_IDS:
        cv2.imwrite(str(out / f'tag_{tid:02d}.png'),
                    make_tag_image(tid, FLOOR_SIZE_M))
    cv2.imwrite(str(out / f'tag_{ROBOT_ID}_robot.png'),
                make_tag_image(ROBOT_ID, ROBOT_SIZE_M))
    print(f'{out}/ に床用 id0-3（15cm）とロボット用 id10（12cm）を出力。'
          '実寸で印刷し、黒枠サイズが概ね合っているか確認する')


if __name__ == '__main__':
    main()
