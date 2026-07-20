"""カメラ画角合わせ用ライブプレビュー（WSLg でウィンドウ表示）。

検出したタグに枠とIDを描画。床タグ4枚（id0-3）が緑になれば
サーベイが通る画角。--guide で configs/camera_truth.yaml の floor_tags
（前回サーベイ合格値）の位置を×印でライブ表示（tag0/tag3 を基準に毎秒更新）。
×印にタグを合わせれば前回の合格配置が再現できる。q で終了。

実行: uv run python rover/preview_camera.py [--guide]
"""
import argparse
import time

import cv2
import numpy as np
import yaml
from pathlib import Path

from truth_core import make_detector, detect_tags, tag_center

FLOOR_IDS = {0, 1, 2, 3}


def guide_homography(dets, size_m, K, dist):
    """検出中の床タグ4枚から コース座標→画素 のホモグラフィを推定。

    refine はタグ4枚必須（tag0/tag3 が軸を定義）。タグを動かしている
    最中は None を返し、ガイドは一時的に消える。"""
    from survey_tags import refine, to_course_frame
    if any(t not in dets for t in FLOOR_IDS):
        return None
    have = {t: dets[t] for t in FLOOR_IDS}
    try:
        tags_mid, _ = refine(have, size_m, K, dist)
    except Exception:
        return None
    tags = to_course_frame(tags_mid)
    src = np.array([tags[t] for t in sorted(FLOOR_IDS)], np.float64)
    dst = np.array([tag_center(dets[t]) for t in sorted(FLOOR_IDS)],
                   np.float64)
    H, _ = cv2.findHomography(src, dst)
    return H


def project(H, p):
    v = H @ np.array([p[0], p[1], 1.0])
    return int(round(v[0] / v[2])), int(round(v[1] / v[2]))


def draw_guide(frame, H, targets, course):
    """前回合格の floor_tags 位置（×印）とコース線を描画。"""
    for a, b in zip(course[:-1], course[1:]):
        cv2.line(frame, project(H, a), project(H, b), (0, 200, 255), 1)
    for tid, p in targets.items():
        x, y = project(H, p)
        inside = 0 <= x < frame.shape[1] and 0 <= y < frame.shape[0]
        color = (0, 0, 255)
        if inside:
            cv2.drawMarker(frame, (x, y), color, cv2.MARKER_TILTED_CROSS,
                           24, 2)
            cv2.putText(frame, f"t{tid}", (x + 8, y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        else:  # 画面外: 端にクランプして矢印代わりの縁マーカー
            cx = int(np.clip(x, 10, frame.shape[1] - 10))
            cy = int(np.clip(y, 10, frame.shape[0] - 10))
            cv2.drawMarker(frame, (cx, cy), color, cv2.MARKER_TRIANGLE_DOWN,
                           18, 2)
            cv2.putText(frame, f"t{tid} OUT", (cx - 30, cy - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--guide", action="store_true",
                    help="前回サーベイ合格の floor_tags 位置を×印で表示")
    ap.add_argument("--config", default="configs/camera_truth.yaml")
    ap.add_argument("--path", default="configs/path_L_turn_1m.yaml")
    a = ap.parse_args()
    guide_cfg = None
    if a.guide:
        cfg = yaml.safe_load(Path(a.config).read_text())
        guide_cfg = dict(
            targets={int(t): tuple(p) for t, p in cfg["floor_tags"].items()},
            size_m=float(cfg["floor_tag_size_m"]),
            K=np.asarray(cfg["camera"]["K"], np.float64),
            dist=np.asarray(cfg["camera"]["dist"], np.float64),
            course=[tuple(w) for w in
                    yaml.safe_load(Path(a.path).read_text())["waypoints"]])
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    det = make_detector()
    print("q で終了。床タグ4枚（id0-3）が全部緑になればOK")
    H = None
    last_solve = 0.0
    while True:
        ok, frame = cap.read()
        if not ok:
            print("フレーム取得失敗")
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = detect_tags(gray, det)
        if guide_cfg and time.time() - last_solve > 1.0:
            last_solve = time.time()
            H2 = guide_homography(tags, guide_cfg["size_m"],
                                  guide_cfg["K"], guide_cfg["dist"])
            if H2 is not None:
                H = H2
        if guide_cfg and H is not None:
            draw_guide(frame, H, guide_cfg["targets"], guide_cfg["course"])
        for tid, corners in tags.items():
            color = (0, 255, 0) if tid in FLOOR_IDS else (255, 200, 0)
            pts = corners.astype(int)
            cv2.polylines(frame, [pts], True, color, 2)
            cx, cy = pts.mean(axis=0).astype(int)
            cv2.putText(frame, f"id{tid}", (cx - 15, cy - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        missing = FLOOR_IDS - tags.keys()
        msg = "ALL FLOOR TAGS OK" if not missing else f"missing: {sorted(missing)}"
        cv2.putText(frame, msg, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0) if not missing else (0, 0, 255), 2)
        cv2.imshow("camera preview (q to quit)", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
