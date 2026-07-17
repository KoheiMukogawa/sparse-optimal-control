# -*- coding: utf-8 -*-
"""survey_tags のテスト（全て合成画像。実カメラ不使用）。"""
import numpy as np
import pytest

from synth_scene import (K_TEST, DIST0, look_down_pose, tag_corners3d,
                         render_scene)
from truth_core import CalibError, detect_tags, make_detector
from survey_tags import FLOOR_IDS, initial_guess, square_corners3d

SIZE = 0.15
# 7/16実配置ふう。tag0→tag3 が+xに揃わない一般配置
TRUE = {0: (0.0, -0.2, 0.1), 1: (0.43, 0.4, -0.3), 2: (1.13, 1.15, 0.05),
        3: (1.19, -0.3, 0.2)}


def to_intermediate(true_tags):
    """真値配置を中間フレーム（tag0原点・tag0→tag3=+x）へ変換した期待値。"""
    p0 = np.array(true_tags[0][:2])
    d = np.array(true_tags[3][:2]) - p0
    ang = np.arctan2(d[1], d[0])
    R = np.array([[np.cos(-ang), -np.sin(-ang)],
                  [np.sin(-ang), np.cos(-ang)]])
    return {tid: R @ (np.array(v[:2]) - p0) for tid, v in true_tags.items()}


def make_det(true_tags, cam_kw=None):
    rvec, tvec = look_down_pose(**(cam_kw or {}))
    img = render_scene((1280, 720), K_TEST, DIST0, rvec, tvec,
                       [(tid, tag_corners3d(v[:2], SIZE, yaw=v[2]))
                        for tid, v in true_tags.items()])
    det = detect_tags(img, make_detector())
    assert set(FLOOR_IDS) <= set(det), '合成シーンで床タグ4枚が検出できていない'
    return det


def test_square_corners3d_matches_synth_scene():
    got = square_corners3d((0.3, -0.1), SIZE, yaw=0.4)
    want = tag_corners3d((0.3, -0.1), SIZE, yaw=0.4)
    assert np.allclose(got, want)


def test_initial_guess_recovers_layout_roughly():
    det = make_det(TRUE)
    _, _, tags0 = initial_guess(det, SIZE, K_TEST, DIST0)
    want = to_intermediate(TRUE)
    assert tags0[0][:2] == (0.0, 0.0)
    assert abs(tags0[3][1]) < 1e-9          # ゲージ: tag3 は y=0
    for tid in FLOOR_IDS:                    # 初期解は5cm以内でよい
        assert np.linalg.norm(np.array(tags0[tid][:2]) - want[tid]) < 0.05


def test_refine_recovers_tags_within_1cm_across_conditions():
    """合成の統計ゲート: カメラ高さ・傾き・配置を振って復元誤差≤1cm。"""
    from survey_tags import refine
    layouts = [
        TRUE,
        {0: (0.1, -0.25, -0.2), 1: (0.5, 0.5, 0.4), 2: (1.2, 1.2, 0.0),
         3: (1.25, -0.2, 0.1)},
    ]
    cams = [dict(), dict(height=2.0, roll_deg=6.0),
            dict(cam_xy=(0.6, 0.9), height=2.4)]
    rng = np.random.default_rng(0)
    worst = 0.0
    for true_tags in layouts:
        want = to_intermediate(true_tags)
        for kw in cams:
            det = make_det(true_tags, cam_kw=kw)
            # レンダリングノイズに加え±0.3pxの隅ノイズも1ケースずつ（spec準拠）
            noisy = {t: c + rng.normal(0, 0.3, c.shape) for t, c in det.items()}
            for d in (det, noisy):
                tags_xy, rms = refine(d, SIZE, K_TEST, DIST0)
                assert rms < 1.0, f'再投影RMS {rms:.2f}px'
                for tid in FLOOR_IDS:
                    e = np.linalg.norm(np.array(tags_xy[tid]) - want[tid])
                    worst = max(worst, e)
    assert worst < 0.01, f'最悪復元誤差 {worst*100:.2f}cm（ゲート1cm）'


def test_refine_rejects_high_residual():
    """検出corner を人工的に壊すと CalibError（黙って通さない）。"""
    from survey_tags import refine
    det = make_det(TRUE)
    det[2] = det[2] + np.array([[8.0, 0.0], [-8.0, 0.0], [0.0, 8.0], [0.0, -8.0]])   # tag2 隅ごとに矛盾注入
    with pytest.raises(CalibError):
        refine(det, SIZE, K_TEST, DIST0)


def test_to_course_frame_offsets_origin():
    from survey_tags import to_course_frame, ORIGIN_OFFSET_M
    tags = {0: (0.0, 0.0), 1: (0.4, 0.6), 2: (1.1, 1.35), 3: (1.2, 0.0)}
    out = to_course_frame(tags)
    assert out[0] == (0.0, -ORIGIN_OFFSET_M)
    assert out[2] == (1.1, 1.35 - ORIGIN_OFFSET_M)


def test_check_layout_rejects_collinear_and_outside_course():
    from survey_tags import check_layout
    L = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]   # path_L_turn_1m と同じ
    ok = {0: (0.0, -0.3), 1: (-0.1, 0.5), 2: (1.2, 1.2), 3: (1.25, -0.35)}
    check_layout(ok, L)                          # 例外なし
    collinear = {0: (0.0, 0.0), 1: (0.4, 0.01), 2: (0.8, 0.0),
                 3: (1.2, 0.01)}
    with pytest.raises(CalibError):
        check_layout(collinear, L)
    onesided = {0: (0.0, -0.2), 1: (0.5, -0.3), 2: (0.9, 0.4),
                3: (1.2, -0.25)}                 # ゴール(1,1)が凸包の外
    with pytest.raises(CalibError):
        check_layout(onesided, L)


def test_update_yaml_replaces_only_floor_tags(tmp_path):
    from survey_tags import update_yaml
    import shutil, yaml
    p = tmp_path / 'camera_truth.yaml'
    shutil.copy('configs/camera_truth.yaml', p)
    tags = {0: (0.0, -0.2), 1: (0.43, 0.4), 2: (1.13, 1.15), 3: (1.19, -0.3)}
    update_yaml(p, tags, rms_px=0.42, when='2026-07-17')
    cfg = yaml.safe_load(p.read_text())
    assert cfg['floor_tags'][2] == [1.13, 1.15]
    assert cfg['robot_tag']['z_m'] == 0.12       # 他キーは無傷
    assert cfg['camera']['image_size'] == [1280, 720]
    assert '2026-07-17' in p.read_text()          # 日付コメント


def test_surveyed_tags_give_robot_pose_within_1cm():
    """サーベイ結果→solve_camera_pose→robot_pose の伝播込みE2E。"""
    from survey_tags import refine, to_course_frame
    from truth_core import robot_pose, solve_camera_pose
    Z_ROBOT = 0.12
    robot_true = (0.5, 0.3, 0.7)   # 中間フレーム基準の真値
    # シーン: 床タグ4枚＋ロボットタグid10（中間フレームで直接定義）
    inter = to_intermediate(TRUE)
    tags_scene = {tid: (inter[tid][0], inter[tid][1], TRUE[tid][2])
                  for tid in FLOOR_IDS}
    rvec, tvec = look_down_pose(cam_xy=(0.6, 0.55))
    img = render_scene(
        (1280, 720), K_TEST, DIST0, rvec, tvec,
        [(tid, tag_corners3d(v[:2], SIZE, yaw=v[2]))
         for tid, v in tags_scene.items()] +
        [(10, tag_corners3d(robot_true[:2], 0.12, yaw=robot_true[2],
                            z=Z_ROBOT))])
    det = detect_tags(img, make_detector())
    # サーベイ（床タグのみ渡す）→ コース座標の floor_tags
    floor = to_course_frame(refine(
        {t: det[t] for t in FLOOR_IDS}, SIZE, K_TEST, DIST0)[0])
    # 走行時と同じ経路: サーベイ結果でカメラ姿勢→ロボットpose
    rv, tv = solve_camera_pose(floor, det, K_TEST, DIST0)
    x, y, th = robot_pose(det[10], K_TEST, DIST0, rv, tv, Z_ROBOT)
    from survey_tags import ORIGIN_OFFSET_M
    want = (robot_true[0], robot_true[1] - ORIGIN_OFFSET_M)
    assert np.hypot(x - want[0], y - want[1]) < 0.01
    assert abs(th - robot_true[2]) < 0.02


def test_average_detections_requires_all_tags():
    from survey_tags import average_detections
    d1 = make_det(TRUE)
    d2 = {tid: c + 0.5 for tid, c in d1.items()}
    avg = average_detections([d1, d2])
    assert np.allclose(avg[0], d1[0] + 0.25)
    with pytest.raises(CalibError):
        average_detections([d1, {0: d1[0]}])   # 欠けフレームは拒否
    with pytest.raises(CalibError):
        average_detections([])                  # 空リスト拒否


def test_run_survey_writes_yaml_and_checks(tmp_path):
    from survey_tags import run_survey
    import shutil, yaml
    p = tmp_path / 'camera_truth.yaml'
    shutil.copy('configs/camera_truth.yaml', p)
    cfg0 = yaml.safe_load(p.read_text())
    cfg0['camera']['K'] = [[700.0, 0, 640], [0, 700.0, 360], [0, 0, 1]]
    cfg0['camera']['dist'] = [0, 0, 0, 0, 0]
    p.write_text(yaml.safe_dump(cfg0, allow_unicode=True, sort_keys=False))
    # Use layout that properly encloses path_L_turn_1m: [(0,0), (1,0), (1,1)]
    # in course frame (intermediate + ORIGIN_OFFSET_M shift)
    layout = {0: (0.0, -0.1, 0.1), 1: (-0.1, 0.7, -0.3), 2: (1.2, 1.4, 0.05),
              3: (1.25, -0.15, 0.2)}
    det = make_det(layout)
    tags = run_survey(p, det, 'configs/path_L_turn_1m.yaml', dry_run=False)
    cfg = yaml.safe_load(p.read_text())
    for tid in (0, 1, 2, 3):
        assert np.allclose(cfg['floor_tags'][tid], tags[tid], atol=5e-4)
    # dry-run は書き換えない
    before = p.read_text()
    run_survey(p, det, 'configs/path_L_turn_1m.yaml', dry_run=True)
    assert p.read_text() == before
