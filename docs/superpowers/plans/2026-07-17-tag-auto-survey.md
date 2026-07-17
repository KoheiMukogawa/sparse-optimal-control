# タグ自動サーベイ実装計画

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 床タグ4枚を毎セッション適当に置き、C270の1コマから floor_tags を自動生成してメジャー実測を廃止する（spec: docs/superpowers/specs/2026-07-17-tag-auto-survey-design.md）。

**Architecture:** 新規 `rover/survey_tags.py` にサーベイ純ロジック＋CLI。タグごとの IPPE_SQUARE で初期解 → 16隅の再投影誤差を scipy.least_squares で一括最適化（拘束: z=0平面・実寸正方形、ゲージ: tag0原点固定・tag0→tag3=+x）→ 規約変換して configs/camera_truth.yaml の floor_tags を書き換え。あわせて `truth_core.solve_camera_pose` にランタイム再投影残差チェックを追加。

**Tech Stack:** Python (uv), OpenCV (cv2.aruco / solvePnP), scipy.optimize.least_squares（全て既存依存）, pytest＋tests/synth_scene.py の合成画像。

## Global Constraints

- 単位はSI（m・rad）。コメント・docstring・エラーメッセージは日本語（既存 rover/ の流儀）
- 新規依存の追加禁止（numpy/opencv/scipy/pyyaml は既存）
- テスト実行は `uv run pytest tests/ -q`（全既存テスト green を維持。実カメラ・実ソケット不使用）
- コミットメッセージは日本語で簡潔に
- tests/ から rover/ へは conftest.py の sys.path 挿入で直接 import できる（`from survey_tags import ...`）
- **合成テストの合格ゲート: floor_tags 復元誤差 ≤1cm**。超える場合は実装を進めず報告（撤退判断は spec 参照）
- 座標規約: +x = tag0中心→tag3中心、+y = +xから反時計回り90°、原点 = tag0中心 + 0.20m·(+y)（→ floor_tags[0] = (0, -0.20) になる）

---

### Task 1: solve_camera_pose にランタイム再投影残差チェック

**Files:**
- Modify: `rover/truth_core.py`（solve_camera_pose、44〜71行）
- Test: `tests/test_truth_core.py`（末尾に追記）

**Interfaces:**
- Consumes: 既存 `solve_camera_pose(floor_tags, detections, K, dist)`、`tests/synth_scene.py` の `look_down_pose / tag_corners3d / render_scene / K_TEST / DIST0`
- Produces: `solve_camera_pose(floor_tags, detections, K, dist, max_residual_px=2.0)`（後方互換。残差超過で CalibError）。残差の定義は**点ごとユークリッド誤差のRMS** `np.sqrt(np.mean(np.sum((proj - img)**2, axis=1)))` — Task 3 でも同じ定義を使う

- [ ] **Step 1: 失敗するテストを書く**

`tests/test_truth_core.py` の末尾に追記:

```python
def test_solve_camera_pose_rejects_stale_floor_tags():
    """floor_tags が実配置とズレている（タグが動いた）と CalibError。"""
    from synth_scene import (K_TEST, DIST0, look_down_pose, tag_corners3d,
                             render_scene)
    from truth_core import (CalibError, detect_tags, make_detector,
                            solve_camera_pose)
    import numpy as np
    import pytest
    true_tags = {0: (0.0, -0.2), 1: (0.43, 0.4), 2: (1.13, 1.15),
                 3: (1.19, -0.3)}
    rvec, tvec = look_down_pose()
    img = render_scene((1280, 720), K_TEST, DIST0, rvec, tvec,
                       [(tid, tag_corners3d(xy, 0.15))
                        for tid, xy in true_tags.items()])
    det = detect_tags(img, make_detector())
    # 正しい座標なら通る
    solve_camera_pose(true_tags, det, K_TEST, DIST0)
    # tag2 が10cm動いた古いyaml相当 → 拒否
    stale = dict(true_tags, **{2: (1.23, 1.15)})
    with pytest.raises(CalibError):
        solve_camera_pose(stale, det, K_TEST, DIST0)
```

- [ ] **Step 2: 失敗を確認**

Run: `uv run pytest tests/test_truth_core.py::test_solve_camera_pose_rejects_stale_floor_tags -q`
Expected: FAIL（CalibError が raise されず `DID NOT RAISE`）

- [ ] **Step 3: 実装**

`rover/truth_core.py` の `solve_camera_pose` を変更。シグネチャに `max_residual_px=2.0` を追加し、`return rvec, tvec` の直前に挿入:

```python
    proj, _ = cv2.projectPoints(
        np.asarray(obj, dtype=np.float64), rvec, tvec, K, dist)
    rms = float(np.sqrt(np.mean(
        np.sum((proj.reshape(-1, 2) - np.asarray(img)) ** 2, axis=1))))
    if rms > max_residual_px:
        raise CalibError(
            f'床タグ再投影残差 {rms:.1f}px（>{max_residual_px}px）: '
            'タグが動いた/floor_tags が古い疑い。survey_tags.py を再実行')
```

docstring にも「残差 max_residual_px 超で CalibError（タグ移動・yaml陳腐化の検出）」を1行追記。

- [ ] **Step 4: 新テストと全既存テストが通ることを確認**

Run: `uv run pytest tests/ -q`
Expected: 全pass（既存の正しい設定のテストは残差が小さく閾値2pxに掛からない）。もし既存テストが掛かる場合は閾値でなくそのテストのシーン設定を疑うこと（正しい座標で2px超は検出品質の問題）。

- [ ] **Step 5: コミット**

```bash
git add rover/truth_core.py tests/test_truth_core.py
git commit -m "solve_camera_poseに再投影残差チェック: タグ移動・古いfloor_tagsを黙って通さない"
```

---

### Task 2: サーベイ初期解（タグ単体PnP → 中間フレーム）

**Files:**
- Create: `rover/survey_tags.py`
- Test: `tests/test_survey_tags.py`（新規）

**Interfaces:**
- Consumes: `truth_core.CalibError / detect_tags / make_detector`
- Produces（Task 3以降が使う）:
  - `FLOOR_IDS = (0, 1, 2, 3)`, `ORIGIN_OFFSET_M = 0.20`
  - `square_corners3d(center_xy, size_m, yaw=0.0) -> np.ndarray(4,3)`（正準順[左上,右上,右下,左下]、z=0）
  - `initial_guess(det, size_m, K, dist) -> (rvec0, tvec0, tags0)`。`det`: `{tag_id: corners(4,2)}`（4枚必須）、`tags0`: `{tag_id: (x, y, yaw)}` 中間フレーム（tag0=(0,0)固定・tag3のy=0）、`rvec0/tvec0`: 世界→カメラ

- [ ] **Step 1: 失敗するテストを書く**

`tests/test_survey_tags.py` を新規作成:

```python
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
```

- [ ] **Step 2: 失敗を確認**

Run: `uv run pytest tests/test_survey_tags.py -q`
Expected: FAIL（`ModuleNotFoundError: survey_tags`）

- [ ] **Step 3: 実装**

`rover/survey_tags.py` を新規作成:

```python
# -*- coding: utf-8 -*-
"""床タグ自動サーベイ: 1コマから floor_tags を推定し yaml を更新する。

毎セッション、タグを適当に置いて本CLIを1回実行すればメジャー実測が不要になる。
アルゴリズム: タグごと IPPE_SQUARE で初期解 → 全16隅の再投影誤差を
一括最小化（拘束: 床平面z=0・実寸正方形、ゲージ: tag0原点・tag0→tag3=+x）。
設計: docs/superpowers/specs/2026-07-17-tag-auto-survey-design.md
"""
import numpy as np
import cv2

from truth_core import CalibError

FLOOR_IDS = (0, 1, 2, 3)
ORIGIN_OFFSET_M = 0.20   # 原点 = tag0中心 + 0.20m·(+y)


def square_corners3d(center_xy, size_m, yaw=0.0):
    """床置きタグの四隅3D座標（正準順[左上,右上,右下,左下]、z=0）。"""
    h = size_m / 2
    u = np.array([np.cos(yaw), np.sin(yaw)])
    v = np.array([-np.sin(yaw), np.cos(yaw)])
    c = np.asarray(center_xy, dtype=np.float64)
    pts2 = [c - h * u + h * v, c + h * u + h * v,
            c + h * u - h * v, c - h * u - h * v]
    return np.array([[p[0], p[1], 0.0] for p in pts2])


def _tag_pose_ippe(corners, size_m, K, dist):
    """タグ1枚の姿勢（タグ正準系→カメラ系）。初期値用途。"""
    h = size_m / 2
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]],
                   dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, np.asarray(corners, dtype=np.float64),
                                  K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        raise CalibError('タグ単体PnP失敗（検出品質を疑う）')
    return rvec, tvec


def initial_guess(det, size_m, K, dist):
    """タグごとPnP → 床平面フィット → 中間フレームの初期値。

    中間フレーム: tag0中心=原点、tag0→tag3の平面内方向=+x、法線=+z。
    返り値: (rvec0, tvec0, tags0)。tags0={id: (x, y, yaw)}、tag3のyは厳密0。
    """
    missing = [t for t in FLOOR_IDS if t not in det]
    if missing:
        raise CalibError(f'床タグ未検出: id={missing}（4枚必要）')
    centers, xaxes = {}, {}
    for tid in FLOOR_IDS:
        rvec, tvec = _tag_pose_ippe(det[tid], size_m, K, dist)
        R, _ = cv2.Rodrigues(rvec)
        centers[tid] = np.asarray(tvec).flatten()
        xaxes[tid] = R[:, 0]
    # 4中心に平面フィット（SVD）。法線はカメラ原点側へ向ける＝世界+z（上）
    P = np.array([centers[t] for t in FLOOR_IDS])
    mid = P.mean(axis=0)
    _, _, Vt = np.linalg.svd(P - mid)
    n = Vt[2]
    if n @ (-mid) < 0:
        n = -n
    x = centers[3] - centers[0]
    x = x - (x @ n) * n
    nx = np.linalg.norm(x)
    if nx < 0.3:
        raise CalibError('tag0とtag3が近すぎる（0.3m以上離して置く）')
    x = x / nx
    y = np.cross(n, x)
    R_cw = np.stack([x, y, n], axis=1)   # 世界軸をカメラ系で表した列
    rvec0, _ = cv2.Rodrigues(R_cw)
    tvec0 = centers[0].reshape(3, 1)     # p_c = R_cw @ p_w + tvec0
    tags0 = {}
    for tid in FLOOR_IDS:
        pw = R_cw.T @ (centers[tid] - centers[0])
        ax = R_cw.T @ xaxes[tid]
        tags0[tid] = (float(pw[0]), float(pw[1]),
                      float(np.arctan2(ax[1], ax[0])))
    tags0[0] = (0.0, 0.0, tags0[0][2])
    tags0[3] = (tags0[3][0], 0.0, tags0[3][2])
    return rvec0, tvec0, tags0
```

- [ ] **Step 4: テストが通ることを確認**

Run: `uv run pytest tests/test_survey_tags.py -q`
Expected: 2 passed

- [ ] **Step 5: コミット**

```bash
git add rover/survey_tags.py tests/test_survey_tags.py
git commit -m "survey_tags: タグ単体PnPと床平面フィットによる初期解"
```

---

### Task 3: 16隅一括最適化（精度ゲート ≤1cm）

**Files:**
- Modify: `rover/survey_tags.py`
- Test: `tests/test_survey_tags.py`（追記）

**Interfaces:**
- Consumes: Task 2 の `initial_guess / square_corners3d / FLOOR_IDS`
- Produces: `refine(det, size_m, K, dist) -> (tags_xy, rms_px)`。`tags_xy`: `{id: (x, y)}` 中間フレーム（tag0=(0,0)・tag3のy=0）、`rms_px`: 点ごとユークリッド誤差のRMS（Task 1 と同定義）。RMS > `MAX_RMS_PX=1.0` で CalibError

- [ ] **Step 1: 失敗するテストを書く（これが1cmゲート）**

`tests/test_survey_tags.py` に追記:

```python
def test_refine_recovers_tags_within_1cm_across_conditions():
    """合成の統計ゲート: カメラ高さ・傾き・配置を振って復元誤差≤1cm。"""
    from survey_tags import refine
    layouts = [
        TRUE,
        {0: (0.1, -0.25, -0.2), 1: (0.5, 0.5, 0.4), 2: (1.2, 1.2, 0.0),
         3: (1.25, -0.2, 0.1)},
    ]
    cams = [dict(), dict(height=2.0, roll_deg=6.0),
            dict(cam_xy=(0.6, 0.9), height=2.2)]
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
    det[2] = det[2] + np.array([8.0, 0.0])   # tag2 を8px平行移動＝矛盾注入
    with pytest.raises(CalibError):
        refine(det, SIZE, K_TEST, DIST0)
```

- [ ] **Step 2: 失敗を確認**

Run: `uv run pytest tests/test_survey_tags.py -q`
Expected: 新規2件が FAIL（`ImportError: refine`）、既存2件は pass

- [ ] **Step 3: 実装**

`rover/survey_tags.py` に追記（import に `from scipy.optimize import least_squares` を追加）:

```python
MAX_RMS_PX = 1.0


def _unpack(p):
    """最適化パラメータ→(rvec, tvec, tags)。ゲージはtag0=(0,0)・tag3のy=0。"""
    tags = {0: (0.0, 0.0, p[6]),
            1: (p[7], p[8], p[9]),
            2: (p[10], p[11], p[12]),
            3: (p[13], 0.0, p[14])}
    return p[:3], p[3:6].reshape(3, 1), tags


def _residuals(p, det, size_m, K, dist):
    rvec, tvec, tags = _unpack(p)
    res = []
    for tid in FLOOR_IDS:
        x, y, yaw = tags[tid]
        proj, _ = cv2.projectPoints(square_corners3d((x, y), size_m, yaw),
                                    rvec, tvec, K, dist)
        res.append((proj.reshape(4, 2) - det[tid]).ravel())
    return np.concatenate(res)


def refine(det, size_m, K, dist):
    """16隅の一括最適化。返り値 ({id: (x, y)} 中間フレーム, 再投影RMS[px])。

    単体PnPの奥行きノイズを床平面拘束＋16点同時フィットで抑える。
    RMS > MAX_RMS_PX なら CalibError（配置・照明・キャリブずれの疑い）。
    """
    rvec0, tvec0, tags0 = initial_guess(det, size_m, K, dist)
    p0 = np.concatenate([
        np.asarray(rvec0).ravel(), np.asarray(tvec0).ravel(),
        [tags0[0][2]],
        [tags0[1][0], tags0[1][1], tags0[1][2]],
        [tags0[2][0], tags0[2][1], tags0[2][2]],
        [tags0[3][0], tags0[3][2]]])
    sol = least_squares(_residuals, p0, args=(det, size_m, K, dist),
                        method='lm')
    err = sol.fun.reshape(-1, 2)
    rms = float(np.sqrt(np.mean(np.sum(err ** 2, axis=1))))
    if rms > MAX_RMS_PX:
        raise CalibError(
            f'サーベイ再投影残差 {rms:.2f}px（>{MAX_RMS_PX}px）: '
            'タグの平坦性・照明・キャリブを疑う。置き直して再実行')
    _, _, tags = _unpack(sol.x)
    return {tid: (float(t[0]), float(t[1]))
            for tid, t in tags.items()}, rms
```

- [ ] **Step 4: テストが通ることを確認（1cmゲート判定）**

Run: `uv run pytest tests/test_survey_tags.py -q`
Expected: 4 passed。**`test_refine_recovers_tags_within_1cm_across_conditions` が FAIL した場合は先へ進まず、最悪誤差の値を添えて報告する**（spec の撤退基準判断: 複数フレーム平均の追加 → 案A撤退）。

- [ ] **Step 5: コミット**

```bash
git add rover/survey_tags.py tests/test_survey_tags.py
git commit -m "survey_tags: 16隅一括最適化で床タグ座標を復元（合成1cmゲート合格）"
```

---

### Task 4: 座標規約・安全チェック・yaml更新

**Files:**
- Modify: `rover/survey_tags.py`
- Modify: `configs/camera_truth.yaml`（floor_tag_size_m 追加）
- Test: `tests/test_survey_tags.py`（追記）

**Interfaces:**
- Consumes: Task 3 の `refine`
- Produces:
  - `to_course_frame(tags_xy) -> {id: (x, y)}`（原点=tag0+0.20m·(+y) へ平行移動＝y から ORIGIN_OFFSET_M を引く）
  - `check_layout(tags_xy, waypoints)`（共線・面積・コース内包。違反は CalibError、メッセージにどのタグをどちらへ動かすか含める）
  - `update_yaml(path, tags_xy, rms_px, when=None)`（floor_tags ブロックのみ置換・他キーとコメント保持）

- [ ] **Step 1: 失敗するテストを書く**

`tests/test_survey_tags.py` に追記:

```python
def test_to_course_frame_offsets_origin():
    from survey_tags import to_course_frame, ORIGIN_OFFSET_M
    tags = {0: (0.0, 0.0), 1: (0.4, 0.6), 2: (1.1, 1.35), 3: (1.2, 0.0)}
    out = to_course_frame(tags)
    assert out[0] == (0.0, -ORIGIN_OFFSET_M)
    assert out[2] == (1.1, 1.35 - ORIGIN_OFFSET_M)


def test_check_layout_rejects_collinear_and_outside_course():
    from survey_tags import check_layout
    L = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]   # path_L_turn_1m と同じ
    ok = {0: (0.0, -0.2), 1: (0.43, 0.4), 2: (1.13, 1.15), 3: (1.19, -0.3)}
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
```

- [ ] **Step 2: 失敗を確認**

Run: `uv run pytest tests/test_survey_tags.py -q`
Expected: 新規3件 FAIL（ImportError）

- [ ] **Step 3: 実装**

`rover/survey_tags.py` に追記（import に `import re`, `import datetime`, `from pathlib import Path` を追加）:

```python
MIN_HULL_AREA_M2 = 0.3


def to_course_frame(tags_xy):
    """中間フレーム→コース座標（原点=tag0中心+0.20m·(+y)）。"""
    return {tid: (float(x), float(y - ORIGIN_OFFSET_M))
            for tid, (x, y) in tags_xy.items()}


def check_layout(tags_xy, waypoints):
    """共線・面積・コース内包チェック。違反は CalibError。

    waypoints: コース座標の経路頂点 [(x, y), ...]（tags_xy と同じ系で渡す）。
    コースが凸包の外＝外挿地帯は実績10-15cm誤差（7/16）のため拒否する。
    """
    hull = cv2.convexHull(
        np.array([tags_xy[t] for t in FLOOR_IDS], dtype=np.float32))
    area = float(cv2.contourArea(hull))
    if area < MIN_HULL_AREA_M2:
        raise CalibError(
            f'床タグ凸包が小さすぎる（{area:.2f}m²<{MIN_HULL_AREA_M2}）: '
            'ほぼ一直線か密集。コースを囲む四隅に広げて置き直す')
    # 経路を5cm刻みでサンプルして全点が凸包内にあるか
    bad = []
    for a, b in zip(waypoints[:-1], waypoints[1:]):
        a, b = np.asarray(a, float), np.asarray(b, float)
        n = max(2, int(np.linalg.norm(b - a) / 0.05) + 1)
        for s in np.linspace(0.0, 1.0, n):
            p = a + s * (b - a)
            if cv2.pointPolygonTest(hull, (float(p[0]), float(p[1])),
                                    False) < 0:
                bad.append(p)
    if bad:
        c = np.mean(bad, axis=0)
        raise CalibError(
            f'コースがタグ凸包の外（外挿地帯・{len(bad)}点、'
            f'中心 x={c[0]:.2f} y={c[1]:.2f} 付近）: '
            'その方向のタグをコースの外側へ動かして囲む配置にする')


def update_yaml(path, tags_xy, rms_px, when=None):
    """configs/camera_truth.yaml の floor_tags ブロックだけ書き換える。"""
    path = Path(path)
    text = path.read_text()
    when = when or datetime.date.today().isoformat()
    block = ['floor_tags:      # id: [x_m, y_m]。survey_tags.py が自動生成',
             f'  # {when} 自動サーベイ（再投影RMS {rms_px:.2f}px）']
    for tid in FLOOR_IDS:
        x, y = tags_xy[tid]
        block.append(f'  {tid}: [{x:.3f}, {y:.3f}]')
    new, n = re.subn(r'floor_tags:.*?(?=\nrobot_tag:)',
                     '\n'.join(block) + '\n', text, flags=re.S)
    if n != 1:
        raise CalibError(f'{path} の floor_tags ブロックを特定できない'
                         '（floor_tags: と robot_tag: の並びが前提）')
    path.write_text(new)
```

- [ ] **Step 4: configs/camera_truth.yaml に floor_tag_size_m を追加**

`floor_tags:` 行の直前に挿入:

```yaml
floor_tag_size_m: 0.150  # 印刷黒枠の実測値[m]。定規で1回測って記入（±0.5mm→1.2m先で4mm）
```

※ 0.150 は仮置き。実機初回に定規実測で上書きする（handoff に記載する）。

- [ ] **Step 5: テストが通ることを確認**

Run: `uv run pytest tests/test_survey_tags.py -q`
Expected: 7 passed

- [ ] **Step 6: コミット**

```bash
git add rover/survey_tags.py tests/test_survey_tags.py configs/camera_truth.yaml
git commit -m "survey_tags: 座標規約変換・配置チェック・yaml自動更新"
```

---

### Task 5: E2E合成テスト（robot_pose への誤差伝播 ≤1cm）

**Files:**
- Test: `tests/test_survey_tags.py`（追記のみ。実装変更なし）

**Interfaces:**
- Consumes: Task 3–4 の `refine / to_course_frame`、既存 `truth_core.solve_camera_pose / robot_pose`

- [ ] **Step 1: E2Eテストを書く**

`tests/test_survey_tags.py` に追記:

```python
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
```

- [ ] **Step 2: テストが通ることを確認**

Run: `uv run pytest tests/test_survey_tags.py -q`
Expected: 8 passed。位置1cm超で FAIL する場合はサーベイ誤差の伝播が原因（Task 3 のゲートと同じ扱いで報告）。
θの規約メモ: robot_true[2] は「中間フレーム＝コース座標系」での機体yaw。to_course_frame は平行移動のみで回転しないため θ の期待値はそのまま。

- [ ] **Step 3: 全テスト実行とコミット**

Run: `uv run pytest tests/ -q`
Expected: 全pass（56本＋新規8本規模）

```bash
git add tests/test_survey_tags.py
git commit -m "survey_tags: robot_poseへの誤差伝播込みE2E合成テスト"
```

---

### Task 6: CLI（カメラ/画像入力・複数フレーム平均・--dry-run）

**Files:**
- Modify: `rover/survey_tags.py`（main / capture 追加）
- Test: `tests/test_survey_tags.py`（追記）

**Interfaces:**
- Consumes: 既存 `truth_live.CameraSource`（`read() -> (t_mono, gray) | None`）、`truth_offline.load_config` は使わない（floor_tags 未確定でも動く必要があるため独自に yaml.safe_load）
- Produces: CLI `uv run python rover/survey_tags.py [--config configs/camera_truth.yaml] [--image <file>] [--frames 5] [--path configs/path_L_turn_1m.yaml] [--dry-run]`
  - `average_detections(dets) -> {id: corners(4,2)}`（複数フレームの検出を平均。全フレームで4枚揃い必須）
  - `run_survey(cfg_path, det, path_file, dry_run) -> {id: (x, y)}`（refine→to_course_frame→check_layout→update_yaml の直列。テストはこれを呼ぶ）

- [ ] **Step 1: 失敗するテストを書く**

`tests/test_survey_tags.py` に追記:

```python
def test_average_detections_requires_all_tags():
    from survey_tags import average_detections
    d1 = make_det(TRUE)
    d2 = {tid: c + 0.5 for tid, c in d1.items()}
    avg = average_detections([d1, d2])
    assert np.allclose(avg[0], d1[0] + 0.25)
    with pytest.raises(CalibError):
        average_detections([d1, {0: d1[0]}])   # 欠けフレームは拒否


def test_run_survey_writes_yaml_and_checks(tmp_path):
    from survey_tags import run_survey
    import shutil, yaml
    p = tmp_path / 'camera_truth.yaml'
    shutil.copy('configs/camera_truth.yaml', p)
    det = make_det(TRUE)
    tags = run_survey(p, det, 'configs/path_L_turn_1m.yaml', dry_run=False)
    cfg = yaml.safe_load(p.read_text())
    for tid in (0, 1, 2, 3):
        assert np.allclose(cfg['floor_tags'][tid], tags[tid], atol=5e-4)
    # dry-run は書き換えない
    before = p.read_text()
    run_survey(p, det, 'configs/path_L_turn_1m.yaml', dry_run=True)
    assert p.read_text() == before
```

※ K_TEST と DIST0 でテストするため、`run_survey` は cfg の `camera.K/dist` を使う。tmp コピーの yaml は C270 の K のままなので、このテストでは cfg の K/dist を K_TEST/DIST0 に書き換えてから渡す必要がある。テスト冒頭に追記:

```python
    cfg0 = yaml.safe_load(p.read_text())
    cfg0['camera']['K'] = [[700.0, 0, 640], [0, 700.0, 360], [0, 0, 1]]
    cfg0['camera']['dist'] = [0, 0, 0, 0, 0]
    p.write_text(yaml.safe_dump(cfg0, allow_unicode=True, sort_keys=False))
```

（safe_dump でコメントは消えるが、このテストで検証するのは run_survey の直列動作。コメント保持は Task 4 の update_yaml テストで担保済み。ただし floor_tags→robot_tag の並び前提は safe_dump でも保たれることに注意——sort_keys=False 必須）

- [ ] **Step 2: 失敗を確認**

Run: `uv run pytest tests/test_survey_tags.py -q`
Expected: 新規2件 FAIL（ImportError）

- [ ] **Step 3: 実装**

`rover/survey_tags.py` に追記（import に `import argparse`, `import yaml` を追加）:

```python
def average_detections(dets):
    """複数フレームの検出を平均（画素ノイズ低減）。全フレーム4枚揃い必須。"""
    for d in dets:
        missing = [t for t in FLOOR_IDS if t not in d]
        if missing:
            raise CalibError(f'フレーム内で床タグ未検出: id={missing}。'
                             '遮蔽・照明を確認して撮り直す')
    return {tid: np.mean([d[tid] for d in dets], axis=0)
            for tid in FLOOR_IDS}


def run_survey(cfg_path, det, path_file, dry_run=False):
    """検出→refine→規約変換→チェック→yaml更新の直列。返り値はコース座標。"""
    cfg = yaml.safe_load(Path(cfg_path).read_text())
    size = cfg.get('floor_tag_size_m')
    if not size:
        raise CalibError('floor_tag_size_m 未記入: 印刷黒枠を定規で1回実測して'
                         ' configs/camera_truth.yaml に記入する')
    K = np.asarray(cfg['camera']['K'], dtype=np.float64)
    dist = np.asarray(cfg['camera']['dist'], dtype=np.float64)
    floor_det = {t: det[t] for t in FLOOR_IDS if t in det}
    tags_mid, rms = refine(floor_det, float(size), K, dist)
    tags = to_course_frame(tags_mid)
    way = yaml.safe_load(Path(path_file).read_text())['waypoints']
    check_layout(tags, [tuple(w) for w in way])
    print(f'サーベイOK: 再投影RMS {rms:.2f}px')
    for tid in FLOOR_IDS:
        print(f'  tag{tid}: ({tags[tid][0]:+.3f}, {tags[tid][1]:+.3f}) m')
    if dry_run:
        print('--dry-run: yaml は更新しない')
    else:
        update_yaml(cfg_path, tags, rms)
        print(f'{cfg_path} の floor_tags を更新した')
    return tags


def _capture_detections(cfg_path, image, frames):
    """--image なら1枚、なければ CameraSource から frames 枚検出する。"""
    detector = None
    from truth_core import make_detector, detect_tags
    detector = make_detector()
    if image:
        gray = cv2.imread(str(image), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            raise CalibError(f'画像を読めない: {image}')
        return [detect_tags(gray, detector)]
    cfg = yaml.safe_load(Path(cfg_path).read_text())
    live = cfg.get('live', {})
    from truth_live import CameraSource
    src = CameraSource(live.get('device', 0), live.get('width', 1280),
                       live.get('height', 720))
    try:
        dets = []
        while len(dets) < frames:
            r = src.read()
            if r is None:
                raise CalibError('カメラからフレームを読めない'
                                 '（usbipd attach を確認）')
            dets.append(detect_tags(r[1], detector))
        return dets
    finally:
        src.release()


def main():
    ap = argparse.ArgumentParser(description='床タグ自動サーベイ')
    ap.add_argument('--config', default='configs/camera_truth.yaml')
    ap.add_argument('--image', default=None,
                    help='静止画から（省略時はライブカメラ）')
    ap.add_argument('--frames', type=int, default=5,
                    help='ライブ時の平均フレーム数')
    ap.add_argument('--path', default='configs/path_L_turn_1m.yaml',
                    help='内包チェックに使うコース')
    ap.add_argument('--dry-run', action='store_true')
    a = ap.parse_args()
    dets = _capture_detections(a.config, a.image, a.frames)
    det = average_detections(dets)
    run_survey(a.config, det, a.path, dry_run=a.dry_run)


if __name__ == '__main__':
    main()
```

- [ ] **Step 4: テストが通ることを確認**

Run: `uv run pytest tests/ -q`
Expected: 全pass

- [ ] **Step 5: コミット**

```bash
git add rover/survey_tags.py tests/test_survey_tags.py
git commit -m "survey_tags: CLI追加（ライブ/画像入力・複数フレーム平均・dry-run）"
```

---

### Task 7: 手順書・handoff 更新

**Files:**
- Modify: `docs/作業記録/全自動バッチ運用手順.md`（「毎セッション」節）
- Modify: `docs/handoff.md`（セッション14に完了を追記）

**Interfaces:** なし（ドキュメントのみ）

- [ ] **Step 1: 手順書の「毎セッション」節を置換**

`docs/作業記録/全自動バッチ運用手順.md` の「## 毎セッション」節の項目3（原点に置く）を以下へ置換し、項目2.5としてサーベイを挿入:

```markdown
## 毎セッション
1. RPi: nav_base 起動（**1つだけ**。多重起動するとモータ指令が競合する）
   `ros2 launch lightrover_ros nav_base.launch.py`
2. RPi: ブリッジ起動（**こちらも毎セッション起動し直す**。laptop側だけ再起動すると
   seq が巻き戻り、ブリッジが全指令を破棄して無言で動かなくなる）
   `source /opt/ros/humble/setup.bash && python3 ~/sparse_control/rover/udp_twist_bridge.py`
3. 床タグ4枚を前回の写真とだいたい同じ配置で置く（コース南側に tag0・tag3、
   北側に tag1・tag2。メジャー・テープ不要）→ サーベイ:
   `uv run python rover/survey_tags.py`
   （残差・配置図を確認。コースが囲めていないとエラーで教えてくれる）
4. ロボットをカメラに写る適当な場所に置く（homing が原点へ自動搬送する）
5. laptop:
   `uv run python rover/run_batch.py configs/batch_<name>.yaml --backend real --auto --outdir results/<日付>_<name>`
   - 開始時に一括許可 [y/N]（全走行数が表示される）
   - 以後は全自動: 走行 → truth_*.csv 保存 → 原点復帰 → 次走行
```

「前提（1回だけ）」に1行追加: `4. 印刷黒枠の実測値を configs/camera_truth.yaml の floor_tag_size_m に記入（定規で1回）`

- [ ] **Step 2: handoff.md 更新**

セッション14の項目に追記: サーベイ実装完了・テスト本数・合成ゲート結果（実測値を記入）・「実機初回は floor_tag_size_m の定規実測と静置照合2点を先にやる」。

- [ ] **Step 3: コミット**

```bash
git add docs/作業記録/全自動バッチ運用手順.md docs/handoff.md
git commit -m "運用手順を自動サーベイ前提に更新"
```

---

## 実機側の残作業（このプランの範囲外・ユーザー同席）

1. `floor_tag_size_m` を定規で実測して記入
2. サーベイ実行 → 静置照合2点（タグ位置など測りやすい既知点にロボット静置、表示値と実測の一致確認）
3. 合格したら Phase 2+3 の初回実機E2E（既存手順の「初回実機E2E」節）へ
