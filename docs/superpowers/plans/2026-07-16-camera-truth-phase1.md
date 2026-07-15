# カメラ真値パイプライン Phase 1 実装計画

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** iPhone俯瞰動画から走行真値 (t,x,y,θ) CSVを出す検出コア＋オフラインCLIを作り、静置照合で±1〜2cm精度を検証できる状態にする。

**Architecture:** `truth_core.py` はI/Oなしの純ロジック（cv2.aruco で AprilTag 36h11 検出→床基準タグ4枚の中心を solvePnP→ロボット上面タグを光線と平面 z=h の交点で厳密にコース座標化）。`truth_offline.py` が動画→CSVのCLI。テストは合成レンダリング画像で±数mm精度を担保。

**Tech Stack:** Python 3.12 / uv / opencv-python（cv2.aruco、AprilTagの検出・画像生成とも）/ numpy / pytest

**Spec:** `docs/superpowers/specs/2026-07-16-camera-truth-pipeline-design.md`

## Global Constraints

- 単位は SI（m・rad）で統一。表示のみ cm・deg（既存コードの流儀）
- コース座標系: 原点=走行原点、x=第1直進方向（経路yamlと同一）
- コメント・docstring・コミットメッセージは日本語、既存ファイルの書式に合わせる
- テストは `uv run pytest tests/ -q`（conftest.py が rover/ を sys.path に追加済み）
- タグ辞書は tag36h11 固定。床基準タグ ID 0〜3、ロボットタグ ID 10
- 精度目標: 合成テストで位置≤1cm・角度≤0.02rad、実写静置照合で±1〜2cm（Phase 1完了条件）

---

### Task 1: 依存追加・設定テンプレート・spec修正

**Files:**
- Modify: `pyproject.toml`（uv経由）
- Create: `configs/camera_truth.yaml`
- Modify: `docs/superpowers/specs/2026-07-16-camera-truth-pipeline-design.md`（依存の行）

**Interfaces:**
- Produces: `configs/camera_truth.yaml`（後続タスクの `load_config()` が読む形式）

- [ ] **Step 1: opencv-python を追加**

Run: `uv add opencv-python`
Expected: pyproject.toml の dependencies に `opencv-python>=4.x` が追加され、`uv run python -c "import cv2; print(cv2.aruco.DICT_APRILTAG_36h11)"` が数値を出力

- [ ] **Step 2: 設定テンプレートを作成**

`configs/camera_truth.yaml`:

```yaml
# カメラ真値計測の設定（設計: specs/2026-07-16-camera-truth-pipeline-design.md）
# K / dist は rover/calibrate_camera.py の出力を貼り付ける。
# floor_tags はタグ中心の実測コース座標 [m]（原点=走行原点, x=第1直進方向）。
# タグの印刷サイズ精度は不要（中心と光線交点のみ使用。サイズは検出可否にだけ効く）。
camera:
  name: iphone_main_1080p   # 使用カメラ・モードを記録（レンズ切替时は要再キャリブ）
  image_size: [1920, 1080]
  K: null        # 3x3 リスト。null のままだと load_config がエラーを出す
  dist: null     # [k1, k2, p1, p2, k3]
floor_tags:      # id: [x_m, y_m]。4枚とも実測して記入
  0: null        # 例: [-0.20, -0.20]
  1: null
  2: null
  3: null
robot_tag:
  id: 10
  z_m: 0.13            # 床→タグ面の高さ（実測して更新）
  yaw_offset_rad: 0.0  # タグ正準+x と機体前方のズレ
```

- [ ] **Step 3: spec の依存行を修正**

`docs/superpowers/specs/2026-07-16-camera-truth-pipeline-design.md` の
`- laptop: \`uv add opencv-python pupil-apriltags\`` を以下に置換:

```markdown
- laptop: `uv add opencv-python`（AprilTagの検出・印刷用画像生成とも cv2.aruco
  の DICT_APRILTAG_36h11 を使用。pupil-apriltags は不要と判明＝依存1つ減）
```

- [ ] **Step 4: コミット**

```bash
git add pyproject.toml uv.lock configs/camera_truth.yaml docs/superpowers/specs/2026-07-16-camera-truth-pipeline-design.md
git commit -m "カメラ真値Phase1: opencv-python追加・設定テンプレ・spec依存修正"
```

---

### Task 2: truth_core 検出関数＋合成シーンヘルパ

**Files:**
- Create: `rover/truth_core.py`
- Create: `tests/synth_scene.py`（テスト専用の合成レンダリング）
- Test: `tests/test_truth_core.py`

**Interfaces:**
- Produces（truth_core）: `make_detector() -> cv2.aruco.ArucoDetector`、
  `detect_tags(gray, detector) -> dict[int, np.ndarray(4,2)]`（corners はマーカー正準向きで[左上,右上,右下,左下]の画素座標）、`tag_center(corners) -> np.ndarray(2,)`、定数 `TAG_DICT_ID`
- Produces（synth_scene）: `look_down_pose(cam_xy, height, roll_deg) -> (rvec, tvec)`、
  `tag_corners3d(center_xy, size_m, yaw=0.0, z=0.0) -> np.ndarray(4,3)`、
  `render_scene(size_px, K, dist, rvec, tvec, tags: list[(id, corners3d)]) -> np.ndarray(H,W)uint8`、
  `K_TEST`（fx=fy=700, cx=640, cy=360）, `DIST0`（ゼロ歪み(5,)）

- [ ] **Step 1: 失敗するテストを書く**

`tests/test_truth_core.py`:

```python
# -*- coding: utf-8 -*-
"""truth_core の合成画像テスト（設計spec: 合成レンダリングで精度担保）。"""
import cv2
import numpy as np

from synth_scene import (DIST0, K_TEST, look_down_pose, render_scene,
                         tag_corners3d)
from truth_core import detect_tags, make_detector, tag_center


def test_render_and_detect_roundtrip():
    """合成画像のタグが正しいIDで検出され、四隅が投影位置と1px以内で一致。"""
    rvec, tvec = look_down_pose()
    c3d = tag_corners3d((0.4, 0.6), 0.15, yaw=0.3)
    img = render_scene((1280, 720), K_TEST, DIST0, rvec, tvec, [(3, c3d)])
    det = detect_tags(img, make_detector())
    assert set(det) == {3}
    proj, _ = cv2.projectPoints(c3d, rvec, tvec, K_TEST, DIST0)
    assert np.allclose(det[3], proj.reshape(4, 2), atol=1.0)


def test_detect_tags_empty_image():
    """タグなし画像 → 空dict（Noneでない）。"""
    img = np.full((720, 1280), 255, np.uint8)
    assert detect_tags(img, make_detector()) == {}


def test_tag_center_is_corner_mean():
    corners = np.array([[0., 0.], [10., 0.], [10., 10.], [0., 10.]])
    assert np.allclose(tag_center(corners), [5.0, 5.0])
```

`tests/synth_scene.py`:

```python
# -*- coding: utf-8 -*-
"""合成シーン: 既知のカメラ姿勢・タグ配置からテスト画像をレンダリングする。

truth_core の精度テスト用。マーカー画像を射影ワープで白背景キャンバスへ
描き込む。歪みゼロ前提（projectPoints と warpPerspective の整合が厳密）。
"""
import cv2
import numpy as np

from truth_core import TAG_DICT_ID

DICT = cv2.aruco.getPredefinedDictionary(TAG_DICT_ID)
K_TEST = np.array([[700.0, 0, 640], [0, 700.0, 360], [0, 0, 1]])
DIST0 = np.zeros(5)
MARKER_PX = 160  # 36h11は8モジュール角 → 20px/モジュール


def look_down_pose(cam_xy=(0.75, 0.75), height=2.4, roll_deg=3.0):
    """ほぼ真下向きカメラの (rvec, tvec)（コース座標→カメラ座標）。

    真下向きはx軸まわりπ回転（カメラz軸=-z_w, y軸=-y_w）、
    roll_deg でわずかな設置傾きを模擬する。
    """
    R, _ = cv2.Rodrigues(np.array([np.pi + np.radians(roll_deg), 0.0, 0.0]))
    C = np.array([cam_xy[0], cam_xy[1], height])
    rvec, _ = cv2.Rodrigues(R)
    tvec = (-R @ C).reshape(3, 1)
    return rvec, tvec


def tag_corners3d(center_xy, size_m, yaw=0.0, z=0.0):
    """床置き(z=0)/ロボット上面(z=h)タグの四隅3D座標（正準順[左上,右上,右下,左下]）。

    タグ正準+x（左辺→右辺）がコースx軸から yaw だけ回転して置かれているとする。
    上から見た正準配置: 左上=(-h,+h), 右上=(+h,+h), 右下=(+h,-h), 左下=(-h,-h)。
    """
    h = size_m / 2
    u = np.array([np.cos(yaw), np.sin(yaw)])   # 正準+x（コース系）
    v = np.array([-np.sin(yaw), np.cos(yaw)])  # 正準+y（コース系, 上から見て）
    c = np.asarray(center_xy, dtype=np.float64)
    pts2 = [c - h * u + h * v, c + h * u + h * v,
            c + h * u - h * v, c - h * u - h * v]
    return np.array([[p[0], p[1], z] for p in pts2])


def render_scene(size_px, K, dist, rvec, tvec, tags):
    """tags: [(tag_id, corners3d(4,3)), ...] を白背景に描画して返す。"""
    img = np.full((size_px[1], size_px[0]), 255, np.uint8)
    pad = MARKER_PX // 8  # 静穏域1モジュール分
    side = MARKER_PX + 2 * pad
    src = np.array([[pad, pad], [pad + MARKER_PX, pad],
                    [pad + MARKER_PX, pad + MARKER_PX],
                    [pad, pad + MARKER_PX]], np.float32)
    for tid, c3d in tags:
        proj, _ = cv2.projectPoints(c3d, rvec, tvec, K, dist)
        proj = proj.reshape(4, 2).astype(np.float32)
        canvas = np.full((side, side), 255, np.uint8)
        canvas[pad:pad + MARKER_PX, pad:pad + MARKER_PX] = \
            cv2.aruco.generateImageMarker(DICT, tid, MARKER_PX)
        # 縮小ワープのエイリアシング対策に軽くぼかす
        canvas = cv2.GaussianBlur(canvas, (5, 5), 1.5)
        H = cv2.getPerspectiveTransform(src, proj)
        warped = cv2.warpPerspective(canvas, H, size_px,
                                     flags=cv2.INTER_LINEAR, borderValue=0)
        mask = cv2.warpPerspective(np.full((side, side), 255, np.uint8), H,
                                   size_px, flags=cv2.INTER_NEAREST)
        img[mask > 127] = warped[mask > 127]
    return img
```

- [ ] **Step 2: テストが失敗することを確認**

Run: `uv run pytest tests/test_truth_core.py -q`
Expected: FAIL（`ModuleNotFoundError: truth_core`）

- [ ] **Step 3: truth_core の検出部を実装**

`rover/truth_core.py`:

```python
# -*- coding: utf-8 -*-
"""カメラ真値計測の純ロジック（I/Oなし）。

AprilTag(36h11) を cv2.aruco で検出し、床基準タグ4枚の中心実測座標から
solvePnP でカメラ外部姿勢を推定、ロボット上面タグ（高さ z_m）を
光線と平面 z=z_m の交点で厳密にコース座標へ変換する。
座標・角度は SI（m・rad）。
設計: docs/superpowers/specs/2026-07-16-camera-truth-pipeline-design.md
"""
import cv2
import numpy as np

TAG_DICT_ID = cv2.aruco.DICT_APRILTAG_36h11


class CalibError(RuntimeError):
    """セッション開始不能（床基準タグ不足・設定不備など）。"""


def make_detector():
    d = cv2.aruco.getPredefinedDictionary(TAG_DICT_ID)
    p = cv2.aruco.DetectorParameters()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    return cv2.aruco.ArucoDetector(d, p)


def detect_tags(gray, detector):
    """グレースケール画像 → {tag_id: corners(4,2)float64}。

    corners はマーカー正準向きで [左上, 右上, 右下, 左下] の画素座標
    （ビットパターンの復号で決まるため、カメラの回転には依存しない）。
    """
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return {}
    return {int(i): c.reshape(4, 2).astype(np.float64)
            for i, c in zip(ids.flatten(), corners)}


def tag_center(corners):
    return corners.mean(axis=0)
```

- [ ] **Step 4: テストが通ることを確認**

Run: `uv run pytest tests/test_truth_core.py -q`
Expected: 3 passed
（検出四隅の1px一致が落ちる場合は synth_scene の MARKER_PX / ぼかし量を調整）

- [ ] **Step 5: コミット**

```bash
git add rover/truth_core.py tests/synth_scene.py tests/test_truth_core.py
git commit -m "truth_core: AprilTag検出＋合成シーンテスト基盤"
```

---

### Task 3: truth_core カメラ外部姿勢（solvePnP）

**Files:**
- Modify: `rover/truth_core.py`
- Test: `tests/test_truth_core.py`

**Interfaces:**
- Consumes: Task 2 の `detect_tags` / `tag_center` / synth_scene 一式
- Produces: `solve_camera_pose(floor_tags: dict[int, tuple], detections, K, dist) -> (rvec, tvec)`（コース→カメラ変換。床タグ4枚未満で `CalibError`）、
  `camera_center(rvec, tvec) -> np.ndarray(3,)`（カメラ中心のコース座標）

- [ ] **Step 1: 失敗するテストを追加**

`tests/test_truth_core.py` に追記:

```python
FLOOR_TAGS = {0: (-0.20, -0.20), 1: (1.70, -0.20),
              2: (1.70, 1.70), 3: (-0.20, 1.70)}


def _floor_scene(rvec, tvec):
    tags = [(tid, tag_corners3d(xy, 0.15, yaw=0.1 * tid))
            for tid, xy in FLOOR_TAGS.items()]
    return render_scene((1280, 720), K_TEST, DIST0, rvec, tvec, tags)


def test_solve_camera_pose_recovers_camera_center():
    """床タグ4枚からカメラ中心 (0.75,0.75,2.4) を2cm以内で復元。"""
    from truth_core import camera_center, solve_camera_pose
    rvec, tvec = look_down_pose()
    det = detect_tags(_floor_scene(rvec, tvec), make_detector())
    rv, tv = solve_camera_pose(FLOOR_TAGS, det, K_TEST, DIST0)
    assert np.allclose(camera_center(rv, tv), [0.75, 0.75, 2.4], atol=0.02)


def test_solve_camera_pose_needs_4_tags():
    """3枚しか写らないと CalibError。"""
    import pytest
    from truth_core import CalibError, solve_camera_pose
    rvec, tvec = look_down_pose()
    det = detect_tags(_floor_scene(rvec, tvec), make_detector())
    det.pop(0)
    with pytest.raises(CalibError):
        solve_camera_pose(FLOOR_TAGS, det, K_TEST, DIST0)
```

- [ ] **Step 2: テストが失敗することを確認**

Run: `uv run pytest tests/test_truth_core.py -q`
Expected: 2 failed（ImportError: solve_camera_pose）, 3 passed

- [ ] **Step 3: 実装**

`rover/truth_core.py` に追記:

```python
def solve_camera_pose(floor_tags, detections, K, dist):
    """床基準タグ中心（コース座標z=0）と検出中心から solvePnP。

    floor_tags: {id: (x_m, y_m)}。検出できた共通タグが4枚未満なら CalibError。
    返り値 (rvec, tvec): コース座標→カメラ座標の剛体変換。
    中心のみ使う（タグの向き・印刷サイズの実測が不要になるため）。
    """
    obj, img = [], []
    for tid, xy in floor_tags.items():
        if tid in detections:
            obj.append([xy[0], xy[1], 0.0])
            img.append(tag_center(detections[tid]))
    if len(obj) < 4:
        raise CalibError(f'床基準タグ検出 {len(obj)}/4 枚（4枚必要）')
    # 正対面付近の平面PnPは IPPE だと画素ノイズがcm級に増幅されるため、
    # ホモグラフィ初期化＋LM精緻化の ITERATIVE を使う（4点共面でOK）
    ok, rvec, tvec = cv2.solvePnP(
        np.asarray(obj, dtype=np.float64), np.asarray(img, dtype=np.float64),
        K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise CalibError('solvePnP 失敗')
    return rvec, tvec


def camera_center(rvec, tvec):
    """カメラ中心のコース座標（設置ズレ検知にも使う）。"""
    R, _ = cv2.Rodrigues(rvec)
    return (-R.T @ np.asarray(tvec).reshape(3, 1)).flatten()
```

- [ ] **Step 4: テストが通ることを確認**

Run: `uv run pytest tests/test_truth_core.py -q`
Expected: 5 passed

- [ ] **Step 5: コミット**

```bash
git add rover/truth_core.py tests/test_truth_core.py
git commit -m "truth_core: 床タグsolvePnPでカメラ外部姿勢を推定"
```

---

### Task 4: truth_core ロボットpose（視差補正つき）

**Files:**
- Modify: `rover/truth_core.py`
- Test: `tests/test_truth_core.py`

**Interfaces:**
- Consumes: Task 3 の `solve_camera_pose` / `camera_center`
- Produces: `pixel_to_plane(pt(2,), K, dist, rvec, tvec, z) -> np.ndarray(2,)`、
  `robot_pose(corners(4,2), K, dist, rvec, tvec, z, yaw_offset=0.0) -> (x, y, theta)`（float3つ、θは[-π,π]）

- [ ] **Step 1: 失敗するテストを追加**

`tests/test_truth_core.py` に追記:

```python
def _full_scene(robot_xy, robot_yaw, z=0.13):
    """床タグ4枚＋ロボットタグ(id10, 12cm, 高さz)のシーンと真のカメラ姿勢。"""
    rvec, tvec = look_down_pose()
    tags = [(tid, tag_corners3d(xy, 0.15, yaw=0.1 * tid))
            for tid, xy in FLOOR_TAGS.items()]
    tags.append((10, tag_corners3d(robot_xy, 0.12, yaw=robot_yaw, z=z)))
    img = render_scene((1280, 720), K_TEST, DIST0, rvec, tvec, tags)
    return img, rvec, tvec


def test_robot_pose_accuracy():
    """コース中央と端の複数姿勢で 位置≤1cm・角度≤0.02rad。"""
    from truth_core import robot_pose, solve_camera_pose
    cases = [((0.75, 0.75), 0.0), ((1.50, 0.20), 2.0), ((0.10, 1.40), -2.5)]
    for xy, yaw in cases:
        img, _, _ = _full_scene(xy, yaw)
        det = detect_tags(img, make_detector())
        rv, tv = solve_camera_pose(FLOOR_TAGS, det, K_TEST, DIST0)
        x, y, th = robot_pose(det[10], K_TEST, DIST0, rv, tv, 0.13)
        assert np.hypot(x - xy[0], y - xy[1]) < 0.01, (xy, yaw)
        dth = (th - yaw + np.pi) % (2 * np.pi) - np.pi
        assert abs(dth) < 0.02, (xy, yaw)


def test_parallax_correction_is_necessary():
    """z=0 のホモグラフィ扱いだとコース端で3cm超ズレる（補正の必要性の担保）。"""
    from truth_core import pixel_to_plane, solve_camera_pose, tag_center
    img, _, _ = _full_scene((1.50, 0.20), 0.0)
    det = detect_tags(img, make_detector())
    rv, tv = solve_camera_pose(FLOOR_TAGS, det, K_TEST, DIST0)
    c_px = tag_center(det[10])
    p_ok = pixel_to_plane(c_px, K_TEST, DIST0, rv, tv, 0.13)
    p_naive = pixel_to_plane(c_px, K_TEST, DIST0, rv, tv, 0.0)
    assert np.linalg.norm(p_ok - p_naive) > 0.03
```

- [ ] **Step 2: テストが失敗することを確認**

Run: `uv run pytest tests/test_truth_core.py -q`
Expected: 2 failed（ImportError: robot_pose / pixel_to_plane）, 5 passed

- [ ] **Step 3: 実装**

`rover/truth_core.py` に追記:

```python
def pixel_to_plane(pt, K, dist, rvec, tvec, z):
    """画素座標 pt を高さ z[m] の水平面へ逆投影（光線と平面の交点）。

    ロボット上面タグは床から z_m 浮いているため、床ホモグラフィだと
    コース端で数cmの視差誤差が出る。カメラ姿勢既知なので厳密に解く。
    """
    xn = cv2.undistortPoints(
        np.asarray(pt, dtype=np.float64).reshape(1, 1, 2), K, dist)[0, 0]
    R, _ = cv2.Rodrigues(rvec)
    cam = (-R.T @ np.asarray(tvec).reshape(3, 1)).flatten()
    ray = R.T @ np.array([xn[0], xn[1], 1.0])
    s = (z - cam[2]) / ray[2]
    return (cam + s * ray)[:2]


def robot_pose(corners, K, dist, rvec, tvec, z, yaw_offset=0.0):
    """ロボット上面タグ corners → (x, y, theta) コース座標。

    θ はタグ正準+x方向（左辺中点→右辺中点）を平面 z へ投影して算出し、
    yaw_offset（タグ+xと機体前方のズレ）を差し引く。θは[-π,π]。
    """
    c = pixel_to_plane(tag_center(corners), K, dist, rvec, tvec, z)
    left = pixel_to_plane((corners[0] + corners[3]) / 2,
                          K, dist, rvec, tvec, z)
    right = pixel_to_plane((corners[1] + corners[2]) / 2,
                           K, dist, rvec, tvec, z)
    d = right - left
    theta = np.arctan2(d[1], d[0]) - yaw_offset
    theta = (theta + np.pi) % (2 * np.pi) - np.pi
    return float(c[0]), float(c[1]), float(theta)
```

- [ ] **Step 4: テストが通ることを確認**

Run: `uv run pytest tests/test_truth_core.py -q`
Expected: 7 passed

- [ ] **Step 5: 全テスト＋コミット**

Run: `uv run pytest tests/ -q` → Expected: 全pass（既存24＋新7）

```bash
git add rover/truth_core.py tests/test_truth_core.py
git commit -m "truth_core: 視差補正つきロボットpose算出（合成テストで≤1cm/0.02rad）"
```

---

### Task 5: カメラ内部パラメータのキャリブレーションCLI

**Files:**
- Create: `rover/calibrate_camera.py`
- Test: `tests/test_calibrate_camera.py`

**Interfaces:**
- Consumes: なし（独立ユーティリティ）
- Produces: `calibrate(grays: list[np.ndarray], cols=9, rows=6, square_m=0.024) -> (K(3,3), dist(5,), err_px)`（チェスボード内側コーナー cols×rows。有効フレーム3枚未満で ValueError）。CLI: `uv run python rover/calibrate_camera.py <video> [--cols 9 --rows 6 --square-m 0.024 --max-frames 30]` → yaml貼り付け用の K / dist を標準出力

- [ ] **Step 1: 失敗するテストを書く**

`tests/test_calibrate_camera.py`:

```python
# -*- coding: utf-8 -*-
"""calibrate_camera の合成チェスボードテスト。"""
import cv2
import numpy as np

from calibrate_camera import calibrate
from synth_scene import DIST0, K_TEST


def _board_img(cols=9, rows=6, sq=60, margin=80):
    """内側コーナー cols×rows の市松ビットマップ（白余白つき）。"""
    pattern = (np.indices((rows + 1, cols + 1)).sum(axis=0) % 2)
    img = np.kron(pattern, np.ones((sq, sq))) * 255
    return cv2.copyMakeBorder(img.astype(np.uint8), margin, margin,
                              margin, margin, cv2.BORDER_CONSTANT, value=255)


def _render_views(n=8, cols=9, rows=6, sq=60, margin=80):
    """ボードを複数姿勢で射影した合成ビュー群（歪みゼロ・K_TEST）。"""
    board = _board_img(cols, rows, sq, margin)
    h, w = board.shape
    # ビットマップ四隅の3D座標: 1マス=0.024m とし、余白も同スケールで換算
    s = 0.024 / sq
    obj4 = np.array([[-margin * s, -margin * s, 0],
                     [(w - margin) * s, -margin * s, 0],
                     [(w - margin) * s, (h - margin) * s, 0],
                     [-margin * s, (h - margin) * s, 0]])
    src = np.array([[0, 0], [w, 0], [w, h], [0, h]], np.float32)
    rng = np.random.default_rng(0)
    views = []
    for _ in range(n):
        rvec = rng.uniform(-0.3, 0.3, 3) + np.array([np.pi, 0, 0])
        tvec = np.array([[0.05], [0.1], [0.9]]) + rng.uniform(-0.03, 0.03, (3, 1))
        # ボード原点をカメラ正面に置く: コース系は使わず board系→cam 直指定
        proj, _ = cv2.projectPoints(obj4, rvec, tvec, K_TEST, DIST0)
        H = cv2.getPerspectiveTransform(src, proj.reshape(4, 2).astype(np.float32))
        views.append(cv2.warpPerspective(board, H, (1280, 720),
                                         borderValue=255))
    return views


def test_calibrate_recovers_intrinsics():
    K, dist, err = calibrate(_render_views(), cols=9, rows=6, square_m=0.024)
    assert abs(K[0, 0] - 700) / 700 < 0.02
    assert abs(K[1, 1] - 700) / 700 < 0.02
    assert err < 1.0
    assert np.all(np.abs(dist) < 0.05)


def test_calibrate_rejects_too_few():
    import pytest
    with pytest.raises(ValueError):
        calibrate([np.full((720, 1280), 255, np.uint8)] * 3)
```

注: `rvec = [π,0,0]±0.3` はボードをカメラへ正対（表向き）させる回転。tvec でカメラ前方 0.9m に置く。

- [ ] **Step 2: テストが失敗することを確認**

Run: `uv run pytest tests/test_calibrate_camera.py -q`
Expected: FAIL（ModuleNotFoundError: calibrate_camera）

- [ ] **Step 3: 実装**

`rover/calibrate_camera.py`:

```python
# -*- coding: utf-8 -*-
"""チェスボード動画からカメラ内部パラメータ (K, dist) を求めるCLI。

使い方:
  uv run python rover/calibrate_camera.py <チェスボード動画>
      [--cols 9] [--rows 6] [--square-m 0.024] [--max-frames 30]
出力: 再投影誤差と、configs/camera_truth.yaml に貼る K / dist のyaml片。
チェスボードは9x6内側コーナー（10x7マス）をA4印刷し、動画は
カメラを固定・ボードを傾けながら全域で撮る（本番と同じ解像度設定で）。
"""
import argparse

import cv2
import numpy as np


def calibrate(grays, cols=9, rows=6, square_m=0.024):
    """グレースケール画像群 → (K, dist(5,), 再投影誤差px)。有効<3枚でValueError。"""
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_m
    obj_pts, img_pts = [], []
    size = None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    for g in grays:
        size = (g.shape[1], g.shape[0])
        ok, corners = cv2.findChessboardCorners(g, (cols, rows))
        if not ok:
            continue
        corners = cv2.cornerSubPix(g, corners, (11, 11), (-1, -1), crit)
        obj_pts.append(objp)
        img_pts.append(corners)
    if len(obj_pts) < 3:
        raise ValueError(f'チェスボード検出 {len(obj_pts)} 枚（3枚以上必要）')
    err, K, dist, _, _ = cv2.calibrateCamera(obj_pts, img_pts, size,
                                             None, None)
    return K, dist.flatten()[:5], err


def main():
    ap = argparse.ArgumentParser(description='カメラ内部パラメータのキャリブ')
    ap.add_argument('video')
    ap.add_argument('--cols', type=int, default=9)
    ap.add_argument('--rows', type=int, default=6)
    ap.add_argument('--square-m', type=float, default=0.024)
    ap.add_argument('--max-frames', type=int, default=30)
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.video)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    step = max(1, total // args.max_frames)
    grays = []
    i = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if i % step == 0:
            grays.append(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        i += 1
    cap.release()

    K, dist, err = calibrate(grays, args.cols, args.rows, args.square_m)
    print(f'フレーム {len(grays)} 枚使用 / 再投影誤差 {err:.3f} px'
          f'（0.5px以下が目安）')
    print('--- configs/camera_truth.yaml に貼り付け ---')
    print(f'  image_size: [{int(2 * K[0, 2])}, {int(2 * K[1, 2])}]  # 概算・撮影解像度を確認')
    print('  K:')
    for row in K:
        print(f'    - [{row[0]:.2f}, {row[1]:.2f}, {row[2]:.2f}]')
    print(f'  dist: [{", ".join(f"{d:.5f}" for d in dist)}]')


if __name__ == '__main__':
    main()
```

- [ ] **Step 4: テストが通ることを確認**

Run: `uv run pytest tests/test_calibrate_camera.py -q`
Expected: 2 passed

- [ ] **Step 5: コミット**

```bash
git add rover/calibrate_camera.py tests/test_calibrate_camera.py
git commit -m "カメラ内部パラメータのキャリブレーションCLI（合成チェスボードテスト付き）"
```

---

### Task 6: 印刷用タグ画像の生成CLI

**Files:**
- Create: `rover/make_tags.py`
- Test: `tests/test_make_tags.py`

**Interfaces:**
- Consumes: `truth_core.TAG_DICT_ID`
- Produces: CLI `uv run python rover/make_tags.py [--outdir docs/tags]` → `tag_00.png`〜`tag_03.png`（床用・目安15cm）と `tag_10_robot.png`（ロボット用・目安12cm）。300dpi相当のピクセルサイズ＋ID/サイズのラベル文字入り。関数 `make_tag_image(tag_id, size_m, dpi=300) -> np.ndarray`

- [ ] **Step 1: 失敗するテストを書く**

`tests/test_make_tags.py`:

```python
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
```

- [ ] **Step 2: テストが失敗することを確認**

Run: `uv run pytest tests/test_make_tags.py -q`
Expected: FAIL（ModuleNotFoundError: make_tags）

- [ ] **Step 3: 実装**

`rover/make_tags.py`:

```python
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
```

- [ ] **Step 4: テストが通ることを確認＋生成物を確認**

Run: `uv run pytest tests/test_make_tags.py -q` → Expected: 1 passed
Run: `uv run python rover/make_tags.py && ls docs/tags/`
Expected: tag_00.png tag_01.png tag_02.png tag_03.png tag_10_robot.png

- [ ] **Step 5: コミット**

```bash
git add rover/make_tags.py tests/test_make_tags.py docs/tags/
git commit -m "印刷用AprilTag生成CLI（床4枚＋ロボット1枚）"
```

---

### Task 7: truth_offline（動画→真値CSV）

**Files:**
- Create: `rover/truth_offline.py`
- Test: `tests/test_truth_offline.py`

**Interfaces:**
- Consumes: truth_core 全関数、configs/camera_truth.yaml の形式
- Produces:
  - `load_config(path) -> dict(K, dist, floor_tags, robot)`（K/dist/floor_tags が null なら CalibError にメッセージ付きで）
  - `run_video(frames_fn, cfg, calib_frames=30) -> (rows, info)`
    - `frames_fn`: 呼ぶたびに `(t_s, gray)` のイテレータを返す callable（2パス用）
    - `rows`: `[(t, x, y, theta, n_tags, quality_px) ...]` 欠測フレームは x,y,theta=None
    - `info`: `dict(cam_center, cam_drift_m, n_frames, n_valid)`
  - CLI: `uv run python rover/truth_offline.py <video> --config configs/camera_truth.yaml [--out CSV] [--path-file configs/path_L_turn.yaml]` → CSV（ヘッダ `t_s,x_m,y_m,theta_rad,n_tags,quality_px`）＋終点サマリ（cm/deg表示）

- [ ] **Step 1: 失敗するテストを書く**

`tests/test_truth_offline.py`:

```python
# -*- coding: utf-8 -*-
"""truth_offline: 合成フレーム列 → 真値rows の検証（動画コーデック非依存）。"""
import numpy as np

from synth_scene import (DIST0, K_TEST, look_down_pose, render_scene,
                         tag_corners3d)
from test_truth_core import FLOOR_TAGS
from truth_offline import run_video

CFG = dict(K=K_TEST, dist=DIST0,
           floor_tags=FLOOR_TAGS,
           robot=dict(id=10, z_m=0.13, yaw_offset_rad=0.0))


def _traj_frames(n=40, drop=(15, 16)):
    """直線移動するロボットの合成フレーム列と真値。dropはロボットタグ無し。"""
    rvec, tvec = look_down_pose()
    floor = [(tid, tag_corners3d(xy, 0.15, yaw=0.1 * tid))
             for tid, xy in FLOOR_TAGS.items()]
    frames, truth = [], []
    for i in range(n):
        x = 0.2 + 0.02 * i
        pose = ((x, 0.5), 0.1)   # (中心xy, yaw)
        tags = list(floor)
        if i not in drop:
            tags.append((10, tag_corners3d(pose[0], 0.12,
                                           yaw=pose[1], z=0.13)))
        frames.append((i / 30.0,
                       render_scene((1280, 720), K_TEST, DIST0,
                                    rvec, tvec, tags)))
        truth.append(pose)
    return frames, truth


def test_run_video_tracks_trajectory():
    frames, truth = _traj_frames()
    rows, info = run_video(lambda: iter(frames), CFG, calib_frames=5)
    assert info['n_frames'] == 40 and info['n_valid'] == 38
    assert info['cam_drift_m'] < 0.01
    for row, ((tx, ty), tyaw) in zip(rows, truth):
        t, x, y, th, n_tags, q = row
        if x is None:
            continue
        assert np.hypot(x - tx, y - ty) < 0.01
        assert abs((th - tyaw + np.pi) % (2 * np.pi) - np.pi) < 0.02


def test_run_video_marks_missing():
    frames, _ = _traj_frames()
    rows, _ = run_video(lambda: iter(frames), CFG, calib_frames=5)
    assert rows[15][1] is None and rows[16][1] is None
```

- [ ] **Step 2: テストが失敗することを確認**

Run: `uv run pytest tests/test_truth_offline.py -q`
Expected: FAIL（ModuleNotFoundError: truth_offline）

- [ ] **Step 3: 実装**

`rover/truth_offline.py`:

```python
# -*- coding: utf-8 -*-
"""俯瞰動画 → コース座標の真値CSV (t,x,y,theta) を出すオフラインCLI。

使い方:
  uv run python rover/truth_offline.py <動画> --config configs/camera_truth.yaml
      [--out 出力CSV] [--calib-frames 30] [--path-file configs/path_L_turn.yaml]

処理: 1パス目で床基準タグ4枚の中心を calib_frames 枚ぶん平均し
solvePnP（カメラは固定前提）。2パス目で全フレームのロボットタグを
視差補正つきでコース座標化してCSVへ。末尾30フレームで床タグから
カメラ姿勢を再推定し、1cm超ズレていたら「カメラが動いた」警告を出す。
iPhoneは「カメラ設定→フォーマット→互換性優先(H.264)」での撮影を推奨。
"""
import argparse
import csv
import math
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import yaml

from truth_core import (CalibError, camera_center, detect_tags,
                        make_detector, robot_pose, solve_camera_pose,
                        tag_center)


def load_config(path):
    """camera_truth.yaml → dict(K, dist, floor_tags, robot)。未記入を検知。"""
    with open(path) as f:
        cfg = yaml.safe_load(f)
    cam = cfg['camera']
    if cam.get('K') is None or cam.get('dist') is None:
        raise CalibError('camera.K/dist が未記入。rover/calibrate_camera.py '
                         'の出力を貼り付けること')
    floor = cfg['floor_tags']
    if any(v is None for v in floor.values()):
        raise CalibError('floor_tags に未記入あり。タグ中心のコース座標[m]を実測して記入')
    return dict(K=np.asarray(cam['K'], dtype=np.float64),
                dist=np.asarray(cam['dist'], dtype=np.float64),
                floor_tags={int(k): tuple(v) for k, v in floor.items()},
                robot=cfg['robot_tag'])


def _solve_from_avg(per_tag, cfg):
    """{tid: [中心px,...]} を平均して solvePnP。擬似検出dictを作って渡す。"""
    det = {tid: np.tile(np.mean(pts, axis=0), (4, 1))
           for tid, pts in per_tag.items()}
    return solve_camera_pose(cfg['floor_tags'], det, cfg['K'], cfg['dist'])


def run_video(frames_fn, cfg, calib_frames=30):
    """フレーム列を2パス処理して (rows, info) を返す。

    rows: [(t, x, y, theta, n_tags, quality_px)]。欠測は x,y,theta=None。
    quality_px はロボットタグの周長（解像度・ブレの代理指標）。
    """
    detector = make_detector()
    # ---- 1パス目: 床タグ中心を平均してカメラ姿勢 ----
    per_tag, used = {}, 0
    for _, gray in frames_fn():
        det = detect_tags(gray, detector)
        if all(t in det for t in cfg['floor_tags']):
            for tid in cfg['floor_tags']:
                per_tag.setdefault(tid, []).append(tag_center(det[tid]))
            used += 1
            if used >= calib_frames:
                break
    if used == 0:
        raise CalibError('床基準タグ4枚が同時に写るフレームがない')
    rvec, tvec = _solve_from_avg(per_tag, cfg)

    # ---- 2パス目: ロボットpose ----
    rid = int(cfg['robot']['id'])
    z = float(cfg['robot']['z_m'])
    yofs = float(cfg['robot'].get('yaw_offset_rad', 0.0))
    rows = []
    tail = deque(maxlen=30)   # 末尾のカメラずれ検知用
    for t, gray in frames_fn():
        det = detect_tags(gray, detector)
        if all(k in det for k in cfg['floor_tags']):
            tail.append({tid: tag_center(det[tid])
                         for tid in cfg['floor_tags']})
        if rid in det:
            x, y, th = robot_pose(det[rid], cfg['K'], cfg['dist'],
                                  rvec, tvec, z, yofs)
            q = float(cv2.arcLength(
                det[rid].astype(np.float32), closed=True))
            rows.append((t, x, y, th, len(det), q))
        else:
            rows.append((t, None, None, None, len(det), 0.0))

    drift = float('nan')
    if tail:
        per_tag2 = {}
        for d in tail:
            for tid, c in d.items():
                per_tag2.setdefault(tid, []).append(c)
        rv2, tv2 = _solve_from_avg(per_tag2, cfg)
        drift = float(np.linalg.norm(
            camera_center(rv2, tv2) - camera_center(rvec, tvec)))
    n_valid = sum(1 for r in rows if r[1] is not None)
    info = dict(cam_center=camera_center(rvec, tvec), cam_drift_m=drift,
                n_frames=len(rows), n_valid=n_valid)
    return rows, info


def _video_frames(path):
    """動画ファイル → (t_s, gray) イテレータを返す callable。"""
    def gen():
        cap = cv2.VideoCapture(str(path))
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        i = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            yield i / fps, cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            i += 1
        cap.release()
    return gen


def _endpoint(rows, window_s=0.5):
    """末尾 window_s の有効行の平均 pose（θは円形平均）。無ければ None。"""
    valid = [r for r in rows if r[1] is not None]
    if not valid:
        return None
    t_end = valid[-1][0]
    win = [r for r in valid if r[0] >= t_end - window_s]
    x = float(np.mean([r[1] for r in win]))
    y = float(np.mean([r[2] for r in win]))
    th = float(np.arctan2(np.mean([np.sin(r[3]) for r in win]),
                          np.mean([np.cos(r[3]) for r in win])))
    return x, y, th


def main():
    ap = argparse.ArgumentParser(description='俯瞰動画→真値CSV')
    ap.add_argument('video')
    ap.add_argument('--config', default='configs/camera_truth.yaml')
    ap.add_argument('--out')
    ap.add_argument('--calib-frames', type=int, default=30)
    ap.add_argument('--path-file', help='終点をゴールと比較する経路yaml')
    args = ap.parse_args()

    cfg = load_config(args.config)
    rows, info = run_video(_video_frames(args.video), cfg,
                           args.calib_frames)
    out = Path(args.out or Path(args.video).with_suffix('.truth.csv'))
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_s', 'x_m', 'y_m', 'theta_rad', 'n_tags', 'quality_px'])
        for t, x, y, th, n, q in rows:
            w.writerow([f'{t:.3f}',
                        '' if x is None else f'{x:.4f}',
                        '' if y is None else f'{y:.4f}',
                        '' if th is None else f'{th:.4f}', n, f'{q:.0f}'])

    print(f'{out}: {info["n_valid"]}/{info["n_frames"]} フレーム有効')
    cc = info['cam_center']
    print(f'カメラ位置 ({cc[0]:.2f}, {cc[1]:.2f}, 高さ {cc[2]:.2f}) m / '
          f'セッション中のずれ {info["cam_drift_m"] * 100:.1f} cm')
    if info['cam_drift_m'] > 0.01:
        print('警告: カメラが動いた可能性。この動画の値は信頼しないこと')
    ep = _endpoint(rows)
    if ep:
        x, y, th = ep
        print(f'終点: x={x * 100:.1f}cm y={y * 100:.1f}cm '
              f'θ={math.degrees(th):.1f}°')
        if args.path_file:
            with open(args.path_file) as f:
                gx, gy = yaml.safe_load(f)['waypoints'][-1]
            d = math.hypot(x - gx, y - gy)
            print(f'ゴール({gx}, {gy})からの距離: {d * 100:.1f} cm')


if __name__ == '__main__':
    main()
```

- [ ] **Step 4: テストが通ることを確認**

Run: `uv run pytest tests/test_truth_offline.py -q`
Expected: 2 passed

- [ ] **Step 5: 全テスト＋コミット**

Run: `uv run pytest tests/ -q` → Expected: 全pass

```bash
git add rover/truth_offline.py tests/test_truth_offline.py
git commit -m "truth_offline: 俯瞰動画→真値CSVのオフラインCLI"
```

---

### Task 8: 精度検証手順ドキュメント＋handoff更新

**Files:**
- Create: `docs/作業記録/カメラ真値_精度検証手順.md`
- Modify: `docs/handoff.md`（セッション先頭に追記）、`CLAUDE.md`（現在地）

**Interfaces:**
- Consumes: Task 1〜7 の成果物一式

- [ ] **Step 1: 検証手順ドキュメントを書く**

`docs/作業記録/カメラ真値_精度検証手順.md`:

```markdown
# カメラ真値の精度検証手順（Phase 1 完了条件）

設計: specs/2026-07-16-camera-truth-pipeline-design.md
合格基準: **静置照合でメジャー実測と位置±1〜2cm・向き±3°一致** → 「真値」認定

## 準備（1回だけ）
1. `uv run python rover/make_tags.py` → docs/tags/ を印刷
   （床用15cm×4枚・ロボット用12cm×1枚。実寸精度は不要、目安でよい）
2. チェスボード（9x6内側コーナー）を印刷し、マス実寸を測る（例24mm）
3. iPhoneを三脚等でコース俯瞰に固定。**設定→カメラ→フォーマット→互換性優先**
   （H.264）にし、レンズは1xのまま固定（0.5x/ズームは使わない＝要再キャリブ）
4. チェスボードを画面全域・傾きを変えつつ30秒撮影 →
   `uv run python rover/calibrate_camera.py <動画> --square-m 0.024`
   → 出力を configs/camera_truth.yaml の K/dist に貼る（再投影誤差 0.5px 以下が目安）
5. 床タグ4枚をコース四隅付近に貼り、**タグ中心のコース座標をメジャーで実測**
   （原点=走行原点・x=第1直進方向）→ yaml の floor_tags に記入
6. ロボットタグを車体上面の中心に前方=タグ上向き（正準+x）で貼り、
   床からタグ面までの高さを実測 → yaml の z_m に記入

## 静置照合（合格判定）
1. 床にテープで既知位置を5点以上マーク（コース中央・四隅・ゴール付近）し、
   コース座標をメジャーで実測記録
2. ロボット（またはタグを貼った箱）を各点に置き、iPhoneで各10秒撮影
3. 各動画を `uv run python rover/truth_offline.py <動画>` にかけ、
   終点サマリとメジャー実測を突き合わせ
4. **全点で位置差≤2cm・向き差≤3°なら合格**。超える場合は
   キャリブやり直し（照明・タグの平坦性・floor_tags の実測を疑う）
5. 結果は results/<日付>_camera_truth_validation/ にCSVと表で保存

## 実走行の試し（合格後）
- L字1本を従来どおり走らせ、動画→CSV→終点を7/15の手計測手順と比較
- 以後、手計測（テープ＋メジャー）はこのパイプラインで置換できる
```

- [ ] **Step 2: handoff.md 先頭にセッション記録を追記・CLAUDE.md 現在地を更新**

`docs/handoff.md` の先頭セクションに追記（当日の他の作業記録に続けて）:

```markdown
- カメラ真値パイプライン Phase 1 実装完了: truth_core（AprilTag→solvePnP→
  視差補正pose）・truth_offline（動画→CSV）・calibrate_camera・make_tags。
  合成画像テストで位置≤1cm/角度≤0.02rad を担保（tests/ 全pass）。
  次: docs/作業記録/カメラ真値_精度検証手順.md に従い実写の静置照合
  （±1〜2cmで真値認定）→ Phase 2（C270ライブ化・run_batch統合）
```

`CLAUDE.md` の「次:」行を更新:

```markdown
- 次: カメラ真値Phase 1の静置照合（手順: docs/作業記録/カメラ真値_精度検証手順.md）
  → Phase 2（C270ライブ化・run_batch統合）→ Phase 3（広角カメラ・自動原点復帰）。
  外乱条件バッチはカメラ真値の後
```

- [ ] **Step 3: 全テスト最終確認**

Run: `uv run pytest tests/ -q`
Expected: 全pass（既存24＋truth系12前後）

- [ ] **Step 4: コミット・push**

```bash
git add docs/作業記録/カメラ真値_精度検証手順.md docs/handoff.md CLAUDE.md
git commit -m "カメラ真値Phase1完了: 精度検証手順とhandoff更新"
git push
```
