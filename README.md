# T6 Calculate the Slope of the Surface

0. Input: Point Cloud
1. 전처리 (노이즈 제거, 다운샘플링)
2. 평면 피팅 (RANSAC)
3. 법선 벡터 계산
4. 법선점(법선 방향의 기준점) 계산 및 해석

Open3D를 사용해 3D 점군 데이터에서 표면 기울기와 방향 정보를 추정


### 1) Preprocessing

- voxel down sampling
- statistical/radius outlier removal
- 관심 영역(ROI) 분리(필요 시)

### 2) Plane Fitting (RANSAC)

Open3D의 `segment_plane`를 사용해 평면 모델 구하기 

```python
plane_model, inliers = pcd.segment_plane(
	distance_threshold=0.5,
	ransac_n=3,
	num_iterations=1000
)

a, b, c, d = plane_model  # ax + by + cz + d = 0
```

- `plane_model = [a, b, c, d]`
- `(a, b, c)`는 평면의 법선 벡터

### 3) Normal Vector

- 평면에서의 법선 벡터: `n = (a, b, c)`
- 단위 법선 벡터: `n_unit = n / ||n||`

### 4) Normal Point

법선점은 보통 기준점 `p0`(예: inlier 중심점, centroid)에서 법선 방향으로 일정 거리 `t`만큼 이동한 점으로 정의
```text
p_normal = p0 + t * n_unit
```

이 값을 이용해 법선 방향 시각화나 표면 방향 비교를 수행


## T5 입력 형태

실제 인터페이스가 꼭 JSON일 필요는 없지만, T6는 아래와 같은 구조의 데이터를 기대합니다.

```json
{
  "hole_id": "H1",
  "frame": "robot_base",
  "unit": "mm",
  "hole_center": [520.3, 140.2, 330.0],
  "scanner_position": [520.3, 140.2, 500.0],
  "scan_range": {
    "type": "radius",
    "value": 25.0
  },
  "points": [
    [501.2, 120.4, 330.1],
    [501.5, 120.8, 330.2],
    [502.0, 121.1, 330.1]
  ]
}
```

T6에서 추가로 필요한 값

- `hole_diameter`: CAD에서 추출한 뚫을 구멍의 직경
- `cad_normal`: CAD 기준 법선 벡터
- `offset`: 구멍 중심에서 법선 방향으로 떨어질 거리
- `expected_noise_std`: 스캐너 노이즈 추정값, RANSAC threshold 설정에 사용

## 처리 흐름

1. T5 payload를 검증합니다.
   - `frame == "robot_base"`
   - `unit == "mm"`
   - `points`가 `(N, 3)` 형태인지 확인

2. `hole_center` 기준으로 point cloud를 crop합니다.
   - 기본 crop 반경은 `hole_diameter * 3`
   - T5의 `scan_range.value`가 더 작으면 그 값을 사용하고 warning을 남깁니다.

3. point cloud를 전처리합니다.
   - Statistical Outlier Removal
   - Voxel Downsampling

4. 평면을 피팅합니다.
   - deterministic RANSAC으로 outlier에 강하게 평면 후보를 찾습니다.
   - RANSAC inlier에 대해 SVD로 최종 평면을 다시 피팅합니다.

5. 법선 방향 부호를 결정합니다.
   - 기본적으로 `scanner_position` 방향을 향하도록 법선을 맞춥니다.
   - 스캐너 방향이 불안정하면 `cad_normal`을 fallback으로 사용합니다.

6. 결과를 검증하고 T7 payload를 만듭니다.
   - RANSAC inlier ratio
   - CAD normal과의 signed/unsigned angle 차이
   - `new_normal_point`
   - `normal`

## T7 전달 payload

T7에는 최소한 아래 값이 전달

```json
{
  "hole_id": "H1",
  "frame": "robot_base",
  "unit": "mm",
  "hole_center": [100.0, 50.0, 30.0],
  "new_normal_point": [86.926, 50.0, 179.429],
  "normal": [-0.087, 0.0, 0.996]
}
```


## 참고

- Open3D 공식 문서: https://www.open3d.org/docs/latest/






