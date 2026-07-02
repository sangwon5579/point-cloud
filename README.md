# Point-Cloud Practice with Open3D

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


## 체크리스트

- 전처리 전/후 포인트 개수 비교
- inlier 비율 확인
- 법선 벡터 정규화 여부 확인
- 법선점 계산식과 시각화 결과 일치 여부 확인

## 참고

- Open3D 공식 문서: https://www.open3d.org/docs/latest/
