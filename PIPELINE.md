## t6_normal_refinement.py

1. T5 Scan Data
2. Payload 검증
3. Point Cloud 생성
4. 구멍 주변 Crop
5. Outlier 제거
6. Voxel Downsampling
7. RANSAC Plane Fitting
8. Plane Normal 계산
9. Normal 방향 보정
10. CAD Normal과 비교
11. T7가 사용할 New Normal Point 생성 

| 역할        | 함수                                              |
| --------- | ----------------------------------------------- |
| 데이터 구조 정의 | `T5ScanData`, `SlopeResult`                     |
| 입력 검증     | `_as_vector3()`, `_as_points()`, `_normalize()` |
| T5 데이터 파싱 | `parse_t5_scan_payload()`                       |
| 전처리       | `_crop_by_radius()`, `_choose_crop_radius()`    |
| 평면 추정     | `_segment_plane_ransac()`                       |
| 최종 계산     | `calculate_slope()`                             |
