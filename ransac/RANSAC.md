# RANSAC - Random Sample Consensus
데이터에 이상치가 많이 섞여 있어도 원하는 모델을 찾아 내는 알고리즘  
Point Cloud에서는 주로 평면이나 직선을 찾을 때 많이 사용  
-> 이상치는 무시하고 대부분의 점이 이루는 평면을 찾는 방법  
특정 임계값 이상의 데이터를 완전히 무시해버리는 특성이 있어 outlier에 강한 알고리즘  

## 핵심 아이디어  
Random하게 최소한의 점만 뽑아 모델을 만든 뒤, 해당 모델이 알마나 많은 점과 잘 맞는지 확인  
이 과정 반복  

### 1. 랜덤하게 점 선택  
평면은 최소 3개의 점 필요 -> 3개의 점 랜덤하게 뽑기

### 2. 평면 생성
3개의 점으로 평면 생성  
ax + by + cz + d = 0  

### 3. 모든 점과 거리 계산
Point Cloud의 모든 점이 이 평면에서 얼마나 떨어져 있는지 계산  

### 4. Threshold 적용  
Inlier - 내점 : Threshold 이내 가까운 점  
Outlier - 이상치 : 멀리 떨저진 점  

### 5. Inlier 개수 세기  
총 점 N개 중 특정 개수가 가까우면 이 평면은 좋은 평면이다 라고 판단  

### 6. 반복
다시 랜덤으로 3점 선택 -> 평면 생성 -> Inlier count -> 수십~수백번 반복  

### 7. 가장 많은 Inlier를 가진 평면 선택  
Inlier가 가장 많은 평면 선택  

# Open3D에서 사용
```python
plane_model, inliers = pcd.segment_plane(
    distance_threshold=0.5,
    ransac_n=3,
    num_iterations=1000
)
```  
  
distance_threshold : 평면으로부터 이 거리 이하인 점을 inlier로 인정  
ransac_n : 한 번의 추정에 사용할 점의 개수(평면은 3이면 충분)  
num_iterations : 랜덤 추출을 몇 번 반복할지  

결과 : plane_model : [a, b, c, d]  
-> ax + by + cz + d =0 의 계수  
(a, b, c) -> 평면의 법선 벡터 (Normal Vector)  


