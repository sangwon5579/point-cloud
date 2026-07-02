# Calculate Slope

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import logging
from typing import Any, Dict, Iterable, List, Optional

import numpy as np
import open3d as o3d

logger = logging.getLogger(__name__)

# T5 데이터 검증 후 저장 
@dataclass(frozen=True)
class T5ScanData:
    hole_id: str
    frame: str
    unit: str
    hole_center: np.ndarray
    point_cloud: o3d.geometry.PointCloud
    scan_radius: Optional[float]
    scanner_position: Optional[np.ndarray] = None

# T6 최종 Output 
@dataclass(frozen=True)
class SlopeResult:
    hole_id: Optional[str]
    frame: str
    unit: str
    hole_center: np.ndarray
    new_normal_point: np.ndarray
    normal: np.ndarray
    inlier_ratio: float
    angle_diff_deg: float
    unsigned_angle_diff_deg: float
    plane_model: List[float]
    input_point_count: int
    cropped_point_count: int
    processed_point_count: int
    ransac_inlier_count: int
    crop_radius: float
    ransac_distance_threshold: float
    warnings: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        """Convert result to JSON-serializable values."""
        data = asdict(self)
        for key in ("hole_center", "new_normal_point", "normal"):
            data[key] = np.asarray(data[key], dtype=float).tolist()
        return data

    def t7_payload(self) -> Dict[str, Any]:
        """Return only the values T7 needs to move the robot arm."""
        return {
            "hole_id": self.hole_id,
            "frame": self.frame,
            "unit": self.unit,
            "hole_center": self.hole_center.tolist(),
            "new_normal_point": self.new_normal_point.tolist(),
            "normal": self.normal.tolist(),
        }

# 입력 검증
# 3차원 벡터인지 확인 
def _as_vector3(value: Any, name: str) -> np.ndarray:
    vector = np.asarray(value, dtype=float)
    if vector.shape != (3,):
        raise ValueError(f"{name} must be a 3D vector.")
    if not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must contain only finite numbers.")
    return vector

# point Cloud가 N*3 형태인지 확인 
def _as_points(value: Iterable[Iterable[float]], name: str) -> np.ndarray:
    points = np.asarray(value, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"{name} must have shape (N, 3).")
    if len(points) == 0:
        raise ValueError(f"{name} must not be empty.")
    if not np.all(np.isfinite(points)):
        raise ValueError(f"{name} must contain only finite numbers.")
    return points

# 벡터를 단위 벡터로 변환  
def _normalize(vector: np.ndarray, name: str) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm <= 1e-12:
        raise ValueError(f"{name} must not be a zero vector.")
    return vector / norm

# Payload 처리
def point_cloud_from_points(points: Iterable[Iterable[float]]) -> o3d.geometry.PointCloud:
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(_as_points(points, "points"))
    return point_cloud

# T5 Payload 처리
def parse_t5_scan_payload(payload: Dict[str, Any]) -> T5ScanData:
    required = ("hole_id", "frame", "unit", "hole_center", "points")
    missing = [key for key in required if key not in payload]
    if missing:
        raise ValueError(f"T5 payload is missing fields: {missing}")

    frame = str(payload["frame"])
    unit = str(payload["unit"])
    if unit != "mm":
        raise ValueError(f"Unsupported unit: {unit!r}. T6 expects 'mm'.")
    if frame != "robot_base":
        raise ValueError(f"Unsupported frame: {frame!r}. T6 expects 'robot_base'.")

    scan_radius = None
    scan_range = payload.get("scan_range")
    if scan_range is not None:
        if scan_range.get("type") != "radius":
            raise ValueError("scan_range.type must be 'radius' when scan_range is provided.")
        scan_radius = float(scan_range["value"])
        if scan_radius <= 0:
            raise ValueError("scan_range.value must be positive.")

    scanner_position = payload.get("scanner_position")
    scanner_vector = (
        _as_vector3(scanner_position, "scanner_position")
        if scanner_position is not None
        else None
    )

    return T5ScanData(
        hole_id=str(payload["hole_id"]),
        frame=frame,
        unit=unit,
        hole_center=_as_vector3(payload["hole_center"], "hole_center"),
        point_cloud=point_cloud_from_points(payload["points"]),
        scan_radius=scan_radius,
        scanner_position=scanner_vector,
    )

# 전처리
# Crop
def _crop_by_radius(
    point_cloud: o3d.geometry.PointCloud,
    hole_center: np.ndarray,
    crop_radius: float,
) -> o3d.geometry.PointCloud:
    points = np.asarray(point_cloud.points)
    distances = np.linalg.norm(points - hole_center, axis=1)
    indices = np.where(distances <= crop_radius)[0]
    return point_cloud.select_by_index(indices.tolist())

# Crop Radius 선택
# 기본적으로는 구멍 직경의 3배 사용하지만, T5에서 제공한 scan_radius가 더 작은 경우에는 그 값을 사용
def _choose_crop_radius(
    hole_diameter: float,
    scan_radius: Optional[float],
    warnings: List[str],
) -> float:
    requested_radius = float(hole_diameter * 3.0)
    if scan_radius is None:
        return requested_radius
    if scan_radius < requested_radius:
        warnings.append(
            "T5 scan radius is smaller than T6 requested crop radius; "
            f"using {scan_radius:.3f} mm instead of {requested_radius:.3f} mm."
        )
        return float(scan_radius)
    return requested_radius

# 평면까지 허용할 최대 거리  
def _choose_ransac_threshold(
    hole_diameter: float,
    expected_noise_std: Optional[float],
    ransac_distance_threshold: Optional[float],
) -> float:
    if ransac_distance_threshold is not None:
        threshold = float(ransac_distance_threshold)
    elif expected_noise_std is not None:
        threshold = max(float(expected_noise_std) * 2.5, 0.05)
    else:
        threshold = max(float(hole_diameter) * 0.02, 0.05)
    if threshold <= 0:
        raise ValueError("ransac_distance_threshold must be positive.")
    return threshold


def _set_open3d_seed(random_seed: Optional[int]) -> None:
    if random_seed is None:
        return
    try:
        o3d.utility.random.seed(int(random_seed))
    except AttributeError:
        logger.debug("Open3D random seed API is unavailable in this version.")

# Plane Fitting
# 점 3개를 통해 평면 방정식 계산
def _plane_from_three_points(points: np.ndarray) -> Optional[List[float]]:
    p0, p1, p2 = points
    normal = np.cross(p1 - p0, p2 - p0)
    norm = float(np.linalg.norm(normal))
    if norm <= 1e-12:
        return None
    normal = normal / norm
    d = -float(np.dot(normal, p0))
    return [float(normal[0]), float(normal[1]), float(normal[2]), d]

# RANSAC 통해 찾은 Inlier들을 이용하여 최종 Plane을 SVD로 다시 계산  
# RANSAC -> SVD -> 더 정확한 평면  
def _fit_plane_svd(points: np.ndarray) -> List[float]:
    if len(points) < 3:
        raise ValueError("At least 3 inlier points are required to fit a plane.")
    centroid = np.mean(points, axis=0)
    _, _, vh = np.linalg.svd(points - centroid, full_matrices=False)
    normal = _normalize(vh[-1], "svd_normal")
    d = -float(np.dot(normal, centroid))
    return [float(normal[0]), float(normal[1]), float(normal[2]), d]


def _point_to_plane_distances(points: np.ndarray, plane_model: List[float]) -> np.ndarray:
    normal = np.asarray(plane_model[:3], dtype=float)
    d = float(plane_model[3])
    return np.abs(points @ normal + d)

# RANSAC Plane Segmentation
def _segment_plane_ransac(
    points: np.ndarray,
    *,
    distance_threshold: float,
    num_iterations: int,
    random_seed: Optional[int],
) -> tuple[List[float], List[int]]:
    if len(points) < 3:
        raise ValueError("At least 3 points are required for RANSAC plane fitting.")
    if num_iterations <= 0:
        raise ValueError("num_iterations must be positive.")

    rng = np.random.default_rng(random_seed)
    best_plane: Optional[List[float]] = None
    best_inliers: np.ndarray = np.array([], dtype=int)
    best_mean_distance = float("inf")

    for _ in range(num_iterations):
        sample_indices = rng.choice(len(points), size=3, replace=False)
        plane = _plane_from_three_points(points[sample_indices])
        if plane is None:
            continue
        distances = _point_to_plane_distances(points, plane)
        inliers = np.where(distances <= distance_threshold)[0]
        if len(inliers) == 0:
            continue
        mean_distance = float(np.mean(distances[inliers]))
        is_better = len(inliers) > len(best_inliers)
        is_tie_break = (
            len(inliers) == len(best_inliers)
            and mean_distance < best_mean_distance
        )
        if is_better or is_tie_break:
            best_plane = plane
            best_inliers = inliers
            best_mean_distance = mean_distance

    if best_plane is None or len(best_inliers) < 3:
        raise ValueError("RANSAC failed to find a valid plane.")

    refined_plane = _fit_plane_svd(points[best_inliers])
    refined_distances = _point_to_plane_distances(points, refined_plane)
    refined_inliers = np.where(refined_distances <= distance_threshold)[0]
    if len(refined_inliers) >= 3:
        refined_plane = _fit_plane_svd(points[refined_inliers])
        return refined_plane, refined_inliers.tolist()
    return refined_plane, best_inliers.tolist()

# 벡터 방향 결정  
def _orient_normal(
    normal: np.ndarray,
    hole_center: np.ndarray,
    scanner_position: Optional[np.ndarray],
    cad_unit: np.ndarray,
    warnings: List[str],
) -> np.ndarray:
    if scanner_position is not None:
        scanner_direction = scanner_position - hole_center
        if np.linalg.norm(scanner_direction) > 1e-9:
            dot_to_scanner = float(np.dot(normal, scanner_direction))
            if abs(dot_to_scanner) > 1e-9:
                return normal if dot_to_scanner > 0.0 else -normal
            warnings.append(
                "Scanner direction is nearly tangent to the fitted plane; "
                "using cad_normal to choose normal sign."
            )
        else:
            warnings.append(
                "scanner_position is too close to hole_center; "
                "using cad_normal to choose normal sign."
            )
    else:
        warnings.append("scanner_position is missing; using cad_normal to choose normal sign.")

    return normal if float(np.dot(normal, cad_unit)) >= 0.0 else -normal

# 입력 검사 -> Crop -> Statistical Outlier Removal -> Voxel Downsampling -> RANSAC -> Normal 계산 -> CAD Normal과 비교 -> 최종 Normal Point 계산
def calculate_slope(
    point_cloud: o3d.geometry.PointCloud,
    hole_center: np.ndarray,
    hole_diameter: float,
    offset: float,
    scanner_position: Optional[np.ndarray],
    cad_normal: np.ndarray,
    *,
    hole_id: Optional[str] = None,
    frame: str = "robot_base",
    unit: str = "mm",
    scan_radius: Optional[float] = None,
    expected_noise_std: Optional[float] = None,
    ransac_distance_threshold: Optional[float] = None,
    voxel_size: Optional[float] = None,
    min_points: int = 50,
    num_iterations: int = 1000,
    random_seed: Optional[int] = None,
) -> SlopeResult:
    """Calculate the corrected T7 target point from a scanned surface."""
    if point_cloud is None or point_cloud.is_empty():
        raise ValueError("point_cloud is empty.")
    if frame != "robot_base":
        raise ValueError(f"Unsupported frame: {frame!r}. T6 expects 'robot_base'.")
    if unit != "mm":
        raise ValueError(f"Unsupported unit: {unit!r}. T6 expects 'mm'.")
    if hole_diameter <= 0:
        raise ValueError("hole_diameter must be positive.")
    if min_points < 3:
        raise ValueError("min_points must be at least 3.")

    center = _as_vector3(hole_center, "hole_center")
    scanner = (
        _as_vector3(scanner_position, "scanner_position")
        if scanner_position is not None
        else None
    )
    cad_unit = _normalize(_as_vector3(cad_normal, "cad_normal"), "cad_normal")
    warnings: List[str] = []

    input_point_count = len(point_cloud.points)
    crop_radius = _choose_crop_radius(hole_diameter, scan_radius, warnings)
    cropped = _crop_by_radius(point_cloud, center, crop_radius)
    cropped_count = len(cropped.points)
    logger.info("Input point count: %d", input_point_count)
    logger.info("Cropped point count within %.3f mm: %d", crop_radius, cropped_count)
    if cropped_count < min_points:
        raise ValueError(
            f"Cropped point count is too small: {cropped_count} < {min_points}."
        )

    nb_neighbors = min(20, max(3, cropped_count - 1))
    filtered, inlier_indices = cropped.remove_statistical_outlier(
        nb_neighbors=nb_neighbors,
        std_ratio=2.0,
    )
    logger.info(
        "Statistical Outlier Removal point count: %d -> %d",
        cropped_count,
        len(inlier_indices),
    )

    actual_voxel_size = (
        float(voxel_size) if voxel_size is not None else max(float(hole_diameter) / 100.0, 0.1)
    )
    if actual_voxel_size <= 0:
        raise ValueError("voxel_size must be positive.")
    downsampled = filtered.voxel_down_sample(voxel_size=actual_voxel_size)
    processed_count = len(downsampled.points)
    logger.info(
        "Voxel Downsampling point count with voxel_size %.3f mm: %d",
        actual_voxel_size,
        processed_count,
    )
    if processed_count < min_points:
        raise ValueError(
            f"Preprocessed point count is too small: {processed_count} < {min_points}."
        )

    threshold = _choose_ransac_threshold(
        hole_diameter=hole_diameter,
        expected_noise_std=expected_noise_std,
        ransac_distance_threshold=ransac_distance_threshold,
    )
    _set_open3d_seed(random_seed)
    processed_points = np.asarray(downsampled.points)
    plane_model, inliers = _segment_plane_ransac(
        processed_points,
        distance_threshold=threshold,
        num_iterations=num_iterations,
        random_seed=random_seed,
    )
    raw_normal = np.asarray(plane_model[:3], dtype=float)
    normal = _normalize(raw_normal, "normal")
    normal = _orient_normal(normal, center, scanner, cad_unit, warnings)

    inlier_ratio = float(len(inliers) / processed_count)
    if inlier_ratio < 0.5:
        warnings.append(f"RANSAC inlier ratio is low: {inlier_ratio:.4f} < 0.5.")

    signed_dot = float(np.clip(np.dot(cad_unit, normal), -1.0, 1.0))
    unsigned_dot = abs(signed_dot)
    angle_diff = float(np.degrees(np.arccos(signed_dot)))
    unsigned_angle_diff = float(np.degrees(np.arccos(unsigned_dot)))
    if angle_diff >= 30.0:
        warnings.append(f"CAD normal signed angle difference is high: {angle_diff:.4f} deg.")
    if unsigned_angle_diff >= 30.0:
        warnings.append(
            "Fitted plane tilt differs from CAD normal by "
            f"{unsigned_angle_diff:.4f} deg."
        )

    new_normal_point = center + normal * float(offset)
    for warning in warnings:
        logger.warning(warning)
    logger.info("RANSAC plane model: %s", [float(v) for v in plane_model])
    logger.info("RANSAC inlier count: %d / %d", len(inliers), processed_count)
    logger.info("RANSAC inlier ratio: %.4f", inlier_ratio)
    logger.info("Calculated normal: %s", normal.tolist())
    logger.info("CAD normal signed angle difference: %.4f deg", angle_diff)
    logger.info("CAD normal unsigned angle difference: %.4f deg", unsigned_angle_diff)
    logger.info("New normal point: %s", new_normal_point.tolist())

    return SlopeResult(
        hole_id=hole_id,
        frame=frame,
        unit=unit,
        hole_center=center,
        new_normal_point=new_normal_point,
        normal=normal,
        inlier_ratio=inlier_ratio,
        angle_diff_deg=angle_diff,
        unsigned_angle_diff_deg=unsigned_angle_diff,
        plane_model=[float(value) for value in plane_model],
        input_point_count=input_point_count,
        cropped_point_count=cropped_count,
        processed_point_count=processed_count,
        ransac_inlier_count=len(inliers),
        crop_radius=crop_radius,
        ransac_distance_threshold=threshold,
        warnings=warnings,
    )

# T5의 딕셔너리를 입력으로 받아서 내부에서 파싱 및 계산 처리 
def calculate_slope_from_t5_payload(
    scan_payload: Dict[str, Any],
    *,
    hole_diameter: float,
    offset: float,
    cad_normal: np.ndarray,
    scanner_position: Optional[np.ndarray] = None,
    expected_noise_std: Optional[float] = None,
    ransac_distance_threshold: Optional[float] = None,
    voxel_size: Optional[float] = None,
    min_points: int = 50,
    num_iterations: int = 1000,
    random_seed: Optional[int] = None,
) -> SlopeResult:
    """Run T6 directly from a T5-style scan payload."""
    scan_data = parse_t5_scan_payload(scan_payload)
    scanner = (
        _as_vector3(scanner_position, "scanner_position")
        if scanner_position is not None
        else scan_data.scanner_position
    )
    return calculate_slope(
        point_cloud=scan_data.point_cloud,
        hole_center=scan_data.hole_center,
        hole_diameter=hole_diameter,
        offset=offset,
        scanner_position=scanner,
        cad_normal=cad_normal,
        hole_id=scan_data.hole_id,
        frame=scan_data.frame,
        unit=scan_data.unit,
        scan_radius=scan_data.scan_radius,
        expected_noise_std=expected_noise_std,
        ransac_distance_threshold=ransac_distance_threshold,
        voxel_size=voxel_size,
        min_points=min_points,
        num_iterations=num_iterations,
        random_seed=random_seed,
    )
