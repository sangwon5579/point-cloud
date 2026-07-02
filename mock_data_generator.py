# Mock scan data generator for testing T6 without a real scanner.

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np
import open3d as o3d

# Check if input is 3-dimentional vector
def _as_vector3(value: np.ndarray, name: str) -> np.ndarray:
    vector = np.asarray(value, dtype=float)
    if vector.shape != (3,):
        raise ValueError(f"{name} must be a 3D vector.")
    return vector

# Calculate the true normal vector
def true_normal_from_x_tilt(tilt_angle_deg: float) -> np.ndarray:
    tilt_rad = np.deg2rad(float(tilt_angle_deg))
    normal = np.array([-np.tan(tilt_rad), 0.0, 1.0], dtype=float)
    return normal / np.linalg.norm(normal)

# Generate a continuous tilted surface around a planned drilling target.
def generate_mock_points(
    hole_center: np.ndarray,
    hole_diameter: float,
    tilt_angle_deg: float,
    *,
    noise_std: float = 0.002,
    num_points: int = 5000,
    scan_radius: Optional[float] = None,
    outlier_ratio: float = 0.0,
    random_seed: Optional[int] = None,
) -> np.ndarray:
    if hole_diameter <= 0:
        raise ValueError("hole_diameter must be positive.")
    if num_points <= 0:
        raise ValueError("num_points must be positive.")
    if noise_std < 0:
        raise ValueError("noise_std must be non-negative.")
    if not 0.0 <= outlier_ratio < 1.0:
        raise ValueError("outlier_ratio must satisfy 0 <= outlier_ratio < 1.")

    center = _as_vector3(hole_center, "hole_center")
    radius = float(scan_radius) if scan_radius is not None else float(hole_diameter * 4.0)
    if radius <= 0:
        raise ValueError("scan_radius must be positive.")

    rng = np.random.default_rng(random_seed)
    inlier_count = max(1, int(round(num_points * (1.0 - outlier_ratio))))
    outlier_count = num_points - inlier_count

    radial = radius * np.sqrt(rng.random(inlier_count))
    theta = rng.uniform(0.0, 2.0 * np.pi, inlier_count)
    local_x = radial * np.cos(theta)
    local_y = radial * np.sin(theta)

    tilt_rad = np.deg2rad(float(tilt_angle_deg))
    local_z = np.tan(tilt_rad) * local_x
    inlier_points = np.column_stack(
        (
            center[0] + local_x,
            center[1] + local_y,
            center[2] + local_z,
        )
    )

    if noise_std > 0.0:
        inlier_points += rng.normal(0.0, noise_std, inlier_points.shape)

    if outlier_count == 0:
        return inlier_points

    outlier_xy = rng.uniform(-radius, radius, size=(outlier_count, 2))
    outlier_z = rng.uniform(-radius * 0.4, radius * 0.4, size=(outlier_count, 1))
    outliers = np.column_stack(
        (
            center[0] + outlier_xy[:, 0],
            center[1] + outlier_xy[:, 1],
            center[2] + outlier_z[:, 0],
        )
    )
    points = np.vstack((inlier_points, outliers))
    rng.shuffle(points, axis=0)
    return points

# generate Open3D point cloud from mock data
def generate_mock_point_cloud(
    hole_center: np.ndarray,
    hole_diameter: float,
    tilt_angle_deg: float,
    *,
    noise_std: float = 0.002,
    num_points: int = 5000,
    scan_radius: Optional[float] = None,
    outlier_ratio: float = 0.0,
    random_seed: Optional[int] = None,
) -> o3d.geometry.PointCloud:
    points = generate_mock_points(
        hole_center=hole_center,
        hole_diameter=hole_diameter,
        tilt_angle_deg=tilt_angle_deg,
        noise_std=noise_std,
        num_points=num_points,
        scan_radius=scan_radius,
        outlier_ratio=outlier_ratio,
        random_seed=random_seed,
    )
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    return point_cloud

# example of input from T5
def generate_t5_scan_payload(
    *,
    hole_id: str,
    hole_center: np.ndarray,
    hole_diameter: float,
    tilt_angle_deg: float,
    scanner_position: np.ndarray,
    scan_radius: Optional[float] = None,
    noise_std: float = 0.002,
    num_points: int = 5000,
    outlier_ratio: float = 0.0,
    random_seed: Optional[int] = None,
) -> Dict[str, Any]:
    radius = float(scan_radius) if scan_radius is not None else float(hole_diameter * 4.0)
    points = generate_mock_points(
        hole_center=hole_center,
        hole_diameter=hole_diameter,
        tilt_angle_deg=tilt_angle_deg,
        noise_std=noise_std,
        num_points=num_points,
        scan_radius=radius,
        outlier_ratio=outlier_ratio,
        random_seed=random_seed,
    )
    return {
        "hole_id": str(hole_id),
        "frame": "robot_base",
        "unit": "mm",
        "hole_center": _as_vector3(hole_center, "hole_center").tolist(),
        "scanner_position": _as_vector3(scanner_position, "scanner_position").tolist(),
        "scan_range": {
            "type": "radius",
            "value": radius,
        },
        "points": points.tolist(),
    }

# save as ply file
def save_point_cloud(point_cloud: o3d.geometry.PointCloud, file_path: str | Path) -> None:
    path = Path(file_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    o3d.io.write_point_cloud(str(path), point_cloud)
