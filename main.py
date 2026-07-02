from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np

from mock_data_generator import (
    generate_t5_scan_payload,
    save_point_cloud,
    true_normal_from_x_tilt,
)
from t6_normal_refinement import (
    calculate_slope_from_t5_payload,
    point_cloud_from_points,
)


PROJECT_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = PROJECT_DIR / "outputs"
RESULTS_PATH = PROJECT_DIR / "results.json"

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
)
logger = logging.getLogger(__name__)


def _angle_between_deg(a: np.ndarray, b: np.ndarray) -> float:
    a_unit = a / np.linalg.norm(a)
    b_unit = b / np.linalg.norm(b)
    dot = float(np.clip(np.dot(a_unit, b_unit), -1.0, 1.0))
    return float(np.degrees(np.arccos(dot)))


def _payload_summary(payload: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "hole_id": payload["hole_id"],
        "frame": payload["frame"],
        "unit": payload["unit"],
        "hole_center": payload["hole_center"],
        "scanner_position": payload.get("scanner_position"),
        "scan_range": payload.get("scan_range"),
        "point_count": len(payload["points"]),
    }


def _save_payload_point_cloud(case_name: str, payload: Dict[str, Any]) -> None:
    point_cloud = point_cloud_from_points(payload["points"])
    save_point_cloud(point_cloud, OUTPUT_DIR / f"{case_name}.ply")


def _run_case(
    *,
    case_name: str,
    hole_id: str,
    hole_center: np.ndarray,
    hole_diameter: float,
    tilt_angle_deg: float,
    offset: float,
    scanner_position: np.ndarray,
    cad_normal: np.ndarray,
    scan_radius: float,
    noise_std: float,
    num_points: int,
    outlier_ratio: float,
    random_seed: int,
    ransac_distance_threshold: Optional[float] = None,
) -> Dict[str, Any]:
    logger.info("Running %s", case_name)
    payload = generate_t5_scan_payload(
        hole_id=hole_id,
        hole_center=hole_center,
        hole_diameter=hole_diameter,
        tilt_angle_deg=tilt_angle_deg,
        scanner_position=scanner_position,
        scan_radius=scan_radius,
        noise_std=noise_std,
        num_points=num_points,
        outlier_ratio=outlier_ratio,
        random_seed=random_seed,
    )
    _save_payload_point_cloud(case_name, payload)

    result = calculate_slope_from_t5_payload(
        payload,
        hole_diameter=hole_diameter,
        offset=offset,
        cad_normal=cad_normal,
        expected_noise_std=noise_std,
        ransac_distance_threshold=ransac_distance_threshold,
        random_seed=random_seed,
    )
    true_normal = true_normal_from_x_tilt(tilt_angle_deg)
    normal_error_deg = _angle_between_deg(result.normal, true_normal)

    return {
        "t5_payload_summary": _payload_summary(payload),
        "expected": {
            "tilt_angle_deg": tilt_angle_deg,
            "true_normal": true_normal.tolist(),
        },
        "t6_result": result.to_dict(),
        "t7_payload": result.t7_payload(),
        "normal_error_deg": normal_error_deg,
    }


def main() -> None:
    hole_center = np.array([100.0, 50.0, 30.0])
    hole_diameter = 10.0
    tilt_angle_deg = 5.0
    offset = 150.0
    scanner_position = np.array([100.0, 50.0, 250.0])
    cad_normal = np.array([0.0, 0.0, 1.0])
    scan_radius = 40.0

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    results: Dict[str, Any] = {}

    results["case_1_clean_scan"] = _run_case(
        case_name="case_1_clean_scan",
        hole_id="H1",
        hole_center=hole_center,
        hole_diameter=hole_diameter,
        tilt_angle_deg=tilt_angle_deg,
        offset=offset,
        scanner_position=scanner_position,
        cad_normal=cad_normal,
        scan_radius=scan_radius,
        noise_std=0.002,
        num_points=5000,
        outlier_ratio=0.0,
        random_seed=1,
    )
    print("Case 1 T7 payload:", results["case_1_clean_scan"]["t7_payload"])

    results["case_2_noisy_scan"] = _run_case(
        case_name="case_2_noisy_scan",
        hole_id="H2",
        hole_center=hole_center,
        hole_diameter=hole_diameter,
        tilt_angle_deg=tilt_angle_deg,
        offset=offset,
        scanner_position=scanner_position,
        cad_normal=cad_normal,
        scan_radius=scan_radius,
        noise_std=0.5,
        num_points=5000,
        outlier_ratio=0.08,
        random_seed=2,
    )
    print("Case 2 T7 payload:", results["case_2_noisy_scan"]["t7_payload"])

    try:
        results["case_3_too_few_points"] = _run_case(
            case_name="case_3_too_few_points",
            hole_id="H3",
            hole_center=hole_center,
            hole_diameter=hole_diameter,
            tilt_angle_deg=tilt_angle_deg,
            offset=offset,
            scanner_position=scanner_position,
            cad_normal=cad_normal,
            scan_radius=scan_radius,
            noise_std=0.002,
            num_points=10,
            outlier_ratio=0.0,
            random_seed=3,
        )
    except ValueError as exc:
        results["case_3_too_few_points"] = {"error": str(exc)}
        print("Case 3 error:", exc)

    with RESULTS_PATH.open("w", encoding="utf-8") as file:
        json.dump(results, file, indent=2, ensure_ascii=False)
    logger.info("Saved %s", RESULTS_PATH)


if __name__ == "__main__":
    main()
