#!/usr/bin/env python3
"""
Standalone shooter ratio characterization with drag.

This script fits the ratio:
  ratio = actual_exit_velocity / expected_exit_velocity_from_wheel_rpm

Unlike simple ballistic back-solves, this uses a drag-aware 2D trajectory model
matching the force form used in scripts/shooter_trajectory_backend.py:
  F_drag = 0.5 * rho * Cd * A * v^2
  a_drag = -(F_drag / m) * v_hat

This script is fully interactive (no required command-line arguments).
You can provide trials from CSV and/or enter them manually.

Each trial needs at minimum:
  rpm, distance_m
Optional per-trial overrides:
  launch_angle_deg, shooter_height_m, target_height_m
"""

from __future__ import annotations

import csv
import json
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

# Matches shooter_trajectory_backend.py gravity constant.
GRAVITY_MPS2 = 9.806

# Defaults mirror scripts/shooter_trajectory_backend.py.
DEFAULT_BALL_MASS_KG = 0.227
DEFAULT_BALL_DIAMETER_M = 5.93 * 0.0254
DEFAULT_DRAG_COEFFICIENT = 0.47
DEFAULT_AIR_DENSITY = 1.204
DEFAULT_WHEEL_RADIUS_M = 2.0 * 0.0254
DEFAULT_SHOOTER_HEIGHT_M = 20.0 * 0.0254
DEFAULT_TARGET_HEIGHT_M = 0.0
DEFAULT_LAUNCH_ANGLE_DEG = 45.0


@dataclass
class DragModelConfig:
    ball_mass_kg: float = DEFAULT_BALL_MASS_KG
    ball_diameter_m: float = DEFAULT_BALL_DIAMETER_M
    drag_coefficient: float = DEFAULT_DRAG_COEFFICIENT
    air_density: float = DEFAULT_AIR_DENSITY

    @property
    def cross_sectional_area_m2(self) -> float:
        radius = self.ball_diameter_m / 2.0
        return math.pi * radius * radius

    @property
    def drag_accel_factor(self) -> float:
        # a_drag = -(0.5 * rho * Cd * A / m) * |v| * v
        return (
            0.5
            * self.air_density
            * self.drag_coefficient
            * self.cross_sectional_area_m2
            / self.ball_mass_kg
        )


@dataclass
class Trial:
    rpm: float
    distance_m: float
    launch_angle_deg: float
    shooter_height_m: float
    target_height_m: float
    source: str


@dataclass
class TrialResult:
    source: str
    rpm: float
    distance_m: float
    launch_angle_deg: float
    shooter_height_m: float
    target_height_m: float
    expected_velocity_mps: float
    actual_velocity_mps: float
    ratio_actual_over_expected: float


def rpm_to_expected_velocity_mps(rpm: float, wheel_radius_m: float) -> float:
    return rpm * 2.0 * math.pi / 60.0 * wheel_radius_m


def _trial_from_values(
    values: list[str],
    default_angle_deg: float,
    default_shooter_height_m: float,
    default_target_height_m: float,
    source: str,
) -> Trial:
    if len(values) not in (2, 3, 4, 5):
        raise ValueError(
            "Trial must be 'rpm,distance' or "
            "'rpm,distance,angle_deg,shooter_height_m,target_height_m'."
        )

    rpm = float(values[0])
    distance_m = float(values[1])
    launch_angle_deg = float(values[2]) if len(values) >= 3 else default_angle_deg
    shooter_height_m = (
        float(values[3]) if len(values) >= 4 else default_shooter_height_m
    )
    target_height_m = float(values[4]) if len(values) >= 5 else default_target_height_m

    if rpm <= 0.0:
        raise ValueError("RPM must be > 0.")
    if distance_m < 0.0:
        raise ValueError("Distance must be >= 0.")
    if not (0.0 <= launch_angle_deg <= 90.0):
        raise ValueError("Launch angle must be within [0, 90] degrees.")

    return Trial(
        rpm=rpm,
        distance_m=distance_m,
        launch_angle_deg=launch_angle_deg,
        shooter_height_m=shooter_height_m,
        target_height_m=target_height_m,
        source=source,
    )


def parse_trial_argument(
    trial_text: str,
    default_angle_deg: float,
    default_shooter_height_m: float,
    default_target_height_m: float,
    source: str,
) -> Trial:
    values = [chunk.strip() for chunk in trial_text.split(",")]
    return _trial_from_values(
        values,
        default_angle_deg,
        default_shooter_height_m,
        default_target_height_m,
        source,
    )


def _read_optional_float(row: dict[str, str], keys: Iterable[str]) -> float | None:
    for key in keys:
        if key in row and row[key] is not None and row[key].strip() != "":
            return float(row[key])
    return None


def read_trials_csv(
    csv_path: Path,
    default_angle_deg: float,
    default_shooter_height_m: float,
    default_target_height_m: float,
) -> list[Trial]:
    trials: list[Trial] = []
    with csv_path.open("r", newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError("CSV file has no header row.")

        for line_number, row in enumerate(reader, start=2):
            rpm = _read_optional_float(row, ("rpm", "wheel_rpm"))
            distance_m = _read_optional_float(row, ("distance_m", "distance"))
            launch_angle_deg = _read_optional_float(
                row, ("launch_angle_deg", "angle_deg", "angle")
            )
            shooter_height_m = _read_optional_float(
                row, ("shooter_height_m", "shooter_height")
            )
            target_height_m = _read_optional_float(
                row, ("target_height_m", "target_height")
            )

            if rpm is None or distance_m is None:
                raise ValueError(
                    f"CSV line {line_number}: missing required rpm and/or distance_m."
                )

            trials.append(
                Trial(
                    rpm=rpm,
                    distance_m=distance_m,
                    launch_angle_deg=(
                        launch_angle_deg
                        if launch_angle_deg is not None
                        else default_angle_deg
                    ),
                    shooter_height_m=(
                        shooter_height_m
                        if shooter_height_m is not None
                        else default_shooter_height_m
                    ),
                    target_height_m=(
                        target_height_m
                        if target_height_m is not None
                        else default_target_height_m
                    ),
                    source=f"csv:{line_number}",
                )
            )
    return trials


def _dynamics(state: tuple[float, float, float, float], drag_factor: float) -> tuple[float, float, float, float]:
    x_m, z_m, vx_mps, vz_mps = state
    del x_m, z_m  # state included for clarity and possible future extension

    speed_mps = math.hypot(vx_mps, vz_mps)
    ax_mps2 = -drag_factor * speed_mps * vx_mps
    az_mps2 = -GRAVITY_MPS2 - drag_factor * speed_mps * vz_mps
    return vx_mps, vz_mps, ax_mps2, az_mps2


def _rk4_step(state: tuple[float, float, float, float], dt_s: float, drag_factor: float) -> tuple[float, float, float, float]:
    k1 = _dynamics(state, drag_factor)
    s2 = tuple(state[i] + 0.5 * dt_s * k1[i] for i in range(4))
    k2 = _dynamics(s2, drag_factor)
    s3 = tuple(state[i] + 0.5 * dt_s * k2[i] for i in range(4))
    k3 = _dynamics(s3, drag_factor)
    s4 = tuple(state[i] + dt_s * k3[i] for i in range(4))
    k4 = _dynamics(s4, drag_factor)
    return tuple(
        state[i] + dt_s / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i])
        for i in range(4)
    )


def simulate_impact_distance_m(
    initial_speed_mps: float,
    launch_angle_deg: float,
    shooter_height_m: float,
    target_height_m: float,
    model: DragModelConfig,
    dt_s: float,
    max_flight_time_s: float,
) -> float | None:
    theta_rad = math.radians(launch_angle_deg)
    vx0_mps = initial_speed_mps * math.cos(theta_rad)
    vz0_mps = initial_speed_mps * math.sin(theta_rad)

    state = (0.0, shooter_height_m, vx0_mps, vz0_mps)
    prev_x_m, prev_z_m = state[0], state[1]
    drag_factor = model.drag_accel_factor

    # If already at/below target, impact distance is zero by definition.
    if shooter_height_m <= target_height_m:
        return 0.0

    steps = max(1, int(math.ceil(max_flight_time_s / dt_s)))
    for _ in range(steps):
        state = _rk4_step(state, dt_s, drag_factor)
        x_m, z_m, _, _ = state

        if prev_z_m >= target_height_m and z_m <= target_height_m:
            if abs(z_m - prev_z_m) < 1e-12:
                return x_m
            frac = (prev_z_m - target_height_m) / (prev_z_m - z_m)
            return prev_x_m + frac * (x_m - prev_x_m)

        prev_x_m, prev_z_m = x_m, z_m

    return None


def _no_drag_speed_guess_mps(
    distance_m: float,
    launch_angle_deg: float,
    shooter_height_m: float,
    target_height_m: float,
) -> float:
    theta_rad = math.radians(launch_angle_deg)
    cos_theta = math.cos(theta_rad)
    tan_theta = math.tan(theta_rad)
    denominator = 2.0 * cos_theta * cos_theta * (
        distance_m * tan_theta + shooter_height_m - target_height_m
    )

    if denominator <= 1e-9:
        return max(1.0, distance_m)

    return math.sqrt(max(1e-9, GRAVITY_MPS2 * distance_m * distance_m / denominator))


def solve_initial_speed_mps(
    distance_m: float,
    launch_angle_deg: float,
    shooter_height_m: float,
    target_height_m: float,
    model: DragModelConfig,
    dt_s: float,
    max_flight_time_s: float,
    max_speed_mps: float,
    tolerance_m: float,
    max_iterations: int,
) -> float:
    if distance_m == 0.0:
        return 0.0

    low_mps = 0.0
    low_distance_m = simulate_impact_distance_m(
        initial_speed_mps=low_mps,
        launch_angle_deg=launch_angle_deg,
        shooter_height_m=shooter_height_m,
        target_height_m=target_height_m,
        model=model,
        dt_s=dt_s,
        max_flight_time_s=max_flight_time_s,
    )

    if low_distance_m is None:
        raise ValueError(
            "Unable to simulate low-speed trajectory. "
            "Try increasing --max-flight-time."
        )

    high_mps = max(
        1.0,
        _no_drag_speed_guess_mps(
            distance_m, launch_angle_deg, shooter_height_m, target_height_m
        ),
    )

    high_distance_m = simulate_impact_distance_m(
        initial_speed_mps=high_mps,
        launch_angle_deg=launch_angle_deg,
        shooter_height_m=shooter_height_m,
        target_height_m=target_height_m,
        model=model,
        dt_s=dt_s,
        max_flight_time_s=max_flight_time_s,
    )

    while (high_distance_m is None or high_distance_m < distance_m) and high_mps < max_speed_mps:
        high_mps *= 1.35
        high_distance_m = simulate_impact_distance_m(
            initial_speed_mps=high_mps,
            launch_angle_deg=launch_angle_deg,
            shooter_height_m=shooter_height_m,
            target_height_m=target_height_m,
            model=model,
            dt_s=dt_s,
            max_flight_time_s=max_flight_time_s,
        )

    if high_distance_m is None or high_distance_m < distance_m:
        raise ValueError(
            "Could not bracket a solution speed. "
            "Increase --max-speed or --max-flight-time, or verify trial inputs."
        )

    for _ in range(max_iterations):
        mid_mps = 0.5 * (low_mps + high_mps)
        mid_distance_m = simulate_impact_distance_m(
            initial_speed_mps=mid_mps,
            launch_angle_deg=launch_angle_deg,
            shooter_height_m=shooter_height_m,
            target_height_m=target_height_m,
            model=model,
            dt_s=dt_s,
            max_flight_time_s=max_flight_time_s,
        )
        if mid_distance_m is None:
            raise ValueError(
                "Trajectory did not impact within max flight time during solve. "
                "Increase --max-flight-time."
            )

        error_m = mid_distance_m - distance_m
        if abs(error_m) <= tolerance_m:
            return mid_mps

        if error_m < 0.0:
            low_mps = mid_mps
        else:
            high_mps = mid_mps

    return 0.5 * (low_mps + high_mps)


def collect_trials_interactive(
    default_angle_deg: float,
    default_shooter_height_m: float,
    default_target_height_m: float,
) -> list[Trial]:
    print()
    print("Enter trials as: rpm,distance_m")
    print(
        "Optional full form: rpm,distance_m,angle_deg,shooter_height_m,target_height_m"
    )
    print("Press ENTER on a blank line to finish.")
    print()

    trials: list[Trial] = []
    count = 1
    while True:
        line = input(f"Trial {count}: ").strip()
        if line == "":
            break

        try:
            trial = parse_trial_argument(
                trial_text=line,
                default_angle_deg=default_angle_deg,
                default_shooter_height_m=default_shooter_height_m,
                default_target_height_m=default_target_height_m,
                source=f"interactive:{count}",
            )
            trials.append(trial)
            count += 1
        except ValueError as err:
            print(f"  Invalid trial: {err}")

    return trials


def characterize(
    trials: list[Trial],
    wheel_radius_m: float,
    model: DragModelConfig,
    dt_s: float,
    max_flight_time_s: float,
    max_speed_mps: float,
    tolerance_m: float,
    max_iterations: int,
) -> list[TrialResult]:
    results: list[TrialResult] = []
    for trial in trials:
        expected_velocity_mps = rpm_to_expected_velocity_mps(trial.rpm, wheel_radius_m)
        actual_velocity_mps = solve_initial_speed_mps(
            distance_m=trial.distance_m,
            launch_angle_deg=trial.launch_angle_deg,
            shooter_height_m=trial.shooter_height_m,
            target_height_m=trial.target_height_m,
            model=model,
            dt_s=dt_s,
            max_flight_time_s=max_flight_time_s,
            max_speed_mps=max_speed_mps,
            tolerance_m=tolerance_m,
            max_iterations=max_iterations,
        )

        ratio = (
            actual_velocity_mps / expected_velocity_mps
            if expected_velocity_mps > 1e-12
            else float("nan")
        )

        results.append(
            TrialResult(
                source=trial.source,
                rpm=trial.rpm,
                distance_m=trial.distance_m,
                launch_angle_deg=trial.launch_angle_deg,
                shooter_height_m=trial.shooter_height_m,
                target_height_m=trial.target_height_m,
                expected_velocity_mps=expected_velocity_mps,
                actual_velocity_mps=actual_velocity_mps,
                ratio_actual_over_expected=ratio,
            )
        )
    return results


def fit_ratio_least_squares(results: list[TrialResult]) -> float:
    numerator = 0.0
    denominator = 0.0
    for r in results:
        if math.isfinite(r.ratio_actual_over_expected):
            numerator += r.expected_velocity_mps * r.actual_velocity_mps
            denominator += r.expected_velocity_mps * r.expected_velocity_mps

    if denominator <= 1e-12:
        return float("nan")
    return numerator / denominator


def print_results(results: list[TrialResult]) -> None:
    print()
    print("=" * 108)
    print(
        f"{'Source':>14}  {'RPM':>7}  {'Dist(m)':>8}  {'Angle':>7}  "
        f"{'v_expected':>10}  {'v_actual':>10}  {'ratio':>8}"
    )
    print("-" * 108)
    for r in results:
        print(
            f"{r.source:>14}  {r.rpm:7.1f}  {r.distance_m:8.3f}  {r.launch_angle_deg:7.2f}  "
            f"{r.expected_velocity_mps:10.4f}  {r.actual_velocity_mps:10.4f}  "
            f"{r.ratio_actual_over_expected:8.4f}"
        )
    print("-" * 108)

    ratios = [r.ratio_actual_over_expected for r in results if math.isfinite(r.ratio_actual_over_expected)]
    if not ratios:
        print("No valid ratios.")
        return

    lsq_ratio = fit_ratio_least_squares(results)
    mean_ratio = sum(ratios) / len(ratios)
    variance = sum((r - mean_ratio) ** 2 for r in ratios) / len(ratios)
    std_ratio = math.sqrt(variance)

    residuals = [
        r.actual_velocity_mps - lsq_ratio * r.expected_velocity_mps
        for r in results
        if math.isfinite(r.ratio_actual_over_expected)
    ]
    rmse = math.sqrt(sum(e * e for e in residuals) / len(residuals))

    print()
    print("Overall fit:")
    print(f"  Least-squares ratio (actual/expected): {lsq_ratio:.5f}")
    print(f"  Mean per-trial ratio:                  {mean_ratio:.5f}")
    print(f"  Std dev per-trial ratio:               {std_ratio:.5f}")
    print(f"  RMSE of v_actual - k*v_expected (m/s): {rmse:.5f}")
    print()


def export_json(
    output_path: Path,
    model: DragModelConfig,
    wheel_radius_m: float,
    dt_s: float,
    max_flight_time_s: float,
    max_speed_mps: float,
    tolerance_m: float,
    max_iterations: int,
    results: list[TrialResult],
) -> None:
    payload = {
        "metadata": {
            "description": "Shooter characterization with drag-aware trajectory inversion",
            "ratio_definition": "actual_exit_velocity / expected_exit_velocity_from_wheel_rpm",
            "wheel_radius_m": wheel_radius_m,
            "integrator_dt_s": dt_s,
            "max_flight_time_s": max_flight_time_s,
            "max_speed_mps": max_speed_mps,
            "solve_tolerance_m": tolerance_m,
            "max_iterations": max_iterations,
            "drag_model": asdict(model),
        },
        "results": [asdict(r) for r in results],
        "summary": {
            "least_squares_ratio": fit_ratio_least_squares(results),
            "mean_ratio": (
                sum(r.ratio_actual_over_expected for r in results)
                / len(results)
                if results
                else float("nan")
            ),
        },
    }

    with output_path.open("w") as handle:
        json.dump(payload, handle, indent=2)


def _prompt_float(
    label: str,
    default: float,
    min_val: float | None = None,
    max_val: float | None = None,
) -> float:
    while True:
        raw = input(f"{label} [{default}]: ").strip()
        if raw == "":
            value = default
        else:
            try:
                value = float(raw)
            except ValueError:
                print("  Enter a valid number.")
                continue

        if min_val is not None and value < min_val:
            print(f"  Value must be >= {min_val}.")
            continue
        if max_val is not None and value > max_val:
            print(f"  Value must be <= {max_val}.")
            continue
        return value


def _prompt_int(label: str, default: int, min_val: int | None = None) -> int:
    while True:
        raw = input(f"{label} [{default}]: ").strip()
        if raw == "":
            value = default
        else:
            try:
                value = int(raw)
            except ValueError:
                print("  Enter a valid integer.")
                continue

        if min_val is not None and value < min_val:
            print(f"  Value must be >= {min_val}.")
            continue
        return value


def _prompt_yes_no(label: str, default_yes: bool) -> bool:
    suffix = "Y/n" if default_yes else "y/N"
    while True:
        raw = input(f"{label} [{suffix}]: ").strip().lower()
        if raw == "":
            return default_yes
        if raw in ("y", "yes"):
            return True
        if raw in ("n", "no"):
            return False
        print("  Enter y or n.")


def main() -> int:
    print("=" * 80)
    print("Shooter Ratio Characterization (Drag-Aware)")
    print("=" * 80)
    print()
    print("This computes: ratio = actual_exit_velocity / expected_exit_velocity")
    print("Expected exit velocity is wheel surface speed from RPM and wheel radius.")
    print()

    wheel_radius_m = _prompt_float(
        "Wheel radius (m)", DEFAULT_WHEEL_RADIUS_M, min_val=1e-6
    )
    default_launch_angle_deg = _prompt_float(
        "Default launch angle (deg)",
        DEFAULT_LAUNCH_ANGLE_DEG,
        min_val=0.0,
        max_val=90.0,
    )
    default_shooter_height_m = _prompt_float(
        "Default shooter height (m)", DEFAULT_SHOOTER_HEIGHT_M, min_val=0.0
    )
    default_target_height_m = _prompt_float(
        "Default target height (m, use 0 for floor)", DEFAULT_TARGET_HEIGHT_M
    )

    print()
    if _prompt_yes_no(
        "Use default drag model constants from shooter_trajectory_backend.py?",
        default_yes=True,
    ):
        model = DragModelConfig()
    else:
        model = DragModelConfig(
            ball_mass_kg=_prompt_float(
                "Ball mass (kg)", DEFAULT_BALL_MASS_KG, min_val=1e-6
            ),
            ball_diameter_m=_prompt_float(
                "Ball diameter (m)", DEFAULT_BALL_DIAMETER_M, min_val=1e-6
            ),
            drag_coefficient=_prompt_float(
                "Drag coefficient", DEFAULT_DRAG_COEFFICIENT, min_val=0.0
            ),
            air_density=_prompt_float(
                "Air density (kg/m^3)", DEFAULT_AIR_DENSITY, min_val=1e-6
            ),
        )

    dt_s = 0.002
    max_flight_time_s = 5.0
    max_speed_mps = 60.0
    tolerance_m = 1e-4
    max_iterations = 80

    print()
    if _prompt_yes_no("Tune advanced solver settings?", default_yes=False):
        dt_s = _prompt_float("Integrator dt (s)", dt_s, min_val=1e-6)
        max_flight_time_s = _prompt_float(
            "Max flight time (s)", max_flight_time_s, min_val=0.05
        )
        max_speed_mps = _prompt_float(
            "Max search speed (m/s)", max_speed_mps, min_val=1.0
        )
        tolerance_m = _prompt_float(
            "Distance solve tolerance (m)", tolerance_m, min_val=1e-8
        )
        max_iterations = _prompt_int(
            "Max bisection iterations", max_iterations, min_val=1
        )

    print()
    trials: list[Trial] = []
    if _prompt_yes_no("Load trials from CSV?", default_yes=False):
        while True:
            csv_raw = input("CSV path: ").strip()
            if csv_raw == "":
                print("  Enter a CSV path.")
                continue

            try:
                csv_path = Path(csv_raw).expanduser()
                csv_trials = read_trials_csv(
                    csv_path=csv_path,
                    default_angle_deg=default_launch_angle_deg,
                    default_shooter_height_m=default_shooter_height_m,
                    default_target_height_m=default_target_height_m,
                )
                trials.extend(csv_trials)
                print(f"Loaded {len(csv_trials)} trial(s) from {csv_path}")
                break
            except Exception as err:
                print(f"  Could not load CSV: {err}")
                if not _prompt_yes_no("Try another CSV path?", default_yes=True):
                    break

    if not trials or _prompt_yes_no("Add manual trials?", default_yes=True):
        trials.extend(
            collect_trials_interactive(
                default_angle_deg=default_launch_angle_deg,
                default_shooter_height_m=default_shooter_height_m,
                default_target_height_m=default_target_height_m,
            )
        )

    if not trials:
        print("No trials provided.")
        return 1

    try:
        results = characterize(
            trials=trials,
            wheel_radius_m=wheel_radius_m,
            model=model,
            dt_s=dt_s,
            max_flight_time_s=max_flight_time_s,
            max_speed_mps=max_speed_mps,
            tolerance_m=tolerance_m,
            max_iterations=max_iterations,
        )
    except ValueError as err:
        print(f"Characterization failed: {err}")
        return 2

    print_results(results)

    if _prompt_yes_no("Export JSON results?", default_yes=False):
        default_export = "shooter_ratio_drag_characterization_results.json"
        output_raw = input(f"Output path [{default_export}]: ").strip()
        output_path = Path(output_raw if output_raw else default_export).expanduser()
        export_json(
            output_path=output_path,
            model=model,
            wheel_radius_m=wheel_radius_m,
            dt_s=dt_s,
            max_flight_time_s=max_flight_time_s,
            max_speed_mps=max_speed_mps,
            tolerance_m=tolerance_m,
            max_iterations=max_iterations,
            results=results,
        )
        print(f"Exported JSON: {output_path}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
