#!/usr/bin/env python3
"""
Generate shooting lookup table for FRC 2026.

Uses the trajectory optimization backend to compute optimal shooting parameters
for distances from 1.5m to 6.0m in 0.1m increments.

Usage:
    python generate_lookup_table.py [--output PATH] [--min MIN] [--max MAX] [--step STEP]

Output:
    JSON file (hub_lookup_vector.json by default) containing a 3D launch
    velocity vector in shooter coordinates where:
    - +x: horizontal toward the target
    - +y: horizontal left (zero for this lookup)
    - +z: up
"""

import argparse
import json
import math
import os
import sys
from typing import Optional

import numpy as np

from shooter_trajectory_backend import (
    ShooterConfig,
    TrajectoryResult,
    solve_trajectory,
    velocity_to_rpm,
)


HUB_DEFAULT_MIN_DISTANCE_M = 1.5
HUB_DEFAULT_MAX_DISTANCE_M = 6.0
HUB_DEFAULT_STEP_M = 0.1

LOB_DEFAULT_MIN_DISTANCE_M = 4.0
LOB_DEFAULT_MAX_DISTANCE_M = 15.0
LOB_DEFAULT_STEP_M = 0.1
LOB_MAX_LAUNCH_SPEED_MPS = 18.0
LOB_PITCH_SEARCH_ITERATIONS = 12
LOB_SPEED_SEARCH_ITERATIONS = 26
LOB_TRAJECTORY_DT_SEC = 0.0025
LOB_MAX_FLIGHT_TIME_SEC = 5.0
LOB_TARGET_HEIGHT_TOLERANCE_M = 1e-3


def ensure_parent_dir(path: str) -> None:
    """Create parent directory for a file path if needed."""
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)


def default_output_path(filename: str) -> str:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(script_dir, "..", "src", "main", "deploy", filename))


def write_lookup_table(
    *,
    output_path: str,
    min_distance: float,
    max_distance: float,
    step: float,
    config: ShooterConfig,
    verbose: bool,
    optimize_steep_entry: bool = False,
) -> None:
    lookup_table = generate_lookup_table(
        min_distance=min_distance,
        max_distance=max_distance,
        step=step,
        config=config,
        verbose=verbose,
        optimize_steep_entry=optimize_steep_entry,
    )
    vector_lookup_table = generate_vector_lookup_table(lookup_table)
    ensure_parent_dir(output_path)
    with open(output_path, "w") as f:
        json.dump(vector_lookup_table, f, indent=2)
    if verbose:
        print(f"Vector lookup table written to: {output_path}")


def generate_default_profiles(verbose: bool) -> None:
    hub_output = default_output_path("hub_lookup_vector.json")
    lob_output = default_output_path("lob_lookup_vector.json")

    if verbose:
        print("Generating HUB lookup profile...")
    hub_config = ShooterConfig()
    write_lookup_table(
        output_path=hub_output,
        min_distance=HUB_DEFAULT_MIN_DISTANCE_M,
        max_distance=HUB_DEFAULT_MAX_DISTANCE_M,
        step=HUB_DEFAULT_STEP_M,
        config=hub_config,
        verbose=verbose,
    )

    if verbose:
        print("\nGenerating LOB lookup profile...")
    lob_config = ShooterConfig()
    lob_config.target_height = 0.0
    write_lookup_table(
        output_path=lob_output,
        min_distance=LOB_DEFAULT_MIN_DISTANCE_M,
        max_distance=LOB_DEFAULT_MAX_DISTANCE_M,
        step=LOB_DEFAULT_STEP_M,
        config=lob_config,
        verbose=verbose,
        optimize_steep_entry=True,
    )


def _compute_drag_accel(vx: float, vz: float, config: ShooterConfig) -> tuple[float, float]:
    speed = math.hypot(vx, vz)
    if speed <= 1e-9:
        return 0.0, -9.806

    drag_scale = (
        0.5
        * config.air_density
        * config.drag_coefficient
        * config.cross_sectional_area
        / config.ball_mass
        * speed
    )
    ax = -drag_scale * vx
    az = -9.806 - drag_scale * vz
    return ax, az


def _rk4_step_2d(state: tuple[float, float, float, float], dt: float, config: ShooterConfig):
    def derivatives(s: tuple[float, float, float, float]):
        _, _, vx, vz = s
        ax, az = _compute_drag_accel(vx, vz, config)
        return vx, vz, ax, az

    x, z, vx, vz = state
    k1 = derivatives((x, z, vx, vz))
    k2 = derivatives(tuple(state[i] + 0.5 * dt * k1[i] for i in range(4)))
    k3 = derivatives(tuple(state[i] + 0.5 * dt * k2[i] for i in range(4)))
    k4 = derivatives(tuple(state[i] + dt * k3[i] for i in range(4)))
    return tuple(state[i] + dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) for i in range(4))


def simulate_to_distance(
    distance: float,
    speed: float,
    pitch_rad: float,
    config: ShooterConfig,
    dt: float = LOB_TRAJECTORY_DT_SEC,
    max_flight_time: float = LOB_MAX_FLIGHT_TIME_SEC,
):
    vx = speed * math.cos(pitch_rad)
    vz = speed * math.sin(pitch_rad)
    state = (0.0, config.shooter_height, vx, vz)
    t = 0.0
    max_steps = max(1, int(max_flight_time / dt))

    for _ in range(max_steps):
        prev_state = state
        prev_t = t
        state = _rk4_step_2d(state, dt, config)
        t += dt

        x0, z0, vx0, vz0 = prev_state
        x1, z1, vx1, vz1 = state
        if x1 >= distance:
            if x1 == x0:
                alpha = 0.0
            else:
                alpha = (distance - x0) / (x1 - x0)
                alpha = max(0.0, min(1.0, alpha))

            return {
                "z": z0 + alpha * (z1 - z0),
                "vx": vx0 + alpha * (vx1 - vx0),
                "vz": vz0 + alpha * (vz1 - vz0),
                "flight_time": prev_t + alpha * (t - prev_t),
            }

        # If we are falling and below the target height before reaching x=distance,
        # this shot will not recover and hit the target.
        if z1 < config.target_height and vz1 < 0.0:
            return None

    return None


def solve_speed_for_pitch(
    distance: float,
    pitch_rad: float,
    config: ShooterConfig,
    max_speed_mps: float,
):
    low_speed = 0.5
    high_speed = max_speed_mps
    target_height = config.target_height

    low_state = simulate_to_distance(distance, low_speed, pitch_rad, config)
    low_height = -math.inf if low_state is None else low_state["z"]
    high_state = simulate_to_distance(distance, high_speed, pitch_rad, config)
    high_height = -math.inf if high_state is None else high_state["z"]

    if high_height < target_height - LOB_TARGET_HEIGHT_TOLERANCE_M:
        return None

    if low_height >= target_height:
        return low_speed, low_state

    for _ in range(LOB_SPEED_SEARCH_ITERATIONS):
        mid_speed = 0.5 * (low_speed + high_speed)
        mid_state = simulate_to_distance(distance, mid_speed, pitch_rad, config)
        mid_height = -math.inf if mid_state is None else mid_state["z"]

        if mid_height >= target_height:
            high_speed = mid_speed
            high_state = mid_state
        else:
            low_speed = mid_speed

    if high_state is None:
        return None

    return high_speed, high_state


def solve_steepest_feasible_lob_trajectory(
    distance: float,
    config: ShooterConfig,
    max_speed_mps: float = LOB_MAX_LAUNCH_SPEED_MPS,
):
    min_pitch_rad = math.radians(config.min_launch_pitch_deg)
    max_pitch_rad = math.radians(config.max_launch_pitch_deg)

    # If minimum pitch is infeasible at max speed, no physically feasible shot exists.
    min_pitch_at_max_speed = simulate_to_distance(distance, max_speed_mps, min_pitch_rad, config)
    if (
        min_pitch_at_max_speed is None
        or min_pitch_at_max_speed["z"] < config.target_height - LOB_TARGET_HEIGHT_TOLERANCE_M
    ):
        return None, None, "No feasible shot within launch-speed limit."

    max_pitch_at_max_speed = simulate_to_distance(distance, max_speed_mps, max_pitch_rad, config)
    if max_pitch_at_max_speed is not None and max_pitch_at_max_speed["z"] >= config.target_height:
        steepest_pitch_rad = max_pitch_rad
    else:
        # Binary search for the highest feasible launch pitch under max_speed_mps.
        low = min_pitch_rad
        high = max_pitch_rad
        for _ in range(LOB_PITCH_SEARCH_ITERATIONS):
            mid = 0.5 * (low + high)
            mid_state = simulate_to_distance(distance, max_speed_mps, mid, config)
            if mid_state is not None and mid_state["z"] >= config.target_height:
                low = mid
            else:
                high = mid
        steepest_pitch_rad = low

    # Evaluate nearby pitches and choose the steepest entry angle as final answer.
    candidate_offsets_deg = (0.0, -0.25, -0.5, -1.0, -2.0)
    candidate_pitches = []
    for offset_deg in candidate_offsets_deg:
        pitch = steepest_pitch_rad + math.radians(offset_deg)
        pitch = max(min_pitch_rad, min(max_pitch_rad, pitch))
        candidate_pitches.append(pitch)
    candidate_pitches = sorted(set(candidate_pitches), reverse=True)

    best = None
    for pitch_rad in candidate_pitches:
        speed_solution = solve_speed_for_pitch(distance, pitch_rad, config, max_speed_mps)
        if speed_solution is None:
            continue

        speed, state = speed_solution
        if state["vz"] >= 0.0:
            continue

        entry_angle_rad = math.atan2(abs(state["vx"]), abs(state["vz"]))
        candidate_result = TrajectoryResult(
            velocity=float(speed),
            pitch_rad=float(pitch_rad),
            yaw_rad=0.0,
            flight_time=float(state["flight_time"]),
            entry_angle_rad=float(entry_angle_rad),
        )
        candidate_score = (candidate_result.entry_angle_rad, candidate_result.flight_time)
        if best is None or candidate_score < best[0]:
            best = (candidate_score, candidate_result)

    if best is None:
        return None, None, "No feasible descending trajectory within launch-speed limit."

    result = best[1]
    return result, math.degrees(result.entry_angle_rad), None


def generate_lookup_table(
    min_distance: float = 1.5,
    max_distance: float = 6.0,
    step: float = 0.1,
    config: Optional[ShooterConfig] = None,
    verbose: bool = True,
    optimize_steep_entry: bool = False,
) -> dict:
    """
    Generate lookup table for shooting parameters.

    Args:
        min_distance: Minimum distance in meters
        max_distance: Maximum distance in meters
        step: Distance step size in meters
        config: Shooter configuration
        verbose: Print progress

    Returns:
        Dictionary containing lookup table data
    """
    if config is None:
        config = ShooterConfig()

    # Generate distance array
    distances = np.arange(min_distance, max_distance + step / 2, step)
    distances = np.round(distances, 2)  # Clean up floating point

    if verbose:
        print("=" * 70)
        print("FRC 2026 Shooting Lookup Table Generator")
        print("=" * 70)
        print()
        print("Configuration:")
        print(f"  Target height:     {config.target_height:.3f} m ({config.target_height / 0.0254:.1f} in)")
        print(f"  Shooter height:    {config.shooter_height:.3f} m ({config.shooter_height / 0.0254:.1f} in)")
        print(f"  Max entry angle:   {config.max_entry_angle_deg:.1f}° from vertical")
        print(
            f"  Launch pitch:      {config.min_launch_pitch_deg:.1f}° to "
            f"{config.max_launch_pitch_deg:.1f}° above horizontal"
        )
        print(f"  Ball mass:         {config.ball_mass:.3f} kg")
        print(f"  Ball diameter:     {config.ball_diameter * 1000:.1f} mm")
        print(f"  Drag coefficient:  {config.drag_coefficient}")
        if optimize_steep_entry:
            print(f"  Max launch speed:  {LOB_MAX_LAUNCH_SPEED_MPS:.1f} m/s")
        print()
        print(f"Distance range: {min_distance:.1f}m to {max_distance:.1f}m, step {step:.2f}m")
        print(f"Total points: {len(distances)}")
        print()
        print("-" * 70)
        print(f"{'Distance':>8}  {'Velocity':>10}  {'RPM':>8}  {'Pitch':>8}  {'Time':>8}  {'Entry':>8}")
        print(f"{'(m)':>8}  {'(m/s)':>10}  {'':>8}  {'(deg)':>8}  {'(s)':>8}  {'(deg)':>8}")
        print("-" * 70)

    # Solve for each distance
    entries = []
    success_count = 0
    failed_distances = []
    for dist in distances:
        failure_reason = None
        selected_entry_limit_deg = config.max_entry_angle_deg
        if optimize_steep_entry:
            result, selected_entry_limit_deg, failure_reason = solve_steepest_feasible_lob_trajectory(
                dist, config
            )
        else:
            try:
                result = solve_trajectory(dist, config, verbose=False)
            except RuntimeError as error:
                result = None
                failure_reason = str(error)

        if result is not None:
            success_count += 1
            rpm = velocity_to_rpm(result.velocity)

            entry = {
                "distance": float(dist),
                "velocity": round(result.velocity, 4),
                "pitch": round(result.pitch_rad, 6),
                "flight_time": round(result.flight_time, 4),
                "entry_angle_deg": round(result.entry_angle_deg, 4),
                "entry_constraint_deg": round(selected_entry_limit_deg, 4),
            }
            entries.append(entry)

            if verbose:
                print(
                    f"{dist:8.2f}  {result.velocity:10.3f}  {rpm:8.0f}  "
                    f"{result.pitch_deg:8.2f}  {result.flight_time:8.3f}  "
                    f"{result.entry_angle_deg:8.2f}"
                )
        else:
            failed_distances.append((float(dist), failure_reason))
            if verbose:
                print(f"{dist:8.2f}  {'NO SOLUTION':^54}")

    if verbose:
        print("-" * 70)
        print(f"Solutions found: {success_count}/{len(distances)}")
        if failed_distances:
            first_failed = failed_distances[0][0]
            last_failed = failed_distances[-1][0]
            print(
                "Skipped distances without feasible solutions: "
                f"{len(failed_distances)} ({first_failed:.2f}m to {last_failed:.2f}m)"
            )
        print()

    if not entries:
        raise RuntimeError(
            "No feasible constrained trajectories in requested range. "
            "Relax constraints or adjust range."
        )

    # Build lookup table structure
    lookup_table = {
        "metadata": {
            "description": "FRC 2026 shooter trajectory lookup table",
            "generated_by": "generate_lookup_table.py",
            "physics_model": "3D trajectory with magnitude-based drag",
            "optimization": (
                f"Steepest feasible entry angle (lob, <= {LOB_MAX_LAUNCH_SPEED_MPS:.1f} m/s launch)"
                if optimize_steep_entry
                else "Minimize flight time (Sleipnir/jormungandr)"
            ),
            "failed_distance_count": len(failed_distances),
        },
        "config": {
            "target_height_m": config.target_height,
            "shooter_height_m": config.shooter_height,
            "max_entry_angle_deg": config.max_entry_angle_deg,
            "min_launch_pitch_deg": config.min_launch_pitch_deg,
            "max_launch_pitch_deg": config.max_launch_pitch_deg,
            "ball_mass_kg": config.ball_mass,
            "ball_diameter_m": config.ball_diameter,
            "drag_coefficient": config.drag_coefficient,
            "air_density_kg_m3": config.air_density,
        },
        "range": {
            "min_distance_m": float(min_distance),
            "max_distance_m": float(max_distance),
            "step_m": float(step),
        },
        "entries": [e for e in entries if e is not None],
    }

    return lookup_table


def generate_vector_lookup_table(lookup_table: dict) -> dict:
    """
    Build a vector lookup table from scalar velocity/pitch entries.

    The generated velocity vectors are in shooter coordinates:
    +x forward, +y left, +z up.

    Args:
        lookup_table: Scalar lookup table dictionary

    Returns:
        Vector lookup table dictionary
    """
    config = lookup_table["config"]
    min_pitch_rad = math.radians(config["min_launch_pitch_deg"])
    max_pitch_rad = math.radians(config["max_launch_pitch_deg"])
    pitch_tolerance_rad = math.radians(0.05)

    vector_entries = []
    for entry in lookup_table["entries"]:
        speed = entry["velocity"]
        pitch = float(entry["pitch"])
        if pitch < min_pitch_rad - pitch_tolerance_rad or pitch > max_pitch_rad + pitch_tolerance_rad:
            raise RuntimeError(
                "Launch pitch out of bounds while generating vector lookup table: "
                f"distance={entry['distance']:.2f}m, pitch={math.degrees(pitch):.3f}°, "
                f"allowed=[{math.degrees(min_pitch_rad):.3f}°, {math.degrees(max_pitch_rad):.3f}°]"
            )
        pitch = max(min_pitch_rad, min(max_pitch_rad, pitch))

        horizontal_speed = speed * math.cos(pitch)
        vertical_speed = speed * math.sin(pitch)
        x = round(horizontal_speed, 5)
        z = round(vertical_speed, 5)

        vector_entries.append(
            {
                "distance": entry["distance"],
                "velocity_vector_mps": {
                    "x": x,
                    "y": 0.0,
                    "z": z,
                },
                "speed_mps": round(speed, 4),
                "pitch_rad": round(pitch, 6),
                "flight_time": entry["flight_time"],
                "entry_angle_deg": entry.get("entry_angle_deg"),
                "entry_constraint_deg": entry.get("entry_constraint_deg"),
            }
        )

    return {
        "metadata": {
            "description": "FRC 2026 shooter trajectory vector lookup table",
            "generated_by": "generate_lookup_table.py",
            "physics_model": lookup_table["metadata"]["physics_model"],
            "optimization": lookup_table["metadata"]["optimization"],
            "frame": "shooter_coordinates",
            "axis_convention": "+x forward, +y left, +z up",
        },
        "config": lookup_table["config"],
        "range": lookup_table["range"],
        "entries": vector_entries,
    }


def main():
    if len(sys.argv) == 1:
        generate_default_profiles(verbose=True)
        return 0

    parser = argparse.ArgumentParser(
        description="Generate FRC 2026 shooter lookup table",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--min", type=float, default=HUB_DEFAULT_MIN_DISTANCE_M,
        help="Minimum distance in meters"
    )
    parser.add_argument(
        "--max", type=float, default=HUB_DEFAULT_MAX_DISTANCE_M,
        help="Maximum distance in meters"
    )
    parser.add_argument(
        "--step", type=float, default=HUB_DEFAULT_STEP_M,
        help="Distance step in meters"
    )
    parser.add_argument(
        "--output", "-o", type=str, default=None,
        help="Output vector JSON file path"
    )
    parser.add_argument(
        "--target-height", type=float, default=None,
        help="Target height in meters (default: 56 3/8 inches)"
    )
    parser.add_argument(
        "--shooter-height", type=float, default=None,
        help="Shooter height in meters (default: 21 inches)"
    )
    parser.add_argument(
        "--entry-angle", type=float, default=None,
        help="Maximum allowed entry angle from vertical in degrees"
    )
    parser.add_argument(
        "--min-launch-pitch", type=float, default=None,
        help="Minimum launch pitch above horizontal in degrees"
    )
    parser.add_argument(
        "--max-launch-pitch", type=float, default=None,
        help="Maximum launch pitch above horizontal in degrees"
    )
    parser.add_argument(
        "--quiet", "-q", action="store_true",
        help="Suppress progress output"
    )

    args = parser.parse_args()

    # Build config
    config = ShooterConfig()
    if args.target_height is not None:
        config.target_height = args.target_height
    if args.shooter_height is not None:
        config.shooter_height = args.shooter_height
    if args.entry_angle is not None:
        config.max_entry_angle_deg = args.entry_angle
    if args.min_launch_pitch is not None:
        config.min_launch_pitch_deg = args.min_launch_pitch
    if args.max_launch_pitch is not None:
        config.max_launch_pitch_deg = args.max_launch_pitch

    # Determine output path
    if args.output:
        output_path = args.output
    else:
        output_path = default_output_path("hub_lookup_vector.json")

    write_lookup_table(
        output_path=output_path,
        min_distance=args.min,
        max_distance=args.max,
        step=args.step,
        config=config,
        verbose=not args.quiet,
    )

    return 0


if __name__ == "__main__":
    sys.exit(main())
