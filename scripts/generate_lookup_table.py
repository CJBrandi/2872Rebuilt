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
    solve_trajectory,
    velocity_to_rpm,
)


def ensure_parent_dir(path: str) -> None:
    """Create parent directory for a file path if needed."""
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)


def generate_lookup_table(
    min_distance: float = 1.5,
    max_distance: float = 15.0,
    step: float = 0.1,
    config: Optional[ShooterConfig] = None,
    verbose: bool = True,
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
        print(f"  Wall angle limit:  {config.wall_angle_deg:.1f}° from vertical")
        print(f"  Entry margin:      {config.entry_angle_margin_deg:.1f}°")
        print(f"  Effective entry:   {config.effective_entry_angle_deg:.1f}° from vertical")
        print(f"  Ball mass:         {config.ball_mass:.3f} kg")
        print(f"  Ball diameter:     {config.ball_diameter * 1000:.1f} mm")
        print(f"  Drag coefficient:  {config.drag_coefficient}")
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

    for dist in distances:
        result = solve_trajectory(dist, config, verbose=False)

        if result is not None:
            success_count += 1
            rpm = velocity_to_rpm(result.velocity)

            entry = {
                "distance": float(dist),
                "velocity": round(result.velocity, 4),
                "pitch": round(result.pitch_rad, 6),
                "flight_time": round(result.flight_time, 4),
            }
            entries.append(entry)

            if verbose:
                print(f"{dist:8.2f}  {result.velocity:10.3f}  {rpm:8.0f}  "
                      f"{result.pitch_deg:8.2f}  {result.flight_time:8.3f}  "
                      f"{result.entry_angle_deg:8.2f}")
        else:
            entries.append(None)
            if verbose:
                print(f"{dist:8.2f}  {'NO SOLUTION':^54}")

    if verbose:
        print("-" * 70)
        print(f"Solutions found: {success_count}/{len(distances)}")
        print()

    # Build lookup table structure
    lookup_table = {
        "metadata": {
            "description": "FRC 2026 shooter trajectory lookup table",
            "generated_by": "generate_lookup_table.py",
            "physics_model": "3D trajectory with magnitude-based drag",
            "optimization": "Minimize flight time (Sleipnir/jormungandr)",
        },
        "config": {
            "target_height_m": config.target_height,
            "shooter_height_m": config.shooter_height,
            "max_entry_angle_deg": config.wall_angle_deg,
            "entry_angle_margin_deg": config.entry_angle_margin_deg,
            "effective_entry_angle_deg": config.effective_entry_angle_deg,
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
    vector_entries = []
    for entry in lookup_table["entries"]:
        speed = entry["velocity"]
        pitch = entry["pitch"]
        horizontal_speed = speed * math.cos(pitch)
        vertical_speed = speed * math.sin(pitch)

        vector_entries.append(
            {
                "distance": entry["distance"],
                "velocity_vector_mps": {
                    "x": round(horizontal_speed, 4),
                    "y": 0.0,
                    "z": round(vertical_speed, 4),
                },
                "speed_mps": round(speed, 4),
                "pitch_rad": round(pitch, 6),
                "flight_time": entry["flight_time"],
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
    parser = argparse.ArgumentParser(
        description="Generate FRC 2026 shooter lookup table",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--min", type=float, default=1.5,
        help="Minimum distance in meters"
    )
    parser.add_argument(
        "--max", type=float, default=6.0,
        help="Maximum distance in meters"
    )
    parser.add_argument(
        "--step", type=float, default=0.1,
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
        help="Shooter height in meters (default: 20 inches)"
    )
    parser.add_argument(
        "--entry-angle", type=float, default=None,
        help="Maximum wall entry angle from vertical in degrees (default: 45°)"
    )
    parser.add_argument(
        "--entry-angle-margin", type=float, default=None,
        help="Safety margin below wall angle in degrees (default: 5°)"
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
        config.wall_angle_deg = args.entry_angle
    if args.entry_angle_margin is not None:
        config.entry_angle_margin_deg = args.entry_angle_margin

    # Generate lookup table
    lookup_table = generate_lookup_table(
        min_distance=args.min,
        max_distance=args.max,
        step=args.step,
        config=config,
        verbose=not args.quiet,
    )

    # Determine output path
    if args.output:
        output_path = args.output
    else:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_path = os.path.join(
            script_dir, "..", "src", "main", "deploy", "hub_lookup_vector.json"
        )
        output_path = os.path.normpath(output_path)

    # Ensure directory exists
    ensure_parent_dir(output_path)

    # Write vector JSON
    vector_lookup_table = generate_vector_lookup_table(lookup_table)
    with open(output_path, 'w') as f:
        json.dump(vector_lookup_table, f, indent=2)
    if not args.quiet:
        print(f"Vector lookup table written to: {output_path}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
