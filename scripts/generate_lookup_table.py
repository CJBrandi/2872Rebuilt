#!/usr/bin/env python3
"""
Generate shooting lookup table for FRC 2026.

Uses the trajectory optimization backend to compute optimal shooting parameters
for distances from 0.5m to 6.0m in 0.1m increments.

Usage:
    python generate_lookup_table.py [--output PATH] [--min MIN] [--max MAX] [--step STEP]

Output:
    JSON file (hub_lookup.json) with lookup table. Each entry contains:
    - distance: horizontal distance to target (m)
    - velocity: ball exit velocity (m/s)
    - pitch: hood/pitch angle (radians)
    - flight_time: time of flight (seconds)
"""

import argparse
import json
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


def generate_lookup_table(
    min_distance: float = 0.5,
    max_distance: float = 6.0,
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
        print(f"  Max entry angle:   {config.wall_angle_deg:.1f}° from vertical")
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


def export_java_constants(lookup_table: dict, output_path: str) -> None:
    """
    Export lookup table as Java constant arrays for direct embedding.

    Args:
        lookup_table: Lookup table dictionary
        output_path: Path to write Java file
    """
    entries = lookup_table["entries"]

    if not entries:
        print("No valid entries to export")
        return

    distances = [e["distance"] for e in entries]
    velocities = [e["velocity"] for e in entries]
    pitches = [e["pitch"] for e in entries]

    with open(output_path, 'w') as f:
        f.write("// Auto-generated by generate_lookup_table_2026.py\n")
        f.write("// DO NOT EDIT MANUALLY\n\n")

        f.write("public final class ShooterLookupTable {\n")
        f.write("    private ShooterLookupTable() {}\n\n")

        # Distance array
        f.write("    /** Distances in meters */\n")
        f.write("    public static final double[] DISTANCES = {\n        ")
        f.write(", ".join(f"{d:.2f}" for d in distances))
        f.write("\n    };\n\n")

        # Velocity array
        f.write("    /** Exit velocities in m/s */\n")
        f.write("    public static final double[] VELOCITIES = {\n        ")
        f.write(", ".join(f"{v:.4f}" for v in velocities))
        f.write("\n    };\n\n")

        # Pitch array
        f.write("    /** Pitch angles in radians */\n")
        f.write("    public static final double[] PITCHES = {\n        ")
        f.write(", ".join(f"{p:.6f}" for p in pitches))
        f.write("\n    };\n")

        f.write("}\n")

    print(f"Java constants written to: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Generate FRC 2026 shooter lookup table",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--min", type=float, default=0.5,
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
        help="Output JSON file path"
    )
    parser.add_argument(
        "--java", type=str, default=None,
        help="Also generate Java constants file at this path"
    )
    parser.add_argument(
        "--target-height", type=float, default=None,
        help="Target height in meters (default: 72 inches)"
    )
    parser.add_argument(
        "--shooter-height", type=float, default=None,
        help="Shooter height in meters (default: 20 inches)"
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
            script_dir, "..", "src", "main", "deploy", "hub_lookup.json"
        )
        output_path = os.path.normpath(output_path)

    # Ensure directory exists
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    # Write JSON
    with open(output_path, 'w') as f:
        json.dump(lookup_table, f, indent=2)

    if not args.quiet:
        print(f"Lookup table written to: {output_path}")

    # Generate Java file if requested
    if args.java:
        export_java_constants(lookup_table, args.java)

    return 0


if __name__ == "__main__":
    sys.exit(main())
