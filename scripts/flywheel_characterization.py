#!/usr/bin/env python3
"""
Interactive Flywheel Characterization Script

Guides users through characterizing their flywheel by:
1. Explaining what to log in AdvantageScope
2. Collecting distance measurements across RPM range
3. Computing efficiency ratios

Usage:
    python flywheel_characterization.py           # Interactive mode
    python flywheel_characterization.py --export  # Export results to JSON
"""

import argparse
import json
import math
import os
import sys
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

# Physical constants
WHEEL_RADIUS_M = 0.0508  # 2 inches in meters (typical)
GRAVITY = 9.81  # m/s^2

# Characterization parameters
RPM_MIN = 1000
RPM_MAX = 4000
RPM_STEP = 500
MEASUREMENTS_PER_RPM = 3


@dataclass
class Measurement:
    """Single distance measurement at a specific RPM."""
    rpm: float
    distance_m: float
    measurement_num: int  # 1, 2, or 3


@dataclass
class CharacterizationResult:
    """Results for a single RPM with multiple measurements."""
    rpm: float
    theoretical_velocity_mps: float
    measurements: list[float] = field(default_factory=list)
    actual_velocities: list[float] = field(default_factory=list)
    efficiencies: list[float] = field(default_factory=list)
    avg_actual_velocity: float = 0.0
    avg_efficiency: float = 0.0
    std_efficiency: float = 0.0


def rpm_to_rad_per_sec(rpm: float) -> float:
    """Convert RPM to radians per second."""
    return rpm * 2 * math.pi / 60


def rad_per_sec_to_rpm(rad_per_sec: float) -> float:
    """Convert radians per second to RPM."""
    return rad_per_sec * 60 / (2 * math.pi)


def theoretical_exit_velocity(wheel_rpm: float, wheel_radius_m: float) -> float:
    """
    Calculate theoretical ball exit velocity from wheel RPM.
    Assumes perfect contact (no slip) between wheel surface and ball.
    v = omega * r
    """
    omega_rad_per_sec = rpm_to_rad_per_sec(wheel_rpm)
    return omega_rad_per_sec * wheel_radius_m


def exit_velocity_from_distance(
    horizontal_distance_m: float,
    launch_angle_deg: float,
    shooter_height_m: float = 0.5,
    target_height_m: float = 0.0,
) -> float:
    """
    Back-calculate exit velocity from shot distance using projectile motion.
    Ignores air drag for simplicity in this characterization.
    """
    angle_rad = math.radians(launch_angle_deg)
    delta_h = target_height_m - shooter_height_m

    cos_theta = math.cos(angle_rad)
    sin_theta = math.sin(angle_rad)

    x = horizontal_distance_m
    numerator = 0.5 * GRAVITY * x**2
    denominator = (cos_theta**2) * (x * math.tan(angle_rad) - delta_h)

    if denominator <= 0:
        raise ValueError("Invalid trajectory - ball cannot reach target")

    v_squared = numerator / denominator
    return math.sqrt(v_squared)


def calculate_efficiency(wheel_rpm: float, actual_velocity_mps: float, wheel_radius_m: float) -> float:
    """Calculate efficiency factor (actual / theoretical)."""
    theoretical = theoretical_exit_velocity(wheel_rpm, wheel_radius_m)
    if theoretical == 0:
        return 0.0
    return actual_velocity_mps / theoretical


def print_header():
    """Print the script header."""
    print("=" * 70)
    print("  FLYWHEEL CHARACTERIZATION TOOL")
    print("=" * 70)
    print()
    print("This script will guide you through characterizing your flywheel")
    print("to determine the efficiency ratio (actual/theoretical exit velocity).")
    print()


def print_advantagescope_instructions():
    """Print instructions for AdvantageScope logging."""
    print("=" * 70)
    print("  STEP 1: ADVANTAGESCOPE SETUP")
    print("=" * 70)
    print()
    print("Before starting, ensure these fields are being logged in your robot code:")
    print()
    print("  Required fields to log:")
    print("    - Shooter/Flywheel/MeasuredVelocityRPM  (actual wheel speed)")
    print("    - Shooter/Flywheel/TargetWheelVelocityRPM  (setpoint)")
    print("    - Shooter/Flywheel/appliedVolts  (motor voltage)")
    print("    - Shooter/Flywheel/currentAmps  (current draw)")
    print()
    print("  In AdvantageScope:")
    print("    1. Open the Log field selector")
    print("    2. Add Shooter/Flywheel/MeasuredVelocityRPM to a line graph")
    print("    3. Watch this value to verify the flywheel stabilizes before shooting")
    print()
    print("  IMPORTANT:")
    print("    - Wait for MeasuredVelocityRPM to reach the target and stabilize")
    print("    - Look for the value to flatline before firing")
    print("    - This ensures consistent, accurate measurements")
    print()
    input("Press ENTER when you're ready to continue...")
    print()


def get_float_input(prompt: str, default: Optional[float] = None, min_val: Optional[float] = None, max_val: Optional[float] = None) -> float:
    """Get a float input from the user with validation."""
    while True:
        try:
            if default is not None:
                user_input = input(f"{prompt} [{default}]: ").strip()
                if user_input == "":
                    return default
            else:
                user_input = input(f"{prompt}: ").strip()
            
            value = float(user_input)
            
            if min_val is not None and value < min_val:
                print(f"  Error: Value must be at least {min_val}")
                continue
            if max_val is not None and value > max_val:
                print(f"  Error: Value must be at most {max_val}")
                continue
                
            return value
        except ValueError:
            print("  Error: Please enter a valid number")


def collect_measurements_interactive(wheel_radius_m: float, shooter_height_m: float, launch_angle_deg: float) -> list[Measurement]:
    """Collect measurements interactively from the user."""
    measurements = []
    
    rpms = list(range(RPM_MIN, RPM_MAX + RPM_STEP, RPM_STEP))
    total_shots = len(rpms) * MEASUREMENTS_PER_RPM
    shot_count = 0
    
    print("=" * 70)
    print("  STEP 2: DATA COLLECTION")
    print("=" * 70)
    print()
    print(f"Configuration:")
    print(f"  RPM range: {RPM_MIN} to {RPM_MAX} (step: {RPM_STEP})")
    print(f"  Measurements per RPM: {MEASUREMENTS_PER_RPM}")
    print(f"  Total shots needed: {total_shots}")
    print()
    print("For each shot:")
    print("  1. Set your flywheel to the target RPM")
    print("  2. Watch MeasuredVelocityRPM in AdvantageScope until it stabilizes")
    print("  3. Fire a shot")
    print("  4. Measure the horizontal distance traveled")
    print("  5. Enter the distance below")
    print()
    input("Press ENTER to start collecting data...")
    print()
    
    for rpm in rpms:
        for measurement_num in range(1, MEASUREMENTS_PER_RPM + 1):
            shot_count += 1
            
            print("-" * 70)
            print(f"Shot {shot_count} of {total_shots}")
            print(f"Target RPM: {rpm} (Measurement {measurement_num} of {MEASUREMENTS_PER_RPM})")
            print("-" * 70)
            print()
            
            print(f"Instructions:")
            print(f"  1. Set flywheel to {rpm} RPM")
            print(f"  2. Watch Shooter/Flywheel/MeasuredVelocityRPM in AdvantageScope")
            print(f"  3. Wait until the value stabilizes (flatline on graph)")
            print(f"  4. Fire your shot")
            print()
            
            distance = get_float_input(
                f"Enter horizontal distance traveled (meters)",
                min_val=0.1
            )
            
            measurements.append(Measurement(rpm=rpm, distance_m=distance, measurement_num=measurement_num))
            
            print(f"  Recorded: {rpm} RPM -> {distance:.2f} m")
            print()
            
            if shot_count < total_shots:
                cont = input("Press ENTER for next shot (or 'q' to quit): ").strip().lower()
                if cont == 'q':
                    print("\nQuitting early. Processing collected data...")
                    return measurements
            print()
    
    return measurements


def analyze_measurements(measurements: list[Measurement], wheel_radius_m: float, shooter_height_m: float, launch_angle_deg: float) -> list[CharacterizationResult]:
    """Analyze collected measurements and compute results."""
    # Group measurements by RPM
    rpm_groups: dict[float, list[Measurement]] = {}
    for m in measurements:
        if m.rpm not in rpm_groups:
            rpm_groups[m.rpm] = []
        rpm_groups[m.rpm].append(m)
    
    results = []
    for rpm in sorted(rpm_groups.keys()):
        group = rpm_groups[rpm]
        theoretical = theoretical_exit_velocity(rpm, wheel_radius_m)
        
        result = CharacterizationResult(
            rpm=rpm,
            theoretical_velocity_mps=theoretical
        )
        
        for m in group:
            try:
                actual_velocity = exit_velocity_from_distance(
                    m.distance_m, launch_angle_deg, shooter_height_m, 0.0
                )
                efficiency = calculate_efficiency(rpm, actual_velocity, wheel_radius_m)
                
                result.measurements.append(m.distance_m)
                result.actual_velocities.append(actual_velocity)
                result.efficiencies.append(efficiency)
            except ValueError as e:
                print(f"  Warning: Could not calculate velocity for RPM {rpm}, measurement {m.measurement_num}: {e}")
        
        if result.efficiencies:
            result.avg_actual_velocity = float(np.mean(result.actual_velocities))
            result.avg_efficiency = float(np.mean(result.efficiencies))
            result.std_efficiency = float(np.std(result.efficiencies))
        
        results.append(result)
    
    return results


def print_results(results: list[CharacterizationResult]):
    """Print the characterization results."""
    print()
    print("=" * 70)
    print("  CHARACTERIZATION RESULTS")
    print("=" * 70)
    print()
    
    print(f"{'RPM':>6}  {'Theoretical':>12}  {'Avg Actual':>12}  {'Measurements':>14}  {'Avg Eff':>10}  {'Std Dev':>10}")
    print(f"{'':>6}  {'Vel (m/s)':>12}  {'Vel (m/s)':>12}  {'(m)':>14}  {'':>10}  {'':>10}")
    print("-" * 70)
    
    for r in results:
        measurements_str = ", ".join([f"{m:.2f}" for m in r.measurements])
        if len(measurements_str) > 14:
            measurements_str = measurements_str[:11] + "..."
        
        print(f"{r.rpm:6.0f}  {r.theoretical_velocity_mps:12.2f}  {r.avg_actual_velocity:12.2f}  {measurements_str:>14}  {r.avg_efficiency:9.1%}  {r.std_efficiency:9.1%}")
    
    print("-" * 70)
    
    # Overall statistics
    all_efficiencies = []
    for r in results:
        all_efficiencies.extend(r.efficiencies)
    
    if all_efficiencies:
        print()
        print("Overall Statistics:")
        print(f"  Mean efficiency:     {np.mean(all_efficiencies):.2%}")
        print(f"  Standard deviation:  {np.std(all_efficiencies):.2%}")
        print(f"  Min efficiency:      {min(all_efficiencies):.2%}")
        print(f"  Max efficiency:      {max(all_efficiencies):.2%}")
    
    print()


def export_to_json(results: list[CharacterizationResult], wheel_radius_m: float, output_path: str):
    """Export results to JSON file."""
    data = {
        "metadata": {
            "description": "Flywheel exit velocity characterization data",
            "wheel_radius_m": wheel_radius_m,
            "rpm_range": {"min": RPM_MIN, "max": RPM_MAX, "step": RPM_STEP},
            "measurements_per_rpm": MEASUREMENTS_PER_RPM,
        },
        "results": [
            {
                "rpm": r.rpm,
                "theoretical_velocity_mps": r.theoretical_velocity_mps,
                "measurements_m": r.measurements,
                "actual_velocities_mps": r.actual_velocities,
                "efficiencies": r.efficiencies,
                "avg_actual_velocity_mps": r.avg_actual_velocity,
                "avg_efficiency": r.avg_efficiency,
                "std_efficiency": r.std_efficiency,
            }
            for r in results
        ],
        "overall": {
            "mean_efficiency": np.mean([e for r in results for e in r.efficiencies]),
            "std_efficiency": np.std([e for r in results for e in r.efficiencies]),
        }
    }
    
    with open(output_path, 'w') as f:
        json.dump(data, f, indent=2)
    print(f"Results exported to: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Interactive flywheel characterization tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--export", "-o", type=str,
        help="Export results to JSON file"
    )
    parser.add_argument(
        "--wheel-radius", type=float, default=WHEEL_RADIUS_M,
        help=f"Wheel radius in meters (default: {WHEEL_RADIUS_M})"
    )
    parser.add_argument(
        "--shooter-height", type=float, default=0.5,
        help="Shooter height in meters (default: 0.5)"
    )
    parser.add_argument(
        "--launch-angle", type=float, default=45.0,
        help="Launch angle in degrees (default: 45)"
    )
    
    args = parser.parse_args()
    
    print_header()
    
    # Setup phase
    print("First, let's configure your flywheel parameters:")
    print()
    wheel_radius_m = get_float_input("Wheel radius (meters)", default=args.wheel_radius, min_val=0.01)
    shooter_height_m = get_float_input("Shooter height (meters)", default=args.shooter_height, min_val=0.0)
    launch_angle_deg = get_float_input("Launch angle (degrees)", default=args.launch_angle, min_val=0.0, max_val=90.0)
    print()
    
    # AdvantageScope instructions
    print_advantagescope_instructions()
    
    # Data collection
    measurements = collect_measurements_interactive(wheel_radius_m, shooter_height_m, launch_angle_deg)
    
    if not measurements:
        print("No measurements collected. Exiting.")
        return 1
    
    # Analysis
    results = analyze_measurements(measurements, wheel_radius_m, shooter_height_m, launch_angle_deg)
    
    # Results
    print_results(results)
    
    # Export
    if args.export:
        export_to_json(results, wheel_radius_m, args.export)
    else:
        export_choice = input("Export results to JSON? (y/n): ").strip().lower()
        if export_choice in ('y', 'yes'):
            default_filename = "flywheel_characterization.json"
            filename = input(f"Filename [{default_filename}]: ").strip()
            if not filename:
                filename = default_filename
            export_to_json(results, wheel_radius_m, filename)
    
    print()
    print("Characterization complete!")
    print("Use the efficiency values to tune your flywheel feedforward.")
    print()
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
