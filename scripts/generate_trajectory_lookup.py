#!/usr/bin/env python3
"""
For each horizontal distance, finds the exit velocity and pitch angle such that:
1. The ball hits the target at the specified height
2. The ball arrives at exactly 45 degrees below horizontal

Uses the same physics model as FuelTrajectory.java (component-wise drag, RK4 integration).

Usage:
    python generate_trajectory_lookup.py [--step STEP] [--min MIN] [--max MAX]

Examples:
    python generate_trajectory_lookup.py                    # Default: 0.25m to 15m, step 0.25m
    python generate_trajectory_lookup.py --step 0.1         # Finer resolution
    python generate_trajectory_lookup.py --step 0.5 --max 12  # Coarser, shorter range
"""

import argparse
import json
import numpy as np
from scipy.optimize import minimize
from dataclasses import dataclass
from typing import Optional, Tuple
import os

# Physical constants (matching FuelTrajectory.java)
GRAVITY = 9.81  # m/s²
AIR_DENSITY = 1.225  # kg/m³ at sea level
BALL_RADIUS = 0.0762  # 3 inches in meters
WHEEL_RADIUS = 0.1016  # 4 inches in meters
BALL_MASS = 0.226796  # 0.5 lb in kg
BALL_CROSS_SECTION = np.pi * BALL_RADIUS ** 2

# Aerodynamic coefficients (component-wise drag model)
DRAG_COEFFICIENT_H = 0.47
DRAG_COEFFICIENT_V = 0.47

# Precompute drag factors
DRAG_FACTOR_H = 0.5 * AIR_DENSITY * DRAG_COEFFICIENT_H * BALL_CROSS_SECTION / BALL_MASS
DRAG_FACTOR_V = 0.5 * AIR_DENSITY * DRAG_COEFFICIENT_V * BALL_CROSS_SECTION / BALL_MASS

# Simulation parameters
SIMULATION_DT = 0.0005  # 0.5ms integration step (matching Java)
MAX_SIMULATION_TIME = 5.0

# Target parameters
TARGET_POSITION = np.array([4.6256194, 4.0346376, 1.75])
TARGET_HEIGHT = TARGET_POSITION[2]
REQUIRED_IMPACT_ANGLE = np.radians(-45)  # 45 degrees below horizontal (negative = descending)

# Shooter parameters
DEFAULT_SHOOTER_HEIGHT = 0.5  # meters


@dataclass
class TrajectoryState:
    """State of the ball at a point in time."""
    time: float
    position: np.ndarray  # [x, z] for 2D
    velocity: np.ndarray  # [vx, vz] for 2D


@dataclass
class TrajectoryResult:
    """Result of trajectory simulation."""
    states: list
    hit_target_height: bool
    time_at_height: float
    position_at_height: Optional[np.ndarray]
    velocity_at_height: Optional[np.ndarray]
    impact_angle: Optional[float]  # radians, negative = descending


def compute_derivatives(state: np.ndarray) -> np.ndarray:
    """
    Compute derivatives for [x, z, vx, vz] state vector.
    Uses component-wise drag matching FuelTrajectory.java.
    """
    vx, vz = state[2], state[3]

    # Gravity
    ax = 0.0
    az = -GRAVITY

    # Component-wise drag
    if abs(vx) > 1e-6:
        ax -= DRAG_FACTOR_H * vx * abs(vx)
    if abs(vz) > 1e-6:
        az -= DRAG_FACTOR_V * vz * abs(vz)

    return np.array([vx, vz, ax, az])


def rk4_step(state: np.ndarray, dt: float) -> np.ndarray:
    """Perform one RK4 integration step."""
    k1 = compute_derivatives(state)
    k2 = compute_derivatives(state + dt / 2 * k1)
    k3 = compute_derivatives(state + dt / 2 * k2)
    k4 = compute_derivatives(state + dt * k3)
    return state + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def simulate_trajectory(
    initial_velocity: float,
    pitch_rad: float,
    shooter_height: float,
    target_height: float
) -> TrajectoryResult:
    """
    Simulate ball trajectory from shooter to when it crosses target height (descending).

    Args:
        initial_velocity: Ball exit speed in m/s
        pitch_rad: Launch angle in radians (positive = upward)
        shooter_height: Height of shooter exit point in meters
        target_height: Target height to find crossing point

    Returns:
        TrajectoryResult with simulation data
    """
    # Initial conditions
    vx = initial_velocity * np.cos(pitch_rad)
    vz = initial_velocity * np.sin(pitch_rad)
    state = np.array([0.0, shooter_height, vx, vz])  # [x, z, vx, vz]

    states = [TrajectoryState(0.0, state[:2].copy(), state[2:].copy())]

    t = 0.0
    max_height_reached = shooter_height
    descending = False

    while t < MAX_SIMULATION_TIME:
        prev_state = state.copy()
        state = rk4_step(state, SIMULATION_DT)
        t += SIMULATION_DT

        # Track if we've started descending
        if state[1] > max_height_reached:
            max_height_reached = state[1]
        elif state[1] < max_height_reached - 0.01:  # Small threshold
            descending = True

        # Check if we crossed target height while descending
        if descending and prev_state[1] >= target_height > state[1]:
            # Interpolate to find exact crossing point
            alpha = (target_height - prev_state[1]) / (state[1] - prev_state[1])
            cross_time = (t - SIMULATION_DT) + alpha * SIMULATION_DT
            cross_pos = prev_state[:2] + alpha * (state[:2] - prev_state[:2])
            cross_vel = prev_state[2:] + alpha * (state[2:] - prev_state[2:])

            # Impact angle (negative = descending)
            impact_angle = np.arctan2(cross_vel[1], cross_vel[0])

            return TrajectoryResult(
                states=states,
                hit_target_height=True,
                time_at_height=cross_time,
                position_at_height=cross_pos,
                velocity_at_height=cross_vel,
                impact_angle=impact_angle
            )

        # Stop if ball hits ground
        if state[1] < 0:
            break

        # Store state periodically
        if len(states) < t / 0.005:
            states.append(TrajectoryState(t, state[:2].copy(), state[2:].copy()))

    return TrajectoryResult(
        states=states,
        hit_target_height=False,
        time_at_height=-1,
        position_at_height=None,
        velocity_at_height=None,
        impact_angle=None
    )


def solve_for_distance(
    target_distance: float,
    target_height: float,
    shooter_height: float,
    required_impact_angle: float = REQUIRED_IMPACT_ANGLE
) -> Optional[Tuple[float, float, float]]:
    """
    Solve for (pitch, velocity) that hits target at specified distance and angle.

    Args:
        target_distance: Horizontal distance to target in meters
        target_height: Target height in meters
        shooter_height: Shooter height in meters
        required_impact_angle: Required impact angle in radians (negative = descending)

    Returns:
        Tuple of (pitch_rad, velocity_mps, flight_time) or None if no solution
    """

    def objective(params):
        """
        Objective function: minimize squared errors in distance and impact angle.
        params = [pitch_rad, velocity_mps]
        """
        pitch, velocity = params

        # Bounds check
        if velocity < 1 or velocity > 50:
            return 1e6
        if pitch < np.radians(10) or pitch > np.radians(85):
            return 1e6

        result = simulate_trajectory(velocity, pitch, shooter_height, target_height)

        if not result.hit_target_height:
            return 1e6

        # Error in horizontal distance
        actual_distance = result.position_at_height[0]
        distance_error = (actual_distance - target_distance) ** 2

        # Error in impact angle
        angle_error = (result.impact_angle - required_impact_angle) ** 2

        # Weight angle error more heavily (in radians^2 vs meters^2)
        return distance_error + 100 * angle_error

    # Try multiple initial guesses
    best_result = None
    best_cost = float('inf')

    # Estimate initial velocity using simple projectile motion (no drag)
    # v = sqrt(g * d / sin(2*theta)) for 45 degree launch
    estimated_v = np.sqrt(GRAVITY * target_distance / 0.9)  # Approximate

    initial_guesses = [
        (np.radians(45), estimated_v),
        (np.radians(50), estimated_v * 1.1),
        (np.radians(55), estimated_v * 1.2),
        (np.radians(60), estimated_v * 1.3),
        (np.radians(40), estimated_v * 0.9),
        (np.radians(65), estimated_v * 1.4),
        (np.radians(70), estimated_v * 1.5),
    ]

    for initial_guess in initial_guesses:
        try:
            result = minimize(
                objective,
                initial_guess,
                method='Nelder-Mead',
                options={'xatol': 1e-6, 'fatol': 1e-8, 'maxiter': 1000}
            )

            if result.fun < best_cost:
                best_cost = result.fun
                best_result = result
        except Exception:
            continue

    if best_result is None or best_cost > 0.01:  # Tolerance: ~10cm distance, ~0.5 deg angle
        return None

    pitch, velocity = best_result.x

    # Verify solution
    verification = simulate_trajectory(velocity, pitch, shooter_height, target_height)
    if not verification.hit_target_height:
        return None

    return (pitch, velocity, verification.time_at_height)


def generate_lookup_table(
    min_distance: float = 1.0,
    max_distance: float = 10.0,
    distance_step: float = 0.25,
    shooter_height: float = DEFAULT_SHOOTER_HEIGHT,
    target_height: float = TARGET_HEIGHT
) -> dict:
    """
    Generate the lookup table for all distances.

    Returns:
        Dictionary in the format expected by TrajectoryLookupTable.java
    """
    distances = np.arange(min_distance, max_distance + distance_step / 2, distance_step)

    # For this use case, we have a single target height
    heights = [target_height]

    data = []
    solutions = {}

    print(f"Generating lookup table for {len(distances)} distances...")
    print(f"Target height: {target_height}m, Shooter height: {shooter_height}m")
    print(f"Required impact angle: {np.degrees(REQUIRED_IMPACT_ANGLE):.1f} degrees")
    print()

    for i, distance in enumerate(distances):
        row = []

        solution = solve_for_distance(distance, target_height, shooter_height)

        if solution is not None:
            pitch, velocity, flight_time = solution

            entry = {
                "pitch": pitch,
                "velocity": velocity
            }
            row.append(entry)
            solutions[distance] = (pitch, velocity, flight_time)

            # Verify the solution
            verify = simulate_trajectory(velocity, pitch, shooter_height, target_height)
            actual_dist = verify.position_at_height[0]
            actual_angle = np.degrees(verify.impact_angle)

            print(f"  d={distance:5.2f}m: pitch={np.degrees(pitch):5.1f}° vel={velocity:5.2f}m/s "
                  f"tof={flight_time:.3f}s | actual_d={actual_dist:.3f}m angle={actual_angle:.1f}°")
        else:
            row.append(None)
            print(f"  d={distance:5.2f}m: NO SOLUTION")

        data.append(row)

    # Build output structure
    lookup_table = {
        "shooter_height": shooter_height,
        "target_height": target_height,
        "target_position": TARGET_POSITION.tolist(),
        "required_impact_angle_deg": np.degrees(REQUIRED_IMPACT_ANGLE),
        "distances": distances.tolist(),
        "heights": heights,
        "data": data
    }

    return lookup_table


def main():
    parser = argparse.ArgumentParser(
        description="Generate trajectory lookup table for FRC shooter",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "--min", type=float, default=0.25,
        help="Minimum distance in meters"
    )
    parser.add_argument(
        "--max", type=float, default=15.0,
        help="Maximum distance in meters"
    )
    parser.add_argument(
        "--step", type=float, default=0.25,
        help="Distance step size in meters"
    )
    parser.add_argument(
        "--shooter-height", type=float, default=DEFAULT_SHOOTER_HEIGHT,
        help="Shooter exit height in meters"
    )
    parser.add_argument(
        "--output", type=str, default=None,
        help="Output file path (default: src/main/deploy/trajectory_lookup.json)"
    )
    args = parser.parse_args()

    print("=" * 60)
    print("Trajectory Lookup Table Generator")
    print("=" * 60)
    print()
    print(f"Target position: ({TARGET_POSITION[0]:.4f}, {TARGET_POSITION[1]:.4f}, {TARGET_POSITION[2]:.4f})")
    print(f"Required impact angle: 45° below horizontal")
    print(f"Distance range: {args.min}m to {args.max}m, step {args.step}m")
    print()

    # Generate lookup table
    lookup_table = generate_lookup_table(
        min_distance=args.min,
        max_distance=args.max,
        distance_step=args.step,
        shooter_height=args.shooter_height,
        target_height=TARGET_HEIGHT
    )

    # Output path
    if args.output:
        output_path = args.output
    else:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_path = os.path.join(script_dir, "..", "src", "main", "deploy", "trajectory_lookup.json")
        output_path = os.path.normpath(output_path)

    # Ensure directory exists
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    # Write JSON
    with open(output_path, 'w') as f:
        json.dump(lookup_table, f, indent=2)

    print()
    print(f"Lookup table written to: {output_path}")
    print()

    # Summary statistics
    valid_entries = sum(1 for row in lookup_table["data"] for entry in row if entry is not None)
    total_entries = len(lookup_table["distances"]) * len(lookup_table["heights"])
    print(f"Valid solutions: {valid_entries}/{total_entries}")

    # Print sample verification
    print()
    print("=" * 60)
    print("Sample Verification")
    print("=" * 60)

    # Pick evenly spaced test distances within the range
    test_distances = np.linspace(args.min, args.max, min(5, len(lookup_table["distances"])))
    test_distances = [round(d / args.step) * args.step for d in test_distances]  # Snap to grid

    for dist in test_distances:
        idx = int(round((dist - args.min) / args.step))
        if 0 <= idx < len(lookup_table["data"]) and lookup_table["data"][idx][0] is not None:
            entry = lookup_table["data"][idx][0]
            pitch = entry["pitch"]
            velocity = entry["velocity"]

            result = simulate_trajectory(velocity, pitch, args.shooter_height, TARGET_HEIGHT)
            if result.hit_target_height:
                print(f"\nDistance {dist}m:")
                print(f"  Pitch: {np.degrees(pitch):.2f}° ({pitch:.4f} rad)")
                print(f"  Velocity: {velocity:.3f} m/s")
                print(f"  Flight time: {result.time_at_height:.3f}s")
                print(f"  Actual horizontal distance: {result.position_at_height[0]:.4f}m")
                print(f"  Impact angle: {np.degrees(result.impact_angle):.2f}°")
                print(f"  Impact velocity: vx={result.velocity_at_height[0]:.2f} vz={result.velocity_at_height[1]:.2f} m/s")


if __name__ == "__main__":
    main()
