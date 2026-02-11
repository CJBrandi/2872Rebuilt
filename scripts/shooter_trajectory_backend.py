#!/usr/bin/env python3
"""
FRC 2026 shooter trajectory optimization backend.

This module provides a reusable API for computing optimal shooting parameters
(velocity, pitch) for a given distance to target using nonlinear trajectory
optimization with accurate magnitude-based drag physics.

Usage:
    from shooter_trajectory_backend import solve_trajectory, ShooterConfig

    config = ShooterConfig()
    result = solve_trajectory(distance=3.0, config=config)
    if result is not None:
        print(f"Velocity: {result.velocity:.2f} m/s, Pitch: {result.pitch_deg:.1f}°")
"""

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np
from numpy.linalg import norm

# Try to import jormungandr (Sleipnir Python bindings)
try:
    from jormungandr.autodiff import block, sqrt
    from jormungandr.optimization import Problem
except ImportError:
    print("Error: jormungandr package not found.")
    print("Please install it with: pip install sleipnirgroup-jormungandr")
    raise


@dataclass
class ShooterConfig:
    """Configuration for the shooter trajectory solver."""

    # Target parameters
    target_height: float = 72.0 * 0.0254  # 1.829 m (72 inches)

    # Hexagon funnel geometry
    hexagon_opening_diameter: float = 42.0 * 0.0254  # 1.067 m (42 inches point-to-point)
    wall_angle_deg: float = 45.0  # degrees from vertical

    # Game piece physics (2026 ball)
    ball_mass: float = 0.227  # kg (0.5 lbs)
    ball_diameter: float = 5.93 * 0.0254  # m
    drag_coefficient: float = 0.47
    air_density: float = 1.204  # kg/m³ at sea level

    # Shooter configuration
    shooter_height: float = 20.0 * 0.0254  # 0.508 m (20 inches)

    # Optimization parameters
    num_timesteps: int = 50
    min_flight_time: float = 0.1  # seconds
    initial_time_guess: float = 1.0  # seconds

    @property
    def ball_radius(self) -> float:
        return self.ball_diameter / 2.0

    @property
    def cross_sectional_area(self) -> float:
        return math.pi * self.ball_radius ** 2

    @property
    def wall_angle_rad(self) -> float:
        return math.radians(self.wall_angle_deg)

    @property
    def hexagon_circumradius(self) -> float:
        return self.hexagon_opening_diameter / 2.0


@dataclass
class TrajectoryResult:
    """Result of trajectory optimization."""

    velocity: float  # m/s - initial ball velocity
    pitch_rad: float  # radians - launch angle above horizontal
    yaw_rad: float  # radians - azimuth angle (0 = straight ahead)
    flight_time: float  # seconds
    entry_angle_rad: float  # radians - angle from vertical at entry

    # Trajectory data for visualization (optional)
    trajectory_x: Optional[np.ndarray] = None
    trajectory_y: Optional[np.ndarray] = None
    trajectory_z: Optional[np.ndarray] = None

    @property
    def pitch_deg(self) -> float:
        return math.degrees(self.pitch_rad)

    @property
    def yaw_deg(self) -> float:
        return math.degrees(self.yaw_rad)

    @property
    def entry_angle_deg(self) -> float:
        return math.degrees(self.entry_angle_rad)


# Gravity vector (pointing down in z)
_GRAVITY = np.array([[0], [0], [9.806]])


def _create_dynamics_function(config: ShooterConfig):
    """Create the dynamics function for the ball in flight."""

    drag_constant = (
        0.5
        * config.air_density
        * config.drag_coefficient
        * config.cross_sectional_area
    )

    def f(x):
        """
        Dynamics function for the ball in flight.

        State vector x = [x, y, z, vx, vy, vz]^T
        Returns state derivative x_dot.
        """
        v = x[3:6, :]  # velocity

        # Velocity magnitude squared
        v_squared = (v.T @ v)[0, 0]

        # Drag force magnitude: F_D = 0.5 * ρ * v² * C_D * A
        drag_force = drag_constant * v_squared

        # Velocity direction unit vector
        v_hat = v / sqrt(v_squared)

        # State derivative:
        # - Position derivative = velocity
        # - Velocity derivative = -g - (F_D/m) * v_hat
        return block([[v], [-_GRAVITY - drag_force / config.ball_mass * v_hat]])

    return f


def _lerp(a: float, b: float, t: float) -> float:
    """Linear interpolation between a and b."""
    return a + t * (b - a)


def solve_trajectory(
    distance: float,
    config: Optional[ShooterConfig] = None,
    return_trajectory: bool = False,
    verbose: bool = False,
    target_height_override: Optional[float] = None,
) -> Optional[TrajectoryResult]:
    """
    Solve for optimal shooting parameters at a given distance.

    Args:
        distance: Horizontal distance from shooter to target in meters
        config: Shooter configuration (uses defaults if None)
        return_trajectory: If True, include trajectory points in result
        verbose: If True, print solver diagnostics
        target_height_override: Override target height (meters). If None, uses config value.

    Returns:
        TrajectoryResult with optimal parameters, or None if no solution found
    """
    if config is None:
        config = ShooterConfig()

    # Create dynamics function
    f = _create_dynamics_function(config)

    # Shooter position (at origin, elevated by shooter_height)
    shooter_pos = np.array([[0.0], [0.0], [config.shooter_height]])

    # Target height: use override if provided, otherwise config value
    target_z = target_height_override if target_height_override is not None else config.target_height

    # Target position (distance ahead, at target height)
    target_pos = np.array([[distance], [0.0], [target_z]])

    # Create optimization problem
    problem = Problem()

    # Duration decision variable
    N = config.num_timesteps
    T = problem.decision_variable()
    problem.subject_to(T >= config.min_flight_time)
    T.set_value(config.initial_time_guess)
    dt = T / N

    # Ball state: [x, y, z, vx, vy, vz] at each timestep
    X = problem.decision_variable(6, N)

    # Extract components
    p = X[:3, :]  # positions
    p_x, p_y, p_z = X[0, :], X[1, :], X[2, :]

    v = X[3:, :]  # velocities
    v_x, v_y, v_z = X[3, :], X[4, :], X[5, :]

    # Initial velocity relative to shooter
    v0_wrt_shooter = X[3:, :1]

    # =========================================================================
    # INITIAL GUESSES
    # =========================================================================

    # Position: linear interpolation with parabolic arc for z
    for k in range(N):
        t = k / (N - 1) if N > 1 else 0
        p_x[k].set_value(_lerp(shooter_pos[0, 0], target_pos[0, 0], t))
        p_y[k].set_value(_lerp(shooter_pos[1, 0], target_pos[1, 0], t))

        # Parabolic arc for height
        z_start = shooter_pos[2, 0]
        z_end = target_pos[2, 0]
        z_peak = max(z_start, z_end) + 0.5  # 0.5m above higher point
        if t < 0.5:
            z_guess = _lerp(z_start, z_peak, t * 2)
        else:
            z_guess = _lerp(z_peak, z_end, (t - 0.5) * 2)
        p_z[k].set_value(z_guess)

    # Velocity: initial guess pointing toward target
    direction = target_pos - shooter_pos
    direction_norm = norm(direction)
    direction_unit = direction / direction_norm
    initial_speed_guess = max(5.0, distance * 2.5)  # Scale with distance
    for k in range(N):
        v[:, k].set_value(initial_speed_guess * direction_unit)

    # =========================================================================
    # CONSTRAINTS
    # =========================================================================

    # 1. Initial position at shooter
    problem.subject_to(p[:, :1] == shooter_pos)

    # 2. Final position at target
    problem.subject_to(p[:, -1:] == target_pos)

    # 3. Entry angle constraint (must enter hexagon funnel)
    # Ball must be descending and within wall angle from vertical
    horizontal_speed_sq = v_x[-1] ** 2 + v_y[-1] ** 2
    vertical_speed = v_z[-1]

    # Must be going downward
    problem.subject_to(vertical_speed < 0.0)

    # Entry angle: sqrt(vx² + vy²) / |vz| ≤ tan(wall_angle)
    max_horizontal_ratio = math.tan(config.wall_angle_rad)
    problem.subject_to(
        horizontal_speed_sq <= vertical_speed ** 2 * max_horizontal_ratio ** 2
    )

    # 4. Dynamics constraints (RK4 integration)
    for k in range(N - 1):
        x_k = X[:, k]
        x_k1 = X[:, k + 1]

        k1 = f(x_k)
        k2 = f(x_k + dt / 2 * k1)
        k3 = f(x_k + dt / 2 * k2)
        k4 = f(x_k + dt * k3)

        problem.subject_to(x_k1 == x_k + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4))

    # 5. Height constraint (stay above ground)
    for k in range(N):
        problem.subject_to(p_z[k] >= 0.0)

    # =========================================================================
    # OBJECTIVE: Minimize flight time
    # =========================================================================

    problem.minimize(T)

    # =========================================================================
    # SOLVE
    # =========================================================================

    try:
        problem.solve(diagnostics=verbose)
    except Exception as e:
        if verbose:
            print(f"Solver failed: {e}")
        return None

    # Check if solution is valid
    flight_time = T.value()
    if flight_time <= 0 or not np.isfinite(flight_time):
        return None

    # =========================================================================
    # EXTRACT RESULTS
    # =========================================================================

    v0 = v0_wrt_shooter.value()
    velocity = float(norm(v0))

    # Angles
    pitch_rad = math.atan2(v0[2, 0], math.hypot(v0[0, 0], v0[1, 0]))
    yaw_rad = math.atan2(v0[1, 0], v0[0, 0])

    # Entry angle (from vertical)
    v_final = v[:, -1].value()
    entry_horizontal = math.hypot(v_final[0, 0], v_final[1, 0])
    entry_vertical = abs(v_final[2, 0])
    entry_angle_rad = math.atan2(entry_horizontal, entry_vertical)

    # Build result
    result = TrajectoryResult(
        velocity=velocity,
        pitch_rad=pitch_rad,
        yaw_rad=yaw_rad,
        flight_time=float(flight_time),
        entry_angle_rad=entry_angle_rad,
    )

    # Include trajectory if requested
    if return_trajectory:
        result.trajectory_x = p_x.value().flatten()
        result.trajectory_y = p_y.value().flatten()
        result.trajectory_z = p_z.value().flatten()

    return result


def velocity_to_rpm(velocity_mps: float, wheel_radius: float = 2.0 * 0.0254) -> float:
    """
    Convert ball exit velocity to wheel RPM.

    Args:
        velocity_mps: Ball exit velocity in m/s
        wheel_radius: Wheel radius in meters (default: 2 inches = 0.0508m)

    Returns:
        Wheel RPM
    """
    angular_velocity = velocity_mps / wheel_radius
    return angular_velocity * 60 / (2 * math.pi)


def rpm_to_velocity(rpm: float, wheel_radius: float = 2.0 * 0.0254) -> float:
    """
    Convert wheel RPM to ball exit velocity.

    Args:
        rpm: Wheel RPM
        wheel_radius: Wheel radius in meters (default: 2 inches = 0.0508m)

    Returns:
        Ball exit velocity in m/s
    """
    angular_velocity = rpm * 2 * math.pi / 60
    return angular_velocity * wheel_radius


def main():
    """Main entry point for standalone usage."""
    import argparse

    parser = argparse.ArgumentParser(
        description="Compute optimal shooting parameters for a given target position.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "horizontal", type=float, nargs="?", default=None,
        help="Horizontal distance to target in meters"
    )
    parser.add_argument(
        "vertical", type=float, nargs="?", default=None,
        help="Vertical distance (target height) in meters"
    )
    parser.add_argument(
        "--shooter-height", type=float, default=None,
        help="Shooter height in meters (default: 20 inches / 0.508m)"
    )
    parser.add_argument(
        "--entry-angle", type=float, default=None,
        help="Maximum entry angle from vertical in degrees (default: 45°)"
    )
    parser.add_argument(
        "--test", action="store_true",
        help="Run test mode with sample distances"
    )
    parser.add_argument(
        "-v", "--verbose", action="store_true",
        help="Show solver diagnostics"
    )

    args = parser.parse_args()

    config = ShooterConfig()
    if args.shooter_height is not None:
        config.shooter_height = args.shooter_height
    if args.entry_angle is not None:
        config.wall_angle_deg = args.entry_angle

    # Test mode
    if args.test or (args.horizontal is None and args.vertical is None):
        print("Testing shooter trajectory backend...")
        print()
        print(f"Config: target_height={config.target_height:.3f}m, "
              f"shooter_height={config.shooter_height:.3f}m, "
              f"entry_angle={config.wall_angle_deg}°")
        print()

        for dist in [1.0, 2.0, 3.0, 4.0, 5.0]:
            result = solve_trajectory(dist, config, verbose=False)
            if result:
                rpm = velocity_to_rpm(result.velocity)
                print(f"d={dist:.1f}m: v={result.velocity:.2f}m/s ({rpm:.0f} RPM), "
                      f"pitch={result.pitch_deg:.1f}°, t={result.flight_time:.3f}s, "
                      f"entry={result.entry_angle_deg:.1f}°")
            else:
                print(f"d={dist:.1f}m: NO SOLUTION")
        return

    # Single query mode
    if args.horizontal is None:
        parser.error("horizontal distance is required (or use --test)")

    horizontal = args.horizontal
    # If vertical not provided, use default target height
    vertical = args.vertical if args.vertical is not None else config.target_height

    result = solve_trajectory(
        distance=horizontal,
        config=config,
        target_height_override=vertical,
        verbose=args.verbose,
    )

    if result is None:
        print("NO SOLUTION FOUND")
        print(f"  Horizontal distance: {horizontal:.3f} m")
        print(f"  Vertical distance:   {vertical:.3f} m")
        print(f"  Shooter height:      {config.shooter_height:.3f} m")
        return

    rpm = velocity_to_rpm(result.velocity)
    print(f"Exit velocity: {result.velocity:.4f} m/s ({rpm:.1f} RPM)")
    print(f"Pitch angle:   {result.pitch_deg:.4f}° ({result.pitch_rad:.6f} rad)")
    print(f"Flight time:   {result.flight_time:.4f} s")
    print(f"Entry angle:   {result.entry_angle_deg:.2f}° from vertical")


if __name__ == "__main__":
    main()
