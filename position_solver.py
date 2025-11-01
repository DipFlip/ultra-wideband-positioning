#!/usr/bin/env python3
"""
3D Position Solver for UWB System
Uses multilateration with least squares optimization and Kalman filtering
"""
import numpy as np
from scipy.optimize import least_squares
import json
import time


class KalmanFilter:
    """Simple Kalman filter for position and velocity smoothing"""

    def __init__(self, process_noise=0.1, measurement_noise=0.2, initial_variance=1.0):
        # State: [x, y, z, vx, vy, vz]
        self.state = np.zeros(6)
        self.covariance = np.eye(6) * initial_variance

        # Process noise (how much we trust the motion model)
        self.Q = np.eye(6) * process_noise
        self.Q[3:, 3:] *= 2  # Higher noise for velocity

        # Measurement noise (how much we trust the measurements)
        self.R = np.eye(3) * measurement_noise

        self.last_update_time = None
        self.initialized = False

    def predict(self, dt):
        """Predict next state based on constant velocity model"""
        # State transition matrix (constant velocity)
        F = np.eye(6)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt

        # Predict state
        self.state = F @ self.state

        # Predict covariance
        self.covariance = F @ self.covariance @ F.T + self.Q

    def update(self, measurement):
        """Update state with new position measurement [x, y, z]"""
        # Measurement matrix (we only measure position, not velocity)
        H = np.zeros((3, 6))
        H[0, 0] = 1
        H[1, 1] = 1
        H[2, 2] = 1

        # Innovation (difference between measurement and prediction)
        y = measurement - H @ self.state

        # Innovation covariance
        S = H @ self.covariance @ H.T + self.R

        # Kalman gain
        K = self.covariance @ H.T @ np.linalg.inv(S)

        # Update state
        self.state = self.state + K @ y

        # Update covariance
        self.covariance = (np.eye(6) - K @ H) @ self.covariance

    def get_position(self):
        """Get filtered position [x, y, z]"""
        return self.state[:3].copy()

    def get_velocity(self):
        """Get estimated velocity [vx, vy, vz]"""
        return self.state[3:].copy()

    def update_process_noise(self, process_noise):
        """Update process noise parameter"""
        self.Q = np.eye(6) * process_noise
        self.Q[3:, 3:] *= 2

    def update_measurement_noise(self, measurement_noise):
        """Update measurement noise parameter"""
        self.R = np.eye(3) * measurement_noise

    def process(self, measurement):
        """Process a new measurement and return filtered position"""
        current_time = time.time()

        if not self.initialized:
            # Initialize filter with first measurement
            self.state[:3] = measurement
            self.state[3:] = 0
            self.initialized = True
            self.last_update_time = current_time
            return self.get_position()

        # Calculate time delta
        dt = current_time - self.last_update_time
        dt = max(0.001, min(dt, 1.0))  # Clamp dt to reasonable range

        # Predict and update
        self.predict(dt)
        self.update(measurement)

        self.last_update_time = current_time
        return self.get_position()


class PositionSolver:
    """3D position solver using multilateration"""

    def __init__(self, config_file='anchor_config.json'):
        with open(config_file, 'r') as f:
            self.config = json.load(f)

        # Extract anchor positions
        self.anchor_positions = {}
        for anchor_id, anchor_data in self.config['anchors'].items():
            if anchor_data.get('enabled', True):
                self.anchor_positions[anchor_id] = np.array(anchor_data['position'])

        # Get positioning parameters
        pos_config = self.config.get('positioning', {})
        self.min_anchors = pos_config.get('min_anchors', 4)
        self.max_residual = pos_config.get('max_residual', 0.5)
        self.outlier_threshold = pos_config.get('outlier_threshold', 0.3)

        # Initialize Kalman filter
        kalman_config = pos_config.get('kalman_filter', {})
        self.use_kalman = kalman_config.get('enabled', True)

        if self.use_kalman:
            self.kalman = KalmanFilter(
                process_noise=kalman_config.get('process_noise', 0.1),
                measurement_noise=kalman_config.get('measurement_noise', 0.2),
                initial_variance=kalman_config.get('initial_variance', 1.0)
            )

        # Get room bounds for validation
        room = self.config.get('room', {})
        self.room_bounds = {
            'x': (0, room.get('width', 10)),
            'y': (0, room.get('depth', 10)),
            'z': (0, room.get('height', 3))
        }

        print(f"Position solver initialized with {len(self.anchor_positions)} anchors")
        for anchor_id, pos in self.anchor_positions.items():
            print(f"  Anchor {anchor_id}: {pos}")

    def residual_function(self, position, anchor_positions, distances):
        """Calculate residuals for least squares optimization"""
        residuals = []
        for anchor_pos, measured_dist in zip(anchor_positions, distances):
            calculated_dist = np.linalg.norm(position - anchor_pos)
            residuals.append(calculated_dist - measured_dist)
        return np.array(residuals)

    def calculate_position_lsq(self, measurements):
        """
        Calculate 3D position using non-linear least squares
        measurements: dict of {anchor_id: distance}
        Returns: (position, success, residual, num_anchors)
        """
        # Filter valid measurements
        valid_measurements = {}
        anchor_positions = []
        distances = []

        for anchor_id, distance in measurements.items():
            if anchor_id in self.anchor_positions and distance > 0:
                valid_measurements[anchor_id] = distance
                anchor_positions.append(self.anchor_positions[anchor_id])
                distances.append(distance)

        num_anchors = len(valid_measurements)

        # Check minimum anchors
        if num_anchors < self.min_anchors:
            return None, False, None, num_anchors

        anchor_positions = np.array(anchor_positions)
        distances = np.array(distances)

        # Initial guess: center of room or weighted centroid of anchors
        initial_guess = np.mean(anchor_positions, axis=0)

        # Use previous Kalman position as initial guess if available
        if self.use_kalman and self.kalman.initialized:
            initial_guess = self.kalman.get_position()

        try:
            # Perform least squares optimization
            result = least_squares(
                self.residual_function,
                initial_guess,
                args=(anchor_positions, distances),
                method='lm',  # Levenberg-Marquardt
                max_nfev=100
            )

            if not result.success:
                return None, False, None, num_anchors

            position = result.x
            residual = np.sqrt(np.mean(result.fun**2))  # RMS residual

            # Validate position is within room bounds (with some margin)
            margin = 1.0  # Allow 1m outside room for edge cases
            if not (self.room_bounds['x'][0] - margin <= position[0] <= self.room_bounds['x'][1] + margin and
                    self.room_bounds['y'][0] - margin <= position[1] <= self.room_bounds['y'][1] + margin and
                    self.room_bounds['z'][0] - margin <= position[2] <= self.room_bounds['z'][1] + margin):
                return None, False, residual, num_anchors

            # Check if residual is acceptable
            if residual > self.max_residual:
                # Try outlier rejection with RANSAC-style approach
                position, success = self.ransac_position(anchor_positions, distances)
                if success:
                    # Recalculate residual
                    residuals = self.residual_function(position, anchor_positions, distances)
                    residual = np.sqrt(np.mean(residuals**2))
                else:
                    return None, False, residual, num_anchors

            return position, True, residual, num_anchors

        except Exception as e:
            print(f"Optimization error: {e}")
            return None, False, None, num_anchors

    def ransac_position(self, anchor_positions, distances, iterations=10):
        """
        RANSAC-style outlier rejection
        Try different subsets of anchors and pick best result
        """
        if len(distances) < self.min_anchors + 1:
            return None, False

        best_position = None
        best_inlier_count = 0

        for _ in range(iterations):
            # Random subset
            indices = np.random.choice(len(distances), self.min_anchors, replace=False)
            subset_positions = anchor_positions[indices]
            subset_distances = distances[indices]

            # Try to solve with subset
            initial_guess = np.mean(subset_positions, axis=0)

            try:
                result = least_squares(
                    self.residual_function,
                    initial_guess,
                    args=(subset_positions, subset_distances),
                    method='lm',
                    max_nfev=50
                )

                if result.success:
                    position = result.x

                    # Count inliers
                    all_residuals = self.residual_function(position, anchor_positions, distances)
                    inliers = np.abs(all_residuals) < self.outlier_threshold
                    inlier_count = np.sum(inliers)

                    if inlier_count > best_inlier_count:
                        best_inlier_count = inlier_count
                        best_position = position

            except:
                continue

        if best_position is not None and best_inlier_count >= self.min_anchors:
            return best_position, True

        return None, False

    def solve(self, measurements):
        """
        Main solve function with Kalman filtering
        measurements: dict of {anchor_id: distance}
        Returns: dict with position, velocity, confidence, etc.
        """
        position, success, residual, num_anchors = self.calculate_position_lsq(measurements)

        result = {
            'success': success,
            'num_anchors': num_anchors,
            'position': None,
            'velocity': None,
            'residual': residual,
            'filtered': False
        }

        if not success or position is None:
            return result

        # Apply Kalman filter if enabled
        if self.use_kalman:
            filtered_position = self.kalman.process(position)
            velocity = self.kalman.get_velocity()

            result['position'] = filtered_position.tolist()
            result['velocity'] = velocity.tolist()
            result['filtered'] = True
        else:
            result['position'] = position.tolist()
            result['velocity'] = [0, 0, 0]

        return result


# Test function
if __name__ == "__main__":
    solver = PositionSolver()

    # Simulate measurements (tag at center of room: 2, 1.5, 1.5)
    test_position = np.array([2.0, 1.5, 1.5])

    print(f"\nTest position: {test_position}")

    # Calculate expected distances
    measurements = {}
    for anchor_id, anchor_pos in solver.anchor_positions.items():
        distance = np.linalg.norm(test_position - anchor_pos)
        # Add small noise
        distance += np.random.normal(0, 0.05)
        measurements[anchor_id] = distance
        print(f"Anchor {anchor_id}: {distance:.3f}m")

    # Solve
    result = solver.solve(measurements)

    print(f"\nResult:")
    print(f"  Success: {result['success']}")
    print(f"  Position: {result['position']}")
    print(f"  Error: {np.linalg.norm(np.array(result['position']) - test_position):.3f}m")
    print(f"  Residual: {result['residual']:.3f}m")
