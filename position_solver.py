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
        # NO MINIMUM - allow very low values for smooth tracking
        self.Q = np.eye(6) * max(0.0, process_noise)
        self.Q[3:, 3:] *= 2  # Higher noise for velocity

        # Measurement noise (how much we trust the measurements)
        # Higher = smoother but more lag
        self.R = np.eye(3) * max(0.01, measurement_noise)

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
        """Update process noise parameter - no minimum constraint"""
        self.Q = np.eye(6) * max(0.0, process_noise)
        self.Q[3:, 3:] *= 2

    def update_measurement_noise(self, measurement_noise):
        """Update measurement noise parameter - allow very low values"""
        self.R = np.eye(3) * max(0.01, measurement_noise)

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
                process_noise=kalman_config.get('process_noise', 0.002),
                measurement_noise=kalman_config.get('measurement_noise', 0.09),
                initial_variance=kalman_config.get('initial_variance', 1.0)
            )
            print(f"  Kalman filter enabled (process_noise={kalman_config.get('process_noise', 0.002)}, measurement_noise={kalman_config.get('measurement_noise', 0.09)})")

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

    def calculate_two_anchor_positions(self, anchor1_pos, dist1, anchor2_pos, dist2):
        """
        Calculate the two candidate positions from two anchor measurements.
        The intersection of two spheres is a circle. We find two points on this circle
        at a reasonable z-height (average of anchor heights or 1.0m).

        Returns: array of two 3D positions, or None if no intersection
        """
        anchor1_pos = np.array(anchor1_pos)
        anchor2_pos = np.array(anchor2_pos)

        # Vector from anchor1 to anchor2
        d_vec = anchor2_pos - anchor1_pos
        d = np.linalg.norm(d_vec)

        # Check if spheres intersect
        if d > dist1 + dist2 or d < abs(dist1 - dist2) or d == 0:
            return None

        # Unit vector from anchor1 to anchor2
        u = d_vec / d

        # Distance from anchor1 to the plane where the circles intersect
        # Using the formula: a = (r1² - r2² + d²) / (2d)
        a = (dist1**2 - dist2**2 + d**2) / (2 * d)

        # Check if the intersection point is valid
        h_squared = dist1**2 - a**2
        if h_squared < 0:
            return None

        h = np.sqrt(h_squared)

        # Point on the line between anchors where the circle lies
        p_center = anchor1_pos + a * u

        # We need a vector perpendicular to u to define the circle plane
        # Choose a vector perpendicular to u
        if abs(u[2]) < 0.9:  # u is not too vertical
            # Use z-axis cross product
            perp = np.cross(u, np.array([0, 0, 1]))
        else:
            # Use x-axis cross product
            perp = np.cross(u, np.array([1, 0, 0]))

        perp = perp / np.linalg.norm(perp)

        # Two candidate positions on the circle
        candidate1 = p_center + h * perp
        candidate2 = p_center - h * perp

        return np.array([candidate1, candidate2])

    def calculate_position_lsq(self, measurements):
        """
        Calculate 3D position using non-linear least squares
        measurements: dict of {anchor_id: distance}
        Returns: (position, success, residual, num_anchors, failure_reason)
        """
        # Filter valid measurements
        valid_measurements = {}
        anchor_positions = []
        distances = []
        anchor_ids = []

        for anchor_id, distance in measurements.items():
            if anchor_id in self.anchor_positions and distance > 0:
                valid_measurements[anchor_id] = distance
                anchor_positions.append(self.anchor_positions[anchor_id])
                distances.append(distance)
                anchor_ids.append(anchor_id)

        num_anchors = len(valid_measurements)

        # Handle 1 anchor case - return sphere visualization data
        if num_anchors == 1:
            # Return the anchor position and distance for visualization
            # The actual position is indeterminate, but we return data for sphere rendering
            return None, False, None, num_anchors, f"Single anchor mode: anchor {anchor_ids[0]} at distance {distances[0]:.2f}m"

        # Handle 2 anchor case - calculate two candidate positions
        if num_anchors == 2:
            candidates = self.calculate_two_anchor_positions(
                anchor_positions[0], distances[0],
                anchor_positions[1], distances[1]
            )
            if candidates is not None:
                # Return both candidates - the visualization will handle this
                return None, False, None, num_anchors, f"Two anchor mode: {len(candidates)} candidate positions"
            else:
                return None, False, None, num_anchors, "Two anchors: spheres do not intersect (check distances)"

        # Check minimum anchors for full positioning
        if num_anchors < self.min_anchors:
            reason = f"Insufficient anchors: {num_anchors}/{self.min_anchors} (valid from {len(measurements)} total)"
            return None, False, None, num_anchors, reason

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
                reason = f"Optimization failed to converge (used {result.nfev} function evaluations)"
                return None, False, None, num_anchors, reason

            position = result.x
            residual = np.sqrt(np.mean(result.fun**2))  # RMS residual

            # Validate position is within room bounds (with some margin)
            margin = 1.0  # Allow 1m outside room for edge cases
            if not (self.room_bounds['x'][0] - margin <= position[0] <= self.room_bounds['x'][1] + margin and
                    self.room_bounds['y'][0] - margin <= position[1] <= self.room_bounds['y'][1] + margin and
                    self.room_bounds['z'][0] - margin <= position[2] <= self.room_bounds['z'][1] + margin):
                reason = f"Position out of bounds: [{position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}] (room: x:{self.room_bounds['x']}, y:{self.room_bounds['y']}, z:{self.room_bounds['z']})"
                return None, False, residual, num_anchors, reason

            # Check if residual is acceptable
            if residual > self.max_residual:
                # Try outlier rejection with RANSAC-style approach
                position, success = self.ransac_position(anchor_positions, distances)
                if success:
                    # Recalculate residual
                    residuals = self.residual_function(position, anchor_positions, distances)
                    residual = np.sqrt(np.mean(residuals**2))
                else:
                    reason = f"High residual error ({residual:.3f}m > {self.max_residual}m) and RANSAC failed to find better solution"
                    return None, False, residual, num_anchors, reason

            return position, True, residual, num_anchors, None

        except Exception as e:
            reason = f"Optimization exception: {str(e)}"
            return None, False, None, num_anchors, reason

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
        # Store measurements for visualization
        valid_measurements = {}
        anchor_positions_list = []
        distances_list = []
        anchor_ids_list = []

        for anchor_id, distance in measurements.items():
            if anchor_id in self.anchor_positions and distance > 0:
                valid_measurements[anchor_id] = distance
                anchor_positions_list.append(self.anchor_positions[anchor_id])
                distances_list.append(distance)
                anchor_ids_list.append(anchor_id)

        num_anchors = len(valid_measurements)

        result = {
            'success': False,
            'num_anchors': num_anchors,
            'position': None,
            'velocity': None,
            'residual': None,
            'filtered': False,
            'failure_reason': None,
            'measurements': valid_measurements,  # For sphere visualization
            'candidate_positions': None  # For 2-anchor mode
        }

        # Handle 1 anchor case
        if num_anchors == 1:
            result['failure_reason'] = f"Single anchor mode: anchor {anchor_ids_list[0]} at distance {distances_list[0]:.2f}m"
            return result

        # Handle 2 anchor case
        if num_anchors == 2:
            candidates = self.calculate_two_anchor_positions(
                anchor_positions_list[0], distances_list[0],
                anchor_positions_list[1], distances_list[1]
            )
            if candidates is not None:
                result['candidate_positions'] = [c.tolist() for c in candidates]
                result['failure_reason'] = f"Two anchor mode: {len(candidates)} candidate positions"
            else:
                result['failure_reason'] = "Two anchors: spheres do not intersect (check distances)"
            return result

        # For 3+ anchors, use the standard positioning
        position, success, residual, _, failure_reason = self.calculate_position_lsq(measurements)

        result['success'] = success
        result['residual'] = residual
        result['failure_reason'] = failure_reason

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
