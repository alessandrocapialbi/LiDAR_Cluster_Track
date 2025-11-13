import numpy as np

class KalmanFilter:
    """
    A simple Kalman Filter for 2D position tracking with velocity and acceleration.

    State vector:
        X = [x, y, z, vx, vy, ax, ay]
        where x, y, z are positions,
              vx, vy are velocities,
              ax, ay are accelerations.

    Measurements:
        z = [x, y, z]
    """

    def __init__(self, dt):
        """
        Initialize the Kalman Filter.

        Args:
            dt (float): Time step between measurements.
        """
        self.dt = dt

        # Initialize state vector [x, y, z, vx, vy, ax, ay]
        self.X = np.zeros(7)

        # State error covariance matrix
        self.P = np.eye(7)

        # State transition matrix F
        self.F = np.eye(7)
        self.F[0, 3] = self.dt      # x = x + vx * dt
        self.F[1, 4] = self.dt      # y = y + vy * dt
        self.F[0, 5] = 0.5 * self.dt ** 2  # x += 0.5 * ax * dt^2
        self.F[1, 6] = 0.5 * self.dt ** 2  # y += 0.5 * ay * dt^2
        self.F[3, 5] = self.dt      # vx += ax * dt
        self.F[4, 6] = self.dt      # vy += ay * dt

        # Observation matrix H (maps state to measurements)
        self.H = np.zeros((3, 7))
        self.H[0, 0] = 1  # measure x
        self.H[1, 1] = 1  # measure y
        self.H[2, 2] = 1  # measure z

        # Process noise covariance
        self.Q = np.eye(7) * 0.01

        # Measurement noise covariance
        self.R = np.eye(3) * 0.01

    def predict(self):
        """
        Predict the next state based on the current state and transition model.
        Updates the state vector X and error covariance P.
        """
        self.X = np.dot(self.F, self.X)
        self.P = np.dot(self.F, np.dot(self.P, self.F.T)) + self.Q

    def update(self, z):
        """
        Update the filter with a new measurement.

        Args:
            z (np.ndarray): Measurement vector [x, y, z].
        """
        # Compute the residual (difference between measurement and prediction)
        y = z - np.dot(self.H, self.X)

        # Compute residual covariance
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R

        # Compute Kalman gain
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))

        # Update state estimate and covariance
        self.X = self.X + np.dot(K, y)
        self.P = self.P - np.dot(K, np.dot(self.H, self.P))

    def get_state(self):
        """
        Get the current estimated position.

        Returns:
            np.ndarray: Estimated position [x, y, z].
        """
        return self.X[:3]
