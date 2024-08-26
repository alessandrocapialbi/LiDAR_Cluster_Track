import numpy as np

class KalmanFilter:
    def __init__(self, dt):
        # Time step (delta t)
        self.dt = dt

        # State vector: [x, y, z, vx, vy, ax, ay]
        self.X = np.zeros(7)

        # State error covariance matrix
        self.P = np.eye(7)

        # State transition matrix
        self.F = np.eye(7)
        self.F[0, 3] = self.dt # x = x + vx * dt
        self.F[1, 4] = self.dt # y = y + vy * dt
        self.F[0, 5] = 0.5 * self.dt ** 2 # x = x + vx * dt + 0.5 * ax * dt^2
        self.F[1, 6] = 0.5 * self.dt ** 2 # y = y + vy * dt + 0.5 * ay * dt^2
        self.F[3, 5] = self.dt # vx = vx + ax * dt
        self.F[4, 6] = self.dt # vy = vy + ay * dt


        # Observation matrix
        self.H = np.zeros((3, 7))
        self.H[0, 0] = 1  # x
        self.H[1, 1] = 1  # y
        self.H[2, 2] = 1  # z

        # Process noise covariance matrix
        self.Q = np.eye(7) * 0.01

        # Measurement noise covariance matrix
        self.R = np.eye(3) * 0.01

    def predict(self):
        # Predict the next state
        self.X = np.dot(self.F, self.X)
        self.P = np.dot(self.F, np.dot(self.P, self.F.T)) + self.Q

    def update(self, z):
        # Update the filter with new measurement z (position x, y, z)
        y = z - np.dot(self.H, self.X)  # Residual (prediction error)
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R  # Residual covariance
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))  # Kalman gain

        # Update state and covariance matrix
        self.X = self.X + np.dot(K, y)
        self.P = self.P - np.dot(K, np.dot(self.H, self.P))

    def get_state(self):
        # Return the current state: position x, y, z
        return self.X[:3]
