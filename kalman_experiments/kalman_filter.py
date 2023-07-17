import numpy as np

# offsets of each variable in the state vector
iX = 0
iVx = 1
iY = 2
iVy = 3
NUMVARS = iVy + 1


class KalmanFilter:
    def __init__(
        self,
        initial_x: int,  # initial position
        initial_vx: float,  # initial velocity
        x_accel_variance: float,
        initial_y: int,
        initial_vy: float,
        y_accel_variance: float,
    ) -> None:
        # mean of state GRV (Gaussian Random Variable)
        self._x = np.zeros(NUMVARS)

        self._x[iX] = initial_x
        self._x[iVx] = initial_vx
        self._x[iY] = initial_y
        self._x[iVy] = initial_vy

        self._a_var = np.zeros(NUMVARS)
        self._a_var[0] = x_accel_variance
        self._a_var[1] = x_accel_variance
        self._a_var[2] = y_accel_variance
        self._a_var[3] = y_accel_variance

        # covariance of state GRV
        self._P = np.eye(NUMVARS)

    def predict(self, dt: float) -> tuple:  # dt = time step (delta t)
        # x = F x
        # P = F P Ft + G Gt a   # Ft = F transpose
        F = np.eye(NUMVARS)
        F[iX, iVx] = dt
        F[iY, iVy] = dt
        new_x = F.dot(self._x)

        G = np.zeros((NUMVARS, 1))
        G[iX] = 0.5 * dt**2
        G[iVx] = dt
        G[iY] = 0.5 * dt**2
        G[iVy] = dt
        new_P = F.dot(self._P).dot(F.T) + G.dot(G.T).dot(self._a_var)

        self._P = new_P
        self._x = new_x

        return self.x_pos, self.y_pos

    def update(
        self,
        meas_x: int,  # meas = measurement
        meas_y: int,
        meas_ax_variance=None,
        meas_ay_variance=None,
    ) -> tuple:
        # y = z - H x
        # S = H P Ht + R
        # K = P Ht S^-1
        # x = x + K y
        # P = (I - K H) * P

        H = np.zeros((2, NUMVARS))
        H[0, iX] = 1
        H[1, iY] = 1

        if meas_ax_variance is None:
            meas_ax_variance = self.x_accel_variance
        if meas_ay_variance is None:
            meas_ay_variance = self.y_accel_variance

        z = np.array([meas_x, meas_y])
        R = np.array([[meas_ax_variance, 0.0], [0.0, meas_ay_variance]])

        y = z - H.dot(self._x)
        S = H.dot(self._P).dot(H.T) + R

        K = self._P.dot(H.T).dot(np.linalg.inv(S))

        new_x = self._x + K.dot(y)
        new_P = (np.eye(NUMVARS) - K.dot(H)).dot(self._P)

        self._P = new_P
        self._x = new_x

        return self.x_pos, self.y_pos

    @property
    def cov(self) -> np.ndarray:
        return self._P

    @property
    def mean(self) -> np.ndarray:
        return self._x

    @property
    def x_pos(self) -> int:
        return int(self._x[iX])

    @property
    def x_vel(self) -> float:
        return self._x[iVx]

    @property
    def x_accel_variance(self) -> float:
        return self._a_var[0]

    @property
    def y_pos(self) -> int:
        return int(self._x[iY])

    @property
    def y_vel(self) -> float:
        return self._x[iVy]

    @property
    def y_accel_variance(self) -> float:
        return self._a_var[2]
