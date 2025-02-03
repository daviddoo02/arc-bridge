import numpy as np
import collections

class KalmanFilter:
    def __init__(self, A, B, C, Q, R, x_init, P_init):
        # x_kp1 = A*xk + B*uk + wk
        # yk = C*xk + vk
        # x0 ~ N(x_init, P_init)
        # wk ~ N(0, Qk)
        # vk ~ N(0, Rk)
        dim_state = A.shape[1]
        dim_control = B.shape[1]
        dim_obs = C.shape[0]
        assert(A.shape == (dim_state, dim_state))
        assert(B.shape == (dim_state, dim_control))
        assert(C.shape == (dim_obs, dim_state))
        assert(Q.shape == (dim_state, dim_state))
        assert(R.shape == (dim_obs, dim_obs))
        assert(x_init.shape == (dim_state, ))
        assert(P_init.shape == (dim_state, dim_state))

        self.A = A.copy()
        self.B = B.copy()
        self.C = C.copy()
        self.Q = Q.copy()
        self.R = R.copy()
        self.I = np.eye(dim_state)
        self.x_init = x_init
        self.P_init = P_init
        self.reset()

    def reset(self, x_init=None):
        if x_init is None:
            self.x = self.x_init.copy()
        else:
            self.x = x_init.copy()
        self.P = self.P_init.copy()

    def predict(self, u):
        # Predict state ahead
        self.x = self.A @ self.x + self.B @ u
        # Update prediction error covariance
        self.P = self.A @ self.P @ self.A.T + self.Q

        return self.x.copy()

    def correct(self, y):
        # Compute the Kalman gain
        K = self.P @ self.C.T @ np.linalg.inv(self.C @ self.P @ self.C.T + self.R)
        # Correct estimation
        self.x = self.x + K @ (y - self.C @ self.x)
        # Update measurement error covariance
        self.P = (self.I - K @ self.C) @ self.P
        
        return self.x.copy()

class MovingWindowFilter:
  """A stable O(1) moving filter for incoming data streams.

  We implement the Neumaier's algorithm to calculate the moving window average,
  which is numerically stable.

  """
  def __init__(self, window_size: int, dim: int = 3):
    """Initializes the class.

    Args:
      window_size: The moving window size.
    """
    assert window_size > 0
    self._window_size = window_size
    self._value_deque = collections.deque(maxlen=window_size)
    # The moving window sum.
    self._sum = np.zeros(dim)
    # The correction term to compensate numerical precision loss during
    # calculation.
    self._correction = np.zeros(dim)

  def _neumaier_sum(self, value: np.ndarray):
    """Update the moving window sum using Neumaier's algorithm.

    For more details please refer to:
    https://en.wikipedia.org/wiki/Kahan_summation_algorithm#Further_enhancements

    Args:
      value: The new value to be added to the window.
    """

    new_sum = self._sum + value
    self._correction = np.where(
        np.abs(self._sum) >= np.abs(value),
        self._correction + (self._sum - new_sum) + value,
        self._correction + (value - new_sum) + self._sum)
    self._sum = new_sum

  def calculate_average(self, new_value: np.ndarray) -> np.ndarray:
    """Computes the moving window average in O(1) time.

    Args:
      new_value: The new value to enter the moving window.

    Returns:
      The average of the values in the window.

    """
    deque_len = len(self._value_deque)
    if deque_len < self._value_deque.maxlen:
      pass
    else:
      # The left most value to be subtracted from the moving sum.
      self._neumaier_sum(-self._value_deque[0])

    self._neumaier_sum(new_value)
    self._value_deque.append(new_value)

    return (self._sum + self._correction) / self._window_size

class HopperStateEstimator(KalmanFilter):

    def __init__(self, dt):
        dt = dt
        dim_state = 6   # (px, py, pz, vx, vy, vz)
        dim_control = 3 # (ax, ay, az)
        dim_obs = 4     # (pz, vx, vy, vz)
        Ac = np.zeros((dim_state, dim_state))
        Ac[:3, 3:] = np.eye(3)
        Bc = np.zeros((dim_state, dim_control))
        Bc[3:] = np.eye(3)
        # print(f"Ac:\n{Ac}")
        # print(f"Bc:\n{Bc}")

        A = np.eye(dim_state) + dt * Ac
        B = dt * Bc
        C = np.zeros((dim_obs, dim_state))
        C[:, 2:] = np.eye(4)
        # print(f"C:\n{C}")

        # Process noise (px, py, pz, vx, vy, vz)
        Q = np.diag([0.002, 0.002, 0.002, 0.02, 0.02, 0.02]) * 1e-3
        # Measurement noise (pz, vx, vy, vz)
        R = np.diag([0.001, 0.1, 0.1, 0.1])
        # TODO scale up any covariance related foot during swing, assume alway in contact

        self.x_init = np.array([0, 0, 0.32, 0, 0, 0]) # initial pos and vel in 3D
        self.P_init = np.eye(dim_state) * 1e-5         # initial state covariance
        super(HopperStateEstimator, self).__init__(A, B, C, Q, R, self.x_init, self.P_init)

        self.l1 = 0.22
        self.l2 = 0.22
        self.p_hip = np.array([0, 0, -0.07])

    def foot_pos_body_frame(self, qj_pos):
        l1 = self.l1
        l2 = self.l2
        th1 = qj_pos[0]
        th2 = qj_pos[1]

        p_foot = np.array([- l1 * np.sin(th1) - l2 * np.sin(th1 + th2), 
                           0,
                           - l1 * np.cos(th1) - l2 * np.cos(th1 + th2)])
        
        return self.p_hip + p_foot

    def foot_vel_body_frame(self, qj_pos, qj_vel):
        l1 = self.l1
        l2 = self.l2
        th1 = qj_pos[0]
        th2 = qj_pos[1]
    
        jacobian = np.array([
            -l1 * np.cos(th1) - l2 * np.cos(th1 + th2), -l2 * np.cos(th1 + th2),
             l1 * np.sin(th1) + l2 * np.sin(th1 + th2),  l2 * np.sin(th1 + th2)
            ]).reshape((2, 2))
        
        v_foot = jacobian @ qj_vel
        
        return np.array([v_foot[0], 0, v_foot[1]])
