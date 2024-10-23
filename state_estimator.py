import numpy as np

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
        self.reset(x_init, P_init)

    def reset(self, x_init, P_init):
        self.x = x_init.copy()
        self.P = P_init.copy()

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

class HopperStateEstimator(KalmanFilter):

    def __init__(self, dt, x_init, P_init):
        dt = dt
        dim_state = 4   # (pz, vx, vy, vz)
        dim_control = 3 # (ax, ay, az)
        dim_obs = 4     # (pz, vx, vy, vz)
        Ac = np.zeros((dim_state, dim_state))
        Ac[:3, -1] = 1
        Bc = np.zeros((dim_state, dim_control))
        Bc[1:] = np.eye(3)
        # print(f"Ac: {Ac}")
        # print(f"Bc: {Bc}")

        A = np.eye(dim_state) + dt * Ac
        B = dt * Bc
        C = np.eye(dim_obs)
        # print(f"C: {C}")

        # Process noise (pz, vx, vy, vz)
        Q = np.diag([0.002, 0.02, 0.02, 0.02])
        # Measurement noise (pz, vx, vy, vz)
        R = np.diag([0.001, 0.1, 0.1, 0.1])
        # TODO scale up any covariance related foot during swing, assume alway in contact

        super(HopperStateEstimator, self).__init__(A, B, C, Q, R, x_init, P_init)

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
