import numpy as np
from sklearn.linear_model import RANSACRegressor

class EKFSlAM:
    def __init__(self, initial_state, initial_covariance):
        self.state = initial_state
        self.covariance = initial_covariance
        self.landmarks = []
    
    @staticmethod
    def line_segment_from_inliers(inlier_points, line):
        if len(inlier_points) < 2:
            return None

        a, b = line
        direction = np.array([1.0, a], dtype=float)
        direction_norm = np.linalg.norm(direction)
        if direction_norm == 0:
            return None

        direction = direction / direction_norm
        base_point = np.array([0.0, b], dtype=float)
        base_projection = base_point @ direction

        projections = np.asarray(inlier_points, dtype=float) @ direction
        min_offset = projections.min() - base_projection
        max_offset = projections.max() - base_projection

        p1 = base_point + min_offset * direction
        p2 = base_point + max_offset * direction
        return tuple(p1), tuple(p2)

    @staticmethod
    def landmark_detection(points: list, min_samples=2, residual_threshold=2.0, min_inliers=10, max_trials=25):
        points = np.array(points)
        if len(points) > 200:
            points = points[::2]

        lines = []
        segments = []
        while len(points) > min_inliers:
            X = points[:, 0].reshape(-1, 1)
            y = points[:, 1]
            model = RANSACRegressor(
                min_samples=min_samples,
                residual_threshold=residual_threshold,
                max_trials=max_trials,
                stop_probability=0.95,
                random_state=0
            )
            model.fit(X, y)
            inlier_mask = model.inlier_mask_
            if np.sum(inlier_mask) < min_inliers:
                break
            # Get line parameters: y = a*x + b
            a = model.estimator_.coef_[0]
            b = model.estimator_.intercept_
            lines.append((a, b))

            inlier_points = points[inlier_mask]
            segment = EKFSlAM.line_segment_from_inliers(inlier_points, (a, b))
            if segment is not None:
                segments.append(segment)

            # Remove inliers and repeat
            points = points[~inlier_mask]
        return lines, segments

    @staticmethod
    def associate_line(detected_line, landmarks, angle_threshold=np.deg2rad(10), rho_threshold=25.0):
        if detected_line is None or len(detected_line) != 2:
            return None

        alpha_d, rho_d = EKFSlAM.line_to_normal_form(detected_line)

        if len(landmarks) == 0:
            return None

        best_index = None
        best_score = float("inf")

        landmarks = np.asarray(landmarks).reshape(-1, 2)

        for index, landmark in enumerate(landmarks):
            if landmark is None or len(landmark) != 2:
                continue

            alpha_l, rho_l = landmark

            angle_diff = abs(EKFSlAM.wrap_angle(alpha_d - alpha_l))
            rho_diff = abs(rho_d - rho_l)

            if angle_diff < angle_threshold and rho_diff < rho_threshold:
                score = angle_diff + rho_diff
                if score < best_score:
                    best_score = score
                    best_index = index

        return best_index
    
    @staticmethod
    def wrap_angle(angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    @staticmethod
    def line_to_normal_form(line):
        a, b = line
        A, B, C = a, -1.0, b   # a*x - y + b = 0
        norm = np.hypot(A, B)

        alpha = np.arctan2(B, A)
        rho = -C / norm

        if rho < 0:
            rho = -rho
            alpha += np.pi

        alpha = EKFSlAM.wrap_angle(alpha)
        return np.array([alpha, rho], dtype=float)

    def add_landmarks(self, new_landmarks, init_cov=None):
        if len(new_landmarks) == 0:
            return

        # convert detected (a,b) lines into (alpha, rho)
        new_landmarks = np.array(
            [self.line_to_normal_form(line) for line in new_landmarks],
            dtype=float
        ).reshape(-1, 2)

        m = new_landmarks.shape[0]

        if init_cov is None:
            init_cov = np.diag([np.deg2rad(15.0)**2, 100.0])
        init_cov = np.asarray(init_cov, dtype=float)
        assert init_cov.shape == (2, 2), "init_cov must be a 2x2 matrix"

        old_n = self.state.size
        P_old = self.covariance
        assert P_old.shape == (old_n, old_n)

        # extend joint state
        self.state = np.concatenate([self.state.reshape(-1), new_landmarks.reshape(-1)])

        # expand covariance
        new_n = old_n + 2 * m
        P_new = np.zeros((new_n, new_n), dtype=float)
        P_new[:old_n, :old_n] = P_old

        for i in range(m):
            i0 = old_n + 2 * i
            P_new[i0:i0+2, i0:i0+2] = init_cov

        self.covariance = P_new

        # derived view of all landmarks
        self.landmarks = self.state[3:].reshape(-1, 2)

    def state_predict(self, control_input, motion_noise=None):
        # predict robot state
        self.state[:3] += control_input
        self.state[2] = self.wrap_angle(self.state[2])

        # predict covariance
        n = self.state.size
        F = np.eye(n)

        if motion_noise is None:
            motion_noise = np.diag([0.1**2, 0.1**2, np.deg2rad(2.0)**2])

        Q = np.zeros((n, n), dtype=float)
        Q[:3, :3] = motion_noise

        self.covariance = F @ self.covariance @ F.T + Q
        return self.state

    def update(self, associated_landmark_idx, detected_line, R=None):
        if R is None:
            R = np.diag([np.deg2rad(2.0)**2, 25.0])

        alpha_meas, rho_meas = self.line_to_normal_form(detected_line)

        x, y, theta = self.state[:3]

        lm_start = 3 + 2 * associated_landmark_idx
        alpha_l, rho_l = self.state[lm_start:lm_start + 2]

        z = np.array([
            self.wrap_angle(alpha_meas - theta),
            rho_meas - (x * np.cos(alpha_meas) + y * np.sin(alpha_meas))
        ], dtype=float)

        z_hat = np.array([
            self.wrap_angle(alpha_l - theta),
            rho_l - (x * np.cos(alpha_l) + y * np.sin(alpha_l))
        ], dtype=float)

        y_err = z - z_hat
        y_err[0] = self.wrap_angle(y_err[0])

        n = self.state.size
        H = np.zeros((2, n), dtype=float)

        H[:, 0:3] = np.array([
            [0.0, 0.0, -1.0],
            [-np.cos(alpha_l), -np.sin(alpha_l), 0.0]
        ])

        H[:, lm_start:lm_start + 2] = np.array([
            [1.0, 0.0],
            [x * np.sin(alpha_l) - y * np.cos(alpha_l), 1.0]
        ])

        P = self.covariance
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)

        self.state = self.state + K @ y_err
        self.state[2] = self.wrap_angle(self.state[2])

        for i in range(3, self.state.size, 2):
            self.state[i] = self.wrap_angle(self.state[i])

        I = np.eye(n)
        self.covariance = (I - K @ H) @ P

        self.landmarks = self.state[3:].reshape(-1, 2)