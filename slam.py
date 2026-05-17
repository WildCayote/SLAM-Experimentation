import numpy as np
from sklearn.linear_model import RANSACRegressor

class EKFSlAM:
    def __init__(self, initial_state, initial_covariance):
        self.state = initial_state
        self.covariance = initial_covariance
        self.landmarks = []
    
    @staticmethod
    def landmark_detection(points: list, min_samples=2, residual_threshold=2.0, min_inliers=10, max_trials=25):
        points = np.array(points)
        if len(points) > 200:
            points = points[::2]

        lines = []
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
            # Remove inliers and repeat
            points = points[~inlier_mask]
        return lines 

    @staticmethod
    def associate_line(detected_line, landmarks, angle_threshold=np.deg2rad(10), intercept_threshold=25.0):
        """
            Match one detected line to the best existing landmark.

            Parameters
            ----------
            detected_line : tuple
                A line in slope-intercept form: (a, b) for y = a*x + b
            landmarks : list
                Existing landmarks, either as tuples (a, b) or dicts like
                {"id": 0, "line": (a, b)}
            angle_threshold : float
                Maximum allowed angle difference in radians
            intercept_threshold : float
                Maximum allowed intercept difference in pixels

            Returns
            -------
            int or None
                Index of the best matching landmark, or None if no match is good enough.
        """

        # no line detcted, or bad line param
        if detected_line is None or len(detected_line) != 2:
            return None
        
        detected_a, detected_b = detected_line

        # no landmarks to compare against
        if len(landmarks) == 0:
            return None

        # convert detected slope to an angle for comparison
        # check if the angle is 90, since tan(90) is undefined/infinite
        if np.isinf(detected_a):
            detected_angle = np.pi / 2
        else:
            detected_angle = np.arctan(detected_a)
        
        # initial values for best match
        best_index = None
        best_score = float('inf')

        for index, landmark in enumerate(landmarks):
            if landmark is None or len(landmark) != 2:
                continue
            landmark_a, landmark_b = landmark

            # same logic for converting to angle for the landmark
            if np.isinf(landmark_a):
                landmark_angle = np.pi / 2 
            else:
                landmark_angle = np.arctan(landmark_a)

            # calculate angle and intercept differences
            angle_diff = abs(detected_angle - landmark_angle)
            angle_diff = min(angle_diff, 2 * np.pi - angle_diff)  # account for angle wrap-around    

            intercept_diff = abs(detected_b - landmark_b)

            if angle_diff < angle_threshold and intercept_diff < intercept_threshold:
                score = angle_diff + intercept_diff
                if score < best_score:
                    best_score = score
                    best_index = index

        return best_index

    def add_landmarks(self, new_landmarks, init_cov=None):
        """
        Append new landmarks to joint state and expand covariance.

        new_landmarks: iterable of (l0,l1) pairs (shape (m,2) or list of tuples)
        """
        if len(new_landmarks) == 0:
            return

        new_landmarks = np.array(new_landmarks, dtype=float).reshape(-1, 2)
        m = new_landmarks.shape[0]

        if init_cov is None:
            init_cov = np.diag([np.deg2rad(15.0)**2, 100.0])
        init_cov = np.asarray(init_cov, dtype=float)
        assert init_cov.shape == (2, 2), "init_cov must be a 2x2 matrix"
        
        # compute old state length from the canonical joint state
        old_n = self.state.size
        P_old = self.covariance
        assert P_old.shape == (old_n, old_n)

        # extend state vector (robot pose + existing landmarks already in self.state)
        self.state = np.concatenate([self.state.reshape(-1), new_landmarks.reshape(-1)])

        # expand covariance
        new_n = old_n + 2 * m
        P_new = np.zeros((new_n, new_n), dtype=float)
        P_new[:old_n, :old_n] = P_old
        for i in range(m):
            i0 = old_n + 2 * i
            P_new[i0:i0+2, i0:i0+2] = init_cov
        self.covariance = P_new

        # keep a derived landmarks array (optional, avoid maintaining twice)
        self.landmarks = self.state[3:].reshape(-1, 2)

    def state_predict(self, control_input):
        self.state[:3] += control_input
        return self.state 

    def update_landmark(self, landmark_idx, measurement, R=None):
        if R is None:
            R = np.diag([0.05**2, 10.0**2])

        z = np.asarray(measurement, dtype=float)

        lm_start = 3 + 2 * landmark_idx
        x_hat = self.state[lm_start:lm_start + 2]

        H = np.zeros((2, self.state.size))
        H[:, lm_start:lm_start + 2] = np.eye(2)

        if self.covariance.shape[0] != self.state.size:
            raise ValueError("Covariance shape does not match state size")

        y = z - x_hat
        S = H @ self.covariance @ H.T + R
        K = self.covariance @ H.T @ np.linalg.inv(S)

        self.state = self.state + K @ y
        self.covariance = (np.eye(self.state.size) - K @ H) @ self.covariance

        self.landmarks = self.state[3:].reshape(-1, 2)