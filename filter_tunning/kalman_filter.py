import numpy as np

class KalmanFilter1D:
    def __init__(self, Q, R, x0=None, P0=None):
        """
        Initializes a Kalman filter with the given parameters.

        Parameters
        ----------
        Q (float): Process error covariance
        R (float): Measurement error covariance
        x0 (float): Initial state estimate
        P0 (float): Initial state covariance
        """

        self.Q = Q
        self.R = R
        if x0 == None:
            self.x = 0
        else:
            self.x = x0
        if P0 == None:
            self.P = 0
        else:
            self.P = P0

    def update(self, z):
        """
        Performs the predict and update steps of the Kalman filter.

        Parameters
        ----------
        z (float): Measurement value

        Returns
        -------
        x (float): The updated state estimate
        """

        # predict step
        x_pred = self.x
        P_pred = self.P + self.Q

        # update step
        K = P_pred / (P_pred + self.R)
        self.x = x_pred + K * (z - x_pred)
        self.P = (1 - K) * P_pred

        return self.x

    def get_state(self):
        """
        Gets the current state estimate.

        Returns
        -------
        x (float): The current state estimate
        """

        return self.x

    def get_covariance(self):
        """
        Gets the current state covariance.

        Returns
        -------
        P (float): The current state covariance
        """

        return self.P

class KalmanFilterND:
    def __init__(self, F, H, Q, R, x0=None, P0=None):
        """
        Initializes a Kalman filter with the given parameters.

        Parameters
        ----------
        F (ndarray): State transition matrix
        H (ndarray): Measurement matrix
        Q (ndarray): Process error covariance matrix
        R (ndarray): Measurement error covariance matrix
        x0 (ndarray): Initial state estimate
        P0 (ndarray): Initial state covariance matrix
        """

        self.F = F
        self.H = H
        self.Q = Q
        self.R = R
        if x0 == None:
            self.x = np.zeros((F.shape[0], 1))
        else:
            self.x = x0
        if P0 == None:
            self.P = np.eye(F.shape[0])
        else:
            self.P = P0

    def update(self, z):
        """
        Performs the predict and update steps of the Kalman filter.

        Parameters
        ----------
        z (ndarray): Measurement vector

        Returns
        -------
        x (ndarray): The updated state estimate
        """

        # Predict step
        x_pred = np.dot(self.F, self.x)
        P_pred = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

        # Update step
        y = z - np.dot(self.H, x_pred)
        S = np.dot(np.dot(self.H, P_pred), self.H.T) + self.R
        K = np.dot(np.dot(P_pred, self.H.T), np.linalg.inv(S))

        self.x = x_pred + np.dot(K, y)
        self.P = np.dot((np.eye(self.x.shape[0]) - np.dot(K, self.H)), P_pred)

    def get_state(self):
        """
        Gets the current state estimate.

        Returns
        -------
        x (ndarray): The current state estimate
        """

        return self.x

    def get_covariance(self):
        """
        Gets the current state covariance matrix.

        Returns
        -------
        P (ndarray): The current state covariance matrix
        """

        return self.P
