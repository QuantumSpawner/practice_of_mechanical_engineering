import numpy as np

class Filter:
    def update(self, x):
        """
        Add a new input sample to the filter, and return the filtered output.

        Parameters
        ----------
        x (float) :The new input sample(s) to filter.

        Returns
        -------
        filtered (float): The filtered output sample(s).
        """
        raise NotImplementedError("Subclasses should implement this method")

class FilterChain:
    def __init__(self, filters):
        """
        Initializes a FilterChain with a list of filters.

        Parameters
        ----------
        filters (list): A list of filter instances to be applied in sequence.
        """
        self.filters = filters

    def update(self, x):
        """
        Add a new input sample to the filter chain, and return the filtered output.

        Parameters
        ----------
        x (float) :The new input sample(s) to filter.

        Returns
        -------
        filtered (float): The filtered output sample(s) after passing through all filters in the chain.
        """
        for filter_instance in self.filters:
            x = filter_instance.update(x)
        return x

class NormalizeFilter(Filter):
    def __init__(self, upper_bound=0, lower_bound=1):
        """
        Initializes a Normalize filter with the given parameters.

        Parameters
        ----------
        upper_bound (float): Upper bound
        lower_bound (float): Lower bound
        """

        self.upper_bound = upper_bound
        self.lower_bound = lower_bound

    def update(self, x):
        """
        Add a new input sample to the filter, and return the filtered output.

        Parameters
        ----------
        x (float) :The new input sample(s) to filter.

        Returns
        -------
        filtered (float): The filtered output sample(s), computed as the exponential moving average of the input samples.
        """

        if x > self.upper_bound:
            self.upper_bound = x
        elif x < self.lower_bound:
            self.lower_bound = x
        return (x - self.lower_bound) / (self.upper_bound - self.lower_bound)

class MovingAverageFilter(Filter):
    def __init__(self, window_size):
        """
        Initialize the moving average filter.

        Parameters
        ----------
        window_size (int): The number of input samples to average over.
        """

        self.window_size = window_size
        self.buffer = np.zeros(window_size)
        self.index = 0
        self.length = 0
    
    def update(self, x):
        """
        Add a new input sample to the filter, and return the filtered output.

        Parameters
        ----------
        x (float) :The new input sample(s) to filter.

        Returns
        -------
        filtered (float): The filtered output sample(s), computed as the mean of the last `window_size` input samples.
        """

        self.buffer[self.index] = x
        if self.length < self.window_size:
            filtered = np.mean(self.buffer[ :self.length + 1])
        else:
            filtered = np.mean(self.buffer)

        self.index = (self.index + 1) % self.window_size
        self.length += 1
        return filtered

class ExponentialFilter(Filter):
    def __init__(self, alpha):
        """
        Initialize the exponential filter.

        Parameters
        ----------
        alpha (float): The smoothing factor. Should be between 0 and 1.
        """

        self.alpha = alpha
        self.filtered = 0
        self.length = 0
    
    def update(self, x):
        """
        Add a new input sample to the filter, and return the filtered output.

        Parameters
        ----------
        x (float) :The new input sample(s) to filter.

        Returns
        -------
        filtered (float): The filtered output sample(s), computed as the exponential moving average of the input samples.
        """

        if(self.length == 0):
            self.filtered = x

        self.filtered = self.alpha * x + (1 - self.alpha) * self.filtered
        self.length += 1
        return self.filtered

class KalmanFilter1D(Filter):
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
