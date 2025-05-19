from enum import Enum
from collections import deque
import numpy as np

class FilterType(Enum):
    MOVING_AVG = "MOVING_AVG"


class Filter(object):
    def __init__(self, window_size, filter_type=FilterType.MOVING_AVG):
        self.window_size = window_size
        self.data = deque()
        self.sum = np.zeros(3)  # NOTE: currently only 3D vectors are supported
        if filter_type == FilterType.MOVING_AVG:
            self.update_method = self.update_moving_average
        else:
            raise ValueError(f"Unsupported filter type: {filter_type}")

    def update(self, vals: np.ndarray):
        if not isinstance(vals, np.ndarray):
            raise ValueError("Input data must be a numpy array.")
        if vals.shape != (3,):
            raise ValueError("Input data must be a 3D vector.")
        return self.update_method(vals)

    def update_moving_average(self, vals):
        self.data.append(vals)
        self.sum += vals
        if len(self.data) > self.window_size:
            self.sum -= self.data.popleft()
        return self.sum / len(self.data)


if __name__ == "__main__":
    # Example usage
    filter = Filter(window_size=5)
    for i in range(10):
        data = np.array([i, i+1, i+2])
        print(f"Input: {data}, Filtered: {filter.update(data)}")