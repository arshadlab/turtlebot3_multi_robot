import math
from rtamt.semantics.abstract_online_operation import AbstractOnlineOperation


class PowOperation(AbstractOnlineOperation):
    def __init__(self):
        pass

    def reset(self):
        pass

    def update(self, sample_left, sample_right):
        sample_result = math.pow(sample_left, sample_right)
        return sample_result