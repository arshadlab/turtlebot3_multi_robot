import math
from rtamt.semantics.abstract_online_operation import AbstractOnlineOperation


class SqrtOperation(AbstractOnlineOperation):
    def __init__(self):
        pass

    def reset(self):
        pass

    def update(self, sample):
        if sample < 0:
            raise Exception('sqrt: input is smaller than 0.')
        sample_result = math.sqrt(sample)
        return sample_result