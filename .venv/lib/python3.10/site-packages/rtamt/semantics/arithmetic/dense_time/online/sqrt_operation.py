import math
from rtamt.semantics.abstract_dense_time_online_operation import AbstractDenseTimeOnlineOperation

class SqrtOperation(AbstractDenseTimeOnlineOperation):
    def __init__(self):
        pass

    def reset(self):
        pass

    def update(self, sample):
        sample_result = []

        for i in sample:
            if i[1] < 0:
                raise Exception('sqrt: input is smaller than 0.')
            out_time = i[0]
            out_value = math.sqrt(i[1])
            sample_result.append([out_time, out_value])

        return sample_result

    def update_final(self, sample, *args, **kargs):
        return self.update(sample, *args, **kargs)