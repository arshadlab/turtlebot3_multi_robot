from rtamt.semantics.abstract_dense_time_online_operation import AbstractDenseTimeOnlineOperation

class NotOperation(AbstractDenseTimeOnlineOperation):
    def __init__(self):
        self.input = []

    def reset(self):
        pass

    def update(self, sample, *args, **kargs):
        sample_result = []

        for i in sample:
            out_time = i[0]
            out_value = - i[1]

            sample_result.append([out_time, out_value])

        return sample_result

    def update_final(self, sample, *args, **kargs):
        return self.update(sample, *args, **kargs)
