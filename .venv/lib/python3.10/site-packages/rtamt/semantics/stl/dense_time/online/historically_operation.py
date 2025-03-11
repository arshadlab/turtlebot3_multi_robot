from rtamt.semantics.abstract_dense_time_online_operation import AbstractDenseTimeOnlineOperation


class HistoricallyOperation(AbstractDenseTimeOnlineOperation):
    def __init__(self):
        self.prev = float("inf")

    def reset(self):
        pass

    def update(self, sample, *args, **kargs):
        result_sample = []

        for i in sample:
            out_time = i[0]
            out_value = min(i[1], self.prev)

            result_sample.append([out_time, out_value])
            self.prev = out_value

        return result_sample

    def update_final(self, sample, *args, **kargs):
        return self.update(sample, *args, **kargs)
