from rtamt.semantics.abstract_dense_time_online_operation import AbstractDenseTimeOnlineOperation
from rtamt.semantics.stl.dense_time.online.since_operation import SinceOperation
from rtamt.semantics.stl.dense_time.online.once_timed_operation import OnceTimedOperation
from rtamt.semantics.stl.dense_time.online.historically_timed_operation import HistoricallyTimedOperation
from rtamt.semantics.stl.dense_time.online.and_operation import AndOperation

class SinceTimedOperation(AbstractDenseTimeOnlineOperation):
    def __init__(self, begin, end):
        self.sample_left_buf = []
        self.sample_right_buf = []
        self.begin = begin
        self.end = end

        self.since = SinceOperation()
        self.hist = HistoricallyTimedOperation(0, self.begin)
        self.once = OnceTimedOperation(self.begin, self.end)
        self.andop = AndOperation()

    def reset(self):
        pass

    def update(self, sample_left, sample_right, *args, **kargs):
        self.sample_left_buf = self.sample_left_buf + sample_left
        self.sample_right_buf = self.sample_right_buf + sample_right

        out1 = self.once.update(sample_right)
        out2 = self.since.update(sample_left, sample_right)
        out3 = self.hist.update(out2)
        sample_result = self.andop.update(out1, out3)

        return sample_result

    def update_final(self, sample_left, sample_right, *args, **kargs):
        self.sample_left_buf = self.sample_left_buf + sample_left
        self.sample_right_buf = self.sample_right_buf + sample_right

        out1 = self.once.update_final(sample_right)
        out2 = self.since.update_final(sample_left, sample_right)
        out3 = self.hist.update_final(out2)
        sample_result = self.andop.update_final(out1, out3)

        return sample_result
