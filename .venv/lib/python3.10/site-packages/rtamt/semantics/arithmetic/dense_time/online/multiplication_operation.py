from rtamt.semantics.abstract_dense_time_online_operation import AbstractDenseTimeOnlineOperation
import rtamt.semantics.stl.dense_time.online.intersection as intersect

class MultiplicationOperation(AbstractDenseTimeOnlineOperation):
    def __init__(self):
        self.sample_left_buf = []
        self.sample_right_buf = []

    def reset(self):
        pass

    def update(self, sample_left, sample_right, *args, **kargs):
        sample_result = []
        self.sample_left_buf = self.sample_left_buf + sample_left
        self.sample_right_buf = self.sample_right_buf + sample_right

        sample_result, last, left, right = intersect.intersection(self.sample_left_buf, self.sample_right_buf, intersect.multiplication)

        self.sample_left_buf = left
        self.sample_right_buf = right
        if sample_result:
            self.last = last

        return sample_result

    def update_final(self, sample_left, sample_right, *args, **kargs):
        return self.update(sample_left, sample_right, *args, **kargs) + [self.last]
