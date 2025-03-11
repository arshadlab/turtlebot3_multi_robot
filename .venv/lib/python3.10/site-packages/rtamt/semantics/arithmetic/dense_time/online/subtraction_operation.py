from rtamt.semantics.abstract_dense_time_online_operation import AbstractDenseTimeOnlineOperation
import rtamt.semantics.stl.dense_time.online.intersection as intersect

class SubtractionOperation(AbstractDenseTimeOnlineOperation):
    def __init__(self):
        self.sample_left = []
        self.sample_right = []

    def reset(self):
        pass

    def update(self, sample_left, sample_right, *args, **kargs):
        self.sample_left = self.sample_left + sample_left
        self.sample_right = self.sample_right + sample_right

        sample_result, last, left, right = intersect.intersection(self.sample_left, self.sample_right, intersect.subtraction)

        self.sample_left = left
        self.sample_right = right
        if sample_result:
            self.last = last

        return sample_result

    def update_final(self, sample_left, sample_right, *args, **kargs):
        return self.update(sample_left, sample_right, *args, **kargs) + [self.last]
