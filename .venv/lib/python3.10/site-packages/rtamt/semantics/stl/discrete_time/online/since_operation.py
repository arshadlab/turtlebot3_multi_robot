from rtamt.semantics.abstract_online_operation import AbstractOnlineOperation

class SinceOperation(AbstractOnlineOperation):
    def __init__(self):
        self.prev_out = -float("inf")

    def reset(self):
        self.__init__()

    def update(self, sample_left, sample_right):
        sample_return = min(sample_left, self.prev_out)
        sample_return = max(sample_return, sample_right)
        self.prev_out = sample_return
        return sample_return
