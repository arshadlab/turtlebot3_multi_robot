import collections
from rtamt.semantics.abstract_online_operation import AbstractOnlineOperation
class PrecedesTimedOperation(AbstractOnlineOperation):
    def __init__(self, begin, end):
        self.begin = begin
        self.end = end

        self.buffer = []
        self.buffer.append(collections.deque(maxlen=(self.end + 1)))
        self.buffer.append(collections.deque(maxlen=(self.end + 1)))

        self.reset()

    def reset(self):
        for i in range(self.end + 1):
            s_sample_left = float("inf")
            s_sample_right = - float("inf")
            self.buffer[0].append(s_sample_left)
            self.buffer[1].append(s_sample_right)

    def update(self, sample_left, sample_right):
        self.buffer[0].append(sample_left)
        self.buffer[1].append(sample_right)
        sample_return = - float("inf")

        for i in range(self.begin, self.end+1):
            sample_left = float("inf")
            sample_right = self.buffer[1][i]
            for j in range(0, i):
                sample_left = min(sample_left, self.buffer[0][j])
            sample_return = max(sample_return, min(sample_left, sample_right))

        return sample_return
