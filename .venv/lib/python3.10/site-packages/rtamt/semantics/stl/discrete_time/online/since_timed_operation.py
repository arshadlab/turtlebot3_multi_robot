import collections
from rtamt.semantics.abstract_online_operation import AbstractOnlineOperation

class SinceTimedOperation(AbstractOnlineOperation):
    def __init__(self, begin, end):
        self.begin = begin
        self.end = end

        self.buffer_sample_left = collections.deque(maxlen=(self.end + 1))
        self.buffer_sample_right = collections.deque(maxlen=(self.end + 1))

        self.reset()

    def reset(self):
        for i in range(self.end + 1):
            s_sample_left = float("inf")
            s_sample_right = - float("inf")
            self.buffer_sample_left.append(s_sample_left)
            self.buffer_sample_right.append(s_sample_right)

    def update(self, sample_left, sample_right):
        self.buffer_sample_left.append(sample_left)
        self.buffer_sample_right.append(sample_right)
        sample_return = - float("inf")

        for i in range(self.end-self.begin+1):
            sample_left = float("inf")
            sample_right = self.buffer_sample_right[i]
            for j in range(i+1,self.end+1):
                sample_left = min(sample_left, self.buffer_sample_left[j])
            sample_return = max(sample_return, min(sample_left, sample_right))

        return sample_return

