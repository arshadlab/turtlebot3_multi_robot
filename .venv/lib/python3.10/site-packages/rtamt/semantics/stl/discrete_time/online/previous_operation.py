from rtamt.semantics.abstract_online_operation import AbstractOnlineOperation

class PreviousOperation(AbstractOnlineOperation):
    def __init__(self):
        self.prev = float("inf")

    def reset(self):
        self.__init__()

    def update(self, sample):
        sample_return = self.prev
        self.prev = sample

        return sample_return
