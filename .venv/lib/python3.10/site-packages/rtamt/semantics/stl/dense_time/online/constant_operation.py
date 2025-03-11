
from rtamt.semantics.abstract_dense_time_online_operation import AbstractDenseTimeOnlineOperation

class ConstantOperation(AbstractDenseTimeOnlineOperation):
    def __init__(self, val):
        self.val = val

    def update(self, *args, **kargs):
        out = [[0, self.val], [float("inf"), self.val]]
        return out

    def update_final(self, *args, **kargs):
        out = [[0, self.val], [float("inf"), self.val]]
        return
