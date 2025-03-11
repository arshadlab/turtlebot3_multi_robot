from rtamt.semantics.abstract_online_operation import AbstractOnlineOperation

class ImpliesOperation(AbstractOnlineOperation):
    def __init__(self):
        pass

    def reset(self):
        pass

    def update(self, sample_left, sample_right):
        sample_return = max(-sample_left, sample_right)
        return sample_return
