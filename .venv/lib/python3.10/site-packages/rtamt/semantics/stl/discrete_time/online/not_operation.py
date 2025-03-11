from rtamt.semantics.abstract_online_operation import AbstractOnlineOperation

class NotOperation(AbstractOnlineOperation):
    def __init__(self):
        pass

    def reset(self):
        pass

    def update(self, sample):
        sample_return = - sample
        return sample_return
