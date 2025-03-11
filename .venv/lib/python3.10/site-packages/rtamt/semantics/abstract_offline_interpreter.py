from abc import abstractmethod

from rtamt.semantics.abstract_interpreter import AbstractInterpreter

class AbstractOfflineInterpreter(AbstractInterpreter):
    def __init__(self):
        super(AbstractOfflineInterpreter, self).__init__()
        return

    @abstractmethod
    def evaluate(self, dataset):
        raise NotImplementedError(self.NOT_IMPLEMENTED)

    def visitSpec(self, node, *args, **kwargs):
        sample_return = self.visit(node, *args, **kwargs)
        self.ast.var_object_dict[node] = sample_return  #TODO subspec name is necessary as a key for var_object_dict.
        return sample_return
