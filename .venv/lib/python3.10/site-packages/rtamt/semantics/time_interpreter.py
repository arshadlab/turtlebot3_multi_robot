from abc import ABCMeta, abstractmethod

from rtamt.exception.exception import RTAMTException


class TimeInterpreter(object):
    __metaclass__ = ABCMeta

    def __init__(self):
        pass

    @abstractmethod
    def dataset_check(self, data):
        pass

    @abstractmethod
    def set_variable_to_ast_from_dataset(self, *args, **kargs):
        pass
