# -*- coding: utf-8 -*-
from abc import abstractmethod

from rtamt.semantics.abstract_online_operation import AbstractOnlineOperation

class AbstractDenseTimeOnlineOperation(AbstractOnlineOperation):
    """
    Abstract Desne TimeOperation: template for online operation
    """

    @abstractmethod
    def update_final(self, node, *args, **kargs):
        raise NotImplementedError(self.NOT_IMPLEMENTED)
