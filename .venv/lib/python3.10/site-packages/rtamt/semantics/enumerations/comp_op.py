# -*- coding: utf-8 -*-
"""
Created on Sun Jul 21 22:39:19 2019

@author: NickovicD
"""
from enum import Enum

class StlComparisonOperator(Enum):
    LESS = 0
    LEQ = 1
    EQUAL = 2
    NEQ = 3
    GREATER = 4
    GEQ = 5

    def __str__(self):
        if self.value == 0:
            return '<'
        elif self.value == 1:
            return '<='
        elif self.value == 2:
            return '=='
        elif self.value == 3:
            return '!='
        elif self.value == 4:
            return '>'
        else:
            return '>='