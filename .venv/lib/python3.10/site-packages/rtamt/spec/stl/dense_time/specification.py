from rtamt.semantics.iastl.dense_time.offline.interpreter import IAStlOutputRobustnessDenseTimeOfflineInterpreter, \
    IAStlInputRobustnessDenseTimeOfflineInterpreter, IAStlInputVacuityDenseTimeOfflineInterpreter, \
    IAStlOutputVacuityDenseTimeOfflineInterpreter
from rtamt.semantics.iastl.dense_time.online.interpreter import IAStlOutputRobustnessDenseTimeOnlineInterpreter, \
    IAStlInputRobustnessDenseTimeOnlineInterpreter, IAStlOutputVacuityDenseTimeOnlineInterpreter, \
    IAStlInputVacuityDenseTimeOnlineInterpreter
from rtamt.pastifier.stl.pastifier import StlPastifier
from rtamt.spec.abstract_specification import AbstractOfflineSpecification, AbstractOnlineSpecification, AbstractOfflineOnlineSpecification

from rtamt.syntax.ast.parser.stl.specification_parser import StlAst

from rtamt.semantics.stl.dense_time.offline.interpreter import StlDenseTimeOfflineInterpreter
from rtamt.semantics.stl.dense_time.online.interpreter import StlDenseTimeOnlineInterpreter

from rtamt.semantics.enumerations.options import *


def StlDenseTimeSpecification(semantics=Semantics.STANDARD, language=Language.PYTHON):
    """
    A class used as a container for STL continuous time specifications
       Inherits STLSpecification

    Attributes:
    """
    if semantics == Semantics.STANDARD and language == Language.PYTHON:
        spec = AbstractOfflineOnlineSpecification(StlAst(), StlDenseTimeOfflineInterpreter(), StlDenseTimeOnlineInterpreter(), pastifier=StlPastifier())
    elif semantics == Semantics.OUTPUT_ROBUSTNESS and language == Language.PYTHON:
        spec = AbstractOfflineOnlineSpecification(StlAst(), IAStlOutputRobustnessDenseTimeOfflineInterpreter(),
                                                  IAStlOutputRobustnessDenseTimeOnlineInterpreter(), pastifier=StlPastifier())
    elif semantics == Semantics.INPUT_ROBUSTNESS and language == Language.PYTHON:
        spec = AbstractOfflineOnlineSpecification(StlAst(), IAStlInputRobustnessDenseTimeOfflineInterpreter(),
                                                  IAStlInputRobustnessDenseTimeOnlineInterpreter(), pastifier=StlPastifier())
    elif semantics == Semantics.INPUT_VACUITY and language == Language.PYTHON:
        spec = AbstractOfflineOnlineSpecification(StlAst(), IAStlInputVacuityDenseTimeOfflineInterpreter(),
                                                  IAStlInputVacuityDenseTimeOnlineInterpreter(), pastifier=StlPastifier())
    elif semantics == Semantics.OUTPUT_VACUITY and language == Language.PYTHON:
        spec = AbstractOfflineOnlineSpecification(StlAst(), IAStlOutputVacuityDenseTimeOfflineInterpreter(),
                                                  IAStlOutputVacuityDenseTimeOnlineInterpreter(), pastifier=StlPastifier())
    else:
        raise Exception()

    return spec

def StlDenseTimeOfflineSpecification():
    spec = AbstractOfflineSpecification(StlAst(), StlDenseTimeOfflineInterpreter())
    return spec

def StlDenseTimeOnlineSpecification():
    spec = AbstractOnlineSpecification(StlAst(), StlDenseTimeOnlineInterpreter(), pastifier=StlPastifier())
    return spec
