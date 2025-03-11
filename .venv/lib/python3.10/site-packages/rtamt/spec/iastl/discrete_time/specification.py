from rtamt.semantics.iastl.discrete_time.offline.interpreter import IAStlOutputRobustnessDiscreteTimeOfflineInterpreter, \
    IAStlInputVacuityDiscreteTimeOfflineInterpreter, IAStlInputRobustnessDiscreteTimeOfflineInterpreter, \
    IAStlOutputVacuityDiscreteTimeOfflineInterpreter
from rtamt.semantics.stl.discrete_time.online.interpreter import StlDiscreteTimeOnlineInterpreter
from rtamt.spec.abstract_specification import AbstractOfflineSpecification, AbstractOnlineSpecification, AbstractOfflineOnlineSpecification

from rtamt.syntax.ast.parser.stl.specification_parser import StlAst


from rtamt.semantics.stl.discrete_time.offline.interpreter import StlDiscreteTimeOfflineInterpreter
from rtamt.semantics.iastl.discrete_time.online.interpreter import IAStlOutputRobustnessDiscreteTimeOnlineInterpreter, \
    IAStlInputVacuityDiscreteTimeOnlineInterpreter, IAStlInputRobustnessDiscreteTimeOnlineInterpreter, \
    IAStlOutputVacuityDiscreteTimeOnlineInterpreter
from rtamt.semantics.enumerations.options import *
from rtamt.pastifier.stl.pastifier import StlPastifier


def IASTLDiscreteTimeSpecification(semantics=Semantics.STANDARD, language=Language.PYTHON):
    """
    A class used as a container for STL continuous time specifications
       Inherits STLSpecification

    Attributes:
    """
    if semantics == Semantics.STANDARD and language == Language.PYTHON:
        spec = AbstractOfflineOnlineSpecification(StlAst(), StlDiscreteTimeOfflineInterpreter(), StlDiscreteTimeOnlineInterpreter(), pastifier=StlPastifier())
    elif semantics == Semantics.OUTPUT_ROBUSTNESS and language == Language.PYTHON:
        spec = AbstractOfflineOnlineSpecification(StlAst(), StlDiscreteTimeOfflineInterpreter(), IAStlOutputRobustnessDiscreteTimeOnlineInterpreter(semantics), pastifier=StlPastifier())
    return spec



def IAStlOutputRobustnessDiscreteTimeOfflineSpecification():
    spec = AbstractOfflineSpecification(StlAst(), IAStlOutputRobustnessDiscreteTimeOfflineInterpreter())
    return spec

def IAStlInputVacuityDiscreteTimeOfflineSpecification():
    spec = AbstractOfflineSpecification(StlAst(), IAStlInputVacuityDiscreteTimeOfflineInterpreter())
    return spec

def IAStlInputRobustnessDiscreteTimeOfflineSpecification():
    spec = AbstractOfflineSpecification(StlAst(), IAStlInputRobustnessDiscreteTimeOfflineInterpreter())
    return spec

def IAStlOutputVacuityDiscreteTimeOfflineSpecification():
    spec = AbstractOfflineSpecification(StlAst(), IAStlOutputVacuityDiscreteTimeOfflineInterpreter())
    return spec

def IAStlOutputRobustnessDiscreteTimeOnlineSpecification():
    spec = AbstractOnlineSpecification(StlAst(), IAStlOutputRobustnessDiscreteTimeOnlineInterpreter(),
                                       pastifier=StlPastifier())
    return spec

def IAStlInputVacuityDiscreteTimeOnlineSpecification():
    spec = AbstractOnlineSpecification(StlAst(), IAStlInputVacuityDiscreteTimeOnlineInterpreter(),
                                       pastifier=StlPastifier())
    return spec

def IAStlInputRobustnessDiscreteTimeOnlineSpecification():
    spec = AbstractOnlineSpecification(StlAst(), IAStlInputRobustnessDiscreteTimeOnlineInterpreter(),
                                       pastifier=StlPastifier())
    return spec

def IAStlOutputVacuityDiscreteTimeOnlineSpecification():
    spec = AbstractOnlineSpecification(StlAst(), IAStlOutputVacuityDiscreteTimeOnlineInterpreter(), pastifier=StlPastifier())
    return spec



