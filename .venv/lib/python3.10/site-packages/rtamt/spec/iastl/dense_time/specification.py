from rtamt.pastifier.stl.pastifier import StlPastifier
from rtamt.spec.abstract_specification import AbstractOfflineSpecification, AbstractOnlineSpecification

from rtamt.semantics.iastl.dense_time.offline.interpreter import IAStlOutputRobustnessDenseTimeOfflineInterpreter, \
    IAStlInputVacuityDenseTimeOfflineInterpreter, IAStlInputRobustnessDenseTimeOfflineInterpreter, \
    IAStlOutputVacuityDenseTimeOfflineInterpreter

from rtamt.semantics.iastl.dense_time.online.interpreter import IAStlOutputRobustnessDenseTimeOnlineInterpreter, \
    IAStlInputVacuityDenseTimeOnlineInterpreter, IAStlInputRobustnessDenseTimeOnlineInterpreter, \
    IAStlOutputVacuityDenseTimeOnlineInterpreter

from rtamt.syntax.ast.parser.stl.specification_parser import StlAst


def IAStlOutputRobustnessDenseTimeOfflineSpecification():
    spec = AbstractOfflineSpecification(StlAst(), IAStlOutputRobustnessDenseTimeOfflineInterpreter())
    return spec

def IAStlInputVacuityDenseTimeOfflineSpecification():
    spec = AbstractOfflineSpecification(StlAst(), IAStlInputVacuityDenseTimeOfflineInterpreter())
    return spec

def IAStlInputRobustnessDenseTimeOfflineSpecification():
    spec = AbstractOfflineSpecification(StlAst(), IAStlInputRobustnessDenseTimeOfflineInterpreter())
    return spec

def IAStlOutputVacuityDenseTimeOfflineSpecification():
    spec = AbstractOfflineSpecification(StlAst(), IAStlOutputVacuityDenseTimeOfflineInterpreter())
    return spec

def IAStlOutputRobustnessDenseTimeOnlineSpecification():
    spec = AbstractOnlineSpecification(StlAst(), IAStlOutputRobustnessDenseTimeOnlineInterpreter(),
                                       pastifier=StlPastifier())
    return spec

def IAStlInputVacuityDenseTimeOnlineSpecification():
    spec = AbstractOnlineSpecification(StlAst(), IAStlInputVacuityDenseTimeOnlineInterpreter(),
                                       pastifier=StlPastifier())
    return spec

def IAStlInputRobustnessDenseTimeOnlineSpecification():
    spec = AbstractOnlineSpecification(StlAst(), IAStlInputRobustnessDenseTimeOnlineInterpreter(),
                                       pastifier=StlPastifier())
    return spec

def IAStlOutputVacuityDenseTimeOnlineSpecification():
    spec = AbstractOnlineSpecification(StlAst(), IAStlOutputVacuityDenseTimeOnlineInterpreter(), pastifier=StlPastifier())
    return spec
