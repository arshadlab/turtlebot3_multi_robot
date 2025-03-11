from rtamt.semantics.abstract_dense_time_online_interpreter import dense_time_online_interpreter_factory
from rtamt.semantics.iastl.dense_time.online.ast_visitor import IAStlOutputRobustnessDenseTimeOnlineAstVisitor, \
    IAStlOutputVacuityDenseTimeOnlineAstVisitor, IAStlInputVacuityDenseTimeOnlineAstVisitor, \
    IAStlInputRobustnessDenseTimeOnlineAstVisitor


def IAStlOutputRobustnessDenseTimeOnlineInterpreter():
    iastlDenseTimeOnlineInterpreter = dense_time_online_interpreter_factory(IAStlOutputRobustnessDenseTimeOnlineAstVisitor)()
    return iastlDenseTimeOnlineInterpreter

def IAStlInputRobustnessDenseTimeOnlineInterpreter():
    iastlDenseTimeOnlineInterpreter = dense_time_online_interpreter_factory(IAStlInputRobustnessDenseTimeOnlineAstVisitor)()
    return iastlDenseTimeOnlineInterpreter

def IAStlOutputVacuityDenseTimeOnlineInterpreter():
    iastlDenseTimeOnlineInterpreter = dense_time_online_interpreter_factory(IAStlOutputVacuityDenseTimeOnlineAstVisitor)()
    return iastlDenseTimeOnlineInterpreter

def IAStlInputVacuityDenseTimeOnlineInterpreter():
    iastlDenseTimeOnlineInterpreter = dense_time_online_interpreter_factory(IAStlInputVacuityDenseTimeOnlineAstVisitor)()
    return iastlDenseTimeOnlineInterpreter
