from rtamt.semantics.iastl.dense_time.offline.ast_visitor import IAStlDenseTimeOfflineAstVisitor, \
    IAStlOutputRobustnessDenseTimeOfflineAstVisitor, IAStlInputRobustnessDenseTimeOfflineAstVisitor, \
    IAStlInputVacuityDenseTimeOfflineAstVisitor, IAStlOutputVacuityDenseTimeOfflineAstVisitor
from rtamt.semantics.abstract_dense_time_offline_interpreter import dense_time_offline_interpreter_factory

def IAStlOutputRobustnessDenseTimeOfflineInterpreter():
    iastlDenseTimeOfflineInterpreter = dense_time_offline_interpreter_factory(IAStlOutputRobustnessDenseTimeOfflineAstVisitor)()
    return iastlDenseTimeOfflineInterpreter

def IAStlInputRobustnessDenseTimeOfflineInterpreter():
    iastlDenseTimeOfflineInterpreter = dense_time_offline_interpreter_factory(IAStlInputRobustnessDenseTimeOfflineAstVisitor)()
    return iastlDenseTimeOfflineInterpreter

def IAStlInputVacuityDenseTimeOfflineInterpreter():
    iastlDenseTimeOfflineInterpreter = dense_time_offline_interpreter_factory(IAStlInputVacuityDenseTimeOfflineAstVisitor)()
    return iastlDenseTimeOfflineInterpreter

def IAStlOutputVacuityDenseTimeOfflineInterpreter():
    iastlDenseTimeOfflineInterpreter = dense_time_offline_interpreter_factory(IAStlOutputVacuityDenseTimeOfflineAstVisitor)()
    return iastlDenseTimeOfflineInterpreter
