from rtamt.semantics.abstract_discrete_time_offline_interpreter import discrete_time_offline_interpreter_factory
from rtamt.semantics.iastl.discrete_time.offline.ast_visitor import IAStlDiscreteTimeOfflineAstVisitor, \
    IAStlOutputRobustnessDiscreteTimeOfflineAstVisitor, IAStlInputRobustnessDiscreteTimeOfflineAstVisitor, \
    IAStlOutputVacuityDiscreteTimeOfflineAstVisitor, IAStlInputVacuityDiscreteTimeOfflineAstVisitor


def IAStlOutputRobustnessDiscreteTimeOfflineInterpreter():
    iastlDiscreteTimeOfflineInterpreter = \
        discrete_time_offline_interpreter_factory(IAStlOutputRobustnessDiscreteTimeOfflineAstVisitor)()
    return iastlDiscreteTimeOfflineInterpreter

def IAStlInputRobustnessDiscreteTimeOfflineInterpreter():
    iastlDiscreteTimeOfflineInterpreter = discrete_time_offline_interpreter_factory(IAStlInputRobustnessDiscreteTimeOfflineAstVisitor)()
    return iastlDiscreteTimeOfflineInterpreter

def IAStlOutputVacuityDiscreteTimeOfflineInterpreter():
    iastlDiscreteTimeOfflineInterpreter = discrete_time_offline_interpreter_factory(IAStlOutputVacuityDiscreteTimeOfflineAstVisitor)()
    return iastlDiscreteTimeOfflineInterpreter

def IAStlInputVacuityDiscreteTimeOfflineInterpreter():
    iastlDiscreteTimeOfflineInterpreter = discrete_time_offline_interpreter_factory(IAStlInputVacuityDiscreteTimeOfflineAstVisitor)()
    return iastlDiscreteTimeOfflineInterpreter
