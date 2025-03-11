from rtamt.semantics.stl.dense_time.online.ast_visitor import StlDenseTimeOnlineAstVisitor
from rtamt.semantics.abstract_dense_time_online_interpreter import dense_time_online_interpreter_factory

def StlDenseTimeOnlineInterpreter():
    stlDenseTimeOnlineInterpreter = dense_time_online_interpreter_factory(StlDenseTimeOnlineAstVisitor)()
    return stlDenseTimeOnlineInterpreter
