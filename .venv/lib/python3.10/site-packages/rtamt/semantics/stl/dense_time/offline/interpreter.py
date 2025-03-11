
from rtamt.semantics.stl.dense_time.offline.ast_visitor import StlDenseTimeOfflineAstVisitor
from rtamt.semantics.abstract_dense_time_offline_interpreter import dense_time_offline_interpreter_factory

def StlDenseTimeOfflineInterpreter():
    stlDenseTimeOfflineInterpreter = dense_time_offline_interpreter_factory(StlDenseTimeOfflineAstVisitor)()
    return stlDenseTimeOfflineInterpreter
