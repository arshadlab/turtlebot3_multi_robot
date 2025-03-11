# -*- coding: utf-8 -*-

from rtamt.syntax.ast.visitor.abstract_ast_visitor import AbstractAstVisitor
from rtamt.semantics.abstract_offline_interpreter import AbstractOfflineInterpreter
from rtamt.semantics.dense_time_interpreter import DenseTimeInterpreter

from rtamt.exception.exception import RTAMTException

class AbstractDenseTimeOfflineInterpreter(AbstractOfflineInterpreter, DenseTimeInterpreter):

    def __init__(self):
        super(AbstractDenseTimeOfflineInterpreter, self).__init__()
        return

    #input format
    #a = [[0, 1.3], [0.7, 3], [1.3, 0.1], [2.1, -2.2]]
    #b = [[0, 2.5], [0.7, 4], [1.3, -1.2], [2.1, 1.7]]
    #dataset = [['a', a], ['b', b]]
    #TODO merge dense and discrete into evaluate AbstractOfflineInterpreter
    def evaluate(self, dataset):
        # check ast exists
        self.exist_ast()

        # update the value of every input variable
        self.set_variable_to_ast_from_dataset(dataset)


        rob = self.visitAst(self.ast)
        # evaluate spec forest

        #rob = self.visitAst(self.ast)[0]

        # reset var_object_dict()
        self.ast.var_object_dict = self.ast.var_object_dict.fromkeys(self.ast.var_object_dict, [])  #TODO I did not understant it.

        return rob[len(rob)-1]


def dense_time_offline_interpreter_factory(AstVisitor):
    if not issubclass(AstVisitor, AbstractAstVisitor):  # type check
        raise RTAMTException('{} is not RTAMT AST visitor'.format(AstVisitor.__name__))

    class DenseTimeOfflineInterpreter(AbstractDenseTimeOfflineInterpreter, AstVisitor):
        def __init__(self, *args, **kwargs):
            AbstractDenseTimeOfflineInterpreter.__init__(self, *args, **kwargs)
            AstVisitor.__init__(self)
    return DenseTimeOfflineInterpreter