# -*- coding: utf-8 -*-

from rtamt.syntax.ast.visitor.abstract_ast_visitor import AbstractAstVisitor
from rtamt.semantics.abstract_offline_interpreter import AbstractOfflineInterpreter
from rtamt.semantics.discrete_time_interpreter import DiscreteTimeInterpreter

from rtamt.exception.exception import RTAMTException

class AbstractDiscreteTimeOfflineInterpreter(AbstractOfflineInterpreter, DiscreteTimeInterpreter):

    def __init__(self):
        super(AbstractDiscreteTimeOfflineInterpreter, self).__init__()
        return

    #input format
    #dataset = {
    #   'time': [0, 1, 2, 3, 4],
    #   'req': [100, -1, -2, 5, -1],
    #   'gnt': [20, -2, 10, 4, -1]
    #}
    #TODO merge dense and discrete into evaluate AbstractOfflineInterpreter
    def evaluate(self, dataset):
        # check ast exists
        self.exist_ast()

        #TODO move both of spec and sub-specs visit into syntax layer.
        # update the value of every input variable
        self.set_variable_to_ast_from_dataset(dataset)

        # evaluate spec forest
        length = len(dataset['time'])
        rob = self.visitAst(self.ast, length)
        rob = rob[len(rob)-1]

        # Check if the difference between two consecutive timestamps is between
        # the accepted tolerance - if not, increase the violation counter
        ts = dataset['time']
        for i in range(len(ts) - 1):
            duration = (ts[i+1] - ts[i]) * self.normalize
        self.update_sampling_violation_counter(duration)

        # convert format
        out_t = [[a[0], a[1]] for a in zip(ts, rob)]
        rob = out_t

        return rob

    def set_variable_to_ast_from_dataset(self, dataset):
        for key in dataset:
            if key != 'time':
                self.ast.var_object_dict[key] = dataset[key]


def discrete_time_offline_interpreter_factory(AstVisitor):
    if not issubclass(AstVisitor, AbstractAstVisitor):  # type check
        raise RTAMTException('{} is not RTAMT AST visitor'.format(AstVisitor.__name__))

    class DiscreteTimeOfflineInterpreter(AbstractDiscreteTimeOfflineInterpreter, AstVisitor):
        def __init__(self, *args, **kwargs):
            super(DiscreteTimeOfflineInterpreter, self).__init__(*args, **kwargs)
    return DiscreteTimeOfflineInterpreter