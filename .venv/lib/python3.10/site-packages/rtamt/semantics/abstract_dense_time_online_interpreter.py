# -*- coding: utf-8 -*-
import operator

from rtamt.syntax.node.ltl.variable import Variable
from rtamt.syntax.node.ltl.constant import Constant

from rtamt.syntax.ast.visitor.abstract_ast_visitor import AbstractAstVisitor
from rtamt.semantics.abstract_online_interpreter import AbstractOnlineInterpreter, AbstractOnlineUpdateVisitor
from rtamt.semantics.dense_time_interpreter import DenseTimeInterpreter

from rtamt.exception.exception import RTAMTException

class AbstractDenseTimeOnlineInterpreter(AbstractOnlineInterpreter, DenseTimeInterpreter):

    def __init__(self):
        AbstractOnlineInterpreter.__init__(self)
        DenseTimeInterpreter.__init__(self)
        self.updateVisitor = DenseTimeOnlineUpdateVisitor()
        self.updateFinalVisitor = DenseTimeOnlineUpdateFinalVisitor()
        return

    #input format
    #a = [[0, 1.3], [0.7, 3], [1.3, 0.1], [2.1, -2.2]]
    #b = [[0, 2.5], [0.7, 4], [1.3, -1.2], [2.1, 1.7]]
    #dataset = [['a', a], ['b', b]]
    #TODO merge dense and discrete into update AbstractOnlineInterpreter
    def update(self, dataset):
        # check ast exists
        self.exist_ast()

        # update the value of every input variable
        self.set_variable_to_ast_from_dataset(dataset)

        # evaluate spec forest
        rob = self.updateVisitor.visitAst(self.ast, self.online_operator_dict, self.ast.var_object_dict)
        rob = rob[len(rob) - 1]

        out = self.ast.var_object_dict[self.ast.out_var]
        if self.ast.out_var_field:
            setattr(out, self.ast.out_var_field, rob)

        self.ast.var_object_dict = self.ast.var_object_dict.fromkeys(self.ast.var_object_dict, [])  #TODO I did not understand it.

        return rob

    def update_final(self, dataset):
        # check ast exists
        self.exist_ast()

        # update the value of every input variable
        self.set_variable_to_ast_from_dataset(dataset)

        # evaluate spec forest
        rob = self.updateFinalVisitor.visitAst(self.ast, self.online_operator_dict, self.ast.var_object_dict)[0]

        self.ast.var_object_dict = self.ast.var_object_dict.fromkeys(self.ast.var_object_dict, [])  #TODO I did not understand it.

        return rob


class DenseTimeOnlineUpdateVisitor(AbstractOnlineUpdateVisitor):
    def visitVariable(self, node, online_operator_dict, var_object_dict):
        vals = var_object_dict[node.var]
        if node.field:  #TODO Tom did not understand this line.
            sample_return = []
            for val in vals:
                sample_return.append([val[0], operator.attrgetter(node.field)(val[1])])
        else:
            sample_return = vals
        return sample_return

    def visitConstant(self, node, online_operator_dict, var_object_dict):
        sample_return = [[0, node.val], [float("inf"), node.val]]
        return sample_return


class DenseTimeOnlineUpdateFinalVisitor(AbstractAstVisitor):
    def visitSpec(self, node, online_operator_dict, var_object_dict):
        sample_return = self.visit(node, online_operator_dict, var_object_dict)
        var_object_dict[node] = sample_return  #TODO subspec name is necessary as a key for var_object_dict.
        return sample_return

    def visitBinary(self, node, online_operator_dict, var_object_dict):
        sample_left  = self.visit(node.children[0], online_operator_dict, var_object_dict)
        sample_right = self.visit(node.children[1], online_operator_dict, var_object_dict)
        operator = online_operator_dict[node.name]
        sample_return = operator.update_final(sample_left, sample_right)
        return sample_return

    def visitUnary(self, node, online_operator_dict, var_object_dict):
        sample = self.visit(node.children[0], online_operator_dict, var_object_dict)
        operator = online_operator_dict[node.name]
        sample_return = operator.update_final(sample)
        return sample_return

    def visitLeaf(self, node, online_operator_dict, var_object_dict):
        if isinstance(node, Constant):
            sample_return = self.visitConstant(node, online_operator_dict, var_object_dict)
        elif isinstance(node, Variable):
            sample_return = self.visitVariable(node, online_operator_dict, var_object_dict)
        return sample_return

    def visitVariable(self, node, online_operator_dict, var_object_dict):
        var = var_object_dict[node.var]
        if node.field:  #TODO Tom did not understand this line.
            sample_return = operator.attrgetter(node.field)(var)
        else:
            sample_return = var
        return sample_return

    def visitConstant(self, node, online_operator_dict, var_object_dict):
        sample_return = [[0, node.val], [float("inf"), node.val]]
        return sample_return


def dense_time_online_interpreter_factory(AstVisitor):
    if not issubclass(AstVisitor, AbstractAstVisitor):  # type check
        raise RTAMTException('{} is not RTAMT AST visitor'.format(AstVisitor.__name__))

    class DenseTimeOnlineInterpreter(AbstractDenseTimeOnlineInterpreter, AstVisitor):
        def __init__(self, *args, **kwargs):
            AbstractDenseTimeOnlineInterpreter.__init__(self, *args, **kwargs)
            AstVisitor.__init__(self)

    return DenseTimeOnlineInterpreter
