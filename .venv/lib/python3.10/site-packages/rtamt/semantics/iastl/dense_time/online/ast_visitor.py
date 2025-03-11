from rtamt.semantics.stl.dense_time.online.ast_visitor import StlDenseTimeOnlineAstVisitor
from rtamt.semantics.iastl.dense_time.online.predicate_operation import PredicateOperation
from rtamt.semantics.enumerations.options import Semantics

class IAStlOutputRobustnessDenseTimeOnlineAstVisitor(StlDenseTimeOnlineAstVisitor):

    def visitPredicate(self, node, *args, **kwargs):
        self.visitChildren(node, *args, **kwargs)
        self.online_operator_dict[node.name] = PredicateOperation(node.operator, Semantics.OUTPUT_ROBUSTNESS,
                                                                  node.in_vars, node.out_vars)

class IAStlInputVacuityDenseTimeOnlineAstVisitor(StlDenseTimeOnlineAstVisitor):

    def visitPredicate(self, node, *args, **kwargs):
        self.visitChildren(node, *args, **kwargs)
        self.online_operator_dict[node.name] = PredicateOperation(node.operator, Semantics.INPUT_VACUITY,
                                                                  node.in_vars, node.out_vars)


class IAStlOutputVacuityDenseTimeOnlineAstVisitor(StlDenseTimeOnlineAstVisitor):

    def visitPredicate(self, node, *args, **kwargs):
        self.visitChildren(node, *args, **kwargs)

        self.online_operator_dict[node.name] = PredicateOperation(node.operator, Semantics.OUTPUT_VACUITY,
                                                                  node.in_vars, node.out_vars)


class IAStlInputRobustnessDenseTimeOnlineAstVisitor(StlDenseTimeOnlineAstVisitor):

    def visitPredicate(self, node, *args, **kwargs):
        self.visitChildren(node, *args, **kwargs)
        self.online_operator_dict[node.name] = PredicateOperation(node.operator, Semantics.INPUT_ROBUSTNESS,
                                                                  node.in_vars, node.out_vars)