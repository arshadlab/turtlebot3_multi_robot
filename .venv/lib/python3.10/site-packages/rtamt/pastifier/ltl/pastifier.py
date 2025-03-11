from rtamt.syntax.ast.visitor.ltl.ast_visitor import LtlAstVisitor

from rtamt.syntax.node.ltl.predicate import Predicate
from rtamt.syntax.node.ltl.variable import Variable
from rtamt.syntax.node.ltl.neg import Neg
from rtamt.syntax.node.ltl.conjunction import Conjunction
from rtamt.syntax.node.ltl.disjunction import Disjunction
from rtamt.syntax.node.ltl.implies import Implies
from rtamt.syntax.node.ltl.iff import Iff
from rtamt.syntax.node.ltl.xor import Xor
from rtamt.syntax.node.ltl.once import Once
from rtamt.syntax.node.ltl.historically import Historically
from rtamt.syntax.node.ltl.since import Since
from rtamt.syntax.node.arithmetic.addition import Addition
from rtamt.syntax.node.arithmetic.subtraction import Subtraction
from rtamt.syntax.node.arithmetic.multiplication import Multiplication
from rtamt.syntax.node.arithmetic.division import Division
from rtamt.syntax.node.arithmetic.abs import Abs
from rtamt.syntax.node.arithmetic.sqrt import Sqrt
from rtamt.syntax.node.arithmetic.exp import Exp
from rtamt.syntax.node.arithmetic.pow import Pow
from rtamt.syntax.node.ltl.fall import Fall
from rtamt.syntax.node.ltl.rise import Rise
from rtamt.syntax.node.ltl.constant import Constant
from rtamt.syntax.node.ltl.previous import Previous

from rtamt.exception.exception import RTAMTException
from rtamt.pastifier.ltl.horizon import LtlHorizon


class LtlPastifier(LtlAstVisitor):

    def __init__(self):
        pass

    def pastify(self, ast):
        h = LtlHorizon()
        horizons = dict()
        for spec in ast.specs:
            horizon = h.visit(spec, None)
            horizons[spec] = horizon
        pastified_specs = []
        for spec in ast.specs:
            horizon = horizons[spec]
            pastified_spec = self.visit(spec, [horizon])
            pastified_specs.append(pastified_spec)
        return pastified_specs

    def visitConstant(self, node, *args, **kwargs):
        node = Constant(node.val)
        return node

    def visitPredicate(self, node, *args, **kwargs):
        child1_node = self.visit(node.children[0], *args, **kwargs)
        child2_node = self.visit(node.children[1], *args, **kwargs)
        node = Predicate(child1_node, child2_node, node.operator)
        return node

    def visitVariable(self, node, *args, **kwargs):
        horizon = args[0]
        node = Variable(node.var, node.field, node.io_type)
        for i in range(horizon):
            node = Previous(node)
        return node

    def visitAddition(self, node, *args, **kwargs):
        child1_node = self.visit(node.children[0], *args, **kwargs)
        child2_node = self.visit(node.children[1], *args, **kwargs)
        node = Addition(child1_node, child2_node)
        return node

    def visitMultiplication(self, node, *args, **kwargs):
        child1_node = self.visit(node.children[0], *args, **kwargs)
        child2_node = self.visit(node.children[1], *args, **kwargs)
        node = Multiplication(child1_node, child2_node)
        return node

    def visitSubtraction(self, node, *args, **kwargs):
        child1_node = self.visit(node.children[0], *args, **kwargs)
        child2_node = self.visit(node.children[1], *args, **kwargs)
        node = Subtraction(child1_node, child2_node)
        return node

    def visitDivision(self, node, *args, **kwargs):
        child1_node = self.visit(node.children[0], *args, **kwargs)
        child2_node = self.visit(node.children[1], *args, **kwargs)
        node = Division(child1_node, child2_node)
        return node

    def visitAbs(self, node, *args, **kwargs):
        child_node = self.visit(node.children[0], *args, **kwargs)
        node = Abs(child_node)
        return node

    def visitSqrt(self, node, *args, **kwargs):
        child_node = self.visit(node.children[0], *args, **kwargs)
        node = Sqrt(child_node)
        return node

    def visitExp(self, node, *args, **kwargs):
        child_node = self.visit(node.children[0], *args, **kwargs)
        node = Exp(child_node)
        return node

    def visitPow(self, node, *args, **kwargs):
        child1_node = self.visit(node.children[0], *args, **kwargs)
        child2_node = self.visit(node.children[1], *args, **kwargs)
        node = Pow(child1_node, child2_node)
        return node

    def visitRise(self, node, *args, **kwargs):
        child_node = self.visit(node.children[0], *args, **kwargs)
        node = Rise(child_node)
        return node

    def visitFall(self, node, *args, **kwargs):
        child_node = self.visit(node.children[0], *args, **kwargs)
        node = Fall(child_node)
        return node

    def visitNot(self, node, *args, **kwargs):
        child_node = self.visit(node.children[0], *args, **kwargs)
        node = Neg(child_node)
        return node

    def visitAnd(self, node, *args, **kwargs):
        child1_node = self.visit(node.children[0], *args, **kwargs)
        child2_node = self.visit(node.children[1], *args, **kwargs)
        node = Conjunction(child1_node, child2_node)
        return node

    def visitOr(self, node, *args, **kwargs):
        child1_node = self.visit(node.children[0], *args, **kwargs)
        child2_node = self.visit(node.children[1], *args, **kwargs)
        node = Disjunction(child1_node, child2_node)
        return node

    def visitImplies(self, node, *args, **kwargs):
        child1_node = self.visit(node.children[0], *args, **kwargs)
        child2_node = self.visit(node.children[1], *args, **kwargs)
        node = Implies(child1_node, child2_node)
        return node

    def visitIff(self, node, *args, **kwargs):
        child1_node = self.visit(node.children[0], *args, **kwargs)
        child2_node = self.visit(node.children[1], *args, **kwargs)
        node = Iff(child1_node, child2_node)
        return node

    def visitXor(self, node, *args, **kwargs):
        child1_node = self.visit(node.children[0], *args, **kwargs)
        child2_node = self.visit(node.children[1], *args, **kwargs)
        node = Xor(child1_node, child2_node)
        return node

    def visitEventually(self, node, *args, **kwargs):
        raise RTAMTException('Cannot pastify an unbounded eventually.')

    def visitAlways(self, node, *args, **kwargs):
        raise RTAMTException('Cannot pastify an unbounded always.')

    def visitUntil(self, node, *args, **kwargs):
        raise RTAMTException('Cannot pastify an unbounded until.')

    def visitOnce(self, node, *args, **kwargs):
        child_node = self.visit(node.children[0], *args, **kwargs)
        node = Once(child_node)
        return node

    def visitPrevious(self, node, *args, **kwargs):
        child_node = self.visit(node.children[0], *args, **kwargs)
        node = Previous(child_node)
        return node

    def visitNext(self, node, *args, **kwargs):
        horizon = args[0] - 1
        child_node = self.visit(node.children[0], horizon)
        return child_node

    def visitHistorically(self, node, *args, **kwargs):
        child_node = self.visit(node.children[0], *args, **kwargs)
        node = Historically(child_node)
        return node

    def visitSince(self, node, *args, **kwargs):
        child_node_1 = self.visit(node.children[0], *args, **kwargs)
        child_node_2 = self.visit(node.children[1], *args, **kwargs)
        node = Since(child_node_1, child_node_2)
        return node

    def visitDefault(self, node):
        raise RTAMTException('LTL Pastifier: encountered unexpected type of object.')