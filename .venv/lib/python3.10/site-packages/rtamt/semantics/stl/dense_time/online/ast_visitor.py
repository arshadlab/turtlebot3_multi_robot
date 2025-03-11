
from rtamt.syntax.ast.visitor.stl.ast_visitor import StlAstVisitor

from rtamt.semantics.arithmetic.dense_time.online.addition_operation import AdditionOperation
from rtamt.semantics.arithmetic.dense_time.online.multiplication_operation import MultiplicationOperation
from rtamt.semantics.arithmetic.dense_time.online.subtraction_operation import SubtractionOperation
from rtamt.semantics.arithmetic.dense_time.online.division_operation import DivisionOperation
from rtamt.semantics.arithmetic.dense_time.online.abs_operation import AbsOperation
from rtamt.semantics.arithmetic.dense_time.online.sqrt_operation import SqrtOperation
from rtamt.semantics.arithmetic.dense_time.online.pow_operation import PowOperation
from rtamt.semantics.arithmetic.dense_time.online.exp_operation import ExpOperation

from rtamt.semantics.stl.dense_time.online.predicate_operation import PredicateOperation
from rtamt.semantics.stl.dense_time.online.and_operation import AndOperation
from rtamt.semantics.stl.dense_time.online.or_operation import OrOperation
from rtamt.semantics.stl.dense_time.online.implies_operation import ImpliesOperation
from rtamt.semantics.stl.dense_time.online.iff_operation import IffOperation
from rtamt.semantics.stl.dense_time.online.xor_operation import XorOperation
from rtamt.semantics.stl.dense_time.online.since_operation import SinceOperation
from rtamt.semantics.stl.dense_time.online.not_operation import NotOperation
from rtamt.semantics.stl.dense_time.online.once_operation import OnceOperation
from rtamt.semantics.stl.dense_time.online.historically_operation import HistoricallyOperation
from rtamt.semantics.stl.dense_time.online.once_timed_operation import OnceTimedOperation
from rtamt.semantics.stl.dense_time.online.historically_timed_operation import HistoricallyTimedOperation
from rtamt.semantics.stl.dense_time.online.since_timed_operation import SinceTimedOperation

from rtamt.exception.exception import RTAMTException

class StlDenseTimeOnlineAstVisitor(StlAstVisitor):
    def visitPredicate(self, node, *args, **kwargs):
        self.visitChildren(node, *args, **kwargs)
        self.online_operator_dict[node.name] = PredicateOperation(node.operator)

    def visitAbs(self, node, *args, **kwargs):
        self.visitChildren(node, *args, **kwargs)
        self.online_operator_dict[node.name] = AbsOperation()

    def visitSqrt(self, node, *args, **kwargs):
        self.visitChildren(node, *args, **kwargs)
        self.online_operator_dict[node.name] = SqrtOperation()

    def visitExp(self, node, *args, **kwargs):
        self.visitChildren(node, *args, **kwargs)
        self.online_operator_dict[node.name] = ExpOperation()

    def visitPow(self, node, *args, **kwargs):
        self.visitChildren(node, *args, **kwargs)
        self.online_operator_dict[node.name] = PowOperation()

    def visitAddition(self, node, *args, **kwargs):
        self.visitChildren(node, *args, **kwargs)
        self.online_operator_dict[node.name] = AdditionOperation()

    def visitSubtraction(self, node, *args, **kwargs):
        self.visitChildren(node, *args, **kwargs)
        self.online_operator_dict[node.name] = SubtractionOperation()

    def visitMultiplication(self, node, *args, **kwargs):
        self.visitChildren(node, *args, **kwargs)
        self.online_operator_dict[node.name] = MultiplicationOperation()

    def visitDivision(self, node, *args, **kwargs):
        self.visitChildren(node, *args, **kwargs)
        self.online_operator_dict[node.name] = DivisionOperation()

    def visitNot(self, node, *args, **kwargs):
        self.visitChildren(node, *args, **kwargs)
        self.online_operator_dict[node.name] = NotOperation()

    def visitAnd(self, node, *args, **kwargs):
        self.visitChildren(node, *args, **kwargs)
        self.online_operator_dict[node.name] = AndOperation()

    def visitOr(self, node, *args, **kwargs):
        self.visitChildren(node, *args, **kwargs)
        self.online_operator_dict[node.name] = OrOperation()

    def visitImplies(self, node, *args, **kwargs):
        self.visitChildren(node, *args, **kwargs)
        self.online_operator_dict[node.name] = ImpliesOperation()

    def visitIff(self, node, *args, **kwargs):
        self.visitChildren(node, *args, **kwargs)
        self.online_operator_dict[node.name] = IffOperation()

    def visitXor(self, node, *args, **kwargs):
        self.visitChildren(node, *args, **kwargs)
        self.online_operator_dict[node.name] = XorOperation()

    def visitEventually(self, node, *args, **kwargs):
        raise RTAMTException('Eventually operator is not implemented in the STL online monitor.')

    def visitAlways(self, node, *args, **kwargs):
        raise RTAMTException('Always operator is not implemented in the STL online monitor.')

    def visitUntil(self, node, *args, **kwargs):
        raise RTAMTException('Until operator is not implemented in the STL online monitor.')

    def visitOnce(self, node, *args, **kwargs):
        self.visitChildren(node, *args, **kwargs)
        self.online_operator_dict[node.name] = OnceOperation()

    def visitHistorically(self, node, *args, **kwargs):
        self.visitChildren(node, *args, **kwargs)
        self.online_operator_dict[node.name] = HistoricallyOperation()

    def visitSince(self, node, *args, **kwargs):
        self.visitChildren(node, *args, **kwargs)
        self.online_operator_dict[node.name] = SinceOperation()

    def visitRise(self, node, *args, **kwargs):
        raise RTAMTException('Rise operator not implemented in STL dense monitor.')

    def visitFall(self, node, *args, **kwargs):
        raise RTAMTException('Fall operator not implemented in STL dense monitor.')

    def visitPrevious(self, node, *args, **kwargs):
        raise RTAMTException('Previous operator not implemented in STL dense-time monitor.')

    def visitNext(self, node, *args, **kwargs):
        raise RTAMTException('Next operator not implemented in STL dense-time monitor.')

    def visitTimedPrecedes(self, node, *args, **kwargs):
        raise RTAMTException('Precedes operator not implemented in STL dense-time monitor.')

    def visitTimedOnce(self, node, *args, **kwargs):
        self.visitChildren(node, *args, **kwargs)
        begin, end = self.time_unit_transformer(node)
        self.online_operator_dict[node.name] = OnceTimedOperation(begin, end)

    def visitTimedHistorically(self, node, *args, **kwargs):
        self.visitChildren(node, *args, **kwargs)
        begin, end = self.time_unit_transformer(node)
        self.online_operator_dict[node.name] = HistoricallyTimedOperation(begin, end)

    def visitTimedSince(self, node, *args, **kwargs):
        self.visitChildren(node, *args, **kwargs)
        begin, end = self.time_unit_transformer(node)
        self.online_operator_dict[node.name] = SinceTimedOperation(begin, end)

    def visitTimedAlways(self, node, *args, **kwargs):
        raise RTAMTException('Bounded always operator not implemented in STL dense-time monitor.')

    def visitTimedEventually(self, node, *args, **kwargs):
        raise RTAMTException('Bounded eventually operator not implemented in STL dense-time monitor.')

    def visitTimedUntil(self, node, *args, **kwargs):
        raise RTAMTException('Bounded until operator not implemented in STL dense-time monitor.')
