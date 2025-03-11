import unittest
import math
from rtamt.semantics.stl.discrete_time.online.constant_operation import ConstantOperation
from rtamt.semantics.stl.discrete_time.online.and_operation import AndOperation
from rtamt.semantics.stl.discrete_time.online.rise_operation import RiseOperation
from rtamt.semantics.stl.discrete_time.online.fall_operation import FallOperation
from rtamt.semantics.stl.discrete_time.online.predicate_operation import PredicateOperation
from rtamt.semantics.stl.discrete_time.online.not_operation import NotOperation
from rtamt.semantics.stl.discrete_time.online.or_operation import OrOperation
from rtamt.semantics.stl.discrete_time.online.implies_operation import ImpliesOperation
from rtamt.semantics.stl.discrete_time.online.iff_operation import IffOperation
from rtamt.semantics.stl.discrete_time.online.xor_operation import XorOperation
from rtamt.semantics.stl.discrete_time.online.always_operation import AlwaysOperation
from rtamt.semantics.stl.discrete_time.online.eventually_operation import EventuallyOperation
from rtamt.semantics.stl.discrete_time.online.historically_operation import HistoricallyOperation
from rtamt.semantics.stl.discrete_time.online.once_operation import OnceOperation
from rtamt.semantics.stl.discrete_time.online.since_operation import SinceOperation
from rtamt.semantics.stl.discrete_time.online.once_timed_operation import OnceTimedOperation
from rtamt.semantics.stl.discrete_time.online.historically_timed_operation import HistoricallyTimedOperation
from rtamt.semantics.stl.discrete_time.online.since_timed_operation import SinceTimedOperation
from rtamt.semantics.stl.discrete_time.online.precedes_timed_operation import PrecedesTimedOperation
from rtamt.semantics.arithmetic.discrete_time.online.subtraction_operation import SubtractionOperation
from rtamt.semantics.arithmetic.discrete_time.online.addition_operation import AdditionOperation
from rtamt.semantics.arithmetic.discrete_time.online.multiplication_operation import MultiplicationOperation
from rtamt.semantics.arithmetic.discrete_time.online.division_operation import DivisionOperation
from rtamt.semantics.arithmetic.discrete_time.online.abs_operation import AbsOperation
from rtamt.semantics.arithmetic.discrete_time.online.sqrt_operation import SqrtOperation
from rtamt.semantics.arithmetic.discrete_time.online.exp_operation import ExpOperation
from rtamt.semantics.arithmetic.discrete_time.online.pow_operation import PowOperation
from rtamt.semantics.stl.discrete_time.online.previous_operation import PreviousOperation
from rtamt.semantics.enumerations.comp_op import StlComparisonOperator

class TestSTLEvaluation(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(TestSTLEvaluation, self).__init__(*args, **kwargs)
        self.left1 = 100
        self.right1 = 20

        self.left2 = -1
        self.right2 = -2

        self.left3 = -2
        self.right3 = 10

        self.left4 = 5
        self.right4 = 4

        self.left5 = -1
        self.right5 = -1

    def test_constant(self):
        oper = ConstantOperation(5)

        out1 = oper.update()
        out2 = oper.update()

        self.assertEqual(out1, 5, "input 1")
        self.assertEqual(out2, 5, "input 2")


    def test_addition(self):
        oper = AdditionOperation()

        out1 = oper.update(self.left1, self.right1)
        out2 = oper.update(self.left2, self.right2)
        out3 = oper.update(self.left3, self.right3)
        out4 = oper.update(self.left4, self.right4)
        out5 = oper.update(self.left5, self.right5)

        self.assertEqual(out1, 120, "input 1")
        self.assertEqual(out2, -3, "input 2")
        self.assertEqual(out3, 8, "input 3")
        self.assertEqual(out4, 9, "input 4")
        self.assertEqual(out5, -2, "input 5")

    def test_subtraction(self):
        oper = SubtractionOperation()

        out1 = oper.update(self.left1, self.right1)
        out2 = oper.update(self.left2, self.right2)
        out3 = oper.update(self.left3, self.right3)
        out4 = oper.update(self.left4, self.right4)
        out5 = oper.update(self.left5, self.right5)

        self.assertEqual(out1, 80, "input 1")
        self.assertEqual(out2, 1, "input 2")
        self.assertEqual(out3, -12, "input 3")
        self.assertEqual(out4, 1, "input 4")
        self.assertEqual(out5, 0, "input 5")

    def test_multiplication(self):
        oper = MultiplicationOperation()

        out1 = oper.update(self.left1, self.right1)
        out2 = oper.update(self.left2, self.right2)
        out3 = oper.update(self.left3, self.right3)
        out4 = oper.update(self.left4, self.right4)
        out5 = oper.update(self.left5, self.right5)

        self.assertEqual(out1, 2000, "input 1")
        self.assertEqual(out2, 2, "input 2")
        self.assertEqual(out3, -20, "input 3")
        self.assertEqual(out4, 20, "input 4")
        self.assertEqual(out5, 1, "input 5")

    def test_division(self):
        oper = DivisionOperation()

        out1 = oper.update(self.left1, self.right1)
        out2 = oper.update(self.left2, self.right2)
        out3 = oper.update(self.left3, self.right3)
        out4 = oper.update(self.left4, self.right4)
        out5 = oper.update(self.left5, self.right5)

        self.assertEqual(out1, 100 / 20, "input 1")
        self.assertEqual(out2, -1 / -2, "input 2")
        self.assertEqual(out3, -2 / 10, "input 3")
        self.assertEqual(out4, 5 / 4, "input 4")
        self.assertEqual(out5, -1 / -1, "input 5")

    def test_abs(self):
        oper = AbsOperation()

        out1 = oper.update(self.left1)
        out2 = oper.update(self.left2)
        out3 = oper.update(self.left3)
        out4 = oper.update(self.left4)
        out5 = oper.update(self.left5)

        self.assertEqual(out1, 100, "input 1")
        self.assertEqual(out2, 1, "input 2")
        self.assertEqual(out3, 2, "input 3")
        self.assertEqual(out4, 5, "input 4")
        self.assertEqual(out5, 1, "input 5")

    def test_sqrt_without_negative(self):
        oper = SqrtOperation()

        out1 = oper.update(4)
        out2 = oper.update(17.3)
        out3 = oper.update(6)
        out4 = oper.update(2)
        out5 = oper.update(0.0001)

        self.assertEqual(out1, math.sqrt(4), "input 1")
        self.assertEqual(out2, math.sqrt(17.3), "input 2")
        self.assertEqual(out3, math.sqrt(6), "input 3")
        self.assertEqual(out4, math.sqrt(2), "input 4")
        self.assertEqual(out5, math.sqrt(0.0001), "input 5")

    def test_sqrt_with_negative(self):
        oper = SqrtOperation()

        self.assertRaises(Exception, oper.update, -4)

    def test_exp(self):
        oper = ExpOperation()

        out1 = oper.update(4)
        out2 = oper.update(17.3)
        out3 = oper.update(6)
        out4 = oper.update(2)
        out5 = oper.update(0.0001)

        self.assertEqual(out1, math.exp(4), "input 1")
        self.assertEqual(out2, math.exp(17.3), "input 2")
        self.assertEqual(out3, math.exp(6), "input 3")
        self.assertEqual(out4, math.exp(2), "input 4")
        self.assertEqual(out5, math.exp(0.0001), "input 5")

    def test_pow(self):
        oper = PowOperation()

        out1 = oper.update(4, 1)
        out2 = oper.update(17.3, 0.3)
        out3 = oper.update(6, 1.2)
        out4 = oper.update(2, 2)
        out5 = oper.update(0.0001, 3)

        self.assertEqual(out1, math.pow(4, 1), "input 1")
        self.assertEqual(out2, math.pow(17.3, 0.3), "input 2")
        self.assertEqual(out3, math.pow(6, 1.2), "input 3")
        self.assertEqual(out4, math.pow(2, 2), "input 4")
        self.assertEqual(out5, math.pow(0.0001, 3), "input 5")

    def test_previous(self):
        oper = PreviousOperation()

        out1 = oper.update(self.left1)
        out2 = oper.update(self.left2)
        out3 = oper.update(self.left3)
        out4 = oper.update(self.left4)
        out5 = oper.update(self.left5)

        self.assertEqual(out1, float("inf"), "input 1")
        self.assertEqual(out2, 100, "input 2")
        self.assertEqual(out3, -1, "input 3")
        self.assertEqual(out4, -2, "input 4")
        self.assertEqual(out5, 5, "input 5")

    def test_and(self):
        oper = AndOperation()

        out1 = oper.update(self.left1, self.right1)
        out2 = oper.update(self.left2, self.right2)
        out3 = oper.update(self.left3, self.right3)
        out4 = oper.update(self.left4, self.right4)
        out5 = oper.update(self.left5, self.right5)

        self.assertEqual(out1, 20, "input 1")
        self.assertEqual(out2, -2, "input 2")
        self.assertEqual(out3, -2, "input 3")
        self.assertEqual(out4, 4, "input 4")
        self.assertEqual(out5, -1, "input 5")

    def test_or(self):
        oper = OrOperation()

        out1 = oper.update(self.left1, self.right1)
        out2 = oper.update(self.left2, self.right2)
        out3 = oper.update(self.left3, self.right3)
        out4 = oper.update(self.left4, self.right4)
        out5 = oper.update(self.left5, self.right5)

        self.assertEqual(out1, 100, "input 1")
        self.assertEqual(out2, -1, "input 2")
        self.assertEqual(out3, 10, "input 3")
        self.assertEqual(out4, 5, "input 4")
        self.assertEqual(out5, -1, "input 5")

    def test_iff(self):
        oper = IffOperation()

        out1 = oper.update(self.left1, self.right1)
        out2 = oper.update(self.left2, self.right2)
        out3 = oper.update(self.left3, self.right3)
        out4 = oper.update(self.left4, self.right4)
        out5 = oper.update(self.left5, self.right5)

        self.assertEqual(out1, -80, "input 1")
        self.assertEqual(out2, -1, "input 2")
        self.assertEqual(out3, -12, "input 3")
        self.assertEqual(out4, -1, "input 4")
        self.assertEqual(out5, 0, "input 5")

    def test_xor(self):
        oper = XorOperation()

        out1 = oper.update(self.left1, self.right1)
        out2 = oper.update(self.left2, self.right2)
        out3 = oper.update(self.left3, self.right3)
        out4 = oper.update(self.left4, self.right4)
        out5 = oper.update(self.left5, self.right5)

        self.assertEqual(out1, 80, "input 1")
        self.assertEqual(out2, 1, "input 2")
        self.assertEqual(out3, 12, "input 3")
        self.assertEqual(out4, 1, "input 4")
        self.assertEqual(out5, 0, "input 5")

    def test_implies(self):
        oper = ImpliesOperation()

        out1 = oper.update(self.left1, self.right1)
        out2 = oper.update(self.left2, self.right2)
        out3 = oper.update(self.left3, self.right3)
        out4 = oper.update(self.left4, self.right4)
        out5 = oper.update(self.left5, self.right5)

        self.assertEqual(out1, 20, "input 1")
        self.assertEqual(out2, 1, "input 2")
        self.assertEqual(out3, 10, "input 3")
        self.assertEqual(out4, 4, "input 4")
        self.assertEqual(out5, 1, "input 5")

    def test_always(self):
        oper = AlwaysOperation()

        out1 = oper.update(self.left1)
        out2 = oper.update(self.left2)
        out3 = oper.update(self.left3)
        out4 = oper.update(self.left4)
        out5 = oper.update(self.left5)

        self.assertEqual(out1, 100, "input 1")
        self.assertEqual(out2, -1, "input 2")
        self.assertEqual(out3, -2, "input 3")
        self.assertEqual(out4, -2, "input 4")
        self.assertEqual(out5, -2, "input 5")

    def test_historically(self):
        oper = HistoricallyOperation()

        out1 = oper.update(self.left1)
        out2 = oper.update(self.left2)
        out3 = oper.update(self.left3)
        out4 = oper.update(self.left4)
        out5 = oper.update(self.left5)

        self.assertEqual(out1, 100, "input 1")
        self.assertEqual(out2, -1, "input 2")
        self.assertEqual(out3, -2, "input 3")
        self.assertEqual(out4, -2, "input 4")
        self.assertEqual(out5, -2, "input 5")

    def test_once(self):
        oper = OnceOperation()

        out1 = oper.update(self.left1)
        out2 = oper.update(self.left2)
        out3 = oper.update(self.left3)
        out4 = oper.update(self.left4)
        out5 = oper.update(self.left5)

        self.assertEqual(out1, 100, "input 1")
        self.assertEqual(out2, 100, "input 2")
        self.assertEqual(out3, 100, "input 3")
        self.assertEqual(out4, 100, "input 4")
        self.assertEqual(out5, 100, "input 5")

    def test_eventually(self):
        oper = EventuallyOperation()

        out1 = oper.update(self.left1)
        out2 = oper.update(self.left2)
        out3 = oper.update(self.left3)
        out4 = oper.update(self.left4)
        out5 = oper.update(self.left5)

        self.assertEqual(out1, 100, "input 1")
        self.assertEqual(out2, 100, "input 2")
        self.assertEqual(out3, 100, "input 3")
        self.assertEqual(out4, 100, "input 4")
        self.assertEqual(out5, 100, "input 5")

    def test_since(self):
        oper = SinceOperation()

        out1 = oper.update(self.left1, self.right1)
        out2 = oper.update(self.left2, self.right2)
        out3 = oper.update(self.left3, self.right3)
        out4 = oper.update(self.left4, self.right4)
        out5 = oper.update(self.left5, self.right5)

        self.assertEqual(out1, 20, "input 1")
        self.assertEqual(out2, -1, "input 2")
        self.assertEqual(out3, 10, "input 3")
        self.assertEqual(out4, 5, "input 4")
        self.assertEqual(out5, -1, "input 5")

    def test_once_0_1(self):
        oper = OnceTimedOperation(0,1)

        out1 = oper.update(self.left1)
        out2 = oper.update(self.left2)
        out3 = oper.update(self.left3)
        out4 = oper.update(self.left4)
        out5 = oper.update(self.left5)

        self.assertEqual(out1, 100, "input 1")
        self.assertEqual(out2, 100, "input 2")
        self.assertEqual(out3, -1, "input 3")
        self.assertEqual(out4, 5, "input 4")
        self.assertEqual(out5, 5, "input 5")

    def test_once_1_2(self):
        oper = OnceTimedOperation(1,2)

        out1 = oper.update(self.left1)
        out2 = oper.update(self.left2)
        out3 = oper.update(self.left3)
        out4 = oper.update(self.left4)
        out5 = oper.update(self.left5)

        self.assertEqual(out1, -float("inf"), "input 1")
        self.assertEqual(out2, 100, "input 2")
        self.assertEqual(out3, 100, "input 3")
        self.assertEqual(out4, -1, "input 4")
        self.assertEqual(out5, 5, "input 5")

    def test_historically_0_1(self):
        oper = HistoricallyTimedOperation(0,1)

        out1 = oper.update(self.left1)
        out2 = oper.update(self.left2)
        out3 = oper.update(self.left3)
        out4 = oper.update(self.left4)
        out5 = oper.update(self.left5)

        self.assertEqual(out1, 100, "input 1")
        self.assertEqual(out2, -1, "input 2")
        self.assertEqual(out3, -2, "input 3")
        self.assertEqual(out4, -2, "input 4")
        self.assertEqual(out5, -1, "input 5")

    def test_historically_1_2(self):
        oper = HistoricallyTimedOperation(1,2)

        out1 = oper.update(self.left1)
        out2 = oper.update(self.left2)
        out3 = oper.update(self.left3)
        out4 = oper.update(self.left4)
        out5 = oper.update(self.left5)

        self.assertEqual(out1, float("inf"), "input 1")
        self.assertEqual(out2, 100, "input 2")
        self.assertEqual(out3, -1, "input 3")
        self.assertEqual(out4, -2, "input 4")
        self.assertEqual(out5, -2, "input 5")

    def test_since_0_1(self):
        oper = SinceTimedOperation(0,1)

        out1 = oper.update(self.left1, self.right1)
        out2 = oper.update(self.left2, self.right2)
        out3 = oper.update(self.left3, self.right3)
        out4 = oper.update(self.left4, self.right4)
        out5 = oper.update(self.left5, self.right5)

        self.assertEqual(out1, 20, "input 1")
        self.assertEqual(out2, -1, "input 2")
        self.assertEqual(out3, 10, "input 3")
        self.assertEqual(out4, 5, "input 4")
        self.assertEqual(out5, -1, "input 5")

    def test_precedes_1_2(self):
        oper = PrecedesTimedOperation(1, 2)

        out1 = oper.update(self.left1, self.right1)
        out2 = oper.update(self.left2, self.right2)
        out3 = oper.update(self.left3, self.right3)
        out4 = oper.update(self.left4, self.right4)
        out5 = oper.update(self.left5, self.right5)

        self.assertEqual(out1, 20, "input 1")
        self.assertEqual(out2, 20, "input 2")
        self.assertEqual(out3, -1, "input 3")
        self.assertEqual(out4, -1, "input 4")
        self.assertEqual(out5, -2, "input 5")

    def test_not(self):
        oper = NotOperation()

        out1 = oper.update(self.left1)
        out2 = oper.update(self.left2)
        out3 = oper.update(self.left3)
        out4 = oper.update(self.left4)
        out5 = oper.update(self.left5)

        self.assertEqual(out1, -100, "input 1")
        self.assertEqual(out2, 1, "input 2")
        self.assertEqual(out3, 2, "input 3")
        self.assertEqual(out4, -5, "input 4")
        self.assertEqual(out5, 1, "input 5")

    def test_rise(self):
        oper = RiseOperation()

        out1 = oper.update(self.left1)
        out2 = oper.update(self.left2)
        out3 = oper.update(self.left3)
        out4 = oper.update(self.left4)
        out5 = oper.update(self.left5)

        self.assertEqual(out1, 100, "input 1")
        self.assertEqual(out2, -100, "input 2")
        self.assertEqual(out3, -2, "input 3")
        self.assertEqual(out4, 2, "input 4")
        self.assertEqual(out5, -5, "input 5")

    def test_fall(self):
        oper = FallOperation()

        out1 = oper.update(self.left1)
        out2 = oper.update(self.left2)
        out3 = oper.update(self.left3)
        out4 = oper.update(self.left4)
        out5 = oper.update(self.left5)

        self.assertEqual(out1, -100, "input 1")
        self.assertEqual(out2, 1, "input 2")
        self.assertEqual(out3, -1, "input 3")
        self.assertEqual(out4, -5, "input 4")
        self.assertEqual(out5, 1, "input 5")


    def test_predicate_leq(self):
        oper = PredicateOperation(StlComparisonOperator.LEQ)

        out1 = oper.update(self.left1, self.right1)
        out2 = oper.update(self.left2, self.right2)
        out3 = oper.update(self.left3, self.right3)
        out4 = oper.update(self.left4, self.right4)
        out5 = oper.update(self.left5, self.right5)

        self.assertEqual(out1, -80, "input 1")
        self.assertEqual(out2, -1, "input 2")
        self.assertEqual(out3, 12, "input 3")
        self.assertEqual(out4, -1, "input 4")
        self.assertEqual(out5, 0, "input 5")

    def test_predicate_less(self):
        oper = PredicateOperation(StlComparisonOperator.LESS)

        out1 = oper.update(self.left1, self.right1)
        out2 = oper.update(self.left2, self.right2)
        out3 = oper.update(self.left3, self.right3)
        out4 = oper.update(self.left4, self.right4)
        out5 = oper.update(self.left5, self.right5)

        self.assertEqual(out1, -80, "input 1")
        self.assertEqual(out2, -1, "input 2")
        self.assertEqual(out3, 12, "input 3")
        self.assertEqual(out4, -1, "input 4")
        self.assertEqual(out5, 0, "input 5")

    def test_predicate_geq(self):
        oper = PredicateOperation(StlComparisonOperator.GEQ)

        out1 = oper.update(self.left1, self.right1)
        out2 = oper.update(self.left2, self.right2)
        out3 = oper.update(self.left3, self.right3)
        out4 = oper.update(self.left4, self.right4)
        out5 = oper.update(self.left5, self.right5)

        self.assertEqual(out1, 80, "input 1")
        self.assertEqual(out2, 1, "input 2")
        self.assertEqual(out3, -12, "input 3")
        self.assertEqual(out4, 1, "input 4")
        self.assertEqual(out5, 0, "input 5")

    def test_predicate_greater(self):
        oper = PredicateOperation(StlComparisonOperator.GREATER)

        out1 = oper.update(self.left1, self.right1)
        out2 = oper.update(self.left2, self.right2)
        out3 = oper.update(self.left3, self.right3)
        out4 = oper.update(self.left4, self.right4)
        out5 = oper.update(self.left5, self.right5)

        self.assertEqual(out1, 80, "input 1")
        self.assertEqual(out2, 1, "input 2")
        self.assertEqual(out3, -12, "input 3")
        self.assertEqual(out4, 1, "input 4")
        self.assertEqual(out5, 0, "input 5")

    def test_predicate_eq(self):
        oper = PredicateOperation(StlComparisonOperator.EQUAL)

        out1 = oper.update(self.left1, self.right1)
        out2 = oper.update(self.left2, self.right2)
        out3 = oper.update(self.left3, self.right3)
        out4 = oper.update(self.left4, self.right4)
        out5 = oper.update(self.left5, self.right5)

        self.assertEqual(out1, -80, "input 1")
        self.assertEqual(out2, -1, "input 2")
        self.assertEqual(out3, -12, "input 3")
        self.assertEqual(out4, -1, "input 4")
        self.assertEqual(out5, 0, "input 5")

    def test_predicate_neq(self):
        oper = PredicateOperation(StlComparisonOperator.NEQ)

        out1 = oper.update(self.left1, self.right1)
        out2 = oper.update(self.left2, self.right2)
        out3 = oper.update(self.left3, self.right3)
        out4 = oper.update(self.left4, self.right4)
        out5 = oper.update(self.left5, self.right5)

        self.assertEqual(out1, 80, "input 1")
        self.assertEqual(out2, 1, "input 2")
        self.assertEqual(out3, 12, "input 3")
        self.assertEqual(out4, 1, "input 4")
        self.assertEqual(out5, 0, "input 5")

if __name__ == '__main__':
    unittest.main()