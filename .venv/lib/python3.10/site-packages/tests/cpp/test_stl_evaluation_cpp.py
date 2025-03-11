import unittest
from rtamt.lib.rtamt_stl_library_wrapper.stl_constant_node import ConstantOperation
from rtamt.lib.rtamt_stl_library_wrapper.stl_comp_op import StlComparisonOperator
from rtamt.lib.rtamt_stl_library_wrapper.stl_addition_node import AdditionOperation
from rtamt.lib.rtamt_stl_library_wrapper.stl_combinatorial_binary_node import CombinatorialBinaryOperation
from rtamt.lib.rtamt_stl_library_wrapper.stl_and_node import AndOperation
from rtamt.lib.rtamt_stl_library_wrapper.stl_rise_node import RiseOperation
from rtamt.lib.rtamt_stl_library_wrapper.stl_fall_node import FallOperation
from rtamt.lib.rtamt_stl_library_wrapper.stl_predicate_node import PredicateOperation
from rtamt.lib.rtamt_stl_library_wrapper.stl_not_node import NotOperation
from rtamt.lib.rtamt_stl_library_wrapper.stl_or_node import OrOperation
from rtamt.lib.rtamt_stl_library_wrapper.stl_implies_node import ImpliesOperation
from rtamt.lib.rtamt_stl_library_wrapper.stl_iff_node import IffOperation
from rtamt.lib.rtamt_stl_library_wrapper.stl_xor_node import XorOperation
from rtamt.lib.rtamt_stl_library_wrapper.stl_always_node import AlwaysOperation
from rtamt.lib.rtamt_stl_library_wrapper.stl_historically_node import HistoricallyOperation
from rtamt.lib.rtamt_stl_library_wrapper.stl_once_node import OnceOperation
from rtamt.lib.rtamt_stl_library_wrapper.stl_since_node import SinceOperation
from rtamt.lib.rtamt_stl_library_wrapper.stl_once_bounded_node import OnceBoundedOperation
from rtamt.lib.rtamt_stl_library_wrapper.stl_historically_bounded_node import HistoricallyBoundedOperation
from rtamt.lib.rtamt_stl_library_wrapper.stl_since_bounded_node import SinceBoundedOperation
from rtamt.lib.rtamt_stl_library_wrapper.stl_precedes_bounded_node import PrecedesBoundedOperation
from rtamt.lib.rtamt_stl_library_wrapper.stl_subtraction_node import SubtractionOperation
from rtamt.lib.rtamt_stl_library_wrapper.stl_multiplication_node import MultiplicationOperation
from rtamt.lib.rtamt_stl_library_wrapper.stl_division_node import DivisionOperation
from rtamt.lib.rtamt_stl_library_wrapper.stl_abs_node import AbsOperation

class TestSTLEvaluationCPP(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(TestSTLEvaluationCPP, self).__init__(*args, **kwargs)
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

        self.assertEqual(out1, 100.0 / 20.0, "input 1")
        self.assertEqual(out2, -1.0 / -2.0, "input 2")
        self.assertEqual(out3, -2.0 / 10.0, "input 3")
        self.assertEqual(out4, 5.0 / 4.0, "input 4")
        self.assertEqual(out5, -1.0 / -1.0, "input 5")

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
        oper = OnceBoundedOperation(0,1)

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
        oper = OnceBoundedOperation(1,2)

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
        oper = HistoricallyBoundedOperation(0,1)

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
        oper = HistoricallyBoundedOperation(1,2)

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
        oper = SinceBoundedOperation(0,1)

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
        oper = PrecedesBoundedOperation(1, 2)

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