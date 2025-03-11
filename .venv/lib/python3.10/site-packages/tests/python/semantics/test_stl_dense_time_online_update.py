import unittest

from rtamt.syntax.ast.parser.stl.specification_parser import StlAst
from rtamt.semantics.stl.dense_time.online.and_operation import AndOperation
from rtamt.semantics.stl.dense_time.online.interpreter import StlDenseTimeOnlineInterpreter
from rtamt.semantics.stl.dense_time.online.not_operation import NotOperation
from rtamt.semantics.stl.dense_time.online.implies_operation import ImpliesOperation
from rtamt.semantics.stl.dense_time.online.iff_operation import IffOperation
from rtamt.semantics.stl.dense_time.online.xor_operation import XorOperation
from rtamt.semantics.stl.dense_time.online.always_operation import AlwaysOperation
from rtamt.semantics.stl.dense_time.online.historically_operation import HistoricallyOperation
from rtamt.semantics.stl.dense_time.online.once_operation import OnceOperation
from rtamt.semantics.stl.dense_time.online.since_operation import SinceOperation
from rtamt.semantics.stl.dense_time.online.once_timed_operation import OnceTimedOperation
from rtamt.semantics.stl.dense_time.online.historically_timed_operation import HistoricallyTimedOperation


class TestSTLBooleanAndTemporalOnline(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(TestSTLBooleanAndTemporalOnline, self).__init__(*args, **kwargs)

    def test_and(self):
         oper = AndOperation()
         in_data_1_1 = [[2, 2], [3.3, 3], [5.7, 4]]
         in_data_2_1 = [[2.5, 5], [4.7, 6]]
         out_expected_1 = [[2.5, 2], [3.3, 3]]
         out_computed_1 = oper.update(in_data_1_1, in_data_2_1)

         self.assertListEqual(out_expected_1, out_computed_1,
                              "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_1, out_computed_1))
         in_data_1_2 = []
         in_data_2_2 = [[5.7, 1]]
         out_expected_2 = [[4.7, 3]]
         out_computed_2 = oper.update(in_data_1_2, in_data_2_2)

         self.assertListEqual(out_expected_2, out_computed_2,
                              "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_2, out_computed_2))

         out_expected_final = [[5.7, 1]]
         out_computed_final = oper.update_final([], [])
         self.assertListEqual(out_expected_final, out_computed_final,
                              "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_final, out_computed_final))

         oper = AndOperation()
         in_data_1_1 = [[2, 2]]
         in_data_2_1 = [[2, 1]]

         out_expected_1 = []
         out_computed_1 = oper.update(in_data_1_1, in_data_2_1)

         self.assertListEqual(out_expected_1, out_computed_1,
                              "Problem with 2nd example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_1, out_computed_1))

         in_data_1_2 = [[3.3, 3]]
         in_data_2_2 = [[3.3, 5]]

         out_expected_2 = [[2, 1]]
         out_computed_2 = oper.update(in_data_1_2, in_data_2_2)

         self.assertListEqual(out_expected_2, out_computed_2,
                              "Problem with 2nd example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_2, out_computed_2))

         in_data_1_3 = [[4.7, 5]]
         in_data_2_3 = [[4.7, 2]]

         out_expected_3 = [[3.3, 3]]
         out_computed_3 = oper.update(in_data_1_3, in_data_2_3)

         self.assertListEqual(out_expected_3, out_computed_3,
                              "Problem with 2nd example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_3, out_computed_3))

         out_expected_final = [[4.7, 2]]
         out_computed_final = oper.update_final([], [])

         self.assertListEqual(out_expected_final, out_computed_final,
                              "Problem with 2nd example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_final, out_computed_final))

         ################################################################################################

         oper = AndOperation()
         in_data_1_1 = [[1, 2], [4.1, 1], [5, 2], [6.1, 1]]
         in_data_2_1 = [[1.2, 1]]
         out_expected_1 = []
         out_computed_1 = oper.update(in_data_1_1, in_data_2_1)

         self.assertListEqual(out_expected_1, out_computed_1,
                              "Problem with 6th example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_1, out_computed_1))

         in_data_1_2 = []
         in_data_2_2 = [[1.2, 1], [3.7, 3], [7.5, 2]]
         out_expected_2 = [[1.2, 1], [3.7, 2], [4.1, 1], [5, 2]]
         out_computed_2 = oper.update(in_data_1_2, in_data_2_2)

         self.assertListEqual(out_expected_2, out_computed_2,
                              "Problem with 6th example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_2, out_computed_2))

         in_data_1_3 = [[6.7, 4], [9.9, 5]]
         in_data_2_3 = [[8.1, 6]]
         out_expected_3 = [[6.1, 1], [6.7, 3], [7.5, 2]]
         out_computed_3 = oper.update(in_data_1_3, in_data_2_3)

         self.assertListEqual(out_expected_3, out_computed_3,
                              "Problem with 6th example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_3, out_computed_3))


         out_expected_final = [[8.1, 4]]
         out_computed_final = oper.update_final([], [])

         self.assertListEqual(out_expected_final, out_computed_final,
                              "Problem with 6th example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_final, out_computed_final))


    def test_or(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a or b'
        ast.parse()
        op = StlDenseTimeOnlineInterpreter()
        op.set_ast(ast)

        a = ['a', [[1, 2], [4.1, 1], [5, 2], [6.1, 1]]]
        b = ['b', [[1.2, 1]]]
        dataset = [a, b]
        out = op.update(dataset)
        out_expected_1 = []

        self.assertListEqual(out_expected_1, out,
                             "Problem with 6th example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected_1, out))

        a = ['a', []]
        b = ['b', [[3.7, 3], [7.5, 2]]]
        dataset = [a, b]
        out = op.update(dataset)
        out_expected_2 = [[1.2, 2], [3.7, 3]]

        self.assertListEqual(out_expected_2, out,
                             "Problem with 6th example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected_2, out))

        a = ['a', [[6.7, 4], [9.9, 5]]]
        b = ['b', [[8.1, 6]]]
        dataset = [a, b]
        out = op.update(dataset)
        out_expected_3 = [[6.1, 3], [6.7, 4]]

        self.assertListEqual(out_expected_3, out,
                             "Problem with 6th example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected_3, out))

        out_expected_final = [[8.1, 6]]
        a = ['a', []]
        b = ['b', []]
        dataset = [a, b]
        out_computed_final = op.update_final(dataset)

        self.assertListEqual(out_expected_final, out_computed_final,
                             "Problem with 6th example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected_final, out_computed_final))


    def test_iff(self):
        oper = IffOperation()
        in_data_1 = [[1, 2], [4.1, 1], [5, 2], [6.1, 1], [6.7, 4], [9.9, 5]]
        in_data_2 = [[1.2, 1], [3.7, 3], [7.5, 2], [8.1, 6]]
        out_expected = [[1.2, -1], [4.1, -2], [5, -1], [6.1, -2], [6.7, -1], [7.5, -2]]
        out_computed = oper.update(in_data_1, in_data_2)

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

        out_expected = [[8.1, -2]]
        out_computed = oper.update_final([], [])

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))
    def test_xor(self):
        oper = XorOperation()
        in_data_1 = [[1, 2], [4.1, 1], [5, 2], [6.1, 1], [6.7, 4], [9.9, 5]]
        in_data_2 = [[1.2, 1], [3.7, 3], [7.5, 2], [8.1, 6]]
        out_expected = [[1.2, 1], [4.1, 2], [5, 1], [6.1, 2], [6.7, 1], [7.5, 2]]
        out_computed = oper.update(in_data_1, in_data_2)

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

        out_expected = [[8.1, 2]]
        out_computed = oper.update_final([], [])

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

    def test_implies(self):
        oper = ImpliesOperation()
        in_data_1 = [[1, 2], [4.1, 1], [5, 2], [6.1, 1], [6.7, 4], [9.9, 5]]
        in_data_2 = [[1.2, 1], [3.7, 3], [7.5, 2], [8.1, 6]]
        out_expected = [[1.2, 1], [3.7, 3], [7.5, 2]]
        out_computed = oper.update(in_data_1, in_data_2)

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

        out_expected = [[8.1, 6]]
        out_computed = oper.update_final([], [])

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))


    def test_always(self):
        oper = AlwaysOperation()
        in_data = [[5, 3], [5.3, 1], [5.75, 2]]
        out_expected = [[5, 3], [5.3, 1], [5.75, 1]]
        out_computed = oper.update(in_data)

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

        in_data = [[6.5, 5], [6.75, 6], [9, 5], [9.25, 4], [10, 2]]
        out_expected = [[6.5, 1], [6.75, 1], [9, 1], [9.25, 1], [10, 1]]
        out_computed = oper.update(in_data)

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

    def test_historically(self):
        oper = HistoricallyOperation()
        in_data = [[5, 3], [5.3, 1], [5.75, 2]]
        out_expected = [[5, 3], [5.3, 1], [5.75, 1]]
        out_computed = oper.update(in_data)

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

        in_data = [[6.5, 5], [6.75, 6], [9, 5], [9.25, 4], [10, 2]]
        out_expected = [[6.5, 1], [6.75, 1], [9, 1], [9.25, 1], [10, 1]]
        out_computed = oper.update(in_data)

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

    def test_once(self):
        oper = OnceOperation()

        in_data = [[5, 3], [5.3, 1], [5.75, 2]]
        out_expected = [[5, 3], [5.3, 3], [5.75, 3]]
        out_computed = oper.update(in_data)

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

        in_data = [[6.5, 5], [6.75, 6]]
        out_expected = [[6.5, 5], [6.75, 6]]
        out_computed = oper.update(in_data)

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

        in_data = [[9, 5], [9.25, 4], [10, 2]]
        out_expected = [[9, 6], [9.25, 6], [10, 6]]
        out_computed = oper.update(in_data)

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

    def test_once_0_1(self):
        oper = OnceTimedOperation(0, 1)
        in_data_1 = [[5, 3], [5.3, 2], [5.75, 1]]
        out_expected_1 = [[5, 3]]
        out_computed_1 = oper.update(in_data_1)

        self.assertListEqual(out_expected_1, out_computed_1,
                                  "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_1, out_computed_1))

        in_data_2 = [[6.5, 5], [6.75, 6], [9, 5]]
        out_expected_2 = [[5.75, 3], [6.3, 2], [6.5, 5], [6.75, 6]]
        out_computed_2 = oper.update(in_data_2)

        self.assertListEqual(out_expected_2, out_computed_2,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected_2, out_computed_2))

        in_data_3 = [[9.25, 4], [10, 2]]
        out_expected_3 = [[9, 6]]
        out_computed_3 = oper.update(in_data_3)

        self.assertListEqual(out_expected_3, out_computed_3,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected_3, out_computed_3))

        out_expected_final = [[10, 5], [10.25, 4], [11, 2]]
        out_computed_final = oper.update_final([])

        self.assertListEqual(out_expected_final, out_computed_final,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected_final, out_computed_final))

    def test_once_1_3(self):
        oper = OnceTimedOperation(1, 3)
        in_data_1 = [[5, 3], [5.3, 2], [5.75, 1]]
        out_expected_1 = [[6, 3]]
        out_computed_1 = oper.update(in_data_1)

        self.assertListEqual(out_expected_1, out_computed_1,
                            "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                            out_expected_1, out_computed_1))

        in_data_2 = [[6.5, 5], [6.75, 6], [9, 5]]
        out_expected_2 = [[6.75, 3], [7.5, 5], [7.75, 6]]
        out_computed_2 = oper.update(in_data_2)

        self.assertListEqual(out_expected_2, out_computed_2,
                            "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                            out_expected_2, out_computed_2))

        in_data_3 = [[9.25, 4], [10, 2]]
        out_expected_3 = [[10, 6]]
        out_computed_3 = oper.update(in_data_3)

        self.assertListEqual(out_expected_3, out_computed_3,
                            "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                out_expected_3, out_computed_3))

        out_expected_final = [[11, 6], [12, 5], [12.25, 4], [13, 2]]
        out_computed_final = oper.update_final([])

        self.assertListEqual(out_expected_final, out_computed_final,
                            "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                out_expected_final, out_computed_final))


    def test_since(self):

        oper = SinceOperation()
        in_data_1 = [[1, 2], [4.1, 1], [5, 2], [6.1, 1], [6.7, 4], [9.9, 5]]
        in_data_2 = [[1.2, 1], [3.7, 3], [7.5, 2], [8.1, 6]]
        out_expected = [[1.2, 1], [3.7, 2], [4.1, 1], [5, 2], [6.1, 1], [6.7, 3], [7.5, 3]]
        out_computed = oper.update(in_data_1, in_data_2)

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 3d example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

        out_expected = [[8.1, 4]]
        out_computed = oper.update_final([], [])

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 3d example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

        oper = SinceOperation()
        in_data_1 = [[1, 2], [4.1, 1]]
        in_data_2 = [[1.2, 1], [3.7, 3], [7.5, 2]]
        out_expected = [[1.2, 1], [3.7, 2]]
        out_computed = oper.update(in_data_1, in_data_2)

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 3d example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

        in_data_1 = [[5, 2], [6.1, 1], [6.7, 4]]
        in_data_2 = []
        out_expected = [[4.1, 1], [5, 2], [6.1, 1]]
        out_computed = oper.update(in_data_1, in_data_2)

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 3d example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

        in_data_1 = [[9.9, 5]]
        in_data_2 = [[8.1, 6]]
        out_expected = [[6.7, 3], [7.5, 3]]
        out_computed = oper.update(in_data_1, in_data_2)

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 3d example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

        out_expected = [[8.1, 4]]
        out_computed = oper.update_final([], [])

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 3d example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

    def test_historically_1_2(self):
        oper = HistoricallyTimedOperation(1, 2)
        in_data = [[5, 3], [5.3, 1], [5.75, 2], [6.5, 5], [6.75, 6], [9, 5], [9.25, 4], [10, 2]]
        out_expected = [[6, 3], [6.3, 1], [7.75, 2], [8.5, 5], [8.75, 6], [10, 5], [10.25, 4]]
        out_computed = oper.update(in_data)

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                             out_expected, out_computed))

        out_expected = [[11, 2]]
        out_computed = oper.update_final([])

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

        oper = HistoricallyTimedOperation(1, 2)
        in_data_1 = [[5, 3], [5.3, 2], [5.75, 1]]
        out_expected_1 = [[6, 3], [6.3, 2]]
        out_computed_1 = oper.update(in_data_1)

        self.assertListEqual(out_expected_1, out_computed_1,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected_1, out_computed_1))

        in_data_2 = [[6.5, 5], [6.75, 6], [9, 5]]
        out_expected_2 = [[6.75, 1], [8.5, 5], [8.75, 6]]
        out_computed_2 = oper.update(in_data_2)

        self.assertListEqual(out_expected_2, out_computed_2,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected_2, out_computed_2))

        in_data_3 = [[9.25, 4], [10, 2]]
        out_expected_3 = [[10, 5], [10.25, 4]]
        out_computed_3 = oper.update(in_data_3)

        self.assertListEqual(out_expected_3, out_computed_3,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected_3, out_computed_3))

        out_expected_final = [[11, 2]]
        out_computed_final = oper.update_final([])

        self.assertListEqual(out_expected_final, out_computed_final,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected_final, out_computed_final))

    #     in_data = [[0, 1], [0.5, 2], [1, 3], [1.5, 4], [2, 5]]
    #     out_expected = [[1, 1], [2.5, 2], [3, 3], [3.5, 4], [4, 5]]
    #     out_computed = oper.offline(in_data)
    #
    #     self.assertListEqual(out_expected, out_computed,
    #                          "Problem with 2nd example:\nExpected output: %s\nComputed output: %s" % (
    #                          out_expected, out_computed))
    #
    #     in_data = [[0, 5], [0.5, 4], [1, 3], [1.5, 2], [2, 1]]
    #     out_expected = [[1, 5], [1.5, 4], [2, 3], [2.5, 2], [4, 1]]
    #     out_computed = oper.offline(in_data)
    #
    #     self.assertListEqual(out_expected, out_computed,
    #                          "Problem with 3rd example:\nExpected output: %s\nComputed output: %s" % (
    #                          out_expected, out_computed))
    #
    #     in_data = [[5, 3], [5.3, 1], [5.75, 2], [6.5, 5], [6.75, 1], [9, 5], [9.25, 4], [10, 2]]
    #     out_expected = [[6, 3], [6.3, 1], [11, 4], [12, 2]]
    #     out_computed = oper.offline(in_data)
    #
    #     self.assertListEqual(out_expected, out_computed,
    #                           "Problem with 4th example:\nExpected output: %s\nComputed output: %s" % (
    #                               out_expected, out_computed))
    #
    #     in_data = [[6, 2], [8, 1], [8.1, 2], [10, 3]]
    #     out_expected = [[7, 2], [9, 1], [10.1, 2], [12, 3]]
    #     out_computed = oper.offline(in_data)
    #
    #     self.assertListEqual(out_expected, out_computed,
    #                          "Problem with 5th example:\nExpected output: %s\nComputed output: %s" % (
    #                              out_expected, out_computed))
    #
    #     in_data = [[6, 2], [8, 3], [8.1, 2], [10, 3]]
    #     out_expected = [[7, 2], [12, 3]]
    #     out_computed = oper.offline(in_data)
    #
    #     self.assertListEqual(out_expected, out_computed,
    #                          "Problem with 6th example:\nExpected output: %s\nComputed output: %s" % (
    #                              out_expected, out_computed))
    #
    #     in_data = []
    #     out_expected = []
    #     out_computed = oper.offline(in_data)
    #
    #     self.assertListEqual(out_expected, out_computed,
    #                          "Problem with 7th example:\nExpected output: %s\nComputed output: %s" % (
    #                              out_expected, out_computed))
    #
    #     in_data = [[2, 5]]
    #     out_expected = []
    #     out_computed = oper.offline(in_data)
    #
    #     self.assertListEqual(out_expected, out_computed,
    #                          "Problem with 8th example:\nExpected output: %s\nComputed output: %s" % (
    #                              out_expected, out_computed))
    #
    #
    # def test_since_0_1(self):
    #     oper = SinceBoundedOperation(0, 1)
    #     in_data_1 = [[0, 3], [2, 4], [4, 6]]
    #     in_data_2 = [[0, -1], [2, 5], [4, 6]]
    #     out_expected = [[0, -1], [2, 4]]
    #     out_computed = oper.offline(in_data_1, in_data_2)
    #
    #     # self.assertListEqual(out_expected, out_computed,
    #     #                      "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
    #     #                          out_expected, out_computed))
    #
    def test_not(self):
         oper = NotOperation()
         in_data_1 = [[5, 3], [5.3, 1]]
         in_data_2 = [[5.75, 2], [6.5, 5], [6.75, 6], [9, 5], [9.25, 4]]
         in_data_3 = [[10, 2]]

         out_expected_1 = [[5, -3], [5.3, -1]]
         out_expected_2 = [[5.75, -2], [6.5, -5], [6.75, -6], [9, -5], [9.25, -4]]
         out_expected_3 = [[10, -2]]
         out_expected_final = []

         out_computed_1 = oper.update(in_data_1)
         out_computed_2 = oper.update(in_data_2)
         out_computed_3 = oper.update(in_data_3)
         out_computed_final = oper.update_final([])


         self.assertListEqual(out_expected_1, out_computed_1,
                              "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_1, out_computed_1))

         self.assertListEqual(out_expected_2, out_computed_2,
                         "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                             out_expected_2, out_computed_2))

         self.assertListEqual(out_expected_3, out_computed_3,
                              "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_3, out_computed_3))

         self.assertListEqual(out_expected_final, out_computed_final,
                              "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_final, out_computed_final))



if __name__ == '__main__':
    unittest.main()