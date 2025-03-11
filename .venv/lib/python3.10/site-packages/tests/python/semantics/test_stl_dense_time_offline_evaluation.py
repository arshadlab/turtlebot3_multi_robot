import unittest
import math

from rtamt.syntax.ast.parser.stl.specification_parser import StlAst
from rtamt.semantics.enumerations.comp_op import StlComparisonOperator
from rtamt.semantics.stl.dense_time.offline.interpreter import StlDenseTimeOfflineInterpreter


class TestSTLDenseTimeOfflineEvaluation(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(TestSTLDenseTimeOfflineEvaluation, self).__init__(*args, **kwargs)

    def test_constant(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = '5.0'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        out = interpreter.evaluate(dict())

        self.assertEqual([[0, 5.0], [float('inf'), 5.0]], out, "constant dense time offline")


    def test_addition(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a+b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 1.3], [0.7, 3], [1.3, 0.1], [2.1, -2.2]]]
        b = ['b', [[0, 2.5], [0.7, 4], [1.3, -1.2], [2.1, 1.7]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[0, 1.3 + 2.5], [0.7, 3 + 4], [1.3, 0.1 + -1.2], [2.1, -2.2 + 1.7]]

        self.assertListEqual(expected, out, "addition dense time offline 1")

        #################################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a+b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 1.3], [0.7, 3], [1.3, 4], [2.1, 3]]]
        b = ['b', [[0, 2.5], [0.7, 4], [1.3, 3], [2.1, 4]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[0, 1.3 + 2.5], [0.7, 3 + 4], [2.1, 3 + 4]]

        self.assertListEqual(expected, out, "addition dense time offline 1")
        #################################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a+b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[1, 1], [3.5, 7], [4.7, 3], [5.3, 5], [6.2, 1]]]
        b = ['b', [[0, 2], [7, 3]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[1, 1 + 2], [3.5, 7+2], [4.7, 3+2], [5.3, 5+2], [6.2, 1+2]]

        self.assertListEqual(expected, out, "addition dense time offline 1")
        #################################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a+b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[1, 1], [3.5, 7], [4.7, 3], [5.3, 5], [6.2, 1]]]
        b = ['b', [[4, 2], [6, 3]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[4, 7+2], [4.7, 3+2], [5.3, 5+2], [6, 5+3]]

        self.assertListEqual(expected, out, "addition dense time offline 1")

        #################################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a+b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[1, 1], [2, 8], [3, 4], [4.5, 7]]]
        b = ['b', [[1.5, 1], [1.7, 2], [2.7, 3], [3, 5], [4, 1]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[1.5, 1+1], [1.7, 2+1], [2, 8+2], [2.7, 8+3], [3, 4+5], [4, 4+1]]

        self.assertListEqual(expected, out, "addition dense time offline 1")

        #################################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a+b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[1, 1], [2, 8], [3, 4], [4.5, 7]]]
        b = ['b', [[5, 1], [6, 2], [7, 3], [8, 5], [9, 1]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = []

        self.assertListEqual(expected, out, "addition dense time offline 1")

        #################################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a+b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', []]
        b = ['b', [[5, 1], [6, 2], [7, 3], [8, 5], [9, 1]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = []

        self.assertListEqual(expected, out, "addition dense time offline 1")

        #################################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a+b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[1, 1], [2, 8], [3, 4], [4.5, 7]]]
        b = ['b', []]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = []

        self.assertListEqual(expected, out, "addition dense time offline 1")


        #################################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a+b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', []]
        b = ['b', []]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = []

        self.assertListEqual(expected, out, "addition dense time offline 1")

        #################################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a+b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 2]]]
        b = ['b', [[0,3]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[0, 2+3]]

        self.assertListEqual(expected, out, "addition dense time offline 1")

        #################################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a+b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 2]]]
        b = ['b', [[0.2,3]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = []

        self.assertListEqual(expected, out, "addition dense time offline 1")


        #################################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a+b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 2], [1.3, 7], [4.5, 2.6], [6.6, 7]]]
        b = ['b', [[5.2, 3]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[5.2, 3+2.6]]

        self.assertListEqual(expected, out, "addition dense time offline 1")

    def test_subtraction(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a-b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 1.3], [0.7, 3], [1.3, 0.1], [2.1, -2.2]]]
        b = ['b', [[0, 2.5], [0.7, 4], [1.3, -1.2], [2.1, 1.7]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[0, 1.3 - 2.5], [0.7, 3 - 4], [1.3, 0.1 - -1.2], [2.1, -2.2 - 1.7]]

        self.assertListEqual(expected, out, "subtraction dense time offline 1")

    def test_multiplication(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a*b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 1.3], [0.7, 3], [1.3, 0.1], [2.1, -2.2]]]
        b = ['b', [[0, 2.5], [0.7, 4], [1.3, -1.2], [2.1, 1.7]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[0, 1.3 * 2.5], [0.7, 3 * 4], [1.3, 0.1 * -1.2], [2.1, -2.2 * 1.7]]

        self.assertListEqual(expected, out, "multiplication dense time offline 1")

    def test_division(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a / b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 1.3], [0.7, 3], [1.3, 0.1], [2.1, -2.2]]]
        b = ['b', [[0, 2.5], [0.7, 4], [1.3, -1.2], [2.1, 1.7]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[0, 1.3 / 2.5], [0.7, 3. / 4.], [1.3, 0.1 / -1.2], [2.1, -2.2 / 1.7]]

        self.assertListEqual(expected, out, "division dense time offline 1")

    def test_abs(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'abs(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[1.3, 4], [3.7, -2.2], [9.4, -33]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)

        expected = [[1.3, 4], [3.7, 2.2], [9.4, 33]]

        self.assertListEqual(out, expected, "abs dense time offline 1")

        #################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'abs(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[1.3, 4]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)

        expected = [[1.3, 4]]

        self.assertListEqual(out, expected, "abs dense time offline 2")

        #################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'abs(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', []]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = []

        self.assertListEqual(out, expected, "abs dense time offline 3")

    def test_sqrt(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'sqrt(abs(a))'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[1.3, 4], [3.7, -2.2], [9.4, -33]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)

        expected = [[1.3, math.sqrt(4)], [3.7, math.sqrt(2.2)], [9.4, math.sqrt(33)]]

        self.assertListEqual(out, expected, "sqrt dense time offline 1")

        #################################################################


        #################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'sqrt(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[1.3, 4]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)

        expected = [[1.3, math.sqrt(4)]]

        self.assertListEqual(out, expected, "sqrt dense time offline 2")

        #################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'sqrt(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', []]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = []

        self.assertListEqual(out, expected, "sqrt dense time offline 3")

    def test_exp(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'exp(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[1.3, 4], [3.7, -2.2], [9.4, -33]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[1.3, math.exp(4)], [3.7, math.exp(-2.2)], [9.4, math.exp(-33)]]

        self.assertListEqual(out, expected, "exp dense time offline 1")

        #################################################################
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'exp(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[1.3, 4]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[1.3, math.exp(4)]]

        self.assertListEqual(out, expected, "exp dense time offline 2")

        #################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'exp(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', []]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = []

        self.assertListEqual(out, expected, "exp dense time offline 3")

    def test_pow(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'pow(a, b)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[1.3, 4], [3.7, 2.2], [9.4, 33]]]
        b = ['b', [[1.3, 2], [3.7, 2.2], [9.4, 2]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[1.3, math.pow(2, 4)], [3.7, math.pow(2.2, 2.2)], [9.4, math.pow(33, 2)]]
        self.assertListEqual(out, expected, "pow dense time offline 1")

        #################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'pow(a, b)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[1.3, 4]]]
        b = ['b', [[1.3, 2]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[1.3, math.pow(2, 4)]]
        self.assertListEqual(out, expected, "pow dense time offline 2")
        #################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'pow(a, b)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', []]
        b = ['b', []]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = []
        self.assertListEqual(out, expected, "pow dense time offline 3")

    def test_and(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a and b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 1.3], [0.7, 3], [1.3, 0.1], [2.1, -2.2]]]
        b = ['b', [[0, 2.5], [0.7, 4], [1.3, -1.2], [2.1, 1.7]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[0, 1.3], [0.7, 3], [1.3, -1.2], [2.1, -2.2]]

        self.assertListEqual(expected, out, "and dense time offline 1")

    def test_or(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a or b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 1.3], [0.7, 3], [1.3, 0.1], [2.1, -2.2]]]
        b = ['b', [[0, 2.5], [0.7, 4], [1.3, -1.2], [2.1, 1.7]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[0, 2.5], [0.7, 4], [1.3, 0.1], [2.1, 1.7]]

        self.assertListEqual(expected, out, "or dense time offline 1")

    def test_iff(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a iff b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 1.3], [0.7, 3], [1.3, 0.1], [2.1, -2.2]]]
        b = ['b', [[0, 2.5], [0.7, 4], [1.3, -1.2], [2.1, 1.7]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[0, 1.3 - 2.5], [0.7, 3 - 4], [1.3, -1.2 - 0.1], [2.1, -2.2 - 1.7]]

        self.assertListEqual(expected, out, "iff dense time offline 1")

    def test_xor(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a xor b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 1.3], [0.7, 3], [1.3, 0.1], [2.1, -2.2]]]
        b = ['b', [[0, 2.5], [0.7, 4], [1.3, -1.2], [2.1, 1.7]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)

        expected = [[0, 2.5-1.3], [0.7, 4-3], [1.3, 1.2 + 0.1], [2.1, 2.2 + 1.7]]

        self.assertListEqual(expected, out, "xor dense time offline 1")


    def test_implies(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a -> b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 1.3], [0.7, 3], [1.3, 0.1], [2.1, -2.2]]]
        b = ['b', [[0, 2.5], [0.7, 4], [1.3, -1.2], [2.1, 1.7]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[0, 2.5], [0.7, 4], [1.3, -0.1], [2.1, 2.2]]

        self.assertListEqual(expected, out, "implies dense time offline 1")

    def test_always(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'G(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 2.5], [0.7, 4], [1.3, -1.2], [2.1, 1.7]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, -1.2], [2.1, 1.7]]

        self.assertListEqual(out, expected, "always dense time offline 1")

        ####################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'G(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 2.5]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 2.5]]

        self.assertListEqual(out, expected, "always dense time offline 2")

        ####################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'G(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', []]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = []

        self.assertListEqual(out, expected, "always dense time offline 3")

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'G(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 2.5], [0.7, 4], [1.3, -1.2], [2.1, -1.7]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, -1.7], [2.1, -1.7]]

        self.assertListEqual(out, expected, "always dense time offline 4")

        ####################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'G(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 2.5], [2.1, 1.7]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 1.7], [2.1, 1.7]]

        self.assertListEqual(out, expected, "always dense time offline 5")

        ####################################################################
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'G(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 1], [0.7, 2], [1.3, 3], [2.1, 4]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 1], [0.7, 2], [1.3, 3], [2.1, 4]]

        self.assertListEqual(out, expected, "always dense time offline 6")

        ####################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'G(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 4], [0.7, 3], [1.3, 2], [2.1, 1]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 1], [2.1, 1]]

        self.assertListEqual(out, expected, "always dense time offline 7")

    def test_historically(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'H(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 2.5], [0.7, 4], [1.3, -1.2], [2.1, 1.7]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 2.5], [1.3, -1.2], [2.1, -1.2]]

        self.assertListEqual(out, expected, "historically dense time offline 1")

        ####################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'H(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 2.5]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 2.5]]

        self.assertListEqual(out, expected, "historically dense time offline 2")

        ####################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'H(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', []]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = []

        self.assertListEqual(out, expected, "historically dense time offline 3")

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'H(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 2.5], [0.7, 4], [1.3, -1.2], [2.1, 6]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 2.5], [1.3, -1.2], [2.1, -1.2]]

        self.assertListEqual(out, expected, "historically dense time offline 4")

        ####################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'H(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 2], [2.1, 2.1]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 2], [2.1, 2]]

        self.assertListEqual(out, expected, "historically dense time offline 5")

        ####################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'H(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 1], [0.7, 2], [1.3, 3], [2.1, 4]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 1], [2.1, 1]]

        self.assertListEqual(out, expected, "historically dense time offline 6")

        ####################################################################
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'H(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 4], [0.7, 3], [1.3, 2], [2.1, 1]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)

        expected = [[0, 4], [0.7, 3], [1.3, 2], [2.1, 1]]

        self.assertListEqual(out, expected, "historically dense time offline 7")

    def test_once(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'once(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 2.5], [0.7, 4], [1.3, -1.2], [2.1, 1.7]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 2.5], [0.7, 4], [2.1, 4]]

        self.assertListEqual(out, expected, "once dense time offline 1")

        ####################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'H(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 2.5]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 2.5]]

        self.assertListEqual(out, expected, "once dense time offline 2")

        ####################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'H(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', []]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = []

        self.assertListEqual(out, expected, "once dense time offline 3")

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'once(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 2.5], [0.7, 4], [1.3, -1.2], [2.1, 6]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 2.5], [0.7, 4], [2.1, 6]]

        self.assertListEqual(out, expected, "once dense time offline 4")

        ####################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'once(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 2.5], [2.1, 2.1]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 2.5], [2.1, 2.5]]

        self.assertListEqual(out, expected, "once dense time offline 5")

        ####################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'once(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 1], [0.7, 2], [1.3, 3], [2.1, 4]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 1], [0.7, 2], [1.3, 3], [2.1, 4]]

        self.assertListEqual(out, expected, "once dense time offline 6")

        ####################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'once(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 4], [0.7, 3], [1.3, 2], [2.1, 1]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 4], [2.1, 4]]

        self.assertListEqual(out, expected, "once dense time offline 7")

    def test_eventually(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'F(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 2.5], [0.7, 4], [1.3, -1.2], [2.1, 1.7]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 4], [1.3, 1.7], [2.1, 1.7]]

        self.assertListEqual(out, expected, "ev dense time offline 1")

        ####################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'F(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 2.5]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 2.5]]

        self.assertListEqual(out, expected, "eventually dense time offline 2")

        ####################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'F(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', []]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = []

        self.assertListEqual(out, expected, "eventually dense time offline 3")

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'F(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 2.5], [0.7, 4], [1.3, -1.2], [2.1, -1.7]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 4], [1.3, -1.2], [2.1, -1.7]]

        self.assertListEqual(out, expected, "eventually dense time offline 4")

        ####################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'F(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 2.5], [2.1, 3]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 3], [2.1, 3]]

        self.assertListEqual(out, expected, "eventually dense time offline 5")

        ####################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'F(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 1], [0.7, 2], [1.3, 3], [2.1, 4]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 4], [2.1, 4]]

        self.assertListEqual(out, expected, "eventually dense time offline 6")

        ####################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'F(a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 4], [0.7, 3], [1.3, 2], [2.1, 1]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 4], [0.7, 3], [1.3, 2], [2.1, 1]]

        self.assertListEqual(out, expected, "eventually dense time offline 7")

    def test_eventually_bounded(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'F[0,1](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 4], [5, 2], [10, 5], [15, 3]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 4], [5, 2], [9, 5], [15, 3]]

        self.assertListEqual(expected, out, "eventually[0,1] offline dense time 1")

        #####################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'F[0,1](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', []]
        dataset = [a]
        out = interpreter.evaluate(dataset)

        expected = []

        self.assertListEqual(expected, out, "eventually[0,1] offline dense time 2")

        #####################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'F[0,1](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 2]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 2]]

        self.assertListEqual(expected, out, "eventually[0,1] offline dense time 3")

        #####################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'F[0,1](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[2, 2]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)

        expected = [[1, 2]]

        self.assertListEqual(expected, out, "eventually[0,1] offline dense time 4")


        #####################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'F[1,2](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 4], [5, 2], [10, 5], [15, 3]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 4], [4, 2], [8, 5], [14, 3]]

        self.assertListEqual(expected, out, "eventually[0,1] offline dense time 5")

        #####################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'F[2,2](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 4], [5, 2], [10, 5], [15, 3]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 4], [3, 2], [8, 5], [13, 3]]
        self.assertListEqual(expected, out, "eventually[0,1] offline dense time 6")

        #####################
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'F[11,11](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 4], [5, 2], [10, 5], [15, 3]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 5], [4, 3]]

        self.assertListEqual(expected, out, "eventually[0,1] offline dense time 7")

        #####################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'F[0,2](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 4], [5, 2], [5.1, 1], [5.2, 0], [6, 5], [15, 3]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 4], [4, 5], [15, 3]]

        self.assertListEqual(expected, out, "eventually[0,1] offline dense time 8")

        #########################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'F[0,2](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 4], [5, 2], [5.1, 1], [5.2, 0], [7, 5], [15, 3]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 4], [5, 5], [15, 3]]

        self.assertListEqual(expected, out, "eventually[0,1] offline dense time 9")

        #########################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'F[0,2](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 4], [5, 2], [5.1, 1], [5.2, 0], [7.1, 5], [15, 3]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 4], [5, 2], [5.1, 5], [15, 3]]

        self.assertListEqual(expected, out, "eventually[0,1] offline dense time 10")

    def test_always_bounded(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'G[0,1](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 4], [5, 2], [10, 5], [15, 3]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 4], [4, 2], [10, 5], [14, 3]]

        self.assertListEqual(expected, out, "always[0,1] offline dense time 1")

        #####################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'G[0,1](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', []]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = []

        self.assertListEqual(expected, out, "always[0,1] offline dense time 2")

        #####################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'G[0,1](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 2]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 2]]

        self.assertListEqual(expected, out, "always[0,1] offline dense time 3")

        #####################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'G[0,1](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[2, 2]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[1, 2]]

        self.assertListEqual(expected, out, "always[0,1] offline dense time 4")


        #####################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'G[1,2](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 4], [5, 2], [10, 5], [15, 3]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 4], [3, 2], [9, 5], [13, 3]]

        self.assertListEqual(expected, out, "always[1,2] offline dense time 5")

        #####################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'G[2,2](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 4], [5, 2], [10, 5], [15, 3]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 4], [3, 2], [8, 5], [13, 3]]

        self.assertListEqual(expected, out, "always[2,2] offline dense time 6")

        #####################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'G[11,11](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 4], [5, 2], [10, 5], [15, 3]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 5], [4, 3]]

        self.assertListEqual(expected, out, "always[0,1] offline dense time 7")

        #####################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'G[0,2](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 4], [5, 6], [5.1, 8], [5.2, 10], [6, 5], [15, 3]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 4], [5, 5], [13, 3]]

        self.assertListEqual(expected, out, "always[0,1] offline dense time 8")

        #########################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'G[0,2](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 4], [5, 8], [5.1, 9], [5.2, 10], [7, 5], [15, 3]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 4], [5, 5], [13, 3]]

        self.assertListEqual(expected, out, "always[0,1] offline dense time 9")

        #########################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'G[0,2](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 4], [5, 8], [5.1, 9], [5.2, 10], [7.1, 5], [15, 3]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 4], [5, 8], [5.1, 5], [13, 3]]

        self.assertListEqual(expected, out, "always[0,1] offline dense time 10")



    def test_since(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a since b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 2], [5, 2]]]
        b = ['b', [[0, 4], [5, 4]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[0, 2], [5, 2]]

        self.assertListEqual(expected, out, "since dense time offline 1")

        ###################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a since b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 2]]]
        b = ['b', [[0, 4], [5, 4]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[0, 2]]

        self.assertListEqual(expected, out, "since dense time offline 2")

        ###################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a since b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[2.3, 2]]]
        b = ['b', [[0, 4], [2, 6], [5, 4]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[2.3, 2]]

        self.assertListEqual(expected, out, "since dense time offline 3")

        ###################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a since b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 1.3], [0.7, 3], [1.3, 0.1], [2.1, -2.2]]]
        b = ['b', [[0, 2.5], [0.7, 4], [1.3, -1.2], [2.1, 1.7]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)

        expected = [[0, 1.3], [0.7, 3], [1.3, 0.1], [2.1, -2.2]]

        self.assertListEqual(expected, out, "since dense time offline 4")

        ###################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a since b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[1, 1], [3.5, 7], [4.7, 3], [5.3, 5], [6.2, 1]]]
        b = ['b', [[0, 2], [7, 3]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)

        expected = [[1, 1], [3.5, 2], [6.2, 1]]

        self.assertListEqual(expected, out, "since dense time offline 4")

        ##########

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a since b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[1, 1], [3.5, 7], [4.7, 3], [5.3, 5], [6.2, 1]]]
        b = ['b', [[4, 2], [6, 3]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[4, 2], [6, 3]]

        self.assertListEqual(expected, out, "since dense time offline 4")

    def test_until(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a  until b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 2], [5, 2]]]
        b = ['b', [[0, 4], [5, 4]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[0, 2], [5, 2]]

        self.assertListEqual(expected, out, "until dense time offline 1")

        ###################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a until b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 2]]]
        b = ['b', [[0, 4], [5, 4]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[0, 2]]

        self.assertListEqual(expected, out, "until dense time offline 2")

        ###################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a until b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[2.3, 2]]]
        b = ['b', [[0, 4], [2, 6], [5, 4]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)

        expected = [[2.3, 2]]

        self.assertListEqual(expected, out, "until dense time offline 3")

        ###################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a until b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 1.3], [0.7, 3], [1.3, 0.1], [2.1, -2.2]]]
        b = ['b', [[0, 2.5], [0.7, 4], [1.3, -1.2], [2.1, 1.7]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[0, 1.3], [0.7, 3], [1.3, -1.2], [2.1, -2.2]]

        self.assertListEqual(expected, out, "until dense time offline 4")

        ###################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a until b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[1, 1], [3.5, 7], [4.7, 3], [5.3, 5], [6.2, 1]]]
        b = ['b', [[0, 2], [7, 3]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[1, 1], [3.5, 2], [6.2, 1]]

        self.assertListEqual(expected, out, "until dense time offline 5")

        ########################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a until b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[1, 1], [3.5, 7], [4.7, 3], [5.3, 5], [6.2, 1]]]
        b = ['b', [[4, 2], [6, 3]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[4, 3], [6, 3]]

        self.assertListEqual(expected, out, "until dense time offline 6")

    def test_until_0_1(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a until[0,1] b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[1, 1], [3.5, 7], [4.7, 3], [5.3, 5], [6.2, 1]]]
        b = ['b', [[4, 2], [6, 3]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)


        expected = [[4, 2], [5, 3]]
        self.assertListEqual(out, expected, "until")

    def test_once_bounded(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'once[0,1](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 4], [5, 2], [10, 5], [15, 3]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 4], [6, 2], [10, 5], [16, 3]]

        self.assertListEqual(expected, out, "once[0,1] offline dense time 1")

        #######################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'once[0,1](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 4], [5, 2], [10, 5], [15, 6]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 4], [6, 2], [10, 5], [15, 6]]

        self.assertListEqual(expected, out, "once[0,1] offline dense time 2")

        #######################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'once[0,1](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', []]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = []

        self.assertListEqual(expected, out, "once[0,1] offline dense time 3")

        #######################################################################
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'once[0,1](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[2, 1]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[2, 1]]

        self.assertListEqual(expected, out, "once[0,1] offline dense time 4")

        #######################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'once[0,1](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 4], [5, 2], [5.5, 5], [15, 6]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 4], [5.5, 5], [15, 6]]

        self.assertListEqual(expected, out, "once[0,1] offline dense time 5")

        #######################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'once[1,2](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 4], [5, 2], [10, 5], [15, 3]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, -float('inf')], [1, 4], [7, 2], [11, 5], [17, 3]]

        self.assertListEqual(expected, out, "once[0,1] offline dense time 6")

        #######################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'once[2,2](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 4], [5, 2], [10, 5], [15, 3]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, -float('inf')], [2, 4], [7, 2], [12, 5], [17, 3]]

        self.assertListEqual(expected, out, "once[0,1] offline dense time 7")

        #######################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'once[1,2](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 4], [5, 6], [5.1, 3], [5.2, 2], [5.3, 1], [5.5, 5], [15, 3]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, -float('inf')], [1, 4], [6, 6], [7.1, 5], [17, 3]]

        self.assertListEqual(expected, out, "once[0,1] offline dense time 8")

        #######################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'once[0,1](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 4], [5, 6], [7, 3], [7.5, 6], [15, 3]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 4], [5, 6], [16, 3]]

        self.assertListEqual(expected, out, "once[0,1] offline dense time 9")

        #######################################################################

    def test_historically_bounded(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'H[0,1](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 4], [5, 2], [10, 5], [15, 3]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 4], [5, 2], [11, 5], [15, 3]]

        self.assertListEqual(expected, out, "historically[0,1] offline dense time 1")

        #######################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'H[0,1](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 4], [5, 2], [10, 5], [15, 6]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 4], [5, 2], [11, 5], [16, 6]]

        self.assertListEqual(expected, out, "historically[0,1] offline dense time 2")

        #######################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'H[0,1](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', []]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = []

        self.assertListEqual(expected, out, "historically[0,1] offline dense time 3")

        #######################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'H[0,1](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[2, 1]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[2, 1]]

        self.assertListEqual(expected, out, "historically[0,1] offline dense time 4")

        #######################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'H[0,1](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 4], [5, 6], [5.5, 3], [15, 6]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, 4], [5.5, 3], [16, 6]]

        self.assertListEqual(expected, out, "historically[0,1] offline dense time 5")

        #######################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'H[1,2](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 4], [5, 2], [10, 5], [15, 3]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, float('inf')], [1, 4], [6, 2], [12, 5], [16, 3]]

        self.assertListEqual(expected, out, "historically[1,2] offline dense time 6")

        #######################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'H[2,2](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 4], [5, 2], [10, 5], [15, 3]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, float('inf')], [2, 4], [7, 2], [12, 5], [17, 3]]

        self.assertListEqual(expected, out, "historically[2,2] offline dense time 7")

        #######################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'H[1,2](a)'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 4], [5, 6], [5.1, 10], [5.2, 11], [5.3, 12], [5.5, 7], [15, 3]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[0, float('inf')], [1, 4], [7, 6], [7.1, 7], [16, 3]]

        self.assertListEqual(expected, out, "historically[1,2] offline dense time 8")

        #######################################################################

    def test_since_0_1(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a since[0,1] b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 2], [10, 2]]]
        b = ['b', [[0, 4], [10, 4]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[0, 2], [10, 2]]

        self.assertListEqual(out, expected, "since[0,1]")

        ########################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a since[1,2] b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 2], [10, 2]]]
        b = ['b', [[0, 4], [10, 4]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[0, -float('inf')], [1, 2], [11, 2]]

        self.assertListEqual(out, expected, "since[0,1]")

        ########################################################################

    def test_until_1_2(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a until[1,2] b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 2], [10, 2]]]
        b = ['b', [[0, 4], [10, 4]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[0, 2], [8, 2]]

        self.assertListEqual(out, expected, "until[0,1]")

    def test_not(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'not a'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[1.3, 4], [3.7, -2.2], [9.4, -33]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[1.3, -4], [3.7, 2.2], [9.4, 33]]

        self.assertListEqual(out, expected, "not dense time offline 1")

        #################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'not a'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[1.3, -4]]]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = [[1.3, 4]]

        self.assertListEqual(out, expected, "not dense time offline 2")

        #################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'not a'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', []]
        dataset = [a]
        out = interpreter.evaluate(dataset)
        expected = []

        self.assertListEqual(out, expected, "not dense time offline 3")

    def test_predicate_leq(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a <= b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 1.3], [0.7, 3], [1.3, 0.1], [2.1, -2.2]]]
        b = ['b', [[0, 2.5], [0.7, 4], [1.3, -1.2], [2.1, 1.7]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[0, 2.5 - 1.3], [0.7, 4 - 3], [1.3, -1.2-0.1], [2.1, 1.7+2.2]]

        self.assertListEqual(expected, out, "leq dense time offline 1")

        #################################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a <= b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 1.3], [0.7, 3], [1.3, 4], [2.1, 3]]]
        b = ['b', [[0, 2.5], [0.7, 4], [1.3, 5], [2.1, 4]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)

        expected = [[0, 2.5-1.3], [0.7, 4-3], [2.1, 4-3]]

        self.assertListEqual(expected, out, "leq dense time offline 2")

        #################################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a <= b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[1, 1], [3.5, 7], [4.7, 3], [5.3, 5], [6.2, 1]]]
        b = ['b', [[0, 2], [7, 3]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[1, 2-1], [3.5, 2-7], [4.7, 2-3], [5.3, 2-5], [6.2, 2-1]]

        self.assertListEqual(expected, out, "leq dense time offline 3")

        #################################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a <= b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[1, 1], [3.5, 7], [4.7, 3], [5.3, 5], [6.2, 1]]]
        b = ['b', [[4, 2], [6, 3]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[4, 2-7], [4.7, 2-3], [5.3, 2-5], [6, 3-5]]

        self.assertListEqual(expected, out, "leq dense time offline 4")

        #################################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a <= b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[1, 1], [2, 8], [3, 4], [4.5, 7]]]
        b = ['b', [[1.5, 1], [1.7, 2], [2.7, 3], [3, 5], [4, 1]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[1.5, 1-1], [1.7, 2-1], [2, 2-8], [2.7, 3-8], [3, 5-4], [4, 1-4]]

        self.assertListEqual(expected, out, "leq dense time offline 5")

        #################################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a <= b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[1, 1], [2, 8], [3, 4], [4.5, 7]]]
        b = ['b', [[5, 1], [6, 2], [7, 3], [8, 5], [9, 1]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = []

        self.assertListEqual(expected, out, "leq dense time offline 6")

        #################################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a <= b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', []]
        b = ['b', [[5, 1], [6, 2], [7, 3], [8, 5], [9, 1]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = []

        self.assertListEqual(expected, out, "leq dense time offline 7")

        #################################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a <= b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[1, 1], [2, 8], [3, 4], [4.5, 7]]]
        b = ['b', []]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = []

        self.assertListEqual(expected, out, "leq dense time offline 8")

        #################################################################################

        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a <= b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', []]
        b = ['b', []]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = []

        self.assertListEqual(expected, out, "leq dense time offline 9")

    def test_predicate_less(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a < b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 1.3], [0.7, 3], [1.3, 0.1], [2.1, -2.2]]]
        b = ['b', [[0, 2.5], [0.7, 4], [1.3, -1.2], [2.1, 1.7]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[0, 2.5 - 1.3], [0.7, 4 - 3], [1.3, -1.2 - 0.1], [2.1, 1.7 + 2.2]]

        self.assertListEqual(expected, out, "less dense time offline 1")

    def test_predicate_geq(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a >= b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 1.3], [0.7, 3], [1.3, 0.1], [2.1, -2.2]]]
        b = ['b', [[0, 2.5], [0.7, 4], [1.3, -1.2], [2.1, 1.7]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[0, 1.3-2.5], [0.7, 3-4], [1.3, 0.1+1.2], [2.1, -2.2-1.7]]

        self.assertListEqual(expected, out, "geq dense time offline 1")

    def test_predicate_greater(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a > b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 1.3], [0.7, 3], [1.3, 0.1], [2.1, -2.2]]]
        b = ['b', [[0, 2.5], [0.7, 4], [1.3, -1.2], [2.1, 1.7]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[0, 1.3 - 2.5], [0.7, 3 - 4], [1.3, 0.1 + 1.2], [2.1, -2.2 - 1.7]]

        self.assertListEqual(expected, out, "greater dense time offline 1")

    def test_predicate_eq(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a == b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 1.3], [0.7, 3], [1.3, 0.1], [2.1, -2.2]]]
        b = ['b', [[0, 2.5], [0.7, 4], [1.3, -1.2], [2.1, 1.7]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)

        expected = [[0, 1.3 - 2.5], [0.7, 3 - 4], [1.3, -0.1 - 1.2], [2.1, -2.2 - 1.7]]

        self.assertListEqual(expected, out, "eq dense time offline 1")

    def test_predicate_neq(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a !== b'
        ast.parse()
        interpreter = StlDenseTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = ['a', [[0, 1.3], [0.7, 3], [1.3, 0.1], [2.1, -2.2]]]
        b = ['b', [[0, 2.5], [0.7, 4], [1.3, -1.2], [2.1, 1.7]]]
        dataset = [a, b]
        out = interpreter.evaluate(dataset)
        expected = [[0, 2.5-1.3], [0.7, 4-3], [1.3, 0.1 + 1.2], [2.1, 2.2+1.7]]

        self.assertListEqual(expected, out, "neq dense time offline 1")

if __name__ == '__main__':
    unittest.main()