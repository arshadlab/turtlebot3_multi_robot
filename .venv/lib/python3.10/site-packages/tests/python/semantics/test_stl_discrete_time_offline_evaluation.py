import unittest
import math

from rtamt.syntax.ast.parser.stl.specification_parser import StlAst
from rtamt.semantics.enumerations.comp_op import StlComparisonOperator
from rtamt.semantics.stl.discrete_time.offline.interpreter import StlDiscreteTimeOfflineInterpreter


class TestSTLEvaluation(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(TestSTLEvaluation, self).__init__(*args, **kwargs)
        self.left = [100, -1, -2, 5, -1]
        self.right = [20, -2, 10, 4, -1]

    def test_constant(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = '5.0'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        dataset = {'time': [0, 1, 2, 3, 4]}

        out = interpreter.evaluate(dataset)

        expected = [[0, 5.0], [1, 5.0], [2, 5.0], [3, 5.0], [4, 5.0]]

        self.assertEqual(expected, out, "constant dense time offline")


    def test_addition(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a + b'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100, -1, -2, 5, -1]
        b = [20, -2, 10, 4, -1]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a, 'b': b}

        out = interpreter.evaluate(dataset)

        expected = [[0, 120], [1, -3], [2, 8], [3, 9], [4, -2]]

        self.assertListEqual(out, expected, "addition")

    def test_subtraction(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a - b'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100, -1, -2, 5, -1]
        b = [20, -2, 10, 4, -1]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a, 'b': b}

        out = interpreter.evaluate(dataset)
        expected = [[0, 80], [1, 1], [2, -12], [3, 1], [4, 0]]

        self.assertListEqual(out, expected, "subtraction")

    def test_multiplication(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a * b'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100, -1, -2, 5, -1]
        b = [20, -2, 10, 4, -1]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a, 'b': b}

        out = interpreter.evaluate(dataset)
        expected = [[0, 2000], [1, 2], [2, -20], [3, 20], [4, 1]]

        self.assertListEqual(out, expected, "multiplication")

    def test_division(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a / b'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100.0, -1.0, -2.0, 5.0, -1.0]
        b = [20.0, -2.0, 10.0, 4.0, -1.0]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a, 'b': b}

        out = interpreter.evaluate(dataset)
        expected = [[0, 100.0/20.0], [1, -1.0/-2.0], [2, -2.0/10.0], [3, 5.0/4.0], [4, -1.0/-1.0]]

        self.assertListEqual(out, expected, "division")

    def test_abs(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'abs(a)'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100, -1, -2, 5, -1]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a}

        out = interpreter.evaluate(dataset)
        expected = [[0, 100], [1, 1], [2, 2], [3, 5], [4, 1]]

        self.assertListEqual(out, expected, "abs")

    def test_sqrt(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'sqrt(a)'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [1, 2.2, 0.5]
        t = [0, 1, 2]

        dataset = {'time': t, 'a': a}

        out = interpreter.evaluate(dataset)
        expected = [[0, math.sqrt(1)], [1, math.sqrt(2.2)], [2, math.sqrt(0.5)]]

        self.assertListEqual(out, expected, "sqrt")

    def test_exp(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'exp(a)'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [1, 2.2, 0.5]
        t = [0, 1, 2]

        dataset = {'time': t, 'a': a}

        out = interpreter.evaluate(dataset)
        expected = [[0, math.exp(1)], [1, math.exp(2.2)], [2, math.exp(0.5)]]

        self.assertListEqual(out, expected, "exp")

    def test_pow(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'pow(a, b)'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [1, 2.2, 0.5]
        b = [2, 0.3, 2]
        t = [0, 1, 2]

        dataset = {'time': t, 'a': a, 'b': b}

        out = interpreter.evaluate(dataset)
        expected = [[0, math.pow(1, 2)], [1, math.pow(2.2, 0.3)], [2, math.pow(0.5, 2)]]

        self.assertListEqual(out, expected, "pow")

    def test_previous(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'prev(a)'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100, -1, -2, 5, -1]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a}

        out = interpreter.evaluate(dataset)
        expected = [[0, float("inf")], [1, 100], [2, -1], [3, -2], [4, 5]]

        self.assertListEqual(out, expected, "previous")

    def test_next(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'next(a)'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100, -1, -2, 5, -1]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a}

        out = interpreter.evaluate(dataset)
        expected = [[0, -1], [1, -2], [2, 5], [3, -1], [4, float("inf")]]

        self.assertListEqual(out, expected, "next")

    def test_and(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a and b'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100, -1, -2, 5, -1]
        b = [20, -2, 10, 4, -1]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a, 'b': b}

        out = interpreter.evaluate(dataset)
        expected = [[0, 20], [1, -2], [2, -2], [3, 4], [4, -1]]
        self.assertListEqual(out, expected, "and")

    def test_or(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a or b'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100, -1, -2, 5, -1]
        b = [20, -2, 10, 4, -1]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a, 'b': b}

        out = interpreter.evaluate(dataset)
        expected = [[0, 100], [1, -1], [2, 10], [3, 5], [4, -1]]

        self.assertListEqual(out, expected, "or")

    def test_iff(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a iff b'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100, -1, -2, 5, -1]
        b = [20, -2, 10, 4, -1]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a, 'b': b}

        out = interpreter.evaluate(dataset)
        expected = [[0, -80], [1, -1], [2, -12], [3, -1], [4, 0]]
        self.assertListEqual(out, expected, "iff")

    def test_xor(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a xor b'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100, -1, -2, 5, -1]
        b = [20, -2, 10, 4, -1]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a, 'b': b}

        out = interpreter.evaluate(dataset)
        expected = [[0, 80], [1, 1], [2, 12], [3, 1], [4, 0]]

        self.assertListEqual(out, expected, "xor")


    def test_implies(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a -> b'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100, -1, -2, 5, -1]
        b = [20, -2, 10, 4, -1]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a, 'b': b}

        out = interpreter.evaluate(dataset)
        expected = [[0, 20], [1, 1], [2, 10], [3, 4], [4, 1]]

        self.assertListEqual(out, expected, "implies")

    def test_always(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'always(a)'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100, -1, -2, 5, -1]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a}

        out = interpreter.evaluate(dataset)
        expected = [[0, -2], [1, -2], [2, -2], [3, -1], [4, -1]]

        self.assertListEqual(out, expected, "always")

    def test_always_0_1(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'always[0,1](a)'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100, -1, -2, 5, -1]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a}

        out = interpreter.evaluate(dataset)
        expected = [[0, -1], [1, -2], [2, -2], [3, -1], [4, -1]]

        self.assertListEqual(out, expected, "always[0,1]")

    def test_historically(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'historically(a)'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100, -1, -2, 5, -1]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a}

        out = interpreter.evaluate(dataset)
        expected = [[0, 100], [1, -1], [2, -2], [3, -2], [4, -2]]

        self.assertListEqual(out, expected, "historically")

    def test_once(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'once(a)'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100, -1, -2, 5, -1]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a}

        out = interpreter.evaluate(dataset)
        expected = [[0, 100], [1, 100], [2, 100], [3, 100], [4, 100]]

        self.assertListEqual(out, expected, "once")

    def test_eventually(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'eventually(a)'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100, -1, -2, 5, -1]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a}

        out = interpreter.evaluate(dataset)
        expected = [[0, 100], [1, 5], [2, 5], [3, 5], [4, -1]]

        self.assertListEqual(out, expected, "eventually")

    def test_eventually_0_1(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'eventually[0,1](a)'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100, -1, -2, 5, -1]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a}

        out = interpreter.evaluate(dataset)
        expected = [[0, 100], [1, -1], [2, 5], [3, 5], [4, -1]]

        self.assertListEqual(out, expected, "eventually[0,1]")

    def test_since(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a since b'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100, -1, -2, 5, -1]
        b = [20, -2, 10, 4, -1]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a, 'b': b}

        out = interpreter.evaluate(dataset)
        expected = [[0, 20], [1, -1], [2, 10], [3, 5], [4, -1]]
        self.assertListEqual(out, expected, "since")

    def test_until(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a until b'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100, -1, -2, 5, -1]
        b = [20, -2, 10, 4, -1]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a, 'b': b}

        out = interpreter.evaluate(dataset)
        expected = [[0, 20], [1, -1], [2, 10], [3, 4], [4, -1]]
        self.assertListEqual(out, expected, "until")

    def test_until_0_1(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a until[0,1] b'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100, -1, -2, 5, -1]
        b = [20, -2, 10, 4, -1]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a, 'b': b}

        out = interpreter.evaluate(dataset)
        expected = [[0, 20], [1, -1], [2, 10], [3, 4], [4, -1]]

        self.assertListEqual(out, expected, "until")

    def test_once_0_1(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'once[0,1](a)'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100, -1, -2, 5, -1]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a}

        out = interpreter.evaluate(dataset)
        expected = [[0, 100], [1, 100], [2, -1], [3, 5], [4, 5]]
        self.assertListEqual(out, expected, "once[0,1]")

    def test_once_1_2(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'once[1,2](a)'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100, -1, -2, 5, -1]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a}

        out = interpreter.evaluate(dataset)
        expected = [[0, -float("inf")], [1, 100], [2, 100], [3, -1], [4, 5]]

        self.assertListEqual(out, expected, "once[1,2]")

    def test_historically_0_1(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'historically[0,1](a)'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100, -1, -2, 5, -1]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a}

        out = interpreter.evaluate(dataset)
        expected = [[0, 100], [1, -1], [2, -2], [3, -2], [4, -1]]

        self.assertListEqual(out, expected, "historically[0,1]")

    def test_historically_1_2(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'historically[1,2](a)'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100, -1, -2, 5, -1]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a}

        out = interpreter.evaluate(dataset)
        expected = [[0, float("inf")], [1, 100], [2, -1], [3, -2], [4, -2]]

        self.assertListEqual(out, expected, "historically[1,2]")

    def test_since_0_1(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a since[0,1] b'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100, -1, -2, 5, -1]
        b = [20, -2, 10, 4, -1]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a, 'b': b}

        out = interpreter.evaluate(dataset)
        expected = [[0, 20], [1, -1], [2, 10], [3, 5], [4, -1]]

        self.assertListEqual(out, expected, "since[0,1]")

    def test_not(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'not(a)'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100, -1, -2, 5, -1]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a}

        out = interpreter.evaluate(dataset)
        expected = [[0, -100], [1, 1], [2, 2], [3, -5], [4, 1]]

        self.assertListEqual(out, expected, "not")

    def test_rise(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'rise(a)'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100, -1, -2, 5, -1]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a}

        out = interpreter.evaluate(dataset)
        expected = [[0, 100], [1, -100], [2, -2], [3, 2], [4, -5]]

        self.assertListEqual(out, expected, "rise")

    def test_fall(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.spec = 'fall(a)'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100, -1, -2, 5, -1]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a}

        out = interpreter.evaluate(dataset)
        expected = [[0, -100], [1, 1], [2, -1], [3, -5], [4, 1]]

        self.assertListEqual(out, expected, "fall")

    def test_predicate_leq(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a <= b'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100, -1, -2, 5, -1]
        b = [20, -2, 10, 4, -1]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a, 'b': b}

        out = interpreter.evaluate(dataset)
        expected = [[0, -80], [1, -1], [2, 12], [3, -1], [4, 0]]

        self.assertListEqual(out, expected, "leq")

    def test_predicate_less(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a < b'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100, -1, -2, 5, -1]
        b = [20, -2, 10, 4, -1]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a, 'b': b}

        out = interpreter.evaluate(dataset)
        expected = [[0, -80], [1, -1], [2, 12], [3, -1], [4, 0]]

        self.assertListEqual(out, expected, "leq")

    def test_predicate_geq(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a >= b'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100, -1, -2, 5, -1]
        b = [20, -2, 10, 4, -1]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a, 'b': b}

        out = interpreter.evaluate(dataset)
        expected = [[0, 80], [1, 1], [2, -12], [3, 1], [4, 0]]


        self.assertListEqual(out, expected, "geq")

    def test_predicate_greater(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a > b'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100, -1, -2, 5, -1]
        b = [20, -2, 10, 4, -1]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a, 'b': b}

        out = interpreter.evaluate(dataset)
        expected = [[0, 80], [1, 1], [2, -12], [3, 1], [4, 0]]

        self.assertListEqual(out, expected, "greater")

    def test_predicate_eq(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a == b'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100, -1, -2, 5, -1]
        b = [20, -2, 10, 4, -1]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a, 'b': b}

        out = interpreter.evaluate(dataset)
        expected = [[0, -80], [1, -1], [2, -12], [3, -1], [4, 0]]

        self.assertListEqual(out, expected, "eq")

    def test_predicate_neq(self):
        ast = StlAst()
        ast.declare_var('a', 'float')
        ast.declare_var('b', 'float')
        ast.spec = 'a !== b'
        ast.parse()
        interpreter = StlDiscreteTimeOfflineInterpreter()
        interpreter.set_ast(ast)

        a = [100, -1, -2, 5, -1]
        b = [20, -2, 10, 4, -1]
        t = [0, 1, 2, 3, 4]

        dataset = {'time': t, 'a': a, 'b': b}

        out = interpreter.evaluate(dataset)
        expected = [[0, 80], [1, 1], [2, 12], [3, 1], [4, 0]]

        self.assertListEqual(out, expected, "neq")

if __name__ == '__main__':
    unittest.main()