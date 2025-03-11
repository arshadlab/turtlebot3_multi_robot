import unittest
import math

from rtamt.spec.stl.discrete_time.specification import StlDiscreteTimeSpecification

class TestStlReset(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestStlReset, self).__init__(*args, **kwargs)

    def test_constant(self):
        spec = StlDiscreteTimeSpecification()
        spec.declare_var('out', 'float')
        spec.spec = 'out = 5'
        spec.parse()

        out = spec.update(0, [])
        self.assertEqual(5, out, 'Constant reset assertion')

        out = spec.update(0, [])
        self.assertEqual(5, out, 'Constant reset assertion')

        spec.reset()

        out = spec.update(0, [])
        self.assertEqual(5, out, 'Constant reset assertion')

    def test_variable(self):
        spec = StlDiscreteTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req'
        spec.parse()

        out = spec.update(0, [['req', 1.1]])
        self.assertEqual(1.1, out, 'Variable reset assertion')

        out = spec.update(1, [['req', 2]])
        self.assertEqual(2, out, 'Variable reset assertion')

        spec.reset()

        out = spec.update(0, [['req', 3.3]])
        self.assertEqual(3.3, out, 'Variable reset assertion')

    def test_abs(self):
        spec = StlDiscreteTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = abs(req)'
        spec.parse()

        out = spec.update(0, [['req', 1.1]])
        self.assertEqual(1.1, out, 'Abs reset assertion')

        out = spec.update(1, [['req', 2]])
        self.assertEqual(2, out, 'Abs reset assertion')

        spec.reset()

        out = spec.update(0, [['req', -3.3]])
        self.assertEqual(3.3, out, 'Abs reset assertion')


    def test_sqrt(self):
        spec = StlDiscreteTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = sqrt(abs(req))'
        spec.parse()

        out = spec.update(0, [['req', 1.1]])
        self.assertEqual(math.sqrt(1.1), out, 'Abs reset assertion')

        out = spec.update(1, [['req', 2]])
        self.assertEqual(math.sqrt(2), out, 'Abs reset assertion')

        spec.reset()

        out = spec.update(0, [['req', -3.3]])
        self.assertEqual(math.sqrt(3.3), out, 'Abs reset assertion')

    def test_exp(self):
        spec = StlDiscreteTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = exp(req)'
        spec.parse()

        out = spec.update(0, [['req', 1.1]])
        self.assertEqual(math.exp(1.1), out, 'Abs reset assertion')

        out = spec.update(1, [['req', 2]])
        self.assertEqual(math.exp(2), out, 'Abs reset assertion')

        spec.reset()

        out = spec.update(0, [['req', -3.3]])
        self.assertEqual(math.exp(-3.3), out, 'Abs reset assertion')

    def test_pow(self):
        spec = StlDiscreteTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = pow(2,req)'
        spec.parse()

        out = spec.update(0, [['req', 1.1]])
        self.assertEqual(math.pow(2,1.1), out, 'Abs reset assertion')

        out = spec.update(1, [['req', 2]])
        self.assertEqual(math.pow(2,2), out, 'Abs reset assertion')

        spec.reset()

        out = spec.update(0, [['req', -3.3]])
        self.assertEqual(math.pow(2,-3.3), out, 'Abs reset assertion')

    def test_addition(self):
        spec = StlDiscreteTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req + gnt'
        spec.parse()

        out = spec.update(0, [['req', 1.1], ['gnt', 2.2]])
        self.assertEqual(1.1 + 2.2, out, 'Addition reset assertion')

        out = spec.update(1, [['req', 2], ['gnt', -1]])
        self.assertEqual(2 - 1, out, 'Addition reset assertion')

        spec.reset()

        out = spec.update(0, [['req', 3.3], ['gnt', 4.3]])
        self.assertEqual(3.3 + 4.3, out, 'Addition reset assertion')

    def test_subtraction(self):
        spec = StlDiscreteTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req - gnt'
        spec.parse()

        out = spec.update(0, [['req', 1.1], ['gnt', 2.2]])
        self.assertEqual(1.1 - 2.2, out, 'Subtraction reset assertion')

        out = spec.update(1, [['req', 2], ['gnt', -1]])
        self.assertEqual(2 + 1, out, 'Subtraction reset assertion')

        spec.reset()

        out = spec.update(0, [['req', 3.3], ['gnt', 4.3]])
        self.assertEqual(3.3 - 4.3, out, 'Subtraction reset assertion')

    def test_multiplication(self):
        spec = StlDiscreteTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req * gnt'
        spec.parse()

        out = spec.update(0, [['req', 1.1], ['gnt', 2.2]])
        self.assertEqual(1.1 * 2.2, out, 'Multiplication reset assertion')

        out = spec.update(1, [['req', 2], ['gnt', -1]])
        self.assertEqual(2 * -1, out, 'Multiplication reset assertion')

        spec.reset()

        out = spec.update(0, [['req', 3.3], ['gnt', 4.3]])
        self.assertEqual(3.3 * 4.3, out, 'Multiplication reset assertion')

    def test_division(self):
        spec = StlDiscreteTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req / gnt'
        spec.parse()

        out = spec.update(0, [['req', 1.1], ['gnt', 2.2]])
        self.assertEqual(1.1 / 2.2, out, 'Division reset assertion')

        out = spec.update(1, [['req', 2], ['gnt', -1]])
        self.assertEqual(2 / -1, out, 'Division reset assertion')

        spec.reset()

        out = spec.update(0, [['req', 3.3], ['gnt', 4.3]])
        self.assertEqual(3.3 / 4.3, out, 'Division reset assertion')

    def test_predicate_leq(self):
        spec = StlDiscreteTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req <= gnt'
        spec.parse()

        out = spec.update(0, [['req', 1.1], ['gnt', 2.2]])
        self.assertEqual(2.2 - 1.1, out, 'Predicate <= reset assertion')

        out = spec.update(1, [['req', 2], ['gnt', -1]])
        self.assertEqual(-1 - 2, out, 'Predicate <= reset assertion')

        spec.reset()

        out = spec.update(0, [['req', 3.3], ['gnt', 4.3]])
        self.assertEqual(4.3 - 3.3, out, 'Predicate <= reset assertion')

    def test_predicate_less(self):
        spec = StlDiscreteTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req < gnt'
        spec.parse()

        out = spec.update(0, [['req', 1.1], ['gnt', 2.2]])
        self.assertEqual(2.2 - 1.1, out, 'Predicate < reset assertion')

        out = spec.update(1, [['req', 2], ['gnt', -1]])
        self.assertEqual(-1 - 2, out, 'Predicate < reset assertion')

        spec.reset()

        out = spec.update(0, [['req', 3.3], ['gnt', 4.3]])
        self.assertEqual(4.3 - 3.3, out, 'Predicate < reset assertion')

    def test_predicate_geq(self):
        spec = StlDiscreteTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req >= gnt'
        spec.parse()

        out = spec.update(0, [['req', 1.1], ['gnt', 2.2]])
        self.assertEqual(-2.2 + 1.1, out, 'Predicate >= reset assertion')

        out = spec.update(1, [['req', 2], ['gnt', -1]])
        self.assertEqual(1 + 2, out, 'Predicate >= reset assertion')

        spec.reset()

        out = spec.update(0, [['req', 3.3], ['gnt', 4.3]])
        self.assertEqual(-4.3 + 3.3, out, 'Predicate >= reset assertion')

    def test_predicate_greater(self):
        spec = StlDiscreteTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req >= gnt'
        spec.parse()

        out = spec.update(0, [['req', 1.1], ['gnt', 2.2]])
        self.assertEqual(-2.2 + 1.1, out, 'Predicate > reset assertion')

        out = spec.update(1, [['req', 2], ['gnt', -1]])
        self.assertEqual(1 + 2, out, 'Predicate > reset assertion')

        spec.reset()

        out = spec.update(0, [['req', 3.3], ['gnt', 4.3]])
        self.assertEqual(-4.3 + 3.3, out, 'Predicate > reset assertion')

    def test_predicate_eq(self):
        spec = StlDiscreteTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req == gnt'
        spec.parse()

        out = spec.update(0, [['req', 1.1], ['gnt', 2.2]])
        self.assertEqual(-(2.2 - 1.1), out, 'Predicate == reset assertion')

        out = spec.update(1, [['req', 2], ['gnt', -1]])
        self.assertEqual(-(1 + 2), out, 'Predicate == reset assertion')

        spec.reset()

        out = spec.update(0, [['req', 3.3], ['gnt', 4.3]])
        self.assertEqual(-4.3 + 3.3, out, 'Predicate == reset assertion')

    def test_predicate_neq(self):
        spec = StlDiscreteTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req !== gnt'
        spec.parse()

        out = spec.update(0, [['req', 1.1], ['gnt', 2.2]])
        self.assertEqual(2.2 - 1.1, out, 'Predicate == reset assertion')

        out = spec.update(1, [['req', 2], ['gnt', -1]])
        self.assertEqual(1 + 2, out, 'Predicate == reset assertion')

        spec.reset()

        out = spec.update(0, [['req', 3.3], ['gnt', 4.3]])
        self.assertEqual(4.3 - 3.3, out, 'Predicate == reset assertion')

    def test_not(self):
        spec = StlDiscreteTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = not(req)'
        spec.parse()

        out = spec.update(0, [['req', 1.1]])
        self.assertEqual(-1.1, out, 'Negation reset assertion')

        out = spec.update(1, [['req', 2]])
        self.assertEqual(-2, out, 'Negation reset assertion')

        spec.reset()

        out = spec.update(0, [['req', -3.3]])
        self.assertEqual(3.3, out, 'Negation reset assertion')

    def test_conjunction(self):
        spec = StlDiscreteTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req and gnt'
        spec.parse()

        out = spec.update(0, [['req', 1.1], ['gnt', 2.2]])
        self.assertEqual(1.1, out, 'And reset assertion')

        out = spec.update(1, [['req', 2], ['gnt', -1]])
        self.assertEqual(-1, out, 'And reset assertion')

        spec.reset()

        out = spec.update(0, [['req', 3.3], ['gnt', 4.3]])
        self.assertEqual(3.3, out, 'And reset assertion')

    def test_disjunction(self):
        spec = StlDiscreteTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req or gnt'
        spec.parse()

        out = spec.update(0, [['req', 1.1], ['gnt', 2.2]])
        self.assertEqual(2.2, out, 'Or reset assertion')

        out = spec.update(1, [['req', 2], ['gnt', -1]])
        self.assertEqual(2, out, 'Or reset assertion')

        spec.reset()

        out = spec.update(0, [['req', 3.3], ['gnt', 4.3]])
        self.assertEqual(4.3, out, 'Or reset assertion')

    def test_implication(self):
        spec = StlDiscreteTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req implies gnt'
        spec.parse()

        out = spec.update(0, [['req', 1.1], ['gnt', 2.2]])
        self.assertEqual(2.2, out, 'Implies reset assertion')

        out = spec.update(1, [['req', 2], ['gnt', -1]])
        self.assertEqual(-1, out, 'Implies reset assertion')

        spec.reset()

        out = spec.update(0, [['req', 3.3], ['gnt', 4.3]])
        self.assertEqual(4.3, out, 'Implies reset assertion')

    def test_iff(self):
        spec = StlDiscreteTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req iff gnt'
        spec.parse()

        out = spec.update(0, [['req', 1.1], ['gnt', 2.2]])
        self.assertEqual(1.1 - 2.2, out, 'Iff reset assertion')

        out = spec.update(1, [['req', 2], ['gnt', -1]])
        self.assertEqual(-1 - 2, out, 'Iff reset assertion')

        spec.reset()

        out = spec.update(0, [['req', 3.3], ['gnt', 4.3]])
        self.assertEqual(3.3 - 4.3, out, 'Iff reset assertion')

    def test_xor(self):
        spec = StlDiscreteTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req xor gnt'
        spec.parse()

        out = spec.update(0, [['req', 1.1], ['gnt', 2.2]])
        self.assertEqual(-(1.1 - 2.2), out, 'Xor reset assertion')

        out = spec.update(1, [['req', 2], ['gnt', -1]])
        self.assertEqual(1 + 2, out, 'Xor reset assertion')

        spec.reset()

        out = spec.update(0, [['req', 3.3], ['gnt', 4.3]])
        self.assertEqual(4.3 - 3.3, out, 'Xor reset assertion')

    def test_rise(self):
        spec = StlDiscreteTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = rise(req)'
        spec.parse()

        out = spec.update(0, [['req', 1.1]])
        self.assertEqual(1.1, out, 'Rise reset assertion')

        out = spec.update(1, [['req', 2]])
        self.assertEqual(-1.1, out, 'Rise reset assertion')

        spec.reset()

        out = spec.update(0, [['req', 4.3]])
        self.assertEqual(4.3, out, 'Rise reset assertion')

    def test_fall(self):
        spec = StlDiscreteTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = fall(req)'
        spec.parse()

        out = spec.update(0, [['req', 1.1]])
        self.assertEqual(-1.1, out, 'Fall reset assertion')

        out = spec.update(1, [['req', 2]])
        self.assertEqual(-2, out, 'Fall reset assertion')

        spec.reset()

        out = spec.update(0, [['req', -3]])
        self.assertEqual(3, out, 'Rise reset assertion')

    def test_prev(self):
        spec = StlDiscreteTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = prev(req)'
        spec.parse()

        out = spec.update(0, [['req', 1.1]])
        self.assertEqual(float("inf"), out, 'Fall reset assertion')

        out = spec.update(1, [['req', 2]])
        self.assertEqual(1.1, out, 'Fall reset assertion')

        spec.reset()

        out = spec.update(0, [['req', -3]])
        self.assertEqual(float("inf"), out, 'Rise reset assertion')

    def test_once(self):
        spec = StlDiscreteTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = once(req)'
        spec.parse()

        out = spec.update(0, [['req', 5]])
        self.assertEqual(5, out, 'Once reset assertion')

        out = spec.update(1, [['req', 2]])
        self.assertEqual(5, out, 'Once reset assertion')

        spec.reset()

        out = spec.update(0, [['req', 4.3]])
        self.assertEqual(4.3, out, 'Once reset assertion')

    def test_historically(self):
        spec = StlDiscreteTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = historically(req)'
        spec.parse()

        out = spec.update(0, [['req', 1.1]])
        self.assertEqual(1.1, out, 'Historically reset assertion')

        out = spec.update(1, [['req', 2]])
        self.assertEqual(1.1, out, 'Historically reset assertion')

        spec.reset()

        out = spec.update(0, [['req', 4.3]])
        self.assertEqual(4.3, out, 'Historically reset assertion')

    # def test_eventually(self):
    #     spec = StlDiscreteTimeSpecification()
    #     spec.declare_var('req', 'float')
    #     spec.declare_var('out', 'float')
    #     spec.spec = 'out = eventually(req)'
    #     spec.parse()
    #
    #     out = spec.update(0, [['req', 5]])
    #     self.assertEqual(5, out, 'Eventually reset assertion')
    #
    #     out = spec.update(1, [['req', 2]])
    #     self.assertEqual(5, out, 'Eventually reset assertion')
    #
    #     spec.reset()
    #
    #     out = spec.update(0, [['req', 4.3]])
    #     self.assertEqual(4.3, out, 'Eventually reset assertion')

    # def test_always(self):
    #     spec = StlDiscreteTimeSpecification()
    #     spec.declare_var('req', 'float')
    #     spec.declare_var('out', 'float')
    #     spec.spec = 'out = always(req)'
    #     spec.parse()
    #
    #     out = spec.update(0, [['req', 1.1]])
    #     self.assertEqual(1.1, out, 'Always reset assertion')
    #
    #     out = spec.update(1, [['req', 2]])
    #     self.assertEqual(1.1, out, 'Always reset assertion')
    #
    #     spec.reset()
    #
    #     out = spec.update(0, [['req', 4.3]])
    #     self.assertEqual(4.3, out, 'Always reset assertion')

    def test_since(self):
        spec = StlDiscreteTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req since gnt'
        spec.parse()

        out = spec.update(0, [['req', 1.1], ['gnt', 2.2]])
        self.assertEqual(2.2, out, 'Since reset assertion')

        out = spec.update(1, [['req', 2], ['gnt', -1]])
        self.assertEqual(2, out, 'Since reset assertion')

        spec.reset()

        out = spec.update(0, [['req', 3.3], ['gnt', 1.6]])
        self.assertEqual(1.6, out, 'Since reset assertion')


    def test_once_0_1(self):
        spec = StlDiscreteTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = once[0:1](req)'
        spec.parse()

        out = spec.update(0, [['req', 5]])
        self.assertEqual(5, out, 'Once [0,1] reset assertion')

        out = spec.update(1, [['req', 4.8]])
        self.assertEqual(5, out, 'Once [0,1] reset assertion')

        spec.reset()

        out = spec.update(0, [['req', 4.3]])
        self.assertEqual(4.3, out, 'Once [0,1] reset assertion')


    def test_historically_0_1(self):
        spec = StlDiscreteTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = historically[0:1](req)'
        spec.parse()

        out = spec.update(0, [['req', 1.1]])
        self.assertEqual(1.1, out, 'Historically [0,1] reset assertion')

        out = spec.update(1, [['req', 2]])
        self.assertEqual(1.1, out, 'Historically [0,1] reset assertion')

        spec.reset()

        out = spec.update(0, [['req', 4.3]])
        self.assertEqual(4.3, out, 'Historically [0,1] reset assertion')

    def test_since_0_1(self):
        spec = StlDiscreteTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req since[0:1] gnt'
        spec.parse()

        out = spec.update(0, [['req', 1.1], ['gnt', 2.2]])
        self.assertEqual(2.2, out, 'Since [0:1] reset assertion')

        out = spec.update(1, [['req', 2], ['gnt', -1]])
        self.assertEqual(2, out, 'Since [0:1] reset assertion')

        spec.reset()

        out = spec.update(0, [['req', 3.3], ['gnt', 1.6]])
        self.assertEqual(1.6, out, 'Since [0:1] reset assertion')

    def test_precedes_0_1(self):
        spec = StlDiscreteTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req until[0:1] gnt'
        spec.parse()
        spec.pastify()

        out = spec.update(0, [['req', 1.1], ['gnt', 2.2]])
        self.assertEqual(2.2, out, 'Precedes [0:1] reset assertion')

        out = spec.update(1, [['req', 2], ['gnt', -1]])
        self.assertEqual(2.2, out, 'Precedes [0:1] reset assertion')

        spec.reset()

        out = spec.update(0, [['req', 3.3], ['gnt', 1.6]])
        self.assertEqual(1.6, out, 'Precedes [0:1] reset assertion')

    if __name__ == '__main__':
        unittest.main()