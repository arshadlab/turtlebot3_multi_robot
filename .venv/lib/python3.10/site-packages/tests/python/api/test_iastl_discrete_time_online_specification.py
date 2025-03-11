import unittest
import rtamt

class TestIastlDiscreteTimeOnlineSpecification(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(TestIastlDiscreteTimeOnlineSpecification, self).__init__(*args, **kwargs)
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

    def test_output_robustness(self):
        spec = rtamt.StlDiscreteTimeSpecification(semantics=rtamt.Semantics.OUTPUT_ROBUSTNESS)
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.set_var_io_type('req', 'input')
        spec.set_var_io_type('gnt', 'output')
        spec.spec = 'out = (req >= 3) implies (gnt >= 0)'

        spec.parse()

        out1 = spec.update(0, [('req', self.left1), ('gnt', self.right1)])
        out2 = spec.update(1, [('req', self.left2), ('gnt', self.right2)])
        out3 = spec.update(2, [('req', self.left3), ('gnt', self.right3)])
        out4 = spec.update(3, [('req', self.left4), ('gnt', self.right4)])
        out5 = spec.update(4, [('req', self.left5), ('gnt', self.right5)])

        self.assertEqual(out1, 20, "input 1")
        self.assertEqual(out2, float("inf"), "input 2")
        self.assertEqual(out3, float("inf"), "input 3")
        self.assertEqual(out4, 4, "input 4")
        self.assertEqual(out5, float("inf"), "input 5")

    def test_input_vacuity(self):
        spec = rtamt.StlDiscreteTimeSpecification(semantics=rtamt.Semantics.INPUT_VACUITY)
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.set_var_io_type('req', 'input')
        spec.set_var_io_type('gnt', 'output')
        spec.spec = 'out = (req >= 3) implies (gnt >= 0)'

        spec.parse()

        out1 = spec.update(0, [('req', self.left1), ('gnt', self.right1)])
        out2 = spec.update(1, [('req', self.left2), ('gnt', self.right2)])
        out3 = spec.update(2, [('req', self.left3), ('gnt', self.right3)])
        out4 = spec.update(3, [('req', self.left4), ('gnt', self.right4)])
        out5 = spec.update(4, [('req', self.left5), ('gnt', self.right5)])

        self.assertEqual(out1, 0, "input 1")
        self.assertEqual(out2, 4, "input 2")
        self.assertEqual(out3, 5, "input 3")
        self.assertEqual(out4, 0, "input 4")
        self.assertEqual(out5, 4, "input 5")

    def test_input_robustness(self):
        spec = rtamt.StlDiscreteTimeSpecification(semantics=rtamt.Semantics.INPUT_ROBUSTNESS)
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.set_var_io_type('req', 'output')
        spec.set_var_io_type('gnt', 'input')
        spec.spec = 'out = (req >= 3) implies (gnt >= 0)'

        spec.parse()

        out1 = spec.update(0, [('req', self.left1), ('gnt', self.right1)])
        out2 = spec.update(1, [('req', self.left2), ('gnt', self.right2)])
        out3 = spec.update(2, [('req', self.left3), ('gnt', self.right3)])
        out4 = spec.update(3, [('req', self.left4), ('gnt', self.right4)])
        out5 = spec.update(4, [('req', self.left5), ('gnt', self.right5)])

        self.assertEqual(out1, 20, "input 1")
        self.assertEqual(out2, float("inf"), "input 2")
        self.assertEqual(out3, float("inf"), "input 3")
        self.assertEqual(out4, 4, "input 4")
        self.assertEqual(out5, float("inf"), "input 5")

    def test_output_vacuity(self):
        spec = rtamt.StlDiscreteTimeSpecification(semantics=rtamt.Semantics.OUTPUT_VACUITY)
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.set_var_io_type('req', 'output')
        spec.set_var_io_type('gnt', 'input')
        spec.spec = 'out = (req >= 3) implies (gnt >= 0)'

        spec.parse()

        out1 = spec.update(0, [('req', self.left1), ('gnt', self.right1)])
        out2 = spec.update(1, [('req', self.left2), ('gnt', self.right2)])
        out3 = spec.update(2, [('req', self.left3), ('gnt', self.right3)])
        out4 = spec.update(3, [('req', self.left4), ('gnt', self.right4)])
        out5 = spec.update(4, [('req', self.left5), ('gnt', self.right5)])

        self.assertEqual(out1, 0, "input 1")
        self.assertEqual(out2, 4, "input 2")
        self.assertEqual(out3, 5, "input 3")
        self.assertEqual(out4, 0, "input 4")
        self.assertEqual(out5, 4, "input 5")

if __name__ == '__main__':
    unittest.main()