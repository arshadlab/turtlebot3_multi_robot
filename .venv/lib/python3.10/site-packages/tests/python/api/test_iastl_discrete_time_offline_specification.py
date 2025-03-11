import unittest
import rtamt

class TestIastlDiscreteTimeOfflineSpecification(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(TestIastlDiscreteTimeOfflineSpecification, self).__init__(*args, **kwargs)
        self.dataset = {
            'time': [0, 1, 2, 3, 4],
            'req': [100, -1, -2, 5, -1],
            'gnt': [20, -2, 10, 4, -1]
        }

    def test_output_robustness(self):
        spec = rtamt.StlDiscreteTimeSpecification(semantics=rtamt.Semantics.OUTPUT_ROBUSTNESS)
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.set_var_io_type('req', 'input')
        spec.set_var_io_type('gnt', 'output')
        spec.spec = 'out = (req >= 3) implies (gnt >= 0)'

        spec.parse()

        out = spec.evaluate(self.dataset)
        expected = [[0, 20], [1, float("inf")], [2, float("inf")], [3, 4], [4, float("inf")]]

        self.assertListEqual(out, expected, "output robustness")

    def test_input_vacuity(self):
        spec = rtamt.StlDiscreteTimeSpecification(semantics=rtamt.Semantics.INPUT_VACUITY)
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.set_var_io_type('req', 'input')
        spec.set_var_io_type('gnt', 'output')
        spec.spec = 'out = (req >= 3) implies (gnt >= 0)'

        spec.parse()

        out = spec.evaluate(self.dataset)
        expected = [[0, 0], [1, 4], [2, 5], [3, 0], [4, 4]]

        self.assertListEqual(out, expected, "input vacuity")

    def test_input_robustness(self):
        spec = rtamt.StlDiscreteTimeSpecification(semantics=rtamt.Semantics.INPUT_ROBUSTNESS)
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.set_var_io_type('req', 'output')
        spec.set_var_io_type('gnt', 'input')
        spec.spec = 'out = (req >= 3) implies (gnt >= 0)'

        spec.parse()

        out = spec.evaluate(self.dataset)
        expected = [[0, 20], [1, float("inf")], [2, float("inf")], [3, 4], [4, float("inf")]]

        self.assertListEqual(out, expected, "input robustness")

    def test_output_vacuity(self):
        spec = rtamt.StlDiscreteTimeSpecification(semantics=rtamt.Semantics.OUTPUT_VACUITY)
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.set_var_io_type('req', 'output')
        spec.set_var_io_type('gnt', 'input')
        spec.spec = 'out = (req >= 3) implies (gnt >= 0)'

        spec.parse()

        out = spec.evaluate(self.dataset)
        expected = [[0, 0], [1, 4], [2, 5], [3, 0], [4, 4]]

        self.assertListEqual(out, expected, "output vacuity")

if __name__ == '__main__':
    unittest.main()