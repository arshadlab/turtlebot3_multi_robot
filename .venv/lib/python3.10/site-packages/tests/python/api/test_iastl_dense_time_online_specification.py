import unittest
import rtamt

class TestIastlDenseTimeOnlineSpecification(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(TestIastlDenseTimeOnlineSpecification, self).__init__(*args, **kwargs)

    def test_output_robustness(self):
        spec = rtamt.StlDenseTimeSpecification(semantics=rtamt.Semantics.OUTPUT_ROBUSTNESS)
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.set_var_io_type('req', 'input')
        spec.set_var_io_type('gnt', 'output')
        spec.spec = 'out = (req >= 3) implies (gnt >= 0)'

        spec.parse()

        left = [[0, 100], [1, -1], [2, -2], [3, 5], [4, -1]]
        right = [[0, 20], [1, -2], [2, 10], [3, 4], [4, -1]]

        out = spec.update(['req', left], ['gnt', right])
        expected = [[0, 20], [1, float("inf")]]

        self.assertEqual(out, expected, "output robustness")

    def test_input_vacuity(self):
        spec = rtamt.StlDenseTimeSpecification(semantics=rtamt.Semantics.INPUT_VACUITY)
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.set_var_io_type('req', 'input')
        spec.set_var_io_type('gnt', 'output')
        spec.spec = 'out = (req >= 3) implies (gnt >= 0)'

        spec.parse()

        left = [[0, 100], [1, -1], [2, -2], [3, 5], [4, -1]]
        right = [[0, 20], [1, -2], [2, 10], [3, 4], [4, -1]]

        out = spec.update(['req', left], ['gnt', right])
        expected = [[0, 0], [1, 4], [2, 5]]

        self.assertEqual(out, expected, "input vacuity")

    def test_input_robustness(self):
        spec = rtamt.StlDenseTimeSpecification(semantics=rtamt.Semantics.INPUT_ROBUSTNESS)
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.set_var_io_type('req', 'output')
        spec.set_var_io_type('gnt', 'input')
        spec.spec = 'out = (req >= 3) implies (gnt >= 0)'

        spec.parse()

        left = [[0, 100], [1, -1], [2, -2], [3, 5], [4, -1]]
        right = [[0, 20], [1, -2], [2, 10], [3, 4], [4, -1]]

        out = spec.update(['req', left], ['gnt', right])
        expected = [[0, 20], [1, float("inf")]]

        self.assertEqual(out, expected, "input robustness")

    def test_output_vacuity(self):
        spec = rtamt.StlDenseTimeSpecification(semantics=rtamt.Semantics.OUTPUT_VACUITY)
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.set_var_io_type('req', 'output')
        spec.set_var_io_type('gnt', 'input')
        spec.spec = 'out = (req >= 3) implies (gnt >= 0)'

        spec.parse()

        left = [[0, 100], [1, -1], [2, -2], [3, 5], [4, -1]]
        right = [[0, 20], [1, -2], [2, 10], [3, 4], [4, -1]]

        out = spec.update(['req', left], ['gnt', right])
        expected = [[0, 0], [1, 4], [2, 5]]

        self.assertEqual(out, expected, "output vacuity")

if __name__ == '__main__':
    unittest.main()