import unittest
import rtamt

class TestSTLSamplingTimeUnits(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestSTLSamplingTimeUnits, self).__init__(*args, **kwargs)

    def test_example(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.name = 'STL Example specification'

        self.assertEqual(spec.name, 'STL Example specification', 'Spec name assertion')

        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')

        spec.spec = 'out = req'

        self.assertEqual(spec.spec, 'out = req', 'Spec assertion')
        self.assertEqual(spec.sampling_tolerance, 0.1, 'Spec sampling tolerance assertion')

        try:
            spec.parse()
            computed = spec.update(0, [['req', 2.2], ['gnt', 1]])
            self.assertEqual(2.2, computed, 'First computation')

            computed = spec.update(1.2, [['req', 4.2], ['gnt', -3.7]])
            self.assertEqual(4.2, computed, 'First computation')
            self.assertEqual(1, spec.sampling_violation_counter, 'Violation counter')
        except rtamt.RTAMTException as err:
            print('STL Parse Exception: {}'.format(err))

if __name__ == '__main__':
    unittest.main()
