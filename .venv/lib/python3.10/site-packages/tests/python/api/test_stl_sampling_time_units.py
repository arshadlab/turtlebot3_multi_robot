import unittest
import rtamt


class TestStlSamplingTimeUnits(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestStlSamplingTimeUnits, self).__init__(*args, **kwargs)

    def test_example(self):
        spec = rtamt.StlDiscreteTimeSpecification()
        spec.name = 'STL Example specification'

        self.assertEqual(spec.name, 'STL Example specification', 'Spec name assertion')

        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')

        spec.spec = 'out = req<=2 and gnt>=3'

        self.assertEqual(spec.spec, 'out = req<=2 and gnt>=3', 'Spec assertion')
        self.assertEqual(spec.sampling_tolerance, 0.1, 'Spec sampling tolerance assertion')

        try:
            spec.parse()
            computed = spec.update(0, [['req', 2.2], ['gnt', 1]])
            self.assertEqual(-2.0, computed, 'First computation')

            computed = spec.update(1.2, [['req', 4.2], ['gnt', -3.7]])
            self.assertEqual(-6.7, computed, 'First computation')
            self.assertEqual(1, spec.sampling_violation_counter, 'Violation counter')
        except rtamt.RTAMTException as err:
            print('STL Parse Exception: {}'.format(err))

    def test_wrong_tolerance(self):
        spec = rtamt.StlDiscreteTimeSpecification()
        with self.assertRaises(Exception):
            spec.set_sampling_period(1, 's', 1.5)

    def test_default_unit_default_sampling_unit_greater_tolerance_ok_samples(self):
        spec = rtamt.StlDiscreteTimeSpecification()
        spec.name = 'STL Example specification'

        self.assertEqual(spec.name, 'STL Example specification', 'Spec name assertion')

        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')

        spec.set_sampling_period(1, 's', 0.5)

        spec.spec = 'out = rise(req)'

        try:
            spec.parse()
            spec.update(0, [['req', 2.2], ['gnt', 1]])
            spec.update(1.11, [['req', 2.2], ['gnt', 1]])
            spec.update(1.99, [['req', 2.2], ['gnt', 1]])
            spec.update(3.38, [['req', 2.2], ['gnt', 1]])
            spec.update(4.39, [['req', 2.2], ['gnt', 1]])

            self.assertEqual(0, spec.sampling_violation_counter, 'Violation counter')
        except rtamt.RTAMTException as err:
            print('STL Parse Exception: {}'.format(err))


if __name__ == '__main__':
    unittest.main()
