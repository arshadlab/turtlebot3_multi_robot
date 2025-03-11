import unittest
import rtamt

class TestStlIntervalStyle(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestStlIntervalStyle, self).__init__(*args, **kwargs)

    def test_interval_style(self):
        spec_comma = rtamt.StlDiscreteTimeSpecification()
        spec_comma.name = 'STL Comma Style'

        spec_column = rtamt.StlDiscreteTimeSpecification()
        spec_column.name = 'STL Column Style'

        spec_comma.declare_var('req', 'float')
        spec_comma.declare_var('gnt', 'float')
        spec_comma.declare_var('out', 'float')

        spec_column.declare_var('req', 'float')
        spec_column.declare_var('gnt', 'float')
        spec_column.declare_var('out', 'float')

        spec_comma.spec = 'out = once[0,2] (req<=2 and gnt>=3)'
        spec_column.spec = 'out = once[0:2] (req<=2 and gnt>=3)'

        try:
            spec_comma.parse()
            spec_column.parse()

            computed_comma = spec_comma.update(0, [['req', 2.2], ['gnt', 1]])
            computed_column = spec_column.update(0, [['req', 2.2], ['gnt', 1]])
            self.assertEqual(computed_comma, computed_column, 'First computation')

            computed_comma = spec_comma.update(1, [['req', 1.2], ['gnt', 3.1]])
            computed_column = spec_column.update(1, [['req', 1.2], ['gnt', 3.1]])
            self.assertEqual(computed_comma, computed_column, 'Second computation')

            computed_comma = spec_comma.update(2, [['req', 3.3], ['gnt', 1.4]])
            computed_column = spec_column.update(2, [['req', 3.3], ['gnt', 1.4]])
            self.assertEqual(computed_comma, computed_column, 'Second computation')

            computed_comma = spec_comma.update(3, [['req', 4.3], ['gnt', 4.4]])
            computed_column = spec_column.update(3, [['req', 4.3], ['gnt', 4.4]])
            self.assertEqual(computed_comma, computed_column, 'Second computation')

        except rtamt.RTAMTException as err:
            print('STL Parse Exception: {}'.format(err))

if __name__ == '__main__':
    unittest.main()
