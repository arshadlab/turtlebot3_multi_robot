import unittest
import math
import rtamt

class TestStlDenseTimeOnlineSpecification(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(TestStlDenseTimeOnlineSpecification, self).__init__(*args, **kwargs)

    def test_and_1(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req and gnt'

        spec.parse()

        in_data_1_1 = [[2, 2], [3.3, 3], [5.7, 4]]
        in_data_2_1 = [[2.5, 5], [4.7, 6]]

        in_data_1_2 = []
        in_data_2_2 = [[5.7, 1]]

        out_expected_1 = [[2.5, 2], [3.3, 3]]
        out_expected_2 = [[4.7, 3]]

        out_computed_1 = spec.update(['req', in_data_1_1], ['gnt', in_data_2_1])
        out_computed_2 = spec.update(['req', in_data_1_2], ['gnt', in_data_2_2])

        self.assertListEqual(out_expected_1, out_computed_1,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected_1, out_computed_1))

        self.assertListEqual(out_expected_2, out_computed_2,
                              "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_2, out_computed_2))

    def test_and_2(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req and gnt'

        spec.parse()

        in_data_1_1 = [[2, 2]]
        in_data_2_1 = [[2, 1]]

        out_expected_1 = []
        out_computed_1 = spec.update(['req', in_data_1_1], ['gnt', in_data_2_1])

        self.assertListEqual(out_expected_1, out_computed_1,
                              "Problem with 2nd example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_1, out_computed_1))

        in_data_1_2 = [[3.3, 3]]
        in_data_2_2 = [[3.3, 5]]

        out_expected_2 = [[2, 1]]
        out_computed_2 = spec.update(['req', in_data_1_2], ['gnt', in_data_2_2])

        self.assertListEqual(out_expected_2, out_computed_2,
                              "Problem with 2nd example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_2, out_computed_2))

        in_data_1_3 = [[4.7, 5]]
        in_data_2_3 = [[4.7, 2]]

        out_expected_3 = [[3.3, 3]]
        out_computed_3 = spec.update(['req', in_data_1_3], ['gnt', in_data_2_3])

        self.assertListEqual(out_expected_3, out_computed_3,
                              "Problem with 2nd example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_3, out_computed_3))

    def test_and_3(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req and gnt'

        spec.parse()

        in_data_1_1 = [[1, 2], [4.1, 1], [5, 2], [6.1, 1]]
        in_data_2_1 = [[1.2, 1]]
        out_expected_1 = []
        out_computed_1 = spec.update(['req', in_data_1_1], ['gnt', in_data_2_1])

        self.assertListEqual(out_expected_1, out_computed_1,
                              "Problem with 6th example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_1, out_computed_1))

        in_data_1_2 = []
        in_data_2_2 = [[1.2, 1], [3.7, 3], [7.5, 2]]
        out_expected_2 = [[1.2, 1], [3.7, 2], [4.1, 1], [5, 2]]
        out_computed_2 = spec.update(['req', in_data_1_2], ['gnt', in_data_2_2])

        self.assertListEqual(out_expected_2, out_computed_2,
                              "Problem with 6th example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_2, out_computed_2))

        in_data_1_3 = [[6.7, 4], [9.9, 5]]
        in_data_2_3 = [[8.1, 6]]
        out_expected_3 = [[6.1, 1], [6.7, 3], [7.5, 2]]
        out_computed_3 = spec.update(['req', in_data_1_3], ['gnt', in_data_2_3])


        self.assertListEqual(out_expected_3, out_computed_3,
                              "Problem with 6th example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_3, out_computed_3))

    def test_or(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req or gnt'

        spec.parse()

        in_data_1_1 = [[1, 2], [4.1, 1], [5, 2], [6.1, 1]]
        in_data_2_1 = [[1.2, 1]]
        out_expected_1 = []
        out_computed_1 = spec.update(['req', in_data_1_1], ['gnt', in_data_2_1])

        self.assertListEqual(out_expected_1, out_computed_1,
                             "Problem with 6th example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected_1, out_computed_1))

        in_data_1_2 = []
        in_data_2_2 = [[3.7, 3], [7.5, 2]]
        out_expected_2 = [[1.2, 2], [3.7, 3]]
        out_computed_2 = spec.update(['req', in_data_1_2], ['gnt', in_data_2_2])

        self.assertListEqual(out_expected_2, out_computed_2,
                             "Problem with 6th example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected_2, out_computed_2))

        in_data_1_3 = [[6.7, 4], [9.9, 5]]
        in_data_2_3 = [[8.1, 6]]
        out_expected_3 = [[6.1, 3], [6.7, 4]]
        out_computed_3 = spec.update(['req', in_data_1_3], ['gnt', in_data_2_3])

        self.assertListEqual(out_expected_3, out_computed_3,
                             "Problem with 6th example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected_3, out_computed_3))


    def test_iff(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req iff gnt'

        spec.parse()

        in_data_1 = [[1, 2], [4.1, 1], [5, 2], [6.1, 1], [6.7, 4], [9.9, 5]]
        in_data_2 = [[1.2, 1], [3.7, 3], [7.5, 2], [8.1, 6]]
        out_expected = [[1.2, -1], [4.1, -2], [5, -1], [6.1, -2], [6.7, -1], [7.5, -2]]
        out_computed = spec.update(['req', in_data_1], ['gnt', in_data_2])

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

    def test_xor(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req xor gnt'

        spec.parse()

        in_data_1 = [[1, 2], [4.1, 1], [5, 2], [6.1, 1], [6.7, 4], [9.9, 5]]
        in_data_2 = [[1.2, 1], [3.7, 3], [7.5, 2], [8.1, 6]]
        out_expected = [[1.2, 1], [4.1, 2], [5, 1], [6.1, 2], [6.7, 1], [7.5, 2]]
        out_computed = spec.update(['req', in_data_1], ['gnt', in_data_2])

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))


    def test_implies(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req -> gnt'

        spec.parse()

        in_data_1 = [[1, 2], [4.1, 1], [5, 2], [6.1, 1], [6.7, 4], [9.9, 5]]
        in_data_2 = [[1.2, 1], [3.7, 3], [7.5, 2], [8.1, 6]]
        out_expected = [[1.2, 1], [3.7, 3], [7.5, 2]]
        out_computed = spec.update(['req', in_data_1], ['gnt', in_data_2])

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

    def test_always_without_pastify(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = always(req)'

        spec.parse()

        self.assertRaises(rtamt.RTAMTException, spec.update, [])

    def test_always_with_pastify(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = always(req)'

        spec.parse()

        self.assertRaises(rtamt.RTAMTException, spec.pastify)


    def test_historically(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = historically req'

        spec.parse()
        in_data = [[5, 3], [5.3, 1], [5.75, 2]]
        out_expected = [[5, 3], [5.3, 1], [5.75, 1]]
        out_computed = spec.update(['req', in_data])

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

        in_data = [[6.5, 5], [6.75, 6], [9, 5], [9.25, 4], [10, 2]]
        out_expected = [[6.5, 1], [6.75, 1], [9, 1], [9.25, 1], [10, 1]]
        out_computed = spec.update(['req', in_data])

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

    def test_once(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = once req'

        spec.parse()

        in_data = [[5, 3], [5.3, 1], [5.75, 2]]
        out_expected = [[5, 3], [5.3, 3], [5.75, 3]]
        out_computed = spec.update(['req', in_data])

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

        in_data = [[6.5, 5], [6.75, 6]]
        out_expected = [[6.5, 5], [6.75, 6]]
        out_computed = spec.update(['req', in_data])

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

        in_data = [[9, 5], [9.25, 4], [10, 2]]
        out_expected = [[9, 6], [9.25, 6], [10, 6]]
        out_computed = spec.update(['req', in_data])

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

    def test_once_0_1(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = once[0,1] req'

        spec.parse()

        in_data_1 = [[5, 3], [5.3, 2], [5.75, 1]]
        out_expected_1 = [[5, 3]]
        out_computed_1 = spec.update(['req', in_data_1])

        self.assertListEqual(out_expected_1, out_computed_1,
                                  "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_1, out_computed_1))

        in_data_2 = [[6.5, 5], [6.75, 6], [9, 5]]
        out_expected_2 = [[5.75, 3], [6.3, 2], [6.5, 5], [6.75, 6]]
        out_computed_2 = spec.update(['req', in_data_2])

        self.assertListEqual(out_expected_2, out_computed_2,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected_2, out_computed_2))

        in_data_3 = [[9.25, 4], [10, 2]]
        out_expected_3 = [[9, 6]]
        out_computed_3 = spec.update(['req', in_data_3])

        self.assertListEqual(out_expected_3, out_computed_3,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected_3, out_computed_3))

    def test_once_1_3(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = once[1,3] req'

        spec.parse()

        in_data_1 = [[5, 3], [5.3, 2], [5.75, 1]]
        out_expected_1 = [[6, 3]]
        out_computed_1 = spec.update(['req', in_data_1])

        self.assertListEqual(out_expected_1, out_computed_1,
                            "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                            out_expected_1, out_computed_1))

        in_data_2 = [[6.5, 5], [6.75, 6], [9, 5]]
        out_expected_2 = [[6.75, 3], [7.5, 5], [7.75, 6]]
        out_computed_2 = spec.update(['req', in_data_2])

        self.assertListEqual(out_expected_2, out_computed_2,
                            "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                            out_expected_2, out_computed_2))

        in_data_3 = [[9.25, 4], [10, 2]]
        out_expected_3 = [[10, 6]]
        out_computed_3 = spec.update(['req', in_data_3])

        self.assertListEqual(out_expected_3, out_computed_3,
                            "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                out_expected_3, out_computed_3))


    def test_since_1(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req since gnt'

        spec.parse()
        in_data_1 = [[1, 2], [4.1, 1], [5, 2], [6.1, 1], [6.7, 4], [9.9, 5]]
        in_data_2 = [[1.2, 1], [3.7, 3], [7.5, 2], [8.1, 6]]
        out_expected = [[1.2, 1], [3.7, 2], [4.1, 1], [5, 2], [6.1, 1], [6.7, 3], [7.5, 3]]
        out_computed = spec.update(['req', in_data_1], ['gnt', in_data_2])

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 3d example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

    def test_since_2(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req since gnt'

        spec.parse()

        in_data_1 = [[1, 2], [4.1, 1]]
        in_data_2 = [[1.2, 1], [3.7, 3], [7.5, 2]]
        out_expected = [[1.2, 1], [3.7, 2]]
        out_computed = spec.update(['req', in_data_1], ['gnt', in_data_2])

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 3d example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

        in_data_1 = [[5, 2], [6.1, 1], [6.7, 4]]
        in_data_2 = []
        out_expected = [[4.1, 1], [5, 2], [6.1, 1]]
        out_computed = spec.update(['req', in_data_1], ['gnt', in_data_2])

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 3d example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

        in_data_1 = [[9.9, 5]]
        in_data_2 = [[8.1, 6]]
        out_expected = [[6.7, 3], [7.5, 3]]
        out_computed = spec.update(['req', in_data_1], ['gnt', in_data_2])

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 3d example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))


    def test_historically_1_2_1(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = historically[1,2] req'

        spec.parse()

        in_data = [[5, 3], [5.3, 1], [5.75, 2], [6.5, 5], [6.75, 6], [9, 5], [9.25, 4], [10, 2]]
        out_expected = [[6, 3], [6.3, 1], [7.75, 2], [8.5, 5], [8.75, 6], [10, 5], [10.25, 4]]
        out_computed = spec.update(['req', in_data])

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                             out_expected, out_computed))

    def test_historically_1_2_2(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = historically[1,2] req'

        spec.parse()

        in_data_1 = [[5, 3], [5.3, 2], [5.75, 1]]
        out_expected_1 = [[6, 3], [6.3, 2]]
        out_computed_1 = spec.update(['req', in_data_1])

        self.assertListEqual(out_expected_1, out_computed_1,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected_1, out_computed_1))

        in_data_2 = [[6.5, 5], [6.75, 6], [9, 5]]
        out_expected_2 = [[6.75, 1], [8.5, 5], [8.75, 6]]
        out_computed_2 = spec.update(['req', in_data_2])

        self.assertListEqual(out_expected_2, out_computed_2,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected_2, out_computed_2))

        in_data_3 = [[9.25, 4], [10, 2]]
        out_expected_3 = [[10, 5], [10.25, 4]]
        out_computed_3 = spec.update(['req', in_data_3])

        self.assertListEqual(out_expected_3, out_computed_3,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected_3, out_computed_3))

    def test_addition(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req + gnt'

        spec.parse()
        in_data_1 = [[1, 2], [4.1, 1], [5, 2], [6.1, 1], [6.7, 4], [9.9, 5]]
        in_data_2 = [[1.2, 1], [3.7, 3], [7.5, 2], [8.1, 6]]
        out_expected = [[1.2, 3], [3.7, 5], [4.1, 4], [5, 5], [6.1, 4], [6.7, 7], [7.5, 6]]
        out_computed = spec.update(['req', in_data_1], ['gnt', in_data_2])

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

    def test_addition_2(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req + 2'

        spec.parse()
        in_data_1 = [[1, 2], [4.1, 1], [5, 2]]
        out_expected = [[1, 4], [4.1, 3]]
        out_computed = spec.update(['req', in_data_1])

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

    def test_subtraction(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req - gnt'

        spec.parse()

        in_data_1 = [[1, 2], [4.1, 1], [5, 2], [6.1, 1], [6.7, 4], [9.9, 5]]
        in_data_2 = [[1.2, 1], [3.7, 3], [7.5, 2], [8.1, 6]]
        out_expected = [[1.2, 1], [3.7, -1], [4.1, -2], [5, -1], [6.1, -2], [6.7, 1], [7.5, 2]]
        out_computed = spec.update(['req', in_data_1], ['gnt', in_data_2])

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

    def test_multiplication(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req * gnt'

        spec.parse()

        in_data_1 = [[1, 2], [4.1, 1], [5, 2], [6.1, 1], [6.7, 4], [9.9, 5]]
        in_data_2 = [[1.2, 1], [3.7, 3], [7.5, 2], [8.1, 6]]
        out_expected = [[1.2, 2], [3.7, 6], [4.1, 3], [5, 6], [6.1, 3], [6.7, 12], [7.5, 8]]
        out_computed = spec.update(['req', in_data_1], ['gnt', in_data_2])

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))


    def test_division(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req / gnt'

        spec.parse()
        in_data_1 = [[1, 2], [4.1, 1], [5, 2], [6.1, 1], [6.7, 4], [9.9, 5]]
        in_data_2 = [[1.2, 1], [3.7, 3], [7.5, 2], [8.1, 6]]
        out_expected = [[1.2, 2], [3.7, 2. / 3.], [4.1, 1. / 3.], [5, 2. / 3.], [6.1, 1. / 3.], [6.7, 4. / 3.],
                        [7.5, 2]]
        out_computed = spec.update(['req', in_data_1], ['gnt', in_data_2])

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

    def test_abs(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = abs(req)'

        spec.parse()

        in_data = [[5, 3], [5.3, -1], [5.75, 2], [6.5, -5], [6.75, 6], [9, 5], [9.25, 4], [10, 2]]
        out_expected = [[5, 3], [5.3, 1], [5.75, 2], [6.5, 5], [6.75, 6], [9, 5], [9.25, 4], [10, 2]]
        out_computed = spec.update(['req', in_data])

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

    def test_sqrt(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = sqrt(abs(req))'

        spec.parse()

        in_data = [[5, 3], [5.3, -1], [5.75, 2], [6.5, -5], [6.75, 6], [9, 5], [9.25, 4], [10, 2]]
        out_expected = [[5, math.sqrt(3)], [5.3, math.sqrt(1)], [5.75, math.sqrt(2)], [6.5, math.sqrt(5)], [6.75, math.sqrt(6)], [9, math.sqrt(5)], [9.25, math.sqrt(4)], [10, math.sqrt(2)]]
        out_computed = spec.update(['req', in_data])

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

    def test_exp(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = exp(req)'

        spec.parse()

        in_data = [[5, 3], [5.3, -1], [5.75, 2], [6.5, -5], [6.75, 6], [9, 5], [9.25, 4], [10, 2]]
        out_expected = [[5, math.exp(3)], [5.3, math.exp(-1)], [5.75, math.exp(2)], [6.5, math.exp(-5)], [6.75, math.exp(6)], [9, math.exp(5)], [9.25, math.exp(4)], [10, math.exp(2)]]
        out_computed = spec.update(['req', in_data])

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))

    def test_pow(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = pow(3,req)'

        spec.parse()

        in_data = [[5, 3], [5.3, -1], [5.75, 2], [6.5, -5], [6.75, 6], [9, 5], [9.25, 4], [10, 2]]
        out_expected = [[5, math.pow(3, 3)], [5.3, math.pow(3, -1)], [5.75, math.pow(3, 2)], [6.5, math.pow(3, -5)],
                        [6.75, math.pow(3, 6)], [9, math.pow(3, 5)], [9.25, math.pow(3, 4)]]
        out_computed = spec.update(['req', in_data])

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected, out_computed))


    def test_not(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = not req'

        spec.parse()

        in_data_1 = [[5, 3], [5.3, 1]]
        in_data_2 = [[5.75, 2], [6.5, 5], [6.75, 6], [9, 5], [9.25, 4]]
        in_data_3 = [[10, 2]]

        out_expected_1 = [[5, -3], [5.3, -1]]
        out_expected_2 = [[5.75, -2], [6.5, -5], [6.75, -6], [9, -5], [9.25, -4]]
        out_expected_3 = [[10, -2]]

        out_computed_1 = spec.update(['req', in_data_1])
        out_computed_2 = spec.update(['req', in_data_2])
        out_computed_3 = spec.update(['req', in_data_3])


        self.assertListEqual(out_expected_1, out_computed_1,
                              "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_1, out_computed_1))

        self.assertListEqual(out_expected_2, out_computed_2,
                         "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                             out_expected_2, out_computed_2))

        self.assertListEqual(out_expected_3, out_computed_3,
                              "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_3, out_computed_3))

    def test_until_without_pastify(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req until gnt'

        spec.parse()

        self.assertRaises(rtamt.RTAMTException, spec.update, [])

    def test_until_with_pastify(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req until gnt'

        spec.parse()

        self.assertRaises(rtamt.RTAMTException, spec.pastify)

    def test_until_0_1_without_pastify(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req until gnt'

        spec.parse()

        self.assertRaises(rtamt.RTAMTException, spec.update, [])

    def test_until_0_1_with_pastify(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req until gnt'

        spec.parse()

        self.assertRaises(rtamt.RTAMTException, spec.pastify)

    def test_next(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = next req'

        spec.parse()

        self.assertRaises(rtamt.RTAMTException, spec.update, ['req', [0.0, 3.0]])

    def test_prev(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = prev req'

        spec.parse()

        self.assertRaises(rtamt.RTAMTException, spec.update, ['req', [0.0, 3.0]])

    def test_rise(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = rise(req)'

        spec.parse()

        self.assertRaises(rtamt.RTAMTException, spec.update, ['req', [0.0, 3.0]])

    def test_fall(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = fall(req)'

        spec.parse()

        self.assertRaises(rtamt.RTAMTException, spec.update, ['req', [0.0, 3.0]])

    def test_always_0_1_without_pastify(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = always[0,1] req'

        spec.parse()

        self.assertRaises(rtamt.RTAMTException, spec.update, ['req', [0.0, 3.0]])

    def test_eventually_0_1_without_pastify(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = eventually[0,1] req'

        spec.parse()

        self.assertRaises(rtamt.RTAMTException, spec.update, ['req', [0.0, 3.0]])

    def test_eventually_0_1_with_pastify(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = eventually[0,1] req'

        spec.parse()
        spec.pastify()

        in_data_1 = [[5, 3], [5.3, 2], [5.75, 1]]
        out_expected_1 = [[5, 3]]
        out_computed_1 = spec.update(['req', in_data_1])

        self.assertListEqual(out_expected_1, out_computed_1,
                                  "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_1, out_computed_1))

        in_data_2 = [[6.5, 5], [6.75, 6], [9, 5]]
        out_expected_2 = [[5.75, 3], [6.3, 2], [6.5, 5], [6.75, 6]]
        out_computed_2 = spec.update(['req', in_data_2])

        self.assertListEqual(out_expected_2, out_computed_2,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected_2, out_computed_2))

        in_data_3 = [[9.25, 4], [10, 2]]
        out_expected_3 = [[9, 6]]
        out_computed_3 = spec.update(['req', in_data_3])

        self.assertListEqual(out_expected_3, out_computed_3,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                 out_expected_3, out_computed_3))

    def test_always_0_1_with_pastify(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = always[0,1] req'

        spec.parse()
        spec.pastify()

        in_data = [[5, 3], [5.3, 1], [5.75, 2], [6.5, 5], [6.75, 6], [9, 5], [9.25, 4], [10, 2]]
        out_expected = [[5, 3], [5.3, 1], [6.75, 2], [7.5, 5], [7.75, 6], [9, 5], [9.25, 4]]
        out_computed = spec.update(['req', in_data])

        self.assertListEqual(out_expected, out_computed,
                             "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                             out_expected, out_computed))


    def test_predicate_leq(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req <= 2'

        spec.parse()

        in_data_1 = [[5, 3], [5.3, 1]]
        in_data_2 = [[5.75, 2], [6.5, 5], [6.75, 6], [9, 5], [9.25, 4]]
        in_data_3 = [[10, 2]]

        out_expected_1 = [[5, -1]]
        out_expected_2 = [[5.3, 1], [5.75, 0], [6.5, -3], [6.75, -4], [9, -3]]
        out_expected_3 = [[9.25, -2]]

        out_computed_1 = spec.update(['req', in_data_1])
        out_computed_2 = spec.update(['req', in_data_2])
        out_computed_3 = spec.update(['req', in_data_3])


        self.assertListEqual(out_expected_1, out_computed_1,
                              "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_1, out_computed_1))

        self.assertListEqual(out_expected_2, out_computed_2,
                         "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                             out_expected_2, out_computed_2))

        self.assertListEqual(out_expected_3, out_computed_3,
                              "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_3, out_computed_3))

    def test_predicate_less(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req < 2'

        spec.parse()

        in_data_1 = [[5, 3], [5.3, 1]]
        in_data_2 = [[5.75, 2], [6.5, 5], [6.75, 6], [9, 5], [9.25, 4]]
        in_data_3 = [[10, 2]]

        out_expected_1 = [[5, -1]]
        out_expected_2 = [[5.3, 1], [5.75, 0], [6.5, -3], [6.75, -4], [9, -3]]
        out_expected_3 = [[9.25, -2]]

        out_computed_1 = spec.update(['req', in_data_1])
        out_computed_2 = spec.update(['req', in_data_2])
        out_computed_3 = spec.update(['req', in_data_3])


        self.assertListEqual(out_expected_1, out_computed_1,
                              "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_1, out_computed_1))

        self.assertListEqual(out_expected_2, out_computed_2,
                         "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                             out_expected_2, out_computed_2))

        self.assertListEqual(out_expected_3, out_computed_3,
                              "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_3, out_computed_3))

    def test_predicate_geq(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req >= 2'

        spec.parse()

        in_data_1 = [[5, 3], [5.3, 1]]
        in_data_2 = [[5.75, 2], [6.5, 5], [6.75, 6], [9, 5], [9.25, 4]]
        in_data_3 = [[10, 2]]

        out_expected_1 = [[5, 1]]
        out_expected_2 = [[5.3, -1], [5.75, 0], [6.5, 3], [6.75, 4], [9, 3]]
        out_expected_3 = [[9.25, 2]]

        out_computed_1 = spec.update(['req', in_data_1])
        out_computed_2 = spec.update(['req', in_data_2])
        out_computed_3 = spec.update(['req', in_data_3])


        self.assertListEqual(out_expected_1, out_computed_1,
                              "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_1, out_computed_1))

        self.assertListEqual(out_expected_2, out_computed_2,
                         "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                             out_expected_2, out_computed_2))

        self.assertListEqual(out_expected_3, out_computed_3,
                              "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_3, out_computed_3))

    def test_predicate_greater(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req > 2'

        spec.parse()

        in_data_1 = [[5, 3], [5.3, 1]]
        in_data_2 = [[5.75, 2], [6.5, 5], [6.75, 6], [9, 5], [9.25, 4]]
        in_data_3 = [[10, 2]]

        out_expected_1 = [[5, 1]]
        out_expected_2 = [[5.3, -1], [5.75, 0], [6.5, 3], [6.75, 4], [9, 3]]
        out_expected_3 = [[9.25, 2]]

        out_computed_1 = spec.update(['req', in_data_1])
        out_computed_2 = spec.update(['req', in_data_2])
        out_computed_3 = spec.update(['req', in_data_3])


        self.assertListEqual(out_expected_1, out_computed_1,
                              "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_1, out_computed_1))

        self.assertListEqual(out_expected_2, out_computed_2,
                         "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                             out_expected_2, out_computed_2))

        self.assertListEqual(out_expected_3, out_computed_3,
                              "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_3, out_computed_3))

    def test_predicate_eq(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req == 2'

        spec.parse()

        in_data_1 = [[5, 3], [5.3, 1]]
        in_data_2 = [[5.75, 2], [6.5, 5], [6.75, 6], [9, 5], [9.25, 4]]
        in_data_3 = [[10, 2]]

        out_expected_1 = [[5, -1]]
        out_expected_2 = [[5.3, -1], [5.75, 0], [6.5, -3], [6.75, -4], [9, -3]]
        out_expected_3 = [[9.25, -2]]

        out_computed_1 = spec.update(['req', in_data_1])
        out_computed_2 = spec.update(['req', in_data_2])
        out_computed_3 = spec.update(['req', in_data_3])


        self.assertListEqual(out_expected_1, out_computed_1,
                              "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_1, out_computed_1))

        self.assertListEqual(out_expected_2, out_computed_2,
                         "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                             out_expected_2, out_computed_2))

        self.assertListEqual(out_expected_3, out_computed_3,
                              "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_3, out_computed_3))

    def test_predicate_neq(self):
        spec = rtamt.StlDenseTimeSpecification()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req !== 2'

        spec.parse()

        in_data_1 = [[5, 3], [5.3, 1]]
        in_data_2 = [[5.75, 2], [6.5, 5], [6.75, 6], [9, 5], [9.25, 4]]
        in_data_3 = [[10, 2]]

        out_expected_1 = [[5, 1]]
        out_expected_2 = [[5.3, 1], [5.75, 0], [6.5, 3], [6.75, 4], [9, 3]]
        out_expected_3 = [[9.25, 2]]

        out_computed_1 = spec.update(['req', in_data_1])
        out_computed_2 = spec.update(['req', in_data_2])
        out_computed_3 = spec.update(['req', in_data_3])


        self.assertListEqual(out_expected_1, out_computed_1,
                              "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_1, out_computed_1))

        self.assertListEqual(out_expected_2, out_computed_2,
                         "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                             out_expected_2, out_computed_2))

        self.assertListEqual(out_expected_3, out_computed_3,
                              "Problem with 1st example:\nExpected output: %s\nComputed output: %s" % (
                                  out_expected_3, out_computed_3))







if __name__ == '__main__':
    unittest.main()