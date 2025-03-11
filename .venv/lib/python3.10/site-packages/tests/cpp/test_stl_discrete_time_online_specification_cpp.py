import unittest
import math
import rtamt

class TestStlDenseTimeOnlineSpecificationCpp(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(TestStlDenseTimeOnlineSpecificationCpp, self).__init__(*args, **kwargs)
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

    def test_constant(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('out', 'float')
        spec.spec = 'out = 5'

        spec.parse()
        out1 = spec.update(0, [])
        out2 = spec.update(1, [])

        self.assertEqual(out1, 5, "input 1")
        self.assertEqual(out2, 5, "input 2")

    def test_constant_2(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_const('c', 'int', 5)
        spec.declare_var('out', 'float')
        spec.spec = 'out = c'

        spec.parse()
        out1 = spec.update(0, [])
        out2 = spec.update(1, [])

        self.assertEqual(out1, 5, "input 1")
        self.assertEqual(out2, 5, "input 2")

    def test_addition(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req + gnt'

        spec.parse()

        out1 = spec.update(0, [('req', self.left1), ('gnt', self.right1)])
        out2 = spec.update(1, [('req', self.left2), ('gnt', self.right2)])
        out3 = spec.update(2, [('req', self.left3), ('gnt', self.right3)])
        out4 = spec.update(3, [('req', self.left4), ('gnt', self.right4)])
        out5 = spec.update(4, [('req', self.left5), ('gnt', self.right5)])

        self.assertEqual(out1, 120, "input 1")
        self.assertEqual(out2, -3, "input 2")
        self.assertEqual(out3, 8, "input 3")
        self.assertEqual(out4, 9, "input 4")
        self.assertEqual(out5, -2, "input 5")

    def test_subtraction(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req - gnt'

        spec.parse()

        out1 = spec.update(0, [('req', self.left1), ('gnt', self.right1)])
        out2 = spec.update(1, [('req', self.left2), ('gnt', self.right2)])
        out3 = spec.update(2, [('req', self.left3), ('gnt', self.right3)])
        out4 = spec.update(3, [('req', self.left4), ('gnt', self.right4)])
        out5 = spec.update(4, [('req', self.left5), ('gnt', self.right5)])

        self.assertEqual(out1, 80, "input 1")
        self.assertEqual(out2, 1, "input 2")
        self.assertEqual(out3, -12, "input 3")
        self.assertEqual(out4, 1, "input 4")
        self.assertEqual(out5, 0, "input 5")

    def test_multiplication(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req * gnt'

        spec.parse()

        out1 = spec.update(0, [('req', self.left1), ('gnt', self.right1)])
        out2 = spec.update(1, [('req', self.left2), ('gnt', self.right2)])
        out3 = spec.update(2, [('req', self.left3), ('gnt', self.right3)])
        out4 = spec.update(3, [('req', self.left4), ('gnt', self.right4)])
        out5 = spec.update(4, [('req', self.left5), ('gnt', self.right5)])

        self.assertEqual(out1, 2000, "input 1")
        self.assertEqual(out2, 2, "input 2")
        self.assertEqual(out3, -20, "input 3")
        self.assertEqual(out4, 20, "input 4")
        self.assertEqual(out5, 1, "input 5")

    def test_division(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req / gnt'

        spec.parse()

        out1 = spec.update(0, [('req', self.left1), ('gnt', self.right1)])
        out2 = spec.update(1, [('req', self.left2), ('gnt', self.right2)])
        out3 = spec.update(2, [('req', self.left3), ('gnt', self.right3)])
        out4 = spec.update(3, [('req', self.left4), ('gnt', self.right4)])
        out5 = spec.update(4, [('req', self.left5), ('gnt', self.right5)])

        self.assertEqual(out1, 100.0 / 20.0, "input 1")
        self.assertEqual(out2, -1.0 / -2.0, "input 2")
        self.assertEqual(out3, -2.0 / 10, "input 3")
        self.assertEqual(out4,  5.0 / 4.0, "input 4")
        self.assertEqual(out5, -1.0 / -1.0, "input 5")

    def test_abs(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = abs(req)'

        spec.parse()

        out1 = spec.update(0, [('req', self.left1)])
        out2 = spec.update(1, [('req', self.left2)])
        out3 = spec.update(2, [('req', self.left3)])
        out4 = spec.update(3, [('req', self.left4)])
        out5 = spec.update(4, [('req', self.left5)])

        self.assertEqual(out1, 100, "input 1")
        self.assertEqual(out2, 1, "input 2")
        self.assertEqual(out3, 2, "input 3")
        self.assertEqual(out4, 5, "input 4")
        self.assertEqual(out5, 1, "input 5")

    def test_sqrt(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = sqrt(req)'

        spec.parse()

        out1 = spec.update(0, [('req', 2)])
        out2 = spec.update(1, [('req', 3.3)])
        out3 = spec.update(2, [('req', 4.5)])
        out4 = spec.update(3, [('req', 0.1)])
        out5 = spec.update(4, [('req', 1)])

        self.assertEqual(out1, math.sqrt(2), "input 1")
        self.assertEqual(out2, math.sqrt(3.3), "input 2")
        self.assertEqual(out3, math.sqrt(4.5), "input 3")
        self.assertEqual(out4, math.sqrt(0.1), "input 4")
        self.assertEqual(out5, math.sqrt(1), "input 5")

    def test_previous(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = prev(req)'

        spec.parse()

        out1 = spec.update(0, [('req', self.left1)])
        out2 = spec.update(1, [('req', self.left2)])
        out3 = spec.update(2, [('req', self.left3)])
        out4 = spec.update(3, [('req', self.left4)])
        out5 = spec.update(4, [('req', self.left5)])

        self.assertEqual(out1, float("inf"), "input 1")
        self.assertEqual(out2, 100, "input 2")
        self.assertEqual(out3, -1, "input 3")
        self.assertEqual(out4, -2, "input 4")
        self.assertEqual(out5, 5, "input 5")

    def test_next_without_pastify(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = next(req)'

        spec.parse()

        self.assertRaises(rtamt.RTAMTException, spec.update, 0, [('req', self.left1)])

    def test_next_with_pastify(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = next(req)'

        spec.parse()
        spec.pastify()

        out1 = spec.update(0, [('req', self.left1)])
        out2 = spec.update(1, [('req', self.left2)])
        out3 = spec.update(2, [('req', self.left3)])
        out4 = spec.update(3, [('req', self.left4)])
        out5 = spec.update(4, [('req', self.left5)])

        self.assertEqual(out1, 100, "input 1")
        self.assertEqual(out2, -1, "input 2")
        self.assertEqual(out3, -2, "input 3")
        self.assertEqual(out4, 5, "input 4")
        self.assertEqual(out5, -1, "input 5")

    def test_and(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req and gnt'

        spec.parse()

        out1 = spec.update(0, [('req', self.left1), ('gnt', self.right1)])
        out2 = spec.update(1, [('req', self.left2), ('gnt', self.right2)])
        out3 = spec.update(2, [('req', self.left3), ('gnt', self.right3)])
        out4 = spec.update(3, [('req', self.left4), ('gnt', self.right4)])
        out5 = spec.update(4, [('req', self.left5), ('gnt', self.right5)])

        self.assertEqual(out1, 20, "input 1")
        self.assertEqual(out2, -2, "input 2")
        self.assertEqual(out3, -2, "input 3")
        self.assertEqual(out4, 4, "input 4")
        self.assertEqual(out5, -1, "input 5")

    def test_or(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req or gnt'

        spec.parse()

        out1 = spec.update(0, [('req', self.left1), ('gnt', self.right1)])
        out2 = spec.update(1, [('req', self.left2), ('gnt', self.right2)])
        out3 = spec.update(2, [('req', self.left3), ('gnt', self.right3)])
        out4 = spec.update(3, [('req', self.left4), ('gnt', self.right4)])
        out5 = spec.update(4, [('req', self.left5), ('gnt', self.right5)])

        self.assertEqual(out1, 100, "input 1")
        self.assertEqual(out2, -1, "input 2")
        self.assertEqual(out3, 10, "input 3")
        self.assertEqual(out4, 5, "input 4")
        self.assertEqual(out5, -1, "input 5")

    def test_iff(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req iff gnt'

        spec.parse()

        out1 = spec.update(0, [('req', self.left1), ('gnt', self.right1)])
        out2 = spec.update(1, [('req', self.left2), ('gnt', self.right2)])
        out3 = spec.update(2, [('req', self.left3), ('gnt', self.right3)])
        out4 = spec.update(3, [('req', self.left4), ('gnt', self.right4)])
        out5 = spec.update(4, [('req', self.left5), ('gnt', self.right5)])

        self.assertEqual(out1, -80, "input 1")
        self.assertEqual(out2, -1, "input 2")
        self.assertEqual(out3, -12, "input 3")
        self.assertEqual(out4, -1, "input 4")
        self.assertEqual(out5, 0, "input 5")

    def test_xor(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req xor gnt'

        spec.parse()

        out1 = spec.update(0, [('req', self.left1), ('gnt', self.right1)])
        out2 = spec.update(1, [('req', self.left2), ('gnt', self.right2)])
        out3 = spec.update(2, [('req', self.left3), ('gnt', self.right3)])
        out4 = spec.update(3, [('req', self.left4), ('gnt', self.right4)])
        out5 = spec.update(4, [('req', self.left5), ('gnt', self.right5)])

        self.assertEqual(out1, 80, "input 1")
        self.assertEqual(out2, 1, "input 2")
        self.assertEqual(out3, 12, "input 3")
        self.assertEqual(out4, 1, "input 4")
        self.assertEqual(out5, 0, "input 5")

    def test_implies(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req -> gnt'

        spec.parse()

        out1 = spec.update(0, [('req', self.left1), ('gnt', self.right1)])
        out2 = spec.update(1, [('req', self.left2), ('gnt', self.right2)])
        out3 = spec.update(2, [('req', self.left3), ('gnt', self.right3)])
        out4 = spec.update(3, [('req', self.left4), ('gnt', self.right4)])
        out5 = spec.update(4, [('req', self.left5), ('gnt', self.right5)])

        self.assertEqual(out1, 20, "input 1")
        self.assertEqual(out2, 1, "input 2")
        self.assertEqual(out3, 10, "input 3")
        self.assertEqual(out4, 4, "input 4")
        self.assertEqual(out5, 1, "input 5")

    def test_always_without_pastify(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = always(req)'

        spec.parse()

        self.assertRaises(rtamt.RTAMTException, spec.update, 0, [('req', self.left1)])

    def test_always_with_pastify(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = always(req)'

        spec.parse()

        self.assertRaises(rtamt.RTAMTException, spec.pastify)

    def test_eventually_without_pastify(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = eventually(req)'

        spec.parse()

        self.assertRaises(rtamt.RTAMTException, spec.update, 0, [('req', self.left1)])

    def test_eventually_with_pastify(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = eventually(req)'

        spec.parse()

        self.assertRaises(rtamt.RTAMTException, spec.pastify)

    def test_historically(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = historically(req)'

        spec.parse()

        out1 = spec.update(0, [('req', self.left1)])
        out2 = spec.update(1, [('req', self.left2)])
        out3 = spec.update(2, [('req', self.left3)])
        out4 = spec.update(3, [('req', self.left4)])
        out5 = spec.update(4, [('req', self.left5)])

        self.assertEqual(out1, 100, "input 1")
        self.assertEqual(out2, -1, "input 2")
        self.assertEqual(out3, -2, "input 3")
        self.assertEqual(out4, -2, "input 4")
        self.assertEqual(out5, -2, "input 5")

    def test_once(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = once(req)'

        spec.parse()

        out1 = spec.update(0, [('req', self.left1)])
        out2 = spec.update(1, [('req', self.left2)])
        out3 = spec.update(2, [('req', self.left3)])
        out4 = spec.update(3, [('req', self.left4)])
        out5 = spec.update(4, [('req', self.left5)])

        self.assertEqual(out1, 100, "input 1")
        self.assertEqual(out2, 100, "input 2")
        self.assertEqual(out3, 100, "input 3")
        self.assertEqual(out4, 100, "input 4")
        self.assertEqual(out5, 100, "input 5")

    def test_since(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req since gnt'

        spec.parse()

        out1 = spec.update(0, [('req', self.left1), ('gnt', self.right1)])
        out2 = spec.update(1, [('req', self.left2), ('gnt', self.right2)])
        out3 = spec.update(2, [('req', self.left3), ('gnt', self.right3)])
        out4 = spec.update(3, [('req', self.left4), ('gnt', self.right4)])
        out5 = spec.update(4, [('req', self.left5), ('gnt', self.right5)])

        self.assertEqual(out1, 20, "input 1")
        self.assertEqual(out2, -1, "input 2")
        self.assertEqual(out3, 10, "input 3")
        self.assertEqual(out4, 5, "input 4")
        self.assertEqual(out5, -1, "input 5")

    def test_until_without_pastify(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req until gnt'

        spec.parse()

        self.assertRaises(rtamt.STLNotImplementedException, spec.update, 0, [('req', self.left1), ('gnt', self.right1)])

    def test_until_with_pastify(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req until gnt'

        spec.parse()

        self.assertRaises(rtamt.RTAMTException, spec.pastify)

    def test_eventually_0_1_without_pastify(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = eventually[0,1](req)'

        spec.parse()

        self.assertRaises(rtamt.RTAMTException, spec.update, 0, [('req', self.left1)])

    def test_eventually_0_1_with_pastify(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = eventually[0,1](req)'

        spec.parse()
        spec.pastify()

        out1 = spec.update(0, [('req', self.left1)])
        out2 = spec.update(1, [('req', self.left2)])
        out3 = spec.update(2, [('req', self.left3)])
        out4 = spec.update(3, [('req', self.left4)])
        out5 = spec.update(4, [('req', self.left5)])

        self.assertEqual(out1, 100, "input 1")
        self.assertEqual(out2, 100, "input 2")
        self.assertEqual(out3, -1, "input 3")
        self.assertEqual(out4, 5, "input 4")
        self.assertEqual(out5, 5, "input 5")

    def test_once_0_1(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = once[0,1](req)'

        spec.parse()

        out1 = spec.update(0, [('req', self.left1)])
        out2 = spec.update(1, [('req', self.left2)])
        out3 = spec.update(2, [('req', self.left3)])
        out4 = spec.update(3, [('req', self.left4)])
        out5 = spec.update(4, [('req', self.left5)])

        self.assertEqual(out1, 100, "input 1")
        self.assertEqual(out2, 100, "input 2")
        self.assertEqual(out3, -1, "input 3")
        self.assertEqual(out4, 5, "input 4")
        self.assertEqual(out5, 5, "input 5")

    def test_once_1_2(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = once[1,2](req)'

        spec.parse()

        out1 = spec.update(0, [('req', self.left1)])
        out2 = spec.update(1, [('req', self.left2)])
        out3 = spec.update(2, [('req', self.left3)])
        out4 = spec.update(3, [('req', self.left4)])
        out5 = spec.update(4, [('req', self.left5)])

        self.assertEqual(out1, -float("inf"), "input 1")
        self.assertEqual(out2, 100, "input 2")
        self.assertEqual(out3, 100, "input 3")
        self.assertEqual(out4, -1, "input 4")
        self.assertEqual(out5, 5, "input 5")

    def test_always_0_1_without_pastify(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = always[0,1](req)'

        spec.parse()

        self.assertRaises(rtamt.RTAMTException, spec.update, 0, [('req', self.left1)])

    def test_always_0_1_with_pastify(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = always[0,1](req)'

        spec.parse()
        spec.pastify()

        out1 = spec.update(0, [('req', self.left1)])
        out2 = spec.update(1, [('req', self.left2)])
        out3 = spec.update(2, [('req', self.left3)])
        out4 = spec.update(3, [('req', self.left4)])
        out5 = spec.update(4, [('req', self.left5)])

        self.assertEqual(out1, 100, "input 1")
        self.assertEqual(out2, -1, "input 2")
        self.assertEqual(out3, -2, "input 3")
        self.assertEqual(out4, -2, "input 4")
        self.assertEqual(out5, -1, "input 5")

    def test_historically_0_1(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = historically[0,1](req)'

        spec.parse()

        out1 = spec.update(0, [('req', self.left1)])
        out2 = spec.update(1, [('req', self.left2)])
        out3 = spec.update(2, [('req', self.left3)])
        out4 = spec.update(3, [('req', self.left4)])
        out5 = spec.update(4, [('req', self.left5)])

        self.assertEqual(out1, 100, "input 1")
        self.assertEqual(out2, -1, "input 2")
        self.assertEqual(out3, -2, "input 3")
        self.assertEqual(out4, -2, "input 4")
        self.assertEqual(out5, -1, "input 5")

    def test_historically_1_2(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = historically[1,2](req)'

        spec.parse()

        out1 = spec.update(0, [('req', self.left1)])
        out2 = spec.update(1, [('req', self.left2)])
        out3 = spec.update(2, [('req', self.left3)])
        out4 = spec.update(3, [('req', self.left4)])
        out5 = spec.update(4, [('req', self.left5)])

        self.assertEqual(out1, float("inf"), "input 1")
        self.assertEqual(out2, 100, "input 2")
        self.assertEqual(out3, -1, "input 3")
        self.assertEqual(out4, -2, "input 4")
        self.assertEqual(out5, -2, "input 5")

    def test_until_0_1_without_pastify(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req until[0,1] gnt'

        spec.parse()

        self.assertRaises(rtamt.RTAMTException, spec.update, 0, [('req', self.left1), ('gnt', self.right1)])

    def test_until_1_2_with_pastify(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req until[1,2] gnt'

        spec.parse()
        spec.pastify()

        out1 = spec.update(0, [('req', self.left1), ('gnt', self.right1)])
        out2 = spec.update(1, [('req', self.left2), ('gnt', self.right2)])
        out3 = spec.update(2, [('req', self.left3), ('gnt', self.right3)])
        out4 = spec.update(3, [('req', self.left4), ('gnt', self.right4)])
        out5 = spec.update(4, [('req', self.left5), ('gnt', self.right5)])

        self.assertEqual(out1, 20, "input 1")
        self.assertEqual(out2, 20, "input 2")
        self.assertEqual(out3, -1, "input 3")
        self.assertEqual(out4, -1, "input 4")
        self.assertEqual(out5, -2, "input 5")

    def test_since_0_1(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req since[0,1] gnt'

        spec.parse()

        out1 = spec.update(0, [('req', self.left1), ('gnt', self.right1)])
        out2 = spec.update(1, [('req', self.left2), ('gnt', self.right2)])
        out3 = spec.update(2, [('req', self.left3), ('gnt', self.right3)])
        out4 = spec.update(3, [('req', self.left4), ('gnt', self.right4)])
        out5 = spec.update(4, [('req', self.left5), ('gnt', self.right5)])

        self.assertEqual(out1, 20, "input 1")
        self.assertEqual(out2, -1, "input 2")
        self.assertEqual(out3, 10, "input 3")
        self.assertEqual(out4, 5, "input 4")
        self.assertEqual(out5, -1, "input 5")

    def test_not(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = not(req)'

        spec.parse()

        out1 = spec.update(0, [('req', self.left1)])
        out2 = spec.update(1, [('req', self.left2)])
        out3 = spec.update(2, [('req', self.left3)])
        out4 = spec.update(3, [('req', self.left4)])
        out5 = spec.update(4, [('req', self.left5)])

        self.assertEqual(out1, -100, "input 1")
        self.assertEqual(out2, 1, "input 2")
        self.assertEqual(out3, 2, "input 3")
        self.assertEqual(out4, -5, "input 4")
        self.assertEqual(out5, 1, "input 5")

    def test_rise(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = rise(req)'

        spec.parse()

        out1 = spec.update(0, [('req', self.left1)])
        out2 = spec.update(1, [('req', self.left2)])
        out3 = spec.update(2, [('req', self.left3)])
        out4 = spec.update(3, [('req', self.left4)])
        out5 = spec.update(4, [('req', self.left5)])

        self.assertEqual(out1, 100, "input 1")
        self.assertEqual(out2, -100, "input 2")
        self.assertEqual(out3, -2, "input 3")
        self.assertEqual(out4, 2, "input 4")
        self.assertEqual(out5, -5, "input 5")

    def test_fall(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = fall(req)'

        spec.parse()

        out1 = spec.update(0, [('req', self.left1)])
        out2 = spec.update(1, [('req', self.left2)])
        out3 = spec.update(2, [('req', self.left3)])
        out4 = spec.update(3, [('req', self.left4)])
        out5 = spec.update(4, [('req', self.left5)])

        self.assertEqual(out1, -100, "input 1")
        self.assertEqual(out2, 1, "input 2")
        self.assertEqual(out3, -1, "input 3")
        self.assertEqual(out4, -5, "input 4")
        self.assertEqual(out5, 1, "input 5")


    def test_predicate_leq(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req <= gnt'

        spec.parse()

        out1 = spec.update(0, [('req', self.left1), ('gnt', self.right1)])
        out2 = spec.update(1, [('req', self.left2), ('gnt', self.right2)])
        out3 = spec.update(2, [('req', self.left3), ('gnt', self.right3)])
        out4 = spec.update(3, [('req', self.left4), ('gnt', self.right4)])
        out5 = spec.update(4, [('req', self.left5), ('gnt', self.right5)])

        self.assertEqual(out1, -80, "input 1")
        self.assertEqual(out2, -1, "input 2")
        self.assertEqual(out3, 12, "input 3")
        self.assertEqual(out4, -1, "input 4")
        self.assertEqual(out5, 0, "input 5")

    def test_predicate_less(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req < gnt'

        spec.parse()

        out1 = spec.update(0, [('req', self.left1), ('gnt', self.right1)])
        out2 = spec.update(1, [('req', self.left2), ('gnt', self.right2)])
        out3 = spec.update(2, [('req', self.left3), ('gnt', self.right3)])
        out4 = spec.update(3, [('req', self.left4), ('gnt', self.right4)])
        out5 = spec.update(4, [('req', self.left5), ('gnt', self.right5)])

        self.assertEqual(out1, -80, "input 1")
        self.assertEqual(out2, -1, "input 2")
        self.assertEqual(out3, 12, "input 3")
        self.assertEqual(out4, -1, "input 4")
        self.assertEqual(out5, 0, "input 5")

    def test_predicate_geq(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req >= gnt'

        spec.parse()

        out1 = spec.update(0, [('req', self.left1), ('gnt', self.right1)])
        out2 = spec.update(1, [('req', self.left2), ('gnt', self.right2)])
        out3 = spec.update(2, [('req', self.left3), ('gnt', self.right3)])
        out4 = spec.update(3, [('req', self.left4), ('gnt', self.right4)])
        out5 = spec.update(4, [('req', self.left5), ('gnt', self.right5)])

        self.assertEqual(out1, 80, "input 1")
        self.assertEqual(out2, 1, "input 2")
        self.assertEqual(out3, -12, "input 3")
        self.assertEqual(out4, 1, "input 4")
        self.assertEqual(out5, 0, "input 5")

    def test_predicate_greater(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req > gnt'

        spec.parse()

        out1 = spec.update(0, [('req', self.left1), ('gnt', self.right1)])
        out2 = spec.update(1, [('req', self.left2), ('gnt', self.right2)])
        out3 = spec.update(2, [('req', self.left3), ('gnt', self.right3)])
        out4 = spec.update(3, [('req', self.left4), ('gnt', self.right4)])
        out5 = spec.update(4, [('req', self.left5), ('gnt', self.right5)])

        self.assertEqual(out1, 80, "input 1")
        self.assertEqual(out2, 1, "input 2")
        self.assertEqual(out3, -12, "input 3")
        self.assertEqual(out4, 1, "input 4")
        self.assertEqual(out5, 0, "input 5")

    def test_predicate_eq(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req == gnt'

        spec.parse()

        out1 = spec.update(0, [('req', self.left1), ('gnt', self.right1)])
        out2 = spec.update(1, [('req', self.left2), ('gnt', self.right2)])
        out3 = spec.update(2, [('req', self.left3), ('gnt', self.right3)])
        out4 = spec.update(3, [('req', self.left4), ('gnt', self.right4)])
        out5 = spec.update(4, [('req', self.left5), ('gnt', self.right5)])

        self.assertEqual(out1, -80, "input 1")
        self.assertEqual(out2, -1, "input 2")
        self.assertEqual(out3, -12, "input 3")
        self.assertEqual(out4, -1, "input 4")
        self.assertEqual(out5, 0, "input 5")

    def test_predicate_neq(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
        spec.declare_var('req', 'float')
        spec.declare_var('gnt', 'float')
        spec.declare_var('out', 'float')
        spec.spec = 'out = req !== gnt'

        spec.parse()

        out1 = spec.update(0, [('req', self.left1), ('gnt', self.right1)])
        out2 = spec.update(1, [('req', self.left2), ('gnt', self.right2)])
        out3 = spec.update(2, [('req', self.left3), ('gnt', self.right3)])
        out4 = spec.update(3, [('req', self.left4), ('gnt', self.right4)])
        out5 = spec.update(4, [('req', self.left5), ('gnt', self.right5)])

        self.assertEqual(out1, 80, "input 1")
        self.assertEqual(out2, 1, "input 2")
        self.assertEqual(out3, 12, "input 3")
        self.assertEqual(out4, 1, "input 4")
        self.assertEqual(out5, 0, "input 5")

if __name__ == '__main__':
    unittest.main()