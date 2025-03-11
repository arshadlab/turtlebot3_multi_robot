import unittest

from rtamt.syntax.ast.parser.ltl.specification_parser import LtlAst
from rtamt.syntax.ast.parser.stl.specification_parser import StlAst
from rtamt.pastifier.stl.pastifier import StlPastifier


class TestStlPastification(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestStlPastification, self).__init__(*args, **kwargs)

    def test_constant(self):
        ast = StlAst()
        ast.spec = '2'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual(str(2.0), ast.specs[0].name, 'Const pastification assertion')

    def test_variable_1(self):
        ast = StlAst()
        ast.declare_var('req', 'float')
        ast.spec = 'req'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('req', ast.specs[0].name, 'Var pastification assertion')

        # ast = StlAst()
        # ast.declare_var('myvar.req.val', 'float')
        # ast.spec = 'myvar.req.val'
        # ast.parse()

        # pastifier = StlPastifier()
        # ast = pastifier.pastify(ast)

        # self.assertEqual('myvar.req.val', ast.specs[0].name, 'Var pastification assertion')

    def test_previous_1(self):
        ast = StlAst()
        ast.declare_var('req', 'float')
        ast.spec = 'prev req'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('previous(req)', ast.specs[0].name, 'Prev pastification assertion')


    def test_next_1(self):
        ast = StlAst()
        ast.declare_var('req', 'float')
        ast.spec = 'X req'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('req', ast.specs[0].name, 'Next pastification assertion')

    def test_abs_1(self):
        ast = StlAst()
        ast.declare_var('req', 'float')
        ast.spec = 'abs(req)'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('abs(req)', ast.specs[0].name, 'Abs pastification assertion')

    def test_sqrt_1(self):
        ast = StlAst()
        ast.declare_var('req', 'float')
        ast.spec = 'sqrt(req)'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('sqrt(req)', ast.specs[0].name, 'Sqrt pastification assertion')


    def test_exp(self):
        ast = LtlAst()
        ast.declare_var('req', 'float')
        ast.spec = 'exp(req)'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('exp(req)', ast.specs[0].name, 'Exp pastification assertion')

    def test_pow(self):
        ast = LtlAst()
        ast.declare_var('req', 'float')
        ast.declare_var('gnt', 'float')
        ast.spec = 'pow(req, gnt)'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('pow(req,gnt)', ast.specs[0].name, 'Pow pastification assertion')

    def test_addition(self):
        ast = LtlAst()
        ast.declare_var('req', 'float')
        ast.declare_var('gnt', 'float')
        ast.spec = 'req + gnt'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('(req)+(gnt)', ast.specs[0].name, 'Addition pastification assertion')

    def test_subtraction(self):
        ast = LtlAst()
        ast.declare_var('req', 'float')
        ast.declare_var('gnt', 'float')
        ast.spec = 'req - gnt'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('(req)-(gnt)', ast.specs[0].name, 'Subtraction pastification assertion')

    def test_multiplication(self):
        ast = LtlAst()
        ast.declare_var('req', 'float')
        ast.declare_var('gnt', 'float')
        ast.spec = 'req * gnt'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('(req)*(gnt)', ast.specs[0].name, 'Multiplication pastification assertion')

    def test_division(self):
        ast = LtlAst()
        ast.declare_var('req', 'float')
        ast.declare_var('gnt', 'float')
        ast.spec = 'req / gnt'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('(req)/(gnt)', ast.specs[0].name, 'Div pastification assertion')

    def test_predicate_leq_1(self):
        ast = LtlAst()
        ast.declare_var('req', 'float')
        ast.declare_var('gnt', 'float')
        ast.spec = 'req <= gnt'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('(req)<=(gnt)', ast.specs[0].name, 'LEQ pastification assertion')

    def test_predicate_less(self):
        ast = LtlAst()
        ast.declare_var('req', 'float')
        ast.declare_var('gnt', 'float')
        ast.spec = 'req < gnt'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('(req)<(gnt)', ast.specs[0].name, 'LESS pastification assertion')

    def test_predicate_geq(self):
        ast = LtlAst()
        ast.declare_var('req', 'float')
        ast.declare_var('gnt', 'float')
        ast.spec = 'req >= gnt'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('(req)>=(gnt)', ast.specs[0].name, 'GEQ pastification assertion')

    def test_predicate_greater(self):
        ast = LtlAst()
        ast.declare_var('req', 'float')
        ast.declare_var('gnt', 'float')
        ast.spec = 'req > gnt'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('(req)>(gnt)', ast.specs[0].name, 'GREATER pastification assertion')

    def test_predicate_eq(self):
        ast = LtlAst()
        ast.declare_var('req', 'float')
        ast.declare_var('gnt', 'float')
        ast.spec = 'req == gnt'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('(req)==(gnt)', ast.specs[0].name, 'EQ pastification assertion')

    def test_predicate_neq(self):
        ast = LtlAst()
        ast.declare_var('req', 'float')
        ast.declare_var('gnt', 'float')
        ast.spec = 'req !== gnt'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('(req)!=(gnt)', ast.specs[0].name, 'NEQ pastification assertion')


    def test_not(self):
        ast = StlAst()
        ast.declare_var('req', 'float')
        ast.spec = 'not req'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('not(req)', ast.specs[0].name, 'Not pastification assertion')


    def test_conjunction(self):
        ast = LtlAst()
        ast.declare_var('req', 'float')
        ast.declare_var('gnt', 'float')
        ast.spec = 'req and gnt'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('(req)and(gnt)', ast.specs[0].name, 'Conjunction pastification assertion')

    def test_disjunction(self):
        ast = LtlAst()
        ast.declare_var('req', 'float')
        ast.declare_var('gnt', 'float')
        ast.spec = 'req or gnt'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('(req)or(gnt)', ast.specs[0].name, 'Disjunction pastification assertion')

    def test_implication(self):
        ast = LtlAst()
        ast.declare_var('req', 'float')
        ast.declare_var('gnt', 'float')
        ast.spec = 'req implies gnt'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('(req)->(gnt)', ast.specs[0].name, '-> pastification assertion')

    def test_iff(self):
        ast = LtlAst()
        ast.declare_var('req', 'float')
        ast.declare_var('gnt', 'float')
        ast.spec = 'req iff gnt'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('(req)<->(gnt)', ast.specs[0].name, 'Iff pastification assertion')

    def test_xor(self):
        ast = LtlAst()
        ast.declare_var('req', 'float')
        ast.declare_var('gnt', 'float')
        ast.spec = 'req xor gnt'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('(req)xor(gnt)', ast.specs[0].name, 'Xor pastification assertion')

    def test_rise(self):
        ast = StlAst()
        ast.declare_var('req', 'float')
        ast.spec = 'rise(req)'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('rise(req)', ast.specs[0].name, 'Rise pastification assertion')

    def test_fall(self):
        ast = StlAst()
        ast.declare_var('req', 'float')
        ast.spec = 'fall(req)'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('fall(req)', ast.specs[0].name, 'Fall pastification assertion')

    def test_once(self):
        ast = StlAst()
        ast.declare_var('req', 'float')
        ast.spec = 'O req'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('once(req)', ast.specs[0].name, 'Once pastification assertion')


    def test_historically(self):
        ast = StlAst()
        ast.declare_var('req', 'float')
        ast.spec = 'H req'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('historically(req)', ast.specs[0].name, 'Historically pastification assertion')

    def test_since(self):
        ast = LtlAst()
        ast.declare_var('req', 'float')
        ast.declare_var('gnt', 'float')
        ast.spec = 'req since gnt'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('(req)since(gnt)', ast.specs[0].name, 'Since pastification assertion')

    def test_once_0_1(self):
        ast = StlAst()
        ast.declare_var('req', 'float')
        ast.spec = 'O[0,1] req'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('once[0,1](req)', ast.specs[0].name, 'Once pastification assertion')

    def test_historically_0_1(self):
        ast = StlAst()
        ast.declare_var('req', 'float')
        ast.spec = 'H[0,1] req'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('historically[0,1](req)', ast.specs[0].name, 'Historically pastification assertion')

    def test_eventually_0_1(self):
        ast = StlAst()
        ast.declare_var('req', 'float')
        ast.spec = 'F[0,1] req'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('once[0,1](req)', ast.specs[0].name, 'Eventually pastification assertion')

    def test_always_0_1(self):
        ast = StlAst()
        ast.declare_var('req', 'float')
        ast.spec = 'always[0,1] req'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('historically[0,1](req)', ast.specs[0].name, 'Always pastification assertion')

    def test_until_0_1(self):
        ast = StlAst()
        ast.declare_var('req', 'float')
        ast.declare_var('gnt', 'float')
        ast.spec = 'req until[0,1] gnt'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('(req)precedes[0,1](gnt)', ast.specs[0].name, 'Until pastification assertion')

    def test_since_0_1(self):
        ast = StlAst()
        ast.declare_var('req', 'float')
        ast.declare_var('gnt', 'float')
        ast.spec = 'req since[0,1] gnt'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('(req)since[0,1](gnt)', ast.specs[0].name, 'Since pastification assertion')

    def test_complex_past_1(self):
        ast = StlAst()
        ast.declare_var('req', 'float')
        ast.declare_var('gnt', 'float')
        ast.spec = 'rise(req) since[2,6] (once[1,2]historically(gnt))'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('(rise(req))since[2,6](once[1,2](historically(gnt)))', ast.specs[0].name, 'Complex pastification assertion')

    def test_complex_bounded_future_2(self):
        ast = StlAst()
        ast.declare_var('req', 'float')
        ast.declare_var('gnt', 'float')
        ast.declare_var('ack', 'float')
        ast.spec = '(req until[1,2] gnt) -> (eventually[0,6] ack)'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('((once[4,4](req))precedes[1,2](once[4,4](gnt)))->(once[0,6](ack))', ast.specs[0].name, 'Complex pastification assertion')

    def test_complex_mixed_1(self):
        ast = StlAst()
        ast.declare_var('req', 'float')
        ast.declare_var('gnt', 'float')
        ast.spec = '(eventually[5,6](req)) -> (eventually[3,3] once[1,2](gnt))'
        ast.parse()

        pastifier = StlPastifier()
        ast = pastifier.pastify(ast)

        self.assertEqual('(once[0,1](req))->(once[1,2](once[3,3](gnt)))', ast.specs[0].name, 'Complex pastification assertion')

    if __name__ == '__main__':
        unittest.main()