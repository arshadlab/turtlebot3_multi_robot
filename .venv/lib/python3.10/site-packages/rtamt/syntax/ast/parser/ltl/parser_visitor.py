
import logging
import operator

from antlr4 import *

from rtamt.antlr.parser.ltl.LtlParserVisitor import LtlParserVisitor

from rtamt.syntax.node.ltl.variable import Variable
from rtamt.syntax.node.ltl.predicate import Predicate
from rtamt.syntax.node.ltl.previous import Previous
from rtamt.syntax.node.ltl.next import Next
from rtamt.syntax.node.ltl.neg import Neg
from rtamt.syntax.node.ltl.until import Until
from rtamt.syntax.node.ltl.conjunction import Conjunction
from rtamt.syntax.node.ltl.disjunction import Disjunction
from rtamt.syntax.node.ltl.implies import Implies
from rtamt.syntax.node.ltl.iff import Iff
from rtamt.syntax.node.ltl.xor import Xor
from rtamt.syntax.node.ltl.always import Always
from rtamt.syntax.node.ltl.eventually import Eventually
from rtamt.syntax.node.ltl.once import Once
from rtamt.syntax.node.ltl.historically import Historically
from rtamt.syntax.node.ltl.since import Since
from rtamt.syntax.node.arithmetic.abs import Abs
from rtamt.syntax.node.arithmetic.sqrt import Sqrt
from rtamt.syntax.node.arithmetic.exp import Exp
from rtamt.syntax.node.arithmetic.pow import Pow
from rtamt.syntax.node.arithmetic.addition import Addition
from rtamt.syntax.node.arithmetic.subtraction import Subtraction
from rtamt.syntax.node.arithmetic.multiplication import Multiplication
from rtamt.syntax.node.arithmetic.division import Division
from rtamt.syntax.node.ltl.fall import Fall
from rtamt.syntax.node.ltl.rise import Rise
from rtamt.syntax.node.ltl.constant import Constant

from rtamt.exception.exception import RTAMTException


class LtlAstParserVisitor(LtlParserVisitor):

    def visitExprPredicate(self, ctx):
        child1 = self.visit(ctx.expression(0))
        child2 = self.visit(ctx.expression(1))
        op_type = self.str_to_op_type(ctx.comparisonOp().getText())
        node = Predicate(child1, child2, op_type)

        return node

    def visitExprId(self, ctx):
        id = ctx.Identifier().getText();

        # Identifier is a constant
        if id in self.const_val_dict:
            val = self.const_val_dict[id]
            node = Constant(float(val))
        # Identifier is either an input variable or a sub-formula
        elif id in self.var_subspec_dict:
                node = self.var_subspec_dict[id]
                return node
        else:
            id_tokens = id.split('.')
            id_head = id_tokens[0]
            id_tokens.pop(0)
            id_tail = '.'.join(id_tokens)

            try:
                var = self.create_var_from_name(id_head)
                if (not id_tail):
                    if (not isinstance(var, (int, float))):
                        raise RTAMTException('Variable {} is not of type int or float'.format(id))
                else:
                    try:
                        value = operator.attrgetter(id_tail)(var)
                        if (not isinstance(value, (int, float))):
                            raise RTAMTException(
                                'The field {0} of the variable {1} is not of type int or float'.format(id, id_head))
                    except AttributeError as err:
                        raise RTAMTException(err)
            except KeyError:
                if id_tail:
                    raise RTAMTException('{0} refers to undeclared variable {1} of unknown type'.format(id, id_head))
                else:
                    var = float()
                    self.var_object_dict[id] = var
                    self.add_var(id)
                    logging.warning('The variable {} is not explicitely declared. It is implicitely declared as a '
                                'variable of type float'.format(id))

            var_io = self.var_io_dict[id_head]
            node = Variable(id_head, id_tail, var_io)

        return node


    def visitVariableDeclaration(self, ctx):
        # fetch the variable name, type and io signature
        var_name = ctx.Identifier().getText()
        var_type = ctx.domainType().getText()

        self.declare_var(var_name, var_type)
        self.var_io_dict[var_name] = 'output'

        # If 'var' is input, add to the set of input vars
        # If 'var' is output, add to the set of output vars
        if (not ctx.ioType() is None):
            if (not ctx.ioType().Input() is None):
                var_iotype = 'input'
            elif (not ctx.ioType().Output() is None):
                var_iotype = 'output'
        self.set_var_io_type(var_name, var_iotype)

        self.visitChildren(ctx)

    def visitConstantDeclaration(self, ctx):
        # fetch the variable name, type and io signature
        const_name = ctx.identifier().getText()
        const_type = ctx.domainType().getText()
        const_value = ctx.literal().getText()

        self.declare_const(const_name, const_type, const_value)

        self.visitChildren(ctx)

    def visitRosTopic(self, ctx):
        var_name = ctx.Identifier(0).getText()
        topic_name = ctx.Identifier(1).getText()
        self.set_var_topic(var_name, topic_name)

    def visitModImport(self, ctx):
        module_name = ctx.Identifier(0).getText()
        var_type = ctx.Identifier(1).getText()
        self.import_module(module_name, var_type)

    def visitExprAddition(self, ctx):
        child1 = self.visit(ctx.real_expression(0))
        child2 = self.visit(ctx.real_expression(1))
        node = Addition(child1, child2)
        return node

    def visitExprSubtraction(self, ctx):
        child1 = self.visit(ctx.real_expression(0))
        child2 = self.visit(ctx.real_expression(1))
        node = Subtraction(child1, child2)
        return node

    def visitExprMultiplication(self, ctx):
        child1 = self.visit(ctx.real_expression(0))
        child2 = self.visit(ctx.real_expression(1))
        node = Multiplication(child1, child2)
        return node

    def visitExprDivision(self, ctx):
        child1 = self.visit(ctx.real_expression(0))
        child2 = self.visit(ctx.real_expression(1))
        node = Division(child1, child2)
        return node

    def visitExprAbs(self, ctx):
        child = self.visit(ctx.real_expression())
        node = Abs(child)
        return node

    def visitExprSqrt(self, ctx):
        child = self.visit(ctx.real_expression())
        node = Sqrt(child)
        return node

    def visitExprExp(self, ctx):
        child = self.visit(ctx.real_expression())
        node = Exp(child)
        return node

    def visitExprPow(self, ctx):
        child1 = self.visit(ctx.real_expression(0))
        child2 = self.visit(ctx.real_expression(1))
        node = Pow(child1, child2)
        return node

    def visitExprNot(self, ctx):
        child = self.visit(ctx.expression())
        node = Neg(child)
        return node

    def visitExprRise(self, ctx):
        child = self.visit(ctx.expression())
        node = Rise(child)
        return node

    def visitExprLiteral(self, ctx):
        val = float(ctx.literal().getText())
        node = Constant(val)
        return node

    def visitExprFall(self, ctx):
        child = self.visit(ctx.expression())
        node = Fall(child)
        return node

    def visitExprAnd(self, ctx):
        child1 = self.visit(ctx.expression(0))
        child2 = self.visit(ctx.expression(1))
        node = Conjunction(child1, child2)
        return node

    def visitExprOr(self, ctx):
        child1 = self.visit(ctx.expression(0))
        child2 = self.visit(ctx.expression(1))
        node = Disjunction(child1, child2)
        return node

    def visitExprImplies(self, ctx):
        child1 = self.visit(ctx.expression(0))
        child2 = self.visit(ctx.expression(1))
        node = Implies(child1, child2)
        return node

    def visitExprIff(self, ctx):
        child1 = self.visit(ctx.expression(0))
        child2 = self.visit(ctx.expression(1))
        node = Iff(child1, child2)
        return node

    def visitExprXor(self, ctx):
        child1 = self.visit(ctx.expression(0))
        child2 = self.visit(ctx.expression(1))
        node = Xor(child1, child2)
        return node

    def visitExprAlways(self, ctx):
        child = self.visit(ctx.expression())
        node = Always(child)
        return node

    def visitExprEv(self, ctx):
        child = self.visit(ctx.expression())
        node = Eventually(child)
        return node

    def visitExprPrevious(self, ctx):
        child = self.visit(ctx.expression())
        node = Previous(child)
        return node

    def visitExprNext(self, ctx):
        child = self.visit(ctx.expression())
        node = Next(child)
        return node

    def visitExpreOnce(self, ctx):
        child = self.visit(ctx.expression())
        node = Once(child)
        return node

    def visitExprHist(self, ctx):
        child = self.visit(ctx.expression())
        node = Historically(child)
        return node

    def visitExprSince(self, ctx):
        child1 = self.visit(ctx.expression(0))
        child2 = self.visit(ctx.expression(1))
        node = Since(child1, child2)
        return node

    def visitExprUntil(self, ctx):
        # Parse children
        child1 = self.visit(ctx.expression(0))
        child2 = self.visit(ctx.expression(1))

        node = Until(child1, child2)
        return node

    def visitExprUnless(self, ctx):
        child1 = self.visit(ctx.expression(0))
        child2 = self.visit(ctx.expression(1))
        interval = self.visit(ctx.interval())

        left = Always(child1, 0, interval.end)
        right = Until(child1, child2)
        node = Disjunction(left, right)

        return node

    def visitExprParen(self, ctx):
        return self.visit(ctx.expression())

    def visitExpr(self, ctx):
        return self.visit(ctx.expression())

    def visitAssertion(self, ctx):
        out = self.visit(ctx.expression())

        implicit = False
        if not ctx.Identifier():
            id = 'out'
            implicit = True
        else:
            id = ctx.Identifier().getText();

        self.var_subspec_dict[id] = out
        id_tokens = id.split('.')
        id_head = id_tokens[0]
        id_tokens.pop(0)
        id_tail = '.'.join(id_tokens)

        try:
            var = self.var_object_dict[id_head]
            var = self.create_var_from_name(id_head)
            if (not id_tail):
                if (not isinstance(var, (int, float))):
                    raise RTAMTException('Variable {} is not of type int or float'.format(id))
            else:
                try:
                    value = operator.attrgetter(id_tail)(var)
                    if (not isinstance(value, (int, float))):
                        raise RTAMTException(
                            'The field {0} of the variable {1} is not of type int or float'.format(id, id_head))
                except AttributeError as err:
                    raise RTAMTException(err)
        except KeyError:
            if id_tail:
                raise RTAMTException('{0} refers to undeclared variable {1} of unknown type'.format(id, id_head))
            else:
                var = float()
                self.var_object_dict[id] = var
                self.add_var(id)
                if not implicit:
                    logging.warning('The variable {} is not explicitly declared. It is implicitly declared as a '
                                    'variable of type float'.format(id))

        self.out_var = id_head
        self.out_var_field = id_tail
        self.free_vars.discard(id_head)
        self.specs.append(out)
        return

    def visitSpecification_file(self, ctx):
        self.visit(ctx.specification())
        return

    def visitSpecification(self, ctx):
        self.visitChildren(ctx)
        try:
            del self.var_subspec_dict[self.out_var + self.out_var_field]
        except KeyError:
            pass
        return

    def visitSpecificationId(self, ctx):
        self.visitChildren(ctx)
        # The specification name is updated only if it is given
        # by the user
        if not ctx.Identifier() is None:
            self.name = ctx.Identifier().getText()

    def str_to_op_type(self, input):
        if input == "<":
            return self.comp_op_mod.StlComparisonOperator.LESS
        elif input == '<=':
            return self.comp_op_mod.StlComparisonOperator.LEQ
        elif input == ">=":
            return self.comp_op_mod.StlComparisonOperator.GEQ
        elif input == ">":
            return self.comp_op_mod.StlComparisonOperator.GREATER
        elif input == "==":
            return self.comp_op_mod.StlComparisonOperator.EQUAL
        else:
            return self.comp_op_mod.StlComparisonOperator.NEQ
