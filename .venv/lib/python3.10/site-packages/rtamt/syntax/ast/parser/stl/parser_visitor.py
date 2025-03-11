from decimal import Decimal
from fractions import Fraction

from rtamt.antlr.parser.stl.StlParserVisitor import StlParserVisitor
from rtamt.syntax.ast.parser.ltl.parser_visitor import LtlAstParserVisitor
from rtamt.semantics.interval.interval import Interval

from rtamt.syntax.node.ltl.disjunction import Disjunction
from rtamt.syntax.node.ltl.always import Always
from rtamt.syntax.node.ltl.eventually import Eventually
from rtamt.syntax.node.ltl.once import Once
from rtamt.syntax.node.ltl.historically import Historically
from rtamt.syntax.node.ltl.since import Since
from rtamt.syntax.node.ltl.until import Until
from rtamt.syntax.node.stl.timed_always import TimedAlways
from rtamt.syntax.node.stl.timed_eventually import TimedEventually
from rtamt.syntax.node.stl.timed_historically import TimedHistorically
from rtamt.syntax.node.stl.timed_once import TimedOnce
from rtamt.syntax.node.stl.timed_since import TimedSince
from rtamt.syntax.node.stl.timed_until import TimedUntil
from rtamt.exception.exception import RTAMTException


class StlAstParserVisitor(LtlAstParserVisitor, StlParserVisitor):

    def __init__(self):

        LtlAstParserVisitor.__init__(self)
        StlParserVisitor.__init__(self)

    def visitExprAlways(self, ctx):
        child = self.visit(ctx.expression())
        if ctx.interval() == None:
            node = Always(child)
        else:
            interval = self.visit(ctx.interval())
            node = TimedAlways(child, interval)
        return node


    def visitExprEv(self, ctx):
        child = self.visit(ctx.expression())
        if ctx.interval() == None:
            node = Eventually(child)
        else:
            interval = self.visit(ctx.interval())
            node = TimedEventually(child, interval)
        return node

    def visitExpreOnce(self, ctx):
        child = self.visit(ctx.expression())
        if ctx.interval() == None:
            node = Once(child)
        else:
            interval = self.visit(ctx.interval())
            node = TimedOnce(child, interval)
        return node

    def visitExprHist(self, ctx):
        child = self.visit(ctx.expression())
        if ctx.interval() == None:
            node = Historically(child)
        else:
            interval = self.visit(ctx.interval())
            node = TimedHistorically(child, interval)
        return node

    def visitExprSince(self, ctx):
        child1 = self.visit(ctx.expression(0))
        child2 = self.visit(ctx.expression(1))
        if ctx.interval() == None:
            node = Since(child1, child2)
        else:
            interval = self.visit(ctx.interval())
            node = TimedSince(child1, child2, interval)
        return node

    def visitExprUntil(self, ctx):
        child1 = self.visit(ctx.expression(0))
        child2 = self.visit(ctx.expression(1))
        if ctx.interval() == None:
            node = Until(child1, child2)
        else:
            interval = self.visit(ctx.interval())
            node = TimedUntil(child1, child2, interval)
        return node

    def visitExprUnless(self, ctx):
        child1 = self.visit(ctx.expression(0))
        child2 = self.visit(ctx.expression(1))
        interval = self.visit(ctx.interval())
        if ctx.interval() == None:
            left = Always(child1)
            right = Until(child1, child2)
            node = Disjunction(left, right)
        else:
            interval_left = Interval(0, interval.end, interval.begin_unit, interval.end_unit)
            left = TimedAlways(child1, interval_left)
            right = TimedUntil(child1, child2, interval)
            node = Disjunction(left, right)
        return node

    def visitConstantTimeLiteral(self, ctx):
        const_name = ctx.Identifier().getText()

        if const_name not in self.const_val_dict:
            raise RTAMTException('Bound {} not declared'.format(const_name))

        val = self.const_val_dict[const_name]

        out = Fraction(Decimal(val))

        if ctx.unit() is None:
            unit = 'default'
        else:
            unit = ctx.unit().getText()

        return out, unit


    def visitIntervalTimeLiteral(self, ctx):
        time_bound = Fraction(Decimal(ctx.literal().getText()))
        if ctx.unit() is None:
            unit = ''
        else:
            unit = ctx.unit().getText()

        return time_bound, unit


    def visitInterval(self, ctx):
        begin, begin_unit = self.visit(ctx.intervalTime(0))
        end, end_unit = self.visit(ctx.intervalTime(1))
        interval = Interval(begin, end, begin_unit, end_unit)
        return interval

    def get_sampling_period(self):
        return self.sampling_period * self.U[self.sampling_period_unit]

    @property
    def unit(self):
        return self.__unit

    @unit.setter
    def unit(self, unit):
        self.__unit = unit
