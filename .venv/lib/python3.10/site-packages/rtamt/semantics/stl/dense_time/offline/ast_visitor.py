import math
import operator
from collections import deque

import rtamt.semantics.stl.dense_time.offline.intersection as intersect

from rtamt.syntax.ast.visitor.stl.ast_visitor import StlAstVisitor
from rtamt.semantics.enumerations.comp_oper import StlComparisonOperator

from rtamt.exception.exception import RTAMTException

def subtraction_operation(sample_left, sample_right):
    sample_return = []
    sample_return, last, left, right = intersect.intersection(sample_left, sample_right, intersect.subtraction)
    if last:
        sample_return.append(last)
    return sample_return


def and_operation(sample_left, sample_right):
    sample_return, last, a, b = intersect.intersection(sample_left, sample_right, intersect.conjunction)
    if last:
        sample_return.append(last)
    return sample_return


def since_operation(sample_left, sample_right):
    iout, last, a, b = intersect.intersection(sample_left, sample_right, intersect.split)
    if last:
        iout.append(last)
    sample_return = []
    prev = -float("inf")
    for i, sample in enumerate(iout):
        t = sample[0]
        o1_val = sample[1][0]
        o2_val = sample[1][1]
        result = max(min(o1_val, o2_val), min(o1_val, prev))
        if result != prev or i == len(iout) - 1:
            sample_return.append([t, result])
        prev = result
    return sample_return


def once_timed_operation(sample, begin, end):
    out = []
    input_list = sample
    ans = []
    prev = []
    residual_start = -float("inf")
    max = - float("inf")

    i = 1

    domain_end = float('inf')
    if input_list:
        domain_end = input_list[len(input_list) - 1][0]

    while i <= len(input_list):
        if i == 1 and begin > 0:
            out.append((0, input_list[0][0] + begin, -float('inf')))

        if i < len(input_list):
            b = (input_list[i - 1][0] + begin, input_list[i][0] + end, input_list[i - 1][1])
        else:
            b = (input_list[i - 1][0] + begin, float('inf'), input_list[i - 1][1])

        if not out:
            out.append(b)
        else:
            a = out[len(out) - 1]
            while (a[2] < b[2]) and (b[0] < a[0]):
                del (out[len(out) - 1])
                a = out[len(out) - 1]
            if not intersect.intersects(a[0], a[1], b[0], b[1]):
                out.append(b)
            else:
                if a[2] >= b[2]:
                    out.append((a[1], b[1], b[2]))
                else:
                    del (out[len(out) - 1])
                    if b[0] > a[0]:
                        out.append((a[0], b[0], a[2]))
                    out.append((b[0], b[1], b[2]))

        i = i + 1

    prev = float("nan")

    for i, b in enumerate(out):
        if b[2] != prev or i == len(out) - 1:
            ans.append([b[0], b[2]])

        prev = b[2]

    return ans


def historically_timed_operation(sample, begin, end):
    out = []
    input_list = sample
    ans = []
    prev = []
    residual_start = float("inf")
    max = float("inf")

    i = 1

    domain_end = float('inf')
    if input_list:
        domain_end = input_list[len(input_list) - 1][0]

    while i <= len(input_list):
        if i == 1 and begin > 0:
            out.append((0, input_list[0][0] + begin, float('inf')))

        if i < len(input_list):
            b = (input_list[i - 1][0] + begin, input_list[i][0] + end, input_list[i - 1][1])
        else:
            b = (input_list[i - 1][0] + begin, float('inf'), input_list[i - 1][1])

        if not out:
            out.append(b)
        else:
            a = out[len(out) - 1]
            while (a[2] > b[2]) and (b[0] < a[0]):
                del (out[len(out) - 1])
                a = out[len(out) - 1]
            if not intersect.intersects(a[0], a[1], b[0], b[1]):
                out.append(b)
            else:
                if a[2] <= b[2]:
                    out.append((a[1], b[1], b[2]))
                else:
                    del (out[len(out) - 1])
                    if b[0] > a[0]:
                        out.append((a[0], b[0], a[2]))
                    out.append((b[0], b[1], b[2]))

        i = i + 1

    prev = float("nan")

    for i, b in enumerate(out):
        if b[2] != prev or i == len(out) - 1:
            ans.append([b[0], b[2]])
        prev = b[2]

    return ans


def since_timed_operation(sample_left, sample_right, begin, end):
    if (begin > 0):
        out1 = once_timed_operation(sample_right, begin, end)
        out2 = since_operation(sample_left, sample_right)
        out3 = historically_timed_operation(out2, 0, begin)
        sample_return = and_operation(out1, out3)
    else:
        out1 = once_timed_operation(sample_right, begin, end)
        out2 = since_operation(sample_left, sample_right)
        sample_return = and_operation(out1, out2)
    return sample_return


def always_timed_operation(sample, begin, end):
    prev = []
    residual_start = float("inf")
    max = float("inf")

    out = []
    input_list = sample
    ans = []

    # i = 1
    i = len(input_list) - 1

    domain_end = float('inf')
    if input_list:
        domain_end = input_list[len(input_list) - 1][0]

    while i >= 0:
        if i == len(input_list) - 1:
            b = (input_list[i][0] - end, float("inf"), input_list[i][1])
        else:
            b = (input_list[i][0] - end, input_list[i + 1][0] - begin, input_list[i][1])

        if not out:
            out.insert(0, b)
        else:
            a = out[0]
            while (a[2] > b[2]) and (b[1] > a[1]):
                out.pop(0)
                a = out[0]
            if not intersect.intersects(a[0], a[1], b[0], b[1]):
                out.insert(0, b)
            else:
                if a[2] <= b[2]:
                    out.insert(0, (b[0], a[0], b[2]))
                else:
                    out.pop(0)
                    if (a[1] > b[1]):
                        out.insert(0, (b[1], a[1], a[2]))
                    out.insert(0, (b[0], b[1], b[2]))

        i = i - 1

    for i, b in enumerate(out):
        if b[0] <= 0 and b[1] > 0:
            ans.append([0, b[2]])
        elif b[0] > 0:
            ans.append([b[0], b[2]])

    return ans

def eventually_timed_operation(sample, begin, end):
    out = []
    input_list = sample
    ans = []
    prev = []
    residual_start = -float("inf")
    max = - float("inf")

    i = len(input_list) - 1

    domain_end = float('inf')
    if input_list:
        domain_end = input_list[len(input_list) - 1][0]

    while i >= 0:
        if i == len(input_list) - 1:
            b = (input_list[i][0] - end, float("inf"), input_list[i][1])
        else:
            b = (input_list[i][0] - end, input_list[i + 1][0] - begin, input_list[i][1])

        if not out:
            out.insert(0, b)
        else:
            a = out[0]
            while (a[2] < b[2]) and (b[1] > a[1]):
                out.pop(0)
                a = out[0]
            if not intersect.intersects(a[0], a[1], b[0], b[1]):
                out.insert(0, b)
            else:
                if a[2] >= b[2]:
                    out.insert(0, (b[0], a[0], b[2]))
                else:
                    out.pop(0)
                    if (a[1] > b[1]):
                        out.insert(0, (b[1], a[1], a[2]))
                    out.insert(0, (b[0], b[1], b[2]))

        i = i - 1

    for i, b in enumerate(out):
        if b[0] <= 0 and b[1] > 0:
            ans.append([0, b[2]])
        elif b[0] > 0:
            ans.append([b[0], b[2]])

    return ans

def until_operation(sample_left, sample_right):
    iout, last, a, b = intersect.intersection(sample_left, sample_right, intersect.split)
    if last:
        iout.append(last)
    sample_return = []
    next = -float("inf")
    for i, sample in reversed(list(enumerate(iout))):
        t = sample[0]
        o1_val = sample[1][0]
        o2_val = sample[1][1]
        result = max(min(o1_val, o2_val), min(o1_val, next))
        if result == next and i < len(iout) - 2:
            sample_return.pop(0)
        sample_return.insert(0, [t, result])
        next = result
    return sample_return


def until_timed_operation(sample_left, sample_right, begin, end):
    if begin > 0:
        out1 = eventually_timed_operation(sample_right, begin, end)
        out2 = until_operation(sample_left, sample_right)
        out3 = always_timed_operation(out2, 0, begin)
        sample_return = and_operation(out1, out3)
    else:
        out1 = eventually_timed_operation(sample_right, begin, end)
        out2 = until_operation(sample_left, sample_right)
        sample_return = and_operation(out1, out2)
    return sample_return


class StlDenseTimeOfflineAstVisitor(StlAstVisitor):

    def visitPredicate(self, node, *args, **kwargs):
        sample_left  = self.visit(node.children[0], *args, **kwargs)
        sample_right = self.visit(node.children[1], *args, **kwargs)

        sample_return = []
        input_list = subtraction_operation(sample_left, sample_right)

        prev = float("nan")
        for i, in_sample in enumerate(input_list):
            if node.operator.value == StlComparisonOperator.EQ.value:
                out_val = - abs(in_sample[1])
            elif node.operator.value == StlComparisonOperator.NEQ.value:
                out_val = abs(in_sample[1])
            elif node.operator.value == StlComparisonOperator.LEQ.value or node.operator.value == StlComparisonOperator.LESS.value:
                out_val = - in_sample[1]
            elif node.operator.value == StlComparisonOperator.GEQ.value or node.operator.value == StlComparisonOperator.GREATER.value:
                out_val = in_sample[1]
            else:
                out_val = float('nan')

            if out_val != prev or i == len(input_list) - 1:
                sample_return.append([in_sample[0], out_val])
            prev = out_val

        return sample_return


    def visitVariable(self, node, *args, **kwargs):
        var = self.ast.var_object_dict[node.var]
        if node.field:
            sample_return = []
            for v in var:
                val = operator.attrgetter(node.field)(v[1])
                sample_return.append([v[0], val])
        else:
            sample_return = var
        return sample_return


    def visitAbs(self, node, *args, **kwargs):
        sample = self.visit(node.children[0], *args, **kwargs)

        sample_return = []
        for i in sample:
            out_time = i[0]
            out_value = abs(i[1])
            sample_return.append([out_time, out_value])
        return sample_return


    def visitSqrt(self, node, *args, **kwargs):
        sample = self.visit(node.children[0], *args, **kwargs)

        sample_return = []
        for i in sample:
            if i[1] < 0:
                raise Exception('sqrt: the input is smaller than 0.')
            out_time = i[0]
            out_value = math.sqrt(i[1])
            sample_return.append([out_time, out_value])
        return sample_return


    def visitExp(self, node, *args, **kwargs):
        sample = self.visit(node.children[0], *args, **kwargs)

        sample_return = []
        for i in sample:
            out_time = i[0]
            out_value = math.exp(i[1])
            sample_return.append([out_time, out_value])
        return sample_return


    def visitPow(self, node, *args, **kwargs):
        sample_left  = self.visit(node.children[0], *args, **kwargs)
        sample_right = self.visit(node.children[1], *args, **kwargs)

        sample_return, last, left, right = intersect.intersection(sample_left, sample_right, intersect.power)
        if last:
            sample_return.append(last)
        return sample_return


    def visitAddition(self, node, *args, **kwargs):
        sample_left  = self.visit(node.children[0], *args, **kwargs)
        sample_right = self.visit(node.children[1], *args, **kwargs)

        sample_return = []
        sample_return, last, left, right = intersect.intersection(sample_left, sample_right, intersect.addition)
        if last:
            sample_return.append(last)
        return sample_return


    def visitSubtraction(self, node, *args, **kwargs):
        sample_left  = self.visit(node.children[0], *args, **kwargs)
        sample_right = self.visit(node.children[1], *args, **kwargs)

        sample_return= subtraction_operation(sample_left, sample_right)
        return sample_return


    def visitMultiplication(self, node, *args, **kwargs):
        sample_left  = self.visit(node.children[0], *args, **kwargs)
        sample_right = self.visit(node.children[1], *args, **kwargs)

        sample_return = []
        sample_return, last, left, right = intersect.intersection(sample_left, sample_right, intersect.multiplication)
        if last:
            sample_return.append(last)
        return sample_return


    def visitDivision(self, node, *args, **kwargs):
        sample_left  = self.visit(node.children[0], *args, **kwargs)
        sample_right = self.visit(node.children[1], *args, **kwargs)

        sample_return = []
        sample_return, last, left, right = intersect.intersection(sample_left, sample_right, intersect.division)
        if last:
            sample_return.append(last)
        return sample_return


    def visitNot(self, node, *args, **kwargs):
        sample =  self.visit(node.children[0], *args, **kwargs)

        sample_return = []
        for i in sample:
            out_time = i[0]
            out_value = - i[1]
            sample_return.append([out_time, out_value])
        return sample_return


    def visitAnd(self, node, *args, **kwargs):
        sample_left  = self.visit(node.children[0], *args, **kwargs)
        sample_right = self.visit(node.children[1], *args, **kwargs)

        sample_return = and_operation(sample_left, sample_right)
        return sample_return


    def visitOr(self, node, *args, **kwargs):
        sample_left  = self.visit(node.children[0], *args, **kwargs)
        sample_right = self.visit(node.children[1], *args, **kwargs)

        sample_return, last, left, right = intersect.intersection(sample_left, sample_right, intersect.disjunction)
        if last:
            sample_return.append(last)
        return sample_return


    def visitImplies(self, node, *args, **kwargs):
        sample_left  = self.visit(node.children[0], *args, **kwargs)
        sample_right = self.visit(node.children[1], *args, **kwargs)

        sample_return, last, left, right = intersect.intersection(sample_left, sample_right, intersect.implication)
        if last:
            sample_return.append(last)
        return sample_return


    def visitIff(self, node, *args, **kwargs):
        sample_left  = self.visit(node.children[0], *args, **kwargs)
        sample_right = self.visit(node.children[1], *args, **kwargs)

        sample_return, last, left, right = intersect.intersection(sample_left, sample_right, intersect.iff)
        if last:
            sample_return.append(last)
        return sample_return


    def visitXor(self, node, *args, **kwargs):
        sample_left  = self.visit(node.children[0], *args, **kwargs)
        sample_right = self.visit(node.children[1], *args, **kwargs)

        sample_return, last, left, right = intersect.intersection(sample_left, sample_right, intersect.xor)
        if last:
            sample_return.append(last)
        return sample_return


    def visitEventually(self, node, *args, **kwargs):
        sample =  self.visit(node.children[0], *args, **kwargs)

        sample_return = []
        self.next = -float("inf")   #TODO don't use self.next
        next = float("nan")
        for i, in_sample in reversed(list(enumerate(sample))):
            out_time = in_sample[0]
            out_value = max(in_sample[1], self.next)
            self.next = out_value
            if out_value == next and i < len(sample) - 2:
                sample_return.pop(0)
            sample_return.insert(0, [out_time, out_value])
            next = out_value
        return sample_return


    def visitAlways(self, node, *args, **kwargs):
        sample =  self.visit(node.children[0], *args, **kwargs)

        sample_return = []
        self.next = float("inf")    #TODO don't use self.next
        next = float("nan")
        for i, in_sample in reversed(list(enumerate(sample))):
            out_time = in_sample[0]
            out_value = min(in_sample[1], self.next)
            self.next = out_value
            if out_value == next and i < len(sample) - 2:
                sample_return.pop(0)
            sample_return.insert(0, [out_time, out_value])
            next = out_value
        return sample_return


    def visitUntil(self, node, *args, **kwargs):
        sample_left  = self.visit(node.children[0], *args, **kwargs)
        sample_right = self.visit(node.children[1], *args, **kwargs)

        sample_return = until_operation(sample_left, sample_right)
        return sample_return

    def visitOnce(self, node, *args, **kwargs):
        sample =  self.visit(node.children[0], *args, **kwargs)

        sample_return = []
        self.prev = - float("inf")  #TODO don't use self.prev
        prev = float("nan")
        for i, in_sample in enumerate(sample):
            out_time = in_sample[0]
            out_value = max(in_sample[1], self.prev)
            self.prev = out_value
            if out_value != prev or i == len(sample) - 1:
                sample_return.append([out_time, out_value])
            prev = out_value
        return sample_return


    def visitHistorically(self, node, *args, **kwargs):
        sample =  self.visit(node.children[0], *args, **kwargs)

        sample_return = []
        self.prev = float("inf")    #TODO don't use self.prev
        prev = float("nan")
        for i, in_sample in enumerate(sample):
            out_time = in_sample[0]
            out_value = min(in_sample[1], self.prev)
            self.prev = out_value
            if out_value != prev or i == len(sample) - 1:
                sample_return.append([out_time, out_value])
            prev = out_value
        return sample_return


    def visitSince(self, node, *args, **kwargs):
        sample_left  = self.visit(node.children[0], *args, **kwargs)
        sample_right = self.visit(node.children[1], *args, **kwargs)

        sample_return = since_operation(sample_left, sample_right)
        return sample_return


    def visitRise(self, node, *args, **kwargs):
        raise RTAMTException('Rise operator not implemented in STL dense monitor.')

    def visitFall(self, node, *args, **kwargs):
        raise RTAMTException('Fall operator not implemented in STL dense monitor.')

    #TODO: this code may not work.
    def visitConstant(self, node, *args, **kwargs):
        sample_return = [[0, node.val], [float("inf"), node.val]]
        return sample_return

    def visitPrevious(self, node, *args, **kwargs):
        raise RTAMTException('Previous operator not implemented in STL dense-time monitor.')


    def visitNext(self, node, *args, **kwargs):
        raise RTAMTException('Next operator not implemented in STL dense-time monitor.')


    def visitTimedPrecedes(self, node, *args, **kwargs):
        raise RTAMTException('Precedes operator not implemented in STL dense-time monitor.')


    def visitTimedOnce(self, node, *args, **kwargs):
        sample = self.visit(node.children[0], *args, **kwargs)
        begin, end = self.time_unit_transformer(node)
        sample_return = once_timed_operation(sample, begin, end)
        return sample_return


    def visitTimedHistorically(self, node, *args, **kwargs):
        sample = self.visit(node.children[0], *args, **kwargs)
        begin, end = self.time_unit_transformer(node)
        sample_return = historically_timed_operation(sample, begin, end)
        return sample_return


    def visitTimedSince(self, node, *args, **kwargs):
        sample_left = self.visit(node.children[0], *args, **kwargs)
        sample_right = self.visit(node.children[1], *args, **kwargs)
        begin, end = self.time_unit_transformer(node)

        sample_return = since_timed_operation(sample_left, sample_right, begin, end)
        return sample_return


    def visitTimedAlways(self, node, *args, **kwargs):
        sample = self.visit(node.children[0], *args, **kwargs)
        begin, end = self.time_unit_transformer(node)
        sample_return = always_timed_operation(sample, begin, end)
        return sample_return


    def visitTimedEventually(self, node, *args, **kwargs):
        sample =  self.visit(node.children[0], *args, **kwargs)
        begin, end = self.time_unit_transformer(node)
        sample_return = eventually_timed_operation(sample, begin, end)
        return sample_return


    def visitTimedUntil(self, node, *args, **kwargs):
        sample_left  = self.visit(node.children[0], *args, **kwargs)
        sample_right = self.visit(node.children[1], *args, **kwargs)
        begin, end = self.time_unit_transformer(node)
        sample_return = until_timed_operation(sample_left, sample_right, begin, end)
        return sample_return
