import math
import operator
import collections

from rtamt.syntax.ast.visitor.stl.ast_visitor import StlAstVisitor
from rtamt.semantics.enumerations.comp_oper import StlComparisonOperator
from rtamt.exception.exception import RTAMTException


class StlDiscreteTimeOfflineAstVisitor(StlAstVisitor):

    def visitPredicate(self, node, *args, **kwargs):
        sample_left  = self.visit(node.children[0], *args, **kwargs)
        sample_right = self.visit(node.children[1], *args, **kwargs)

        sample_return = []
        for i in range(len(sample_left)):
            if node.operator.value == StlComparisonOperator.EQ.value:
                val = -abs(sample_left[i] - sample_right[i])
            elif node.operator.value == StlComparisonOperator.NEQ.value:
                val = abs(sample_left[i] - sample_right[i])
            elif node.operator.value == StlComparisonOperator.LEQ.value or node.operator.value == StlComparisonOperator.LESS.value:
                val = sample_right[i] - sample_left[i]
            elif node.operator.value == StlComparisonOperator.GEQ.value or node.operator.value == StlComparisonOperator.GREATER.value:
                val = sample_left[i] - sample_right[i]
            else:
                raise RTAMTException('Unknown predicate operation')
            sample_return.append(val)
        return sample_return


    def visitVariable(self, node, *args, **kwargs):
        var = self.ast.var_object_dict[node.var]
        if node.field:  #TODO Tom did not understand this line.
            sample_return = []
            for v in var:
                sample_return.append(operator.attrgetter(node.field)(v))
        else:
            sample_return = var
        return sample_return


    def visitAbs(self, node, *args, **kwargs):
        sample = self.visit(node.children[0], *args, **kwargs)

        sample_return = []
        for i in sample:
            out_sample = abs(i)
            sample_return.append(out_sample)
        return sample_return

    def visitSqrt(self, node, *args, **kwargs):
        sample = self.visit(node.children[0], *args, **kwargs)

        sample_return = []
        for i in sample:
            out_sample = math.sqrt(i)
            sample_return.append(out_sample)
        return sample_return

    def visitExp(self, node, *args, **kwargs):
        sample = self.visit(node.children[0], *args, **kwargs)

        sample_return = []
        for i in sample:
            out_sample = math.exp(i)
            sample_return.append(out_sample)
        return sample_return

    def visitPow(self, node, *args, **kwargs):
        sample_1 = self.visit(node.children[0], *args, **kwargs)
        sample_2 = self.visit(node.children[1], *args, **kwargs)

        sample_return = []
        for i in range(len(sample_1)):
            out_sample = math.pow(sample_1[i], sample_2[i])
            sample_return.append(out_sample)
        return sample_return


    def visitAddition(self, node, *args, **kwargs):
        sample_left  = self.visit(node.children[0], *args, **kwargs)
        sample_right = self.visit(node.children[1], *args, **kwargs)

        sample_return = []
        for i in range(len(sample_left)):
            out_sample = sample_left[i] + sample_right[i]
            sample_return.append(out_sample)
        return sample_return


    def visitSubtraction(self, node, *args, **kwargs):
        sample_left  = self.visit(node.children[0], *args, **kwargs)
        sample_right = self.visit(node.children[1], *args, **kwargs)

        sample_return = []
        for i in range(len(sample_left)):
            out_sample = sample_left[i] - sample_right[i]
            sample_return.append(out_sample)
        return sample_return


    def visitMultiplication(self, node, *args, **kwargs):
        sample_left  = self.visit(node.children[0], *args, **kwargs)
        sample_right = self.visit(node.children[1], *args, **kwargs)

        sample_return = []
        for i in range(len(sample_left)):
            out_sample = sample_left[i] * sample_right[i]
            sample_return.append(out_sample)
        return sample_return


    def visitDivision(self, node, *args, **kwargs):
        sample_left  = self.visit(node.children[0], *args, **kwargs)
        sample_right = self.visit(node.children[1], *args, **kwargs)

        sample_return = []
        for i in range(len(sample_left)):
            out_sample = sample_left[i] / sample_right[i]
            sample_return.append(out_sample)
        return sample_return


    def visitNot(self, node, *args, **kwargs):
        sample = self.visit(node.children[0], *args, **kwargs)

        sample_return = [ -i for i in sample]
        return sample_return


    def visitAnd(self, node, *args, **kwargs):
        sample_left  = self.visit(node.children[0], *args, **kwargs)
        sample_right = self.visit(node.children[1], *args, **kwargs)

        sample_return = list(map(min, zip(sample_left, sample_right)))
        return sample_return


    def visitOr(self, node, *args, **kwargs):
        sample_left  = self.visit(node.children[0], *args, **kwargs)
        sample_right = self.visit(node.children[1], *args, **kwargs)

        sample_return = list(map(max, zip(sample_left, sample_right)))
        return sample_return


    def visitImplies(self, node, *args, **kwargs):
        sample_left  = self.visit(node.children[0], *args, **kwargs)
        sample_right = self.visit(node.children[1], *args, **kwargs)

        sample_return = [max(-l,r) for l,r in zip(sample_left, sample_right)]
        return sample_return


    def visitIff(self, node, *args, **kwargs):
        sample_left  = self.visit(node.children[0], *args, **kwargs)
        sample_right = self.visit(node.children[1], *args, **kwargs)

        sample_return = [-abs(l-r) for l,r in zip(sample_left, sample_right)]
        return sample_return


    def visitXor(self, node, *args, **kwargs):
        sample_left  = self.visit(node.children[0], *args, **kwargs)
        sample_right = self.visit(node.children[1], *args, **kwargs)

        sample_return = [abs(l-r) for l,r in zip(sample_left, sample_right)]
        return sample_return


    def visitEventually(self, node, *args, **kwargs):
        sample = self.visit(node.children[0], *args, **kwargs)

        sample_return = []
        prev_out = -float("inf")
        for i in reversed(sample):
            out_sample = max(i, prev_out)
            prev_out = out_sample
            sample_return.append(out_sample)
        sample_return.reverse()
        return sample_return


    def visitAlways(self, node, *args, **kwargs):
        sample = self.visit(node.children[0], *args, **kwargs)

        sample_return = []
        prev_out = float("inf")
        for i in reversed(sample):
            out_sample = min(i, prev_out)
            prev_out = out_sample
            sample_return.append(out_sample)
        sample_return.reverse()
        return sample_return


    def visitUntil(self, node, *args, **kwargs):
        sample_left  = self.visit(node.children[0], *args, **kwargs)
        sample_right = self.visit(node.children[1], *args, **kwargs)

        sample_return = []
        next_out = -float("inf")
        for i in range(len(sample_left)-1, -1, -1):
            out_sample = min(sample_left[i], next_out)
            out_sample = max(out_sample, sample_right[i])
            next_out = out_sample
            sample_return.append(out_sample)
        sample_return.reverse()
        return sample_return


    def visitOnce(self, node, *args, **kwargs):
        sample = self.visit(node.children[0], *args, **kwargs)

        sample_return = []
        prev_out = -float("inf")
        for i in sample:
            out_sample = max(i, prev_out)
            prev_out = out_sample
            sample_return.append(out_sample)
        return sample_return


    def visitHistorically(self, node, *args, **kwargs):
        sample = self.visit(node.children[0], *args, **kwargs)

        sample_return = []
        prev_out = float("inf")
        for i in sample:
            out_sample = min(i, prev_out)
            prev_out = out_sample
            sample_return.append(out_sample)
        return sample_return


    def visitSince(self, node, *args, **kwargs):
        sample_left  = self.visit(node.children[0], *args, **kwargs)
        sample_right = self.visit(node.children[1], *args, **kwargs)

        sample_return = []
        prev_out = -float("inf")
        for i in range(len(sample_left)):
            out_sample = min(sample_left[i], prev_out)
            out_sample = max(out_sample, sample_right[i])
            prev_out = out_sample
            sample_return.append(out_sample)
        return sample_return


    def visitRise(self, node, *args, **kwargs):
        sample = self.visit(node.children[0], *args, **kwargs)

        prev = sample[:-1]
        prev.insert(0,-float("inf"))
        sample_return = [min(-p,s) for p,s in zip(prev, sample)]
        return sample_return


    def visitFall(self, node, *args, **kwargs):
        sample = self.visit(node.children[0], *args, **kwargs)

        prev = sample[:-1]
        prev.insert(0,float("inf"))
        sample_return = [min(p,-s) for p,s in zip(prev, sample)]
        return sample_return


    def visitConstant(self, node, *args, **kwargs):
        length = args[0]
        return [node.val]*length


    def visitPrevious(self, node, *args, **kwargs):
        sample = self.visit(node.children[0], *args, **kwargs)

        sample_return = []
        prev = float("inf")
        for i in sample:
            out_sample = prev
            prev = i
            sample_return.append(out_sample)
        return sample_return


    def visitNext(self, node, *args, **kwargs):
        sample = self.visit(node.children[0], *args, **kwargs)

        sample_return = sample[1:]
        sample_return.append(float("inf"))
        return sample_return


    def visitTimedPrecedes(self, node, *args, **kwargs):
        raise RTAMTException('Offline does not need visitTimedPrecedes')


    def visitTimedOnce(self, node, *args, **kwargs):
        sample = self.visit(node.children[0], *args, **kwargs)
        begin, end = self.time_unit_transformer(node)

        sample = [-float("inf") for j in range(end)] + sample
        sample_return = [max(sample[j - end:j - begin+ 1]) for j in range(end, len(sample))]
        return sample_return


    def visitTimedHistorically(self, node, *args, **kwargs):
        sample = self.visit(node.children[0], *args, **kwargs)
        begin, end = self.time_unit_transformer(node)

        sample = [float("inf") for j in range(end)] + sample
        sample_return = [min(sample[j - end:j - begin + 1]) for j in range(end, len(sample))]
        return sample_return

    def visitTimedSince(self, node, *args, **kwargs):
        sample_left  = self.visit(node.children[0], *args, **kwargs)
        sample_right = self.visit(node.children[1], *args, **kwargs)
        begin, end = self.time_unit_transformer(node)

        sample_return = []
        buffer_left = collections.deque(maxlen=(end + 1))
        buffer_right = collections.deque(maxlen=(end + 1))

        for i in range(end + 1):
            s_left = float("inf")
            s_right = - float("inf")
            buffer_left.append(s_left)
            buffer_right.append(s_right)

        for i in range(len(sample_left)):
            buffer_left.append(sample_left[i])
            buffer_right.append(sample_right[i])
            out_sample = - float("inf")
            

            for j in range(end-begin+1):
                c_left = float("inf")
                c_right = buffer_right[j]
                for k in range(j+1, end+1):
                    c_left = min(c_left, buffer_left[k])
                out_sample = max(out_sample, min(c_left, c_right))
            sample_return.append(out_sample)
        return sample_return


    def visitTimedAlways(self, node, *args, **kwargs):
        sample = self.visit(node.children[0], *args, **kwargs)
        begin, end = self.time_unit_transformer(node)
        sample_len = len(sample)
        if sample_len <= end:
            sample += [float('inf')] * (end - sample_len + 1)

        diff = end - begin
        sample_return  = [min(sample[j:j+diff+1]) for j in range(begin, end+1)]
        tmp  = [min(sample[j:j+diff+1]) for j in range(end+1,len(sample))]
        sample_return += tmp
        tmp  = [float("inf") for j in range(len(sample)-len(sample_return))]
        sample_return += tmp
        return sample_return[0:sample_len]

    def visitTimedEventually(self, node, *args, **kwargs):
        sample = self.visit(node.children[0], *args, **kwargs)
        begin, end = self.time_unit_transformer(node)
        sample_len = len(sample)

        if sample_len <= end:
            sample += [-float('inf')] * (end - sample_len + 1)

        diff = end - begin
        sample_return  = [max(sample[j:j+diff+1]) for j in range(begin, end+1)]
        tmp = [max(sample[j:j+diff+1]) for j in range(end+1,len(sample))]
        sample_return += tmp
        tmp = [-float("inf") for j in range(len(sample)-len(sample_return))]
        sample_return += tmp
        return sample_return[0:sample_len]


    def visitTimedUntil(self, node, *args, **kwargs):
        sample_left  = self.visit(node.children[0], *args, **kwargs)
        sample_right = self.visit(node.children[1], *args, **kwargs)
        begin, end = self.time_unit_transformer(node)

        sample_return = []
        buffer_left = collections.deque(maxlen=(end + 1))
        buffer_right = collections.deque(maxlen=(end + 1))

        for i in range(end + 1):
            s_left = float("inf")
            s_right = - float("inf")
            buffer_left.append(s_left)
            buffer_right.append(s_right)
        for i in range(len(sample_left)-1, -1, -1):
            buffer_left.append(sample_left[i])
            buffer_right.append(sample_right[i])
            out_sample = - float("inf")

            for j in range(end-begin+1):
                c_left = float("inf")
                c_right = buffer_right[j]
                for k in range(j+1, end+1):
                    c_left = min(c_left, buffer_left[k])
                out_sample = max(out_sample, min(c_left, c_right))
            sample_return.append(out_sample)
        sample_return.reverse()
        return sample_return
