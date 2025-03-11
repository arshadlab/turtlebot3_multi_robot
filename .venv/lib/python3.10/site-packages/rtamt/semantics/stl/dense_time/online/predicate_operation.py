from rtamt.semantics.abstract_dense_time_online_operation import AbstractDenseTimeOnlineOperation
from rtamt.semantics.enumerations.comp_oper import StlComparisonOperator
from rtamt.semantics.arithmetic.dense_time.online.subtraction_operation import SubtractionOperation
from rtamt.exception.exception import RTAMTException


class PredicateOperation(AbstractDenseTimeOnlineOperation):
    def __init__(self, comparison_op):
        self.sub = SubtractionOperation()
        self.comparison_op = comparison_op

    def reset(self):
        pass

    def update(self, sample_left, sample_right, *args, **kargs):
        sample_result = []
        input_list = self.sub.update(sample_left, sample_right, *args, **kargs)

        prev = float('nan')
        for i in input_list:
            if self.comparison_op.value == StlComparisonOperator.EQ.value:
                out_val = - abs(i[1])
            elif self.comparison_op.value == StlComparisonOperator.NEQ.value:
                out_val = abs(i[1])
            elif self.comparison_op.value == StlComparisonOperator.LEQ.value or self.comparison_op.value == StlComparisonOperator.LESS.value:
                out_val = - i[1]
            elif self.comparison_op.value == StlComparisonOperator.GEQ.value or self.comparison_op.value == StlComparisonOperator.GREATER.value:
                out_val = i[1]
            else:
                out_val = float('nan')

            if out_val != prev:
                sample_result.append([i[0], out_val])
            prev = out_val

        return sample_result

    def update_final(self, node, sample_left, sample_right, *args, **kargs):
        sample_result = []
        input_list = self.sub.update_final(sample_left, sample_right)

        prev = float('nan')
        for i in input_list:
            if node.operator.value == StlComparisonOperator.EQ.value:
                out_val = - abs(i[1])
            elif node.operator.value == StlComparisonOperator.NEQ.value:
                out_val = abs(i[1])
            elif node.operator.value == StlComparisonOperator.LEQ.value or node.operator.value == StlComparisonOperator.LESS.value:
                out_val = - i[1]
            elif node.operator.value == StlComparisonOperator.GEQ.value or node.operator.value == StlComparisonOperator.GREATER.value:
                out_val = i[1]
            else:
                out_val = float('nan')

            if out_val != prev:
                sample_result.append([i[0], out_val])
            prev = out_val

        return sample_result

    def sat(self, sample_left, sample_right, *args, **kargs):
        sample_result = []
        input_list = self.sub.update(sample_left, sample_right)

        prev = float('nan')
        for i, in_sample in enumerate(input_list):
            if self.comparison_op.value == StlComparisonOperator.EQ.value:
                out_val = True if in_sample[1] == 0 else False
                rval = -abs(in_sample[1])
            elif self.comparison_op.value == StlComparisonOperator.NEQ.value:
                out_val = False if in_sample[1] == 0 else True
                rval = abs(in_sample[1])
            elif self.comparison_op.value == StlComparisonOperator.LEQ.value:
                out_val = True if in_sample[1] <= 0 else False
                rval = - in_sample[1]
            elif self.comparison_op.value == StlComparisonOperator.LESS.value:
                out_val = True if in_sample[1] < 0 else False
                rval = - in_sample[1]
            elif self.comparison_op.value == StlComparisonOperator.GEQ.value:
                out_val = True if in_sample[1] >= 0 else False
                rval = in_sample[1]
            elif self.comparison_op.value == StlComparisonOperator.GREATER.value:
                out_val = True if in_sample[1] > 0 else False
                rval = in_sample[1]
            else:
                raise RTAMTException('Unknown predicate operation')

            if rval != prev or i == len(input_list) - 1:
                sample_result.append([in_sample[0], out_val])
            prev = out_val

        return sample_result

    def sat_final(self, sample_left, sample_right, *args, **kargs):
        sample_result = []
        input_list = self.sub.update_final(sample_left, sample_right)

        prev = float('nan')
        for i, in_sample in enumerate(input_list):
            if self.comparison_op.value == StlComparisonOperator.EQ.value:
                out_val = True if in_sample[1] == 0 else False
                rval = -abs(in_sample[1])
            elif self.comparison_op.value == StlComparisonOperator.NEQ.value:
                out_val = False if in_sample[1] == 0 else True
                rval = abs(in_sample[1])
            elif self.comparison_op.value == StlComparisonOperator.LEQ.value:
                out_val = True if in_sample[1] <= 0 else False
                rval = - in_sample[1]
            elif self.comparison_op.value == StlComparisonOperator.LESS.value:
                out_val = True if in_sample[1] < 0 else False
                rval = - in_sample[1]
            elif self.comparison_op.value == StlComparisonOperator.GEQ.value:
                out_val = True if in_sample[1] >= 0 else False
                rval = in_sample[1]
            elif self.comparison_op.value == StlComparisonOperator.GREATER.value:
                out_val = True if in_sample[1] > 0 else False
                rval = in_sample[1]
            else:
                raise RTAMTException('Unknown predicate operation')

            if rval != prev or i == len(input_list) - 1:
                sample_result.append([in_sample[0], out_val])
            prev = out_val

        return sample_result
