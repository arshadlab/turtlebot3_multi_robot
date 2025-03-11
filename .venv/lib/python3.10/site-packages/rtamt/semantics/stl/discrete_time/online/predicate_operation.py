from rtamt.semantics.abstract_online_operation import AbstractOnlineOperation
from rtamt.semantics.enumerations.comp_oper import StlComparisonOperator
from rtamt.exception.exception import RTAMTException


class PredicateOperation(AbstractOnlineOperation):
    def __init__(self, comparison_op):
        self.comparison_op = comparison_op

    def reset(self):
        pass

    def update(self, sample_left, sample_right):
        if self.comparison_op.value == StlComparisonOperator.EQ.value:
            sample_return = - abs(sample_left - sample_right)
        elif self.comparison_op.value == StlComparisonOperator.NEQ.value:
            sample_return = abs(sample_left - sample_right)
        elif self.comparison_op.value == StlComparisonOperator.LEQ.value or self.comparison_op.value == StlComparisonOperator.LESS.value:
            sample_return = sample_right - sample_left
        elif self.comparison_op.value == StlComparisonOperator.GEQ.value or self.comparison_op.value == StlComparisonOperator.GREATER.value:
            sample_return = sample_left - sample_right
        else:
            raise RTAMTException('Unknown predicate operation')

        return sample_return

    def sat(self, sample_left, sample_right):
        if self.comparison_op.value == StlComparisonOperator.EQ.value:
            sample_return = sample_left == sample_right
        elif self.comparison_op.value == StlComparisonOperator.NEQ.value:
            sample_return = sample_left != sample_right
        elif self.comparison_op.value == StlComparisonOperator.GEQ.value:
            sample_return = sample_left >= sample_right
        elif self.comparison_op.value == StlComparisonOperator.GREATER.value:
            sample_return = sample_left > sample_right
        elif self.comparison_op.value == StlComparisonOperator.LEQ.value:
            sample_return = sample_left <= sample_right
        elif self.comparison_op.value == StlComparisonOperator.LESS.value:
            sample_return = sample_left < sample_right
        else:
            raise RTAMTException('Unknown predicate operation')

        return sample_return
