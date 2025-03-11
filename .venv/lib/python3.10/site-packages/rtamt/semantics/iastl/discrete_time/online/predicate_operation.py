from rtamt.semantics.stl.discrete_time.online.predicate_operation import PredicateOperation as StlPredicateOperation
from rtamt.semantics.enumerations.options import Semantics


class PredicateOperation(StlPredicateOperation):
    def __init__(self, comparison_op, semantics, in_vars, out_vars):
        StlPredicateOperation.__init__(self, comparison_op)
        self.semantics = semantics
        self.in_vars = in_vars
        self.out_vars = out_vars

    def update(self, sample_left, sample_right):
        out_sample = StlPredicateOperation.update(self, sample_left, sample_right)

        sat_sample = self.sat(sample_left, sample_right)
        if (self.semantics == Semantics.OUTPUT_ROBUSTNESS and not self.out_vars) or (
                self.semantics == Semantics.INPUT_ROBUSTNESS and not self.in_vars):
            out_sample = float('inf') if sat_sample == True else -float("inf")
        elif (self.semantics == Semantics.INPUT_VACUITY and not self.in_vars) or (
                self.semantics == Semantics.OUTPUT_VACUITY and not self.out_vars):
            out_sample = 0

        return out_sample
