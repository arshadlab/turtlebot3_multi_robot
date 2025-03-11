from rtamt.semantics.stl.dense_time.online.predicate_operation import PredicateOperation as StlPredicateOperation
from rtamt.semantics.enumerations.options import Semantics


class PredicateOperation(StlPredicateOperation):
    def __init__(self, comparison_op, semantics, in_vars, out_vars):
        StlPredicateOperation.__init__(self, comparison_op)
        self.semantics = semantics
        self.in_vars = in_vars
        self.out_vars = out_vars

    def update(self, sample_left, sample_right):
        samples = StlPredicateOperation.update(self, sample_left, sample_right)
        sat_sample = StlPredicateOperation.sat(self, sample_left, sample_right)
        out_sample = []

        if (self.semantics == Semantics.OUTPUT_ROBUSTNESS and not self.out_vars) or (
                self.semantics == Semantics.INPUT_ROBUSTNESS and not self.in_vars):
            for i in range(len(sat_sample)):
                sample = float('inf') if sat_sample[i][1] == True else -float("inf")
                out_sample.append([sat_sample[i][0], sample])
        elif (self.semantics == Semantics.INPUT_VACUITY and not self.in_vars) or (
                self.semantics == Semantics.OUTPUT_VACUITY and not self.out_vars):
            for i in range(len(sat_sample)):
                sample = 0
                out_sample.append([sat_sample[i][0], sample])
        else:
            out_sample = samples

        return out_sample

    def update_final(self, sample_left, sample_right):
        samples = StlPredicateOperation.update_final(self, sample_left, sample_right)
        sat_sample = StlPredicateOperation.sat_final(self, sample_left, sample_right)
        out_sample = []

        if (self.semantics == Semantics.OUTPUT_ROBUSTNESS and not self.out_vars) or (
                self.semantics == Semantics.INPUT_ROBUSTNESS and not self.in_vars):
            for i in range(len(sat_sample)):
                sample = float('inf') if sat_sample[i][1] == True else -float("inf")
                out_sample.append([sat_sample[i][0], sample])
        elif (self.semantics == Semantics.INPUT_VACUITY and not self.in_vars) or (
                self.semantics == Semantics.OUTPUT_VACUITY and not self.out_vars):
            for i in range(len(sat_sample)):
                sample = 0
                out_sample.append([sat_sample[i][0], sample])
        else:
            out_sample = samples

        return out_sample
