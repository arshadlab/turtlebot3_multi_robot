from rtamt.semantics.enumerations.comp_oper import StlComparisonOperator
from rtamt.semantics.stl.dense_time.offline.ast_visitor import StlDenseTimeOfflineAstVisitor, subtraction_operation


class IAStlDenseTimeOfflineAstVisitor(StlDenseTimeOfflineAstVisitor):

    def visitPredicate(self, node, *args, **kwargs):
        sample_left = self.visit(node.children[0], *args, **kwargs)
        sample_right = self.visit(node.children[1], *args, **kwargs)

        out_samples = []
        sat_samples = []

        input_list = subtraction_operation(sample_left, sample_right)

        prev = float("nan")
        for i, in_sample in enumerate(input_list):
            if node.operator.value == StlComparisonOperator.EQ.value:
                sat_val = True if in_sample[1] == 0 else False
                out_val = - abs(in_sample[1])
            elif node.operator.value == StlComparisonOperator.NEQ.value:
                sat_val = True if abs(in_sample[1]) > 0 else False
                out_val = abs(in_sample[1])
            elif node.operator.value == StlComparisonOperator.LEQ.value:
                sat_val = True if in_sample[1] <= 0 else False
                out_val = - in_sample[1]
            elif node.operator.value == StlComparisonOperator.LESS.value:
                sat_val = True if in_sample[1] < 0 else False
                out_val = - in_sample[1]
            elif node.operator.value == StlComparisonOperator.GEQ.value:
                sat_val = True if in_sample[1] >= 0 else False
                out_val = in_sample[1]
            elif node.operator.value == StlComparisonOperator.GREATER.value:
                sat_val = True if in_sample[1] > 0 else False
                out_val = in_sample[1]
            else:
                out_val = float('nan')

            if out_val != prev or i == len(input_list) - 1:
                out_samples.append([in_sample[0], out_val])
                sat_samples.append([in_sample[0], sat_val])
            prev = out_val

        return out_samples, sat_samples

class IAStlOutputRobustnessDenseTimeOfflineAstVisitor(IAStlDenseTimeOfflineAstVisitor):

    def visitPredicate(self, node, *args, **kwargs):
        out_sample, sat_samples = IAStlDenseTimeOfflineAstVisitor.visitPredicate(self, node, *args, **kwargs)
        out = []

        if not node.out_vars:
            for i, sample in enumerate(sat_samples):
                val = float("inf") if sample == True else -float("inf")
                out.append([out_sample[i][0], val])
        else:
            out = out_sample

        return out


class IAStlInputRobustnessDenseTimeOfflineAstVisitor(IAStlDenseTimeOfflineAstVisitor):

    def visitPredicate(self, node, *args, **kwargs):
        out_sample, sat_samples = IAStlDenseTimeOfflineAstVisitor.visitPredicate(self, node, *args, **kwargs)
        out = []

        if not node.in_vars:
            for i, sample in enumerate(sat_samples):
                val = float("inf") if sample == True else - float("inf")
                out.append([out_sample[i][0], val])
        else:
            out = out_sample

        return out


class IAStlInputVacuityDenseTimeOfflineAstVisitor(IAStlDenseTimeOfflineAstVisitor):

    def visitPredicate(self, node, *args, **kwargs):
        out_sample, sat_samples = IAStlDenseTimeOfflineAstVisitor.visitPredicate(self, node, *args, **kwargs)
        out = []

        if not node.in_vars:
            for i, sample in enumerate(sat_samples):
                out.append([out_sample[i][0], 0.0])
        else:
            out = out_sample

        return out


class IAStlOutputVacuityDenseTimeOfflineAstVisitor(IAStlDenseTimeOfflineAstVisitor):

    def visitPredicate(self, node, *args, **kwargs):
        out_sample, sat_samples = IAStlDenseTimeOfflineAstVisitor.visitPredicate(self, node, *args, **kwargs)
        out = []

        if not node.out_vars:
            for i, sample in enumerate(sat_samples):
                out.append([out_sample[i][0], 0.0])
        else:
            out = out_sample

        return out
