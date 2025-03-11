from rtamt.semantics.time_interpreter import TimeInterpreter

class DenseTimeInterpreter(TimeInterpreter):

    def __init__(self):
        super(DenseTimeInterpreter, self).__init__()
        return

    #input format
    #a = [[0, 1.3], [0.7, 3], [1.3, 0.1], [2.1, -2.2]]
    #b = [[0, 2.5], [0.7, 4], [1.3, -1.2], [2.1, 1.7]]
    #dataset = [['a', a], ['b', b]]
    def dataset_check(self, dataset):
        #TODO check that data fromat more.
        #TODO chage to dict format
        pass

    def set_variable_to_ast_from_dataset(self, dataset):
        for data in dataset:
            var_name = data[0]
            var_object = data[1]
            self.ast.var_object_dict[var_name] = var_object

    def time_unit_transformer(self, node):
        b = node.begin
        e = node.end
        b_unit = node.begin_unit
        e_unit = node.end_unit
        if len(node.begin_unit) == 0:
            if len(node.end_unit) > 0:
                b_unit = node.end_unit
            else:
                b_unit = self.ast.unit
                e_unit = self.ast.unit

        b = b * (self.ast.U[self.ast.unit] / self.ast.U[b_unit])
        e = e * (self.ast.U[self.ast.unit] / self.ast.U[e_unit])

        return b, e
