from rtamt.syntax.node.binary_node import BinaryNode

class Since(BinaryNode):
    """A class for storing STL Since nodes
                Inherits TemporalNode
    """
    def __init__(self, child1, child2):
        """Constructor for Since node

            Parameters:
                child1 : stl.Node
                child2 : stl.Node
        """

        super(Since, self).__init__(child1, child2)

        self.in_vars = child1.in_vars + child2.in_vars
        self.out_vars = child1.out_vars + child2.out_vars
        self.name = '(' + child1.name + ')since(' + child2.name + ')'


