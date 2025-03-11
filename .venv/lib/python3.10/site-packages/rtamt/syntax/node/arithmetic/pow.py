from rtamt.syntax.node.binary_node import BinaryNode

class Pow(BinaryNode):
    """A class for storing Pow nodes
        Inherits Node
    """
    def __init__(self, child1, child2):
        """Constructor for Pow node

            Parameters:
                child1 : stl.Node
                child2 : stl.Node
        """
        super(Pow, self).__init__(child1, child2)

        self.in_vars = child1.in_vars + child2.in_vars
        self.out_vars = child1.out_vars + child2.out_vars

        self.name = 'pow(' + child1.name + ',' + child2.name + ')'


