from rtamt.syntax.node.unary_node import UnaryNode

class Fall(UnaryNode):
    """A class for storing STL Neg nodes
        Inherits Node
    """
    def __init__(self, child):
        """Constructor for Neg node

            Parameters:
                child : stl.Node
        """
        super(Fall, self).__init__(child)
        self.in_vars = child.in_vars
        self.out_vars = child.out_vars

        self.name = 'fall(' + child.name + ')'




