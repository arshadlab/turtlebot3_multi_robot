from rtamt.syntax.node.unary_node import UnaryNode

class Previous(UnaryNode):
    """A class for storing STL Previous nodes
        Inherits Node
    """
    def __init__(self, child):
        """Constructor for Previous node

            Parameters:
                child : stl.Node
        """
        super(Previous, self).__init__(child)
        self.in_vars = child.in_vars
        self.out_vars = child.out_vars

        self.name = 'previous(' + child.name + ')'




