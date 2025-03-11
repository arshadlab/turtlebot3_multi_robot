from rtamt.syntax.node.unary_node import UnaryNode

class Exp(UnaryNode):
    """A class for storing Exp nodes
        Inherits Node
    """
    def __init__(self, child):
        """Constructor for Exp node

            Parameters:
                child : stl.Node
        """
        super(Exp, self).__init__(child)

        self.in_vars = child.in_vars
        self.out_vars = child.out_vars

        self.name = 'exp(' + child.name + ')'


