from rtamt.syntax.node.unary_node import UnaryNode

class Historically(UnaryNode):
    """A class for storing STL Historically nodes
         Inherits TemporalNode
    """
    def __init__(self, child):
        """Constructor for Historically node

            Parameters:
                child : stl.Node
        """
        super(Historically, self).__init__(child)
        self.in_vars = child.in_vars
        self.out_vars = child.out_vars
        self.name = 'historically(' + child.name + ')'

