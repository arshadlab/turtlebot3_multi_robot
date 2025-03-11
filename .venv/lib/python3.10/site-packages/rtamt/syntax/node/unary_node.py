from rtamt.syntax.node.abstract_node import AbstractNode


class UnaryNode(AbstractNode):

    def __init__(self, child):
        """Constructor for Node"""
        super(UnaryNode, self).__init__()
        self.add_child(child)



