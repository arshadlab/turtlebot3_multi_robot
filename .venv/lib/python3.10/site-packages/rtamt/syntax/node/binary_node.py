from rtamt.syntax.node.abstract_node import AbstractNode


class BinaryNode(AbstractNode):

    def __init__(self, left_child, right_child):
        """Constructor for Node"""
        super(BinaryNode, self).__init__()
        self.add_child(left_child)
        self.add_child(right_child)



