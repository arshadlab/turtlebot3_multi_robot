import unittest

from rtamt.spec.stl.discrete_time.specification import StlDiscreteTimeOfflineSpecification
from rtamt.spec.stl.dense_time.specification import StlDenseTimeOfflineSpecification
from rtamt.spec.stl.discrete_time.specification import StlDiscreteTimeOnlineSpecification
from rtamt.spec.stl.dense_time.specification import StlDenseTimeOnlineSpecification


class TestSemantics(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(TestSemantics, self).__init__(*args, **kwargs)


    def test_stl_discrete_time_offline(self):
        spec = StlDiscreteTimeOfflineSpecification()
        spec.declare_var('a', 'float')
        spec.spec = 'always(a>=2)'
        spec.parse()

        dataset = {
            'time': [0, 1, 2, 3, 4],
            'a': [100, -1, -2, 5, -1]
        }
        rob = spec.evaluate(dataset)

    def test_stl_dense_time_offline(self):
        spec = StlDenseTimeOfflineSpecification()
        spec.declare_var('a', 'float')
        spec.spec = 'always(a>=2)'
        spec.parse()

        a = [[0, 1.3], [0.7, 3], [1.3, 0.1], [2.1, -2.2]]
        rob = spec.evaluate(['a', a])

    def test_stl_discrete_time_online(self):
        spec = StlDiscreteTimeOnlineSpecification()
        spec.declare_var('a', 'float')
        spec.spec = 'historically[0,1](a>=2)'
        spec.parse()

        a = [100, -1, -2, 5, -1]
        robs = []

        for i in range(len(a)):
            rob = spec.update(i, [('a', a[i])])
            robs.append(rob)

        spec.reset()

    def test_stl_dense_time_online(self):
        spec = StlDenseTimeOnlineSpecification()
        spec.declare_var('a', 'float')
        spec.spec = 'historically[0,1](a>=2)'
        spec.parse()

        a = []
        a.append([[5, 3], [5.3, 1]])
        a.append([[5.75, 2], [6.5, 5], [6.75, 6], [9, 5], [9.25, 4]])
        a.append([[10, 2]])

        robs = []
        for i in range(len(a)):
            rob = spec.update(['a', a[i]])
            robs.append(rob)

        # spec.reset()

if __name__ == '__main__':
    unittest.main()