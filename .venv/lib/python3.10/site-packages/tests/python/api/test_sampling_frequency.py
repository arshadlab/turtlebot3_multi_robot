import unittest
import rtamt


class TestSamplingFrequency(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(TestSamplingFrequency, self).__init__(*args, **kwargs)


    def test_stl_discrete_time_offline(self):
        spec = rtamt.StlDiscreteTimeOfflineSpecification()
        spec.set_sampling_period(1, 'ms')
        freq = spec.get_sampling_frequency()
        self.assertEqual(1000.0, freq, 'Sampling frequency')

        spec = rtamt.StlDiscreteTimeOfflineSpecification()
        spec.set_sampling_period(100, 'ms')
        freq = spec.get_sampling_frequency()
        self.assertEqual(10.0, freq, 'Sampling frequency')

        spec = rtamt.StlDiscreteTimeOfflineSpecification()
        spec.set_sampling_period(10, 's')
        freq = spec.get_sampling_frequency()
        self.assertEqual(0.1, freq, 'Sampling frequency')

    def test_stl_discrete_time_online(self):
        spec = rtamt.StlDiscreteTimeOnlineSpecification()
        spec.set_sampling_period(1, 'ms')
        freq = spec.get_sampling_frequency()
        self.assertEqual(1000.0, freq, 'Sampling frequency')

        spec = rtamt.StlDiscreteTimeOnlineSpecification()
        spec.set_sampling_period(100, 'ms')
        freq = spec.get_sampling_frequency()
        self.assertEqual(10.0, freq, 'Sampling frequency')

        spec = rtamt.StlDiscreteTimeOnlineSpecification()
        spec.set_sampling_period(10, 's')
        freq = spec.get_sampling_frequency()
        self.assertEqual(0.1, freq, 'Sampling frequency')

    def test_stl_discrete_time(self):
        spec = rtamt.StlDiscreteTimeSpecification()
        spec.set_sampling_period(1, 'ms')
        freq = spec.get_sampling_frequency()
        self.assertEqual(1000.0, freq, 'Sampling frequency')

        spec = rtamt.StlDiscreteTimeSpecification()
        spec.set_sampling_period(100, 'ms')
        freq = spec.get_sampling_frequency()
        self.assertEqual(10.0, freq, 'Sampling frequency')

        spec = rtamt.StlDiscreteTimeSpecification()
        spec.set_sampling_period(10, 's')
        freq = spec.get_sampling_frequency()
        self.assertEqual(0.1, freq, 'Sampling frequency')


if __name__ == '__main__':
    unittest.main()