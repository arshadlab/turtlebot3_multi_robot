import unittest
import rtamt

class TestEnums(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestEnums, self).__init__(*args, **kwargs)

    def test_enums(self):
        semantics = rtamt.Semantics.STANDARD
        language = rtamt.Language.PYTHON
        time_interpretation = rtamt.TimeInterpretation.DISCRETE
        self.assertEqual('standard', str(semantics), 'Semantics')
        self.assertEqual('python', str(language), 'Language')
        self.assertEqual('discrete_time', str(time_interpretation), 'TimeInterpretation')

if __name__ == '__main__':
    unittest.main()
