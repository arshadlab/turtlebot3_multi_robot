import logging

class Interval(object):
    """A class for storing integer intervals

        Attributes
        --------------
        begin : int
            beginning of the interval
        end : int
            end of the interval

        Methods
        --------------
        begin, end
            Getter for begin and end
            The object is immutable - the setter issues a warning message and does nothing
        """
    def __init__(self, begin, end, begin_unit="", end_unit=""):
        """Constructor for Interval
        Parameters:
            begin : int
                Beginning of the interval
            end : int
                End of the interval
        """
        self.begin = begin
        self.end = end
        self.begin_unit = begin_unit
        self.end_unit = end_unit

    @property
    def begin(self):
        """Getter for begin"""
        return self.__begin

    @begin.setter
    def begin(self, begin):
        self.__begin = begin

    @property
    def end(self):
        """Getter for end"""
        return self.__end

    @end.setter
    def end(self, end):
        self.__end = end

    @property
    def begin_unit(self):
        """Getter for begin_unit"""
        return self.__begin_unit

    @begin_unit.setter
    def begin_unit(self, begin_unit):
        self.__begin_unit = begin_unit

    @property
    def end_unit(self):
        """Getter for end_unit"""
        return self.__end_unit

    @end_unit.setter
    def end_unit(self, end_unit):
        self.__end_unit = end_unit