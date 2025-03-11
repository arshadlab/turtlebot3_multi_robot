class Time:
    def __init__(self):
        self.sec = 0
        self.msec = 0

    @property
    def sec(self):
        return self.__sec

    @sec.setter
    def sec(self, sec):
        self.__sec = sec

    @property
    def msec(self):
        return self.__msec

    @msec.setter
    def msec(self, msec):
        self.__msec = msec


class Sample:
    def __init__(self):
        self.seq = 0;
        self.time = Time()
        self.value = 0.0

    @property
    def seq(self):
        return self.__seq

    @seq.setter
    def seq(self, seq):
        self.__seq = seq

    @property
    def time(self):
        return self.__time

    @time.setter
    def time(self, time):
        self.__time = time

    @property
    def value(self):
        return self.__value

    @value.setter
    def value(self, value):
        self.__value = value