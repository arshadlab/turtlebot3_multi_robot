class RTAMTException(Exception):
    def __init__(self, *args):
        if args:
            self.message = args[0]
        else:
            self.message = None

    def __str__(self):
        if self.message:
            return 'RTAMT Exception: {0} '.format(self.message)
        else:
            return 'RTAMT Exception has been raised'

