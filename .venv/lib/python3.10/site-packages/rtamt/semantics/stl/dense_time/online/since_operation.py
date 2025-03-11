from rtamt.semantics.abstract_dense_time_online_operation import AbstractDenseTimeOnlineOperation

class SinceOperation(AbstractDenseTimeOnlineOperation):
    def __init__(self):
        self.sample_left_buf = []
        self.sample_right_buf = []
        self.prev = -float("inf")
        self.last = []

    def reset(self):
        pass

    def update(self, sample_left, sample_right, *args, **kargs):
        sample_result = []
        a = self.sample_left_buf + sample_left
        b = self.sample_right_buf + sample_right

        i = j = 1

        last = self.last

        while len(a) > 1 and len(b) > 1:
            a_start = a[i - 1][0]
            a_end = a[i][0]
            b_start = b[j - 1][0]
            b_end = b[j][0]

            a_val = a[i - 1][1]
            b_val = b[j - 1][1]

            a_val_next = a[i][1]
            b_val_next = b[j][1]

            if a_end < b_end:
                last_val = max(min(a_val_next, b_val), min(a_val_next, self.prev))
                del (a[i - 1])
            elif a_end > b_end:
                last_val = max(min(a_val, b_val_next), min(a_val, self.prev))
                del (b[j - 1])
            else:
                last_val = max(min(a_val_next, b_val_next), min(a_val_next, self.prev))
                del (a[i - 1])
                del (b[j - 1])

            lo = max(a_start, b_start)
            hi = min(a_end, b_end)

            val = float("nan")
            if lo < hi:
                val = max(min(a_val, b_val), min(a_val, self.prev))
                #if not (self.last == [lo, val]):
                sample_result.append([lo, val])
                self.prev = val
                last = [hi, last_val]

        self.sample_left_buf = a
        self.sample_right_buf = b
        self.last = last

        return sample_result

    def update_final(self, sample_left, sample_right, *args, **kargs):
        return self.update(sample_left, sample_right, *args, **kargs) + [self.last]
