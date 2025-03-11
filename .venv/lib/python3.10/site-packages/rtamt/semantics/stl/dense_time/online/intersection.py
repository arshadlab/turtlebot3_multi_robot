import math

def interval_union(a, b, method):
    out = []

    a_min = a[0]
    a_max = a[1]
    b_min = b[0]
    b_max = b[1]

    lo = max(a_min, b_min)
    hi = min(a_max, b_max)

    if lo <= hi:
        val = method(a[2], b[2])
        out.append((lo, hi, val))
        out.append((hi, b_max, b[2]))
    else:
        out.append(b)

    return b


def union(a_list, b):
    i = len(a_list)
    out = []

    if not a_list:
        out = [b]

    while i > 0:
        a = a_list[i]
        if b[0] >= a[0]:
            current = interval_union(a, b)
            out.append(current)
            i = i - 1
        else:
            i = 0
    return out;


def intersects(x1, x2, y1, y2):
    if x1 <= y2 and y1 <= x2:
        return True
    else:
        return False

def intersection(a, b, method):
    ans = []
    a = list(a)
    b = list(b)
    i = j = 1

    hi = None

    prev = float("nan")
    last = []
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
            last_val = method(a_val_next, b_val)
            del (a[i - 1])
        elif a_end > b_end:
            last_val = method(a_val, b_val_next)
            del (b[j - 1])
        else:
            last_val = method(a_val_next, b_val_next)
            del (a[i - 1])
            del (b[j - 1])

        lo = max(a_start, b_start)
        hi = min(a_end, b_end)

        val = float("nan")

        if lo < hi:
            val = method(a_val, b_val)
            if val != prev:
                ans.append([lo, val])
            prev = val
            last = [hi, last_val]

    return ans, last, a, b


def disjunction(a, b):
    return max(a, b)


def conjunction(a, b):
    return min(a, b)


def implication(a, b):
    return max(-a, b)


def xor(a, b):
    return abs(a - b)


def iff(a, b):
    return -abs(a - b)


def addition(a, b):
    return a + b


def subtraction(a, b):
    return a - b


def multiplication(a, b):
    return a * b

def power(a, b):
    return math.pow(a, b)


def division(a, b):
    return float(a) / float(b)

def split(a, b):
    return [a,b]