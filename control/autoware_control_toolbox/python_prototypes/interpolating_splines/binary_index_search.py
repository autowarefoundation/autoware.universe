import numpy as np
import time


def binary_search(z, nstart, nend) -> (int, int):
    # locate the interval of z given a range

    left = nstart
    right = nend

    if z == nend:
        zleft = nend - 1
        zright = nend

    else:
        # binary search
        while (right > left + 1):
            mid: int = np.floor((left + right) / 2)
            if z < mid:
                right = mid

            else:
                left = mid

        zleft = left
        zright = right

    return int(zleft), int(zright)


if __name__ == "__main__":
    ntop = 400;
    nlist = np.arange(1, ntop)

    z = np.random.randint(0, ntop) + np.random.randn()
    # z = 55

    zleft, zright = binary_search(z, 0, ntop)
    print(f'{zleft} < {z} < {zright}')
