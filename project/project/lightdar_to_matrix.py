import numpy as np
import pandas as pd
import math


def _round_up_of_half(number):
    # rounds up to nearest half
    return math.ceil(number * 2) / 2


def _cenvert_line_to_matrix(map, alhpa, lenght, pose_x, pose_y):
    if map == None:
        map = np.zeros((1000, 1000))
        map.fill(3)

    for i in range(1, int(_round_up_of_half(lenght) * 2) + 1, 1):
        if i / 2 < lenght:
            x = math.floor(math.cos(alhpa) * (i / 2)) + pose_x
            y = math.floor(math.sin(alhpa) * (i / 2)) + pose_y
            print(f"x:{x}", f"y:{y}", "free")
            if map[y, x] == 3:
                map[y, x] = 1

        if i / 2 > lenght:
            x = math.floor(math.cos(alhpa) * (i / 2)) + pose_x
            y = math.floor(math.sin(alhpa) * (i / 2)) + pose_y
            print(f"x:{x}", f"y:{y}", "occupied")
            map[y, x] = 2

    return map


if __name__ == "__main__":
    map = _cenvert_line_to_matrix(None, 1.047, 2.23, 2, 2)

    print(map)
