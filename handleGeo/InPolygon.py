from numba import njit

@njit
def is_inside_polygon(point, polygon_coords):
    xp, yp = point
    cnt = 0
    n = len(polygon_coords)

    for i in range(n):
        x1, y1 = polygon_coords[i]
        x2, y2 = polygon_coords[(i + 1) % n]

        if (yp < y1) != (yp < y2) and xp < x1 + ((yp - y1) / (y2 - y1)) * (x2 - x1):
            cnt += 1

    return cnt % 2 == 1

