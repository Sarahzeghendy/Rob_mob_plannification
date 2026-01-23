import numpy as np
import cv2

def occgrid_to_2d(data_1d, width, height):
    arr = np.array(data_1d, dtype=np.int16).reshape((height, width))
    return arr

def inflate_occupancy(occ_2d: np.ndarray, inflation_radius_cells: int, occ_thresh: int = 50):
    if inflation_radius_cells <= 0:
        return occ_2d.copy()

    obstacle_mask = (occ_2d >= occ_thresh).astype(np.uint8)  
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (2 * inflation_radius_cells + 1, 2 * inflation_radius_cells + 1)
    )
    inflated_mask = cv2.dilate(obstacle_mask, kernel)

    out = occ_2d.copy()
    out[inflated_mask == 1] = 100
    return out

def world_to_cell(x, y, origin_x, origin_y, res):
    c = int((x - origin_x) / res)
    r = int((y - origin_y) / res)
    return (r, c)

def cell_to_world(r, c, origin_x, origin_y, res):
    x = origin_x + (c + 0.5) * res
    y = origin_y + (r + 0.5) * res
    return (x, y)
