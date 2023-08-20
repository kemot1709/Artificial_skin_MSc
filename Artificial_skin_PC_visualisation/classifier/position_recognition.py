import numpy as np
import cv2
import matplotlib.pyplot as plt

from item.item import ItemPlacement, ItemShape, ItemType
from sensor.params import ImageMask
from debug import debug, DBGLevel


def get_image_centroid(image, output_type):
    moments = cv2.moments(image)
    centroid_x = (moments["m10"] / moments["m00"])
    centroid_y = (moments["m01"] / moments["m00"])

    if output_type is float:
        centroid_x = round(centroid_x, 2)
        centroid_y = round(centroid_y, 2)
    elif output_type is int:
        centroid_x = int(round(centroid_x))
        centroid_y = int(round(centroid_y))
    else:
        centroid_x = int(round(centroid_x))
        centroid_y = int(round(centroid_y))

    return [centroid_x, centroid_y]


def stretch_image(image, stretch_x, stretch_y):
    image = np.uint8(image)
    stretched_image = cv2.resize(image, None, fx=stretch_x, fy=stretch_y, interpolation=cv2.INTER_LINEAR)
    stretched_image = np.uint8(stretched_image)
    return stretched_image


def find_histogram_peak(histogram):
    idx_max_val = np.argmax(histogram)
    idx_all_max_vals = np.where(histogram == histogram[idx_max_val])

    # Multipeak
    if len(idx_all_max_vals[0]) > 1:
        for i in range(len(idx_all_max_vals[0]) - 1):
            if idx_all_max_vals[0][i + 1] - idx_all_max_vals[0][i] != 1:
                return -1

        # Neighbour peaks
        idx_avg = np.average(idx_all_max_vals)
        return idx_avg

    return idx_max_val


def get_histogram_of_weight_from_point(image, point):
    # Calculate the distances from each white pixel to the centroid on the original image
    distances = np.uint8(np.round(np.sqrt(
            (np.where(image > 0)[1] - point[0]) ** 2 + (
                    np.where(image > 0)[0] - point[1]) ** 2)))

    # Collect intensity values of the original image at corresponding distances
    intensity_values = np.uint32(image[image > 0])

    nrs, vals = np.unique(distances, return_counts=True)

    # If there is lack of some values add them to make full histogram (especially near 0)
    if len(nrs) is not max(nrs) + 1:
        new_nrs = []
        new_vals = []
        for i in range(max(nrs) + 1):
            new_nrs.append(i)
            if i in nrs:
                nr_id = np.where(nrs == i)[0][0]
                new_vals.append(vals[nr_id])
            else:
                new_vals.append(1)
        nrs = np.array(new_nrs)
        vals = np.array(new_vals)

    # Create a histogram of intensity values at corresponding distances
    hist_values, bins = np.histogram(distances, bins=len(nrs), weights=intensity_values, range=(0, nrs.max()))
    hist_values_weight = np.uint16(np.round(hist_values / vals))

    # Plot the cumulative histogram
    # plt.plot(bins[:-1], hist_values_weight)
    # plt.xlabel('Distance from Center')
    # plt.ylabel('Cumulative Intensity')
    # plt.title('Cumulative Histogram of Intensity vs. Distance from Center')
    # plt.show()

    return hist_values_weight


def check_item_on_edge(image, mask):
    image = np.uint8(image)
    max_image_val = np.max(image)
    e_image = np.pad(image, [(1, 1), (1, 1)], mode='constant', constant_values=0)
    e_image_shape = list(e_image.shape)

    for i in range(1, e_image_shape[0] - 1):
        for j in range(1, e_image_shape[1] - 1):
            if mask.e_bordered_mask[i, j] == 0 and e_image[i, j] > max_image_val / 10.0:
                # Bordered mask must be used, because normal mask is used to trimming image
                return True
    return False


def random_staff(image, mask, item=None):
    image = np.uint8(image)
    stretched_e_image = stretch_image(image, 1.5, 2.5)
    # blur = cv2.blur(image, (3,3))
    blur = cv2.GaussianBlur(stretched_e_image, (3, 3), 0)
    edges = cv2.Canny(blur, 10, 40)

    e_image = np.pad(edges, [(1, 1), (1, 1)], mode='constant', constant_values=0)
    e_image_shape = list(e_image.shape)

    # Find circles
    # circles = cv2.HoughCircles(
    #         e_image, cv2.HOUGH_GRADIENT, dp=.5, minDist=3,
    #         param1=40, param2=2)
    #
    # circ = np.zeros(list(stretched_e_image.shape))
    # circ = np.uint8(circ)
    # if circles is not None:
    #     circles = np.uint8(np.around(circles))
    #     for circle in circles[0, :]:
    #         center = (circle[0], circle[1])
    #         radius = circle[2]
    #         cv2.circle(circ, center, radius, 255)
    #     return True
    # elif item.shape is ItemShape.round and item.type is not ItemType.drug:
    #     return False
    # return False

    return True


def find_sides_of_table(mask_image):
    # Get last column and find first and the last '1'
    last_col = mask_image[:, -1]
    side_edges = np.argwhere(np.diff(np.r_[0, last_col, 0])).reshape(-1, 2)
    if len(side_edges) > 1:
        debug(DBGLevel.WARN, "Table have some border inconsistance")

    edge = side_edges[0]
    edge[1] -= 1
    return edge


def recognise_position(image, image_mask, field_size):
    s_image = stretch_image(image, field_size[0], field_size[1])  # field size = [1.5, 2.5]
    # TODO clean this section
    # s_image_mask = stretch_image(image_mask, field_size[0], field_size[1])
    mask = ImageMask()
    s_image_mask = mask.getStretchedMask()
    ##############################
    s_side_edges = find_sides_of_table(s_image_mask)

    is_border = check_item_on_edge(image, mask)
    s_centroid_point = get_image_centroid(s_image, int)
    hist = get_histogram_of_weight_from_point(s_image, s_centroid_point)
    peak = find_histogram_peak(hist)

    if not is_border:
        if s_centroid_point[1] < s_side_edges[0] or s_centroid_point[1] > s_side_edges[1]:
            return ItemPlacement.side
        else:
            return ItemPlacement.center

    return ItemPlacement.unknown
