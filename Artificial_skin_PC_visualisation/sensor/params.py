import cv2
import numpy as np


class Params:
    field_dim1 = 10
    field_dim2 = 20
    field_cover_dim1 = 15
    field_cover_dim2 = 25
    field_cover_area = field_cover_dim1 * field_cover_dim2


class ImageMask:
    mask = np.zeros([16, 16])
    roi_points = np.array([[0, 7], [4, 0], [11, 0], [15, 7], [15, 15], [0, 15]])
    cv2.fillPoly(mask, [roi_points], 1)

    def getMask(self):
        return self.mask
