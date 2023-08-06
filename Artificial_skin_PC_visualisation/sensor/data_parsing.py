import numpy as np
from cv_bridge import CvBridge


def cast_data_to_uint8(rows, columns, raw_data):
    data_uint8 = [[0 for x in range(columns)] for y in range(rows)]

    for i in range(columns):
        for j in range(rows):
            val_uint8 = int(255 - raw_data[i][j] * 255 / 4095)

            if val_uint8 > 255:
                val_uint8 = 255
            if val_uint8 < 0:
                val_uint8 = 0

            data_uint8[i][j] = val_uint8
    return data_uint8


def parse_data_to_np_image(rows, columns, raw_data):
    if not all(i <= 255 for i in raw_data):
        raw_data = cast_data_to_uint8(rows, columns, raw_data)
    np_image = np.array(raw_data, dtype=np.uint8)
    return np_image


def parse_np_image_to_msg(np_image):
    bridge = CvBridge()
    msg = bridge.cv2_to_imgmsg(np_image, "mono8")
    return msg


def mask_np_image(np_image, mask):
    # TODO
    pass


def process_raw_image_through_calibration(rows, columns, image, calibration_image):
    # TODO implement all possible compensation methods
    mult_b = 0.98
    calibrated_image = [[0 for x in range(columns)] for y in range(rows)]

    for i in range(columns):
        for j in range(rows):
            calibrated_image[i][j] = image[i][j]
            if calibrated_image[i][j] > mult_b * calibration_image[i][j]:
                calibrated_image[i][j] = mult_b * calibration_image[i][j]
            calibrated_image[i][j] = calibrated_image[i][j] * 4095 / (mult_b * calibration_image[i][j])
    return calibrated_image
