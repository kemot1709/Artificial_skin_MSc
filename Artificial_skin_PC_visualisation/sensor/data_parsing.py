import numpy as np


def flatten(seq):
    out_list = []
    for element in seq:
        element_type = type(element)
        if element_type is tuple or element_type is list or element_type is np.ndarray:
            for element_in_element in flatten(element):
                out_list.append(element_in_element)
        else:
            out_list.append(element)
    return out_list


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
    for i in flatten(raw_data):
        if not i <= 255:
            raw_data = cast_data_to_uint8(rows, columns, raw_data)
            break
    np_image = np.array(raw_data, dtype=np.uint8)
    return np_image


def mask_np_image(np_image, mask):
    # TODO
    pass


def compensate_raw_image(rows, columns, image_to_compensate, calibration_image):
    # TODO implement all possible compensation methods
    mult_b = 0.98
    calibrated_image = [[0 for x in range(columns)] for y in range(rows)]

    for i in range(columns):
        for j in range(rows):
            calibrated_image[i][j] = image_to_compensate[i][j]
            if calibrated_image[i][j] > mult_b * calibration_image[i][j]:
                calibrated_image[i][j] = mult_b * calibration_image[i][j]
            calibrated_image[i][j] = calibrated_image[i][j] * 4095 / (mult_b * calibration_image[i][j])
    return calibrated_image
