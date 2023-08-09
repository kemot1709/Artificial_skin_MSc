import numpy as np
from item.item import ItemPlacement


def set_nearby_occupied(image, location):
    image_shape = list(image.shape)

    image[location[0], location[1]] = 1
    if location[0] - 1 >= 0:
        image[location[0] - 1, location[1]] = 1
    if location[1] - 1 >= 0:
        image[location[0], location[1] - 1] = 1
    if location[0] + 1 < image_shape[0]:
        image[location[0] + 1, location[1]] = 1
    if location[1] + 1 < image_shape[1]:
        image[location[0], location[1] + 1] = 1
    if location[0] - 1 >= 0 and location[1] - 1 >= 0:
        image[location[0] - 1, location[1] - 1] = 1
    if location[1] - 1 >= 0 and location[0] + 1 < image_shape[0]:
        image[location[0] + 1, location[1] - 1] = 1
    if location[0] + 1 < image_shape[0] and location[1] + 1 < image_shape[1]:
        image[location[0] + 1, location[1] + 1] = 1
    if location[1] + 1 < image_shape[1] and location[0] - 1 >= 0:
        image[location[0] - 1, location[1] + 1] = 1
    return image


def check_item_on_edge(image, image_mask):
    image_shape = list(image_mask.shape)
    border_pixels = np.zeros(image_shape)
    for i in range(image_shape[0]):
        for j in range(image_shape[1]):
            if not image_mask[i, j]:
                border_pixels = set_nearby_occupied(border_pixels, [i, j])
            if i == 0 or j == 0 or i == image_shape[0] - 1 or j == image_shape[1] - 1:
                border_pixels[i, j] = 1
    border_pixels = border_pixels.astype(int)

    # TODO check if object is on '1'
    for i in range(image_shape[0]):
        for j in range(image_shape[1]):
            if border_pixels[i, j] and image[i, j] > 10:
                return True
    return False


def recognise_position(image, image_mask, field_size):
    is_border = check_item_on_edge(image, image_mask)

    return ItemPlacement.unknown
