import os
from item.item import Item


def loadItems(path):
    item_id = 1
    item_list = []

    files = os.listdir(path)
    for file in files:
        # Omit calibration images
        if file.startswith("c_"):
            continue

        new_item = Item()
        new_item.getLabelsFromFilename(path, file, item_id)
        item_list.append(new_item)
        item_id += 1
    return item_list
