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


def selectDesiredItems(all_items, item_filter):
    selected_items = []

    for item in all_items:
        if item.type in item_filter:
            selected_items.append(item)

    return selected_items
