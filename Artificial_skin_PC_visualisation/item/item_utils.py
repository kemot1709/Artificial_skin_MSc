import os
from item.item import Item


def loadItems(path, mask=None):
    item_id = 1
    item_list = []

    files = os.listdir(path)
    for file in files:
        # Omit calibration images
        if file.startswith("c_"):
            continue

        new_item = Item(mask)
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


def selectDesiredPlacement(all_items, placement_filter):
    selected_items = []

    for item in all_items:
        if item.placement in placement_filter:
            selected_items.append(item)

    return selected_items


def selectCorruptedItems(all_items):
    corrupted_items = []

    for item in all_items:
        if item.potentially_corrupted is True:
            corrupted_items.append(item)

    return corrupted_items


def ommitCorruptedItems(all_items):
    good_items = []

    for item in all_items:
        if item.potentially_corrupted is False:
            good_items.append(item)

    return good_items
