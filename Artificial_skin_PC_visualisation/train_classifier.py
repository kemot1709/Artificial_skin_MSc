import os
from item.item import Item


global_id = 1
path = "c_img"
itemList = []
trainingSet = []
validationSet = []

files = os.listdir(path)
for file in files:
    # Omit calibration images
    if file.startswith("c_"):
        continue

    newItem = Item()
    newItem.getLabelsFromFilename(path, file, global_id)
    itemList.append(newItem)

    if global_id % 4 == 0:
        validationSet.append(newItem)
    else:
        trainingSet.append(newItem)

    global_id = global_id + 1
