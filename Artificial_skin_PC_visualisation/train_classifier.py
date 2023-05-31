import os
import numpy as np
from collections import Counter

from item.item import Item
from classifier.utils import ImageParser, LabelsMap
from classifier.image_recognition import Classifier

global_id = 1
path = "c_img"
itemList = []
trainingSet = []
validationSet = []
testSet = []

files = os.listdir(path)
for file in files:
    # Omit calibration images
    if file.startswith("c_"):
        continue

    newItem = Item()
    newItem.getLabelsFromFilename(path, file, global_id)
    itemList.append(newItem)

    if global_id % 4 == 0:
        if global_id % 8 == 0:
            validationSet.append(newItem)
        else:
            testSet.append(newItem)
    else:
        trainingSet.append(newItem)

    global_id = global_id + 1

parser = ImageParser()

x_train = parser.parseImagesToArray(trainingSet)
y_train = parser.parseLabelsToArray(trainingSet)
x_val = parser.parseImagesToArray(validationSet)
y_val = parser.parseLabelsToArray(validationSet)
x_test = parser.parseImagesToArray(testSet)
y_test = parser.parseLabelsToArray(testSet)

num_classes = y_train.shape[1]

classifier = Classifier(num_classes)
classifier.trainModel(x_train, y_train, x_val, y_val)
# classifier.predict(x_test)
table = classifier.evaluationTable(x_test, y_test)

dupa = parser.parseOrnToNames([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10])

table.columns = dupa
table.index = dupa
pass
