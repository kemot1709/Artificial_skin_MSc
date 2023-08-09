import numpy as np
import pandas as pd

from item.item_utils import loadItems, selectDesiredItems, selectDesiredPlacement
from item.item import ItemType, ItemPlacement
from classifier.image_utils import ImageParser, LabelsMap, splitDataToTraining
from classifier.image_recognition import Classifier

path = "c_img_v2"

itemList = loadItems(path)
# itemList = selectDesiredItems(itemList, [ItemType.book, ItemType.mug_full, ItemType.mug_empty, ItemType.plate_full,
#                                          ItemType.plate_empty, ItemType.phone])
# itemList = selectDesiredPlacement(itemList, [ItemPlacement.center, ItemPlacement.side])
[trainingSet, validationSet, testSet] = splitDataToTraining(itemList, 6, 1, 1)

# Parsing images and labels so keras can use them
parser = ImageParser()
x_train = parser.parseImagesToArray(trainingSet)
y_train = parser.parseLabelsToArray(trainingSet)
x_val = parser.parseImagesToArray(validationSet)
y_val = parser.parseLabelsToArray(validationSet)
x_test = parser.parseImagesToArray(testSet)
y_test = parser.parseLabelsToArray(testSet)

classifier = Classifier(y_train.shape[1])
classifier.trainModel(x_train, y_train, x_val, y_val)
table = classifier.evaluationTable(x_test, y_test)

column_names = parser.parseOrdinalNumbersToNames(list(range(0, y_train.shape[1])))
table.columns = column_names
table.index = column_names
print(table.to_string())

elo = classifier.predict(np.array([x_test[5]]))
eluwina = pd.DataFrame(elo)
eluwina.columns = column_names
print(eluwina.to_string())
print(testSet[5].type)
