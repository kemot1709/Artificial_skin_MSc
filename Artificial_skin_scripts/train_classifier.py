import numpy as np
import pandas as pd

from item.item_utils import loadItems, selectDesiredItems, selectDesiredPlacement
from item.item import ItemType, ItemPlacement
from classifier.image_utils import ImageParser, LabelsMap, splitDataToTraining
from classifier.image_recognition import Classifier
from languages.en import itemTranslationDict

path = "c_img_v2"

itemList = loadItems(path)
itemList = selectDesiredItems(itemList, [ItemType.book,
                                         ItemType.food_tray,
                                         ItemType.mug_full, ItemType.mug_empty,
                                         ItemType.plate_full, ItemType.plate_empty, ItemType.phone, ItemType.drug,
                                         ItemType.hand_any, ItemType.hand_hard, ItemType.hand_mid, ItemType.hand_light])
itemList = selectDesiredPlacement(itemList, [ItemPlacement.center, ItemPlacement.side])

for item in itemList:
    if item.type == ItemType.mug_empty or item.type == ItemType.mug_full:
        item.type = ItemType.mug_any
    if item.type == ItemType.plate_empty or item.type == ItemType.plate_full:
        item.type = ItemType.plate_any
    if item.type == ItemType.hand_light or item.type == ItemType.hand_mid or item.type == ItemType.hand_hard:
        item.type = ItemType.hand_any

[trainingSet, validationSet, testSet] = splitDataToTraining(itemList, 6, 1, 1)

# Parsing images and labels so keras can use them
parser = ImageParser()
x_train = parser.parseImagesToArray(trainingSet)
y_train = parser.parseLabelsToArray(trainingSet)
x_val = parser.parseImagesToArray(validationSet)
y_val = parser.parseLabelsToArray(validationSet)
x_test = parser.parseImagesToArray(testSet)
y_test = parser.parseLabelsToArray(testSet)

classifier = Classifier(y_train.shape[1], parser.parseOrdinalNumbersToItemTypes(list(range(0, y_train.shape[1]))))
###############
classifier.import_model("classifier/models/test_model.keras")
# classifier.trainModel(x_train, y_train, x_val, y_val)
###############
table = classifier.evaluationTable(x_test, y_test)

column_names = parser.parseOrdinalNumbersToNames(list(range(0, y_train.shape[1])))
table.columns = column_names
table.index = column_names
print(table.to_string())

im = np.array([x_test[1]])
elo = classifier.predict(im)
elo_ = classifier.predict_items_with_confidence(im, 0.8, classifier.output_types)
eluwina = pd.DataFrame(elo)
eluwina.columns = column_names
print(eluwina.to_string())
print(testSet[5].type)
print(itemTranslationDict[elo_[0]])

# classifier.export_model("classifier/models/test_model.keras")
