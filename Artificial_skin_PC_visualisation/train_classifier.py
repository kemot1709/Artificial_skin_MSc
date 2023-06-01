from item.utils import loadItems
from classifier.utils import ImageParser, LabelsMap, splitDataToTraining
from classifier.image_recognition import Classifier

path = "c_img"

itemList = loadItems(path)
[trainingSet, validationSet, testSet] = splitDataToTraining(itemList, 4, 2, 2)

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
# classifier.predict(x_test)
table = classifier.evaluationTable(x_test, y_test)

column_names = parser.parseOrdinalNumbersToNames(list(range(0, y_train.shape[1])))
table.columns = column_names
table.index = column_names
print(table.to_string())
