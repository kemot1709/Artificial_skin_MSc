from keras.models import Sequential, load_model
from keras.layers import Conv2D, MaxPooling2D, Flatten, Dense

import os
import pickle
import pandas as pd
import numpy as np

from item.item import ItemType
from debug.debug import *


class Classifier:
    model = None
    output_types = None

    def __init__(self, num_classes=2, class_items=[ItemType.unknown, ItemType.unknown]):
        # Define the model architecture
        self.model = Sequential()
        self.model.add(Conv2D(32, (3, 3), activation='relu', input_shape=(16, 16, 1)))
        self.model.add(MaxPooling2D((2, 2)))
        # self.model.add(Conv2D(64, (3, 3), activation='relu'))
        # self.model.add(MaxPooling2D((2, 2)))
        # self.model.add(Conv2D(64, (3, 3), activation='relu'))
        self.model.add(Flatten())
        self.model.add(Dense(64, activation='relu'))
        self.model.add(Dense(num_classes, activation='softmax'))

        # Compile the model
        self.model.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])

        self.output_types = class_items

    def trainModel(self, x_train, y_train, x_val, y_val):
        self.model.fit(x_train, y_train, batch_size=32, epochs=10, validation_data=(x_val, y_val))

    def predict(self, images):
        # For every image returns array of probabilities for that item
        predictions = self.model.predict(images)
        return predictions

    def predict_items_with_confidence(self, images, confidence_treshold, map_of_types):
        try:
            # For every image returns one most probable item, if probability value is greater than treshold
            predictions = self.model.predict(images)
            item_predictions = []
            for prediction in predictions:
                max_val = np.argmax(prediction)
                if prediction[max_val] > confidence_treshold:
                    item_predictions.append(ItemType(map_of_types[max_val]))
                else:
                    item_predictions.append(ItemType.unknown)
            return item_predictions
        except:
            debug(DBGLevel.ERROR, "Item prediction failed")
            return [[ItemType.unknown]]

    def export_model(self, filename):
        filename = os.path.splitext(filename)[0]
        # self.model.summary()
        self.model.save(filename + ".keras")
        with open(str(filename + ".names"), 'wb') as pickle_file:
            pickle.dump(self.output_types, pickle_file)
        debug(DBGLevel.WARN, "Model successfully exported")

    def import_model(self, filename):
        filename = os.path.splitext(filename)[0]
        self.model = load_model(filename + ".keras")
        # self.model.summary()
        with open(str(filename + ".names"), 'rb') as pickle_file:
            self.output_types = pickle.load(pickle_file)
        debug(DBGLevel.WARN, "Model successfully imported")

    def evaluate(self, images, labels):
        loss, accuracy = self.model.evaluate(images, labels)
        return accuracy

    def evaluationTable(self, images, labels):
        predictions = self.predict(images)

        # TODO change to panda
        # df = pd.DataFrame(predictions)

        pre_sum = np.zeros((labels.shape[1], labels.shape[1]))
        pre_nr = np.zeros((labels.shape[1], 1))

        for i, line in enumerate(predictions, start=0):
            np.add.at(pre_sum, np.argmax(labels[i]), line)
            np.add.at(pre_nr, np.argmax(labels[i]), 1)

        predictions_merged = pd.DataFrame(pre_sum / pre_nr)
        return predictions_merged
