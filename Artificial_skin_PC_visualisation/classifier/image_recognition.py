from keras.models import Sequential, load_model
from keras.layers import Conv2D, MaxPooling2D, Flatten, Dense

import pandas as pd
import numpy as np


class Classifier:
    model = None

    def __init__(self, num_classes=2):
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

    def trainModel(self, x_train, y_train, x_val, y_val):
        self.model.fit(x_train, y_train, batch_size=32, epochs=10, validation_data=(x_val, y_val))

    def predict(self, images):
        predictions = self.model.predict(images)
        return predictions

    def export_model(self, filename):
        self.model.save(filename)

    def import_model(self, filename):
        self.model = load_model(filename)

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
