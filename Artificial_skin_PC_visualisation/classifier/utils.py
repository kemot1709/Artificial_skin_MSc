import numpy as np
import pandas as pd
import keras

from item.item import Item, ItemType
from item.en import itemTranslationDict


class LabelsMap:
    instance = None
    map = None
    reverse_map = None

    # Singleton
    def __new__(cls, *args, **kwargs):
        if not isinstance(cls.instance, cls):
            cls.instance = object.__new__(cls)
        return cls.instance

    def __init__(self, labelList):
        self.setMapFromLabels(labelList)

    def setMapFromLabels(self, labellist):
        df = pd.DataFrame([labellist])
        v = df.stack().unique()
        v.sort()
        f = pd.factorize(v)
        self.map = pd.Series(f[0], f[1])
        self.reverse_map = pd.Series(f[1], f[0])

    def mapLabelsToOrdinalNumbers(self, labellist):
        df = pd.DataFrame([labellist])
        ordinalNumbers = df.stack().map(self.map).unstack()
        return ordinalNumbers

    def mapOrdinalNumbersToLabels(self, ordinalNumbers):
        df = pd.DataFrame([ordinalNumbers])
        labels = df.stack().map(self.reverse_map).unstack()
        return labels

    def mapOrdinalNumbersToLabelNames(self, ordinalNumbers):
        labels = self.mapOrdinalNumbersToLabels(ordinalNumbers)
        labels = labels.values
        labelNames = []
        for label in labels:
            for jebanyPython in label:
                elo = itemTranslationDict[ItemType(jebanyPython)]
                labelNames.append(elo)
        return labelNames


class ImageParser:
    map = None

    def parseImagesToArray(self, itemlist):
        imageList = []
        for item in itemlist:
            imageList.append(np.array(item.image))
        imageList = np.array(imageList)
        return imageList

    def parseLabelsToArray(self, itemlist):
        labelList = []
        for item in itemlist:
            labelList.append(item.type.value)

        self.map = LabelsMap(labelList)
        labelList = self.map.mapLabelsToOrdinalNumbers(labelList)
        labelList = np.array(keras.utils.to_categorical(labelList))
        labelList = np.hstack(labelList)
        labelList = np.array(labelList)
        return labelList

    def parseOrnToNames(self, list):
        return self.map.mapOrdinalNumbersToLabelNames(list)
