import re
import os
from enum import Enum
from PIL import Image as im
import numpy as np


class ItemType(Enum):
    none = 0
    unknown = 1
    nothing = 10
    phone = 11
    book = 12
    plate_any = 20
    plate_empty = 21
    plate_full = 22
    mug_any = 30
    mug_empty = 31
    mug_full = 32
    hand_any = 40
    hand_light = 41
    hand_mid = 42
    hand_hard = 43
    drug = 51
    food_tray = 61


class FoodTrayType(Enum):
    none = 0
    half = 1
    two_half = 2
    half_straight = 3
    half_diagonal = 4
    full = 5
    full_corner = 6
    three_part = 7


class ItemShape(Enum):
    none = 0
    round = 1
    rectangle = 2


class ItemShapeDetails(Enum):
    none = 0
    flat = 1
    edge = 2


class ItemPlacement(Enum):
    unknown = 0
    center = 1
    edge = 2
    partially_out = 3


itemDictionary = {
        "book":       ItemType.book,
        "foodtray":   ItemType.food_tray,
        "fullmug":    ItemType.mug_full,
        "emptymug":   ItemType.mug_empty,
        "fullplate":  ItemType.plate_full,
        "emptyplate": ItemType.plate_empty,
        "phone":      ItemType.phone,
        "drugmug":    ItemType.drug,
        "hand":       ItemType.hand_any,
}

foodTrayDictionary = {
        "y":      FoodTrayType.three_part,
        "o":      FoodTrayType.full,
        "hslash": FoodTrayType.half_diagonal,
        "hsolo":  FoodTrayType.half,
        "hline":  FoodTrayType.half_straight,
        "hduo":   FoodTrayType.two_half,
        "corner": FoodTrayType.full_corner,
}

itemShapeDetailsDictionary = {
        "edge": ItemShapeDetails.edge,
        "flat": ItemShapeDetails.flat,
}

handTypeDictionary = {
        "soft": ItemType.hand_light,
        "mid":  ItemType.hand_mid,
        "hard": ItemType.hand_hard,
}

placementDictionary = {
        "center": ItemType.hand_light,
        "edge":   ItemType.hand_mid,
        "out":    ItemType.hand_hard,
}


class Item:
    def __init__(self):
        self.id = 0
        self.filename = None
        self.type = ItemType.none
        self.details = None
        self.weight = 0
        self.shape = ItemShape.none
        self.shape_details = ItemShapeDetails.none
        self.placement = ItemPlacement.unknown
        self.dim1 = 0
        self.dim2 = 0
        self.image = None
        self.image_calibration = None
        self.image_mask = None
        self.image_extracted = None

    def getLabelsFromFilename(self, path: str, filename: str, image_id: int):
        if "v2" in path:
            image_labels_version = 2
        else:
            image_labels_version = 1

        self.id = image_id
        self.filename = filename
        self.image = im.open(path + "/" + filename)
        if os.path.isfile(path + "/c_" + filename):
            self.image_calibration = im.open(path + "/c_" + filename)
        self.setExtractedImage()

        words = re.split(r'_|\.', filename)
        if image_labels_version == 1:
            if words[0] in itemDictionary:
                self.type = itemDictionary[words[0]]

            if words[1].isnumeric():
                self.weight = words[1]

            if self.type == ItemType.food_tray:
                if words[2] in foodTrayDictionary:
                    self.details = foodTrayDictionary[words[2]]
            elif self.type == ItemType.hand_any:
                if words[2] in handTypeDictionary:
                    self.type = handTypeDictionary[words[2]]
            elif words[2].startswith("r"):
                if words[2][1:].isnumeric():
                    self.shape = ItemShape.round
                    self.dim1 = words[2][1:]
                    if words[3] and words[3] in itemShapeDetailsDictionary:
                        self.shape_details = itemShapeDetailsDictionary[words[3]]
            elif "x" in words[2]:
                subwords = words[2].split("x")
                if subwords[0].isnumeric() and subwords[1].isnumeric():
                    self.dim1 = subwords[0]
                    self.dim2 = subwords[1]
                    self.shape = ItemShape.rectangle

        elif image_labels_version == 2:
            for word in words:
                if word.startswith("n"):
                    self.type = itemDictionary[word[1:]]

                if word.startswith("w") and word[1:].isnumeric():
                    self.weight = int(word[1:])

                if word.startswith("d"):
                    word = word[1:]
                    if "x" in word:
                        subwords = word.split("x")
                        if subwords[0].isnumeric() and subwords[1].isnumeric():
                            self.dim1 = int(subwords[0])
                            self.dim2 = int(subwords[1])
                            self.shape = ItemShape.rectangle
                    else:
                        if word.isnumeric():
                            self.dim1 = int(word)
                            self.shape = ItemShape.round

                if word.startswith("s"):
                    word = word[1:]
                    if self.type == ItemType.hand_any:
                        if word in handTypeDictionary:
                            self.details = handTypeDictionary[word]
                    if self.type == ItemType.food_tray:
                        if word in foodTrayDictionary:
                            self.details = foodTrayDictionary[word]

                if word.startswith("p"):
                    word = word[1:]
                    if word in placementDictionary:
                        self.placement = placementDictionary[word]

                if word.startswith("t"):
                    word = word[1:]
                    if word in itemShapeDetailsDictionary:
                        self.placement = itemShapeDetailsDictionary[word]

    def setMask(self, mask):
        self.image_mask = mask
        self.setExtractedImage()

    def removeMask(self):
        self.image_mask = None
        self.setExtractedImage()

    def setExtractedImage(self):
        source = np.array(self.image, dtype=int)
        calibration = np.array(self.image_calibration, dtype=int)
        extracted = source - calibration
        if self.image_mask is not None:
            mask = np.array(self.image_mask, dtype=int)
        else:
            mask = np.full((16, 16), 1, dtype=int)

        for i in range(16):
            for j in range(16):
                if extracted[i][j] < 0 or mask[i][j] == 0:
                    extracted[i][j] = 0

    def getExtractedImage(self):
        return self.image_extracted
