import re
import os
from enum import Enum
from PIL import Image as im


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
    square = 2


class ItemShapeDetails(Enum):
    none = 0
    flat = 1
    edge = 2


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


class Item:
    def __init__(self):
        self.id = 0
        self.filename = ""
        self.type = ItemType.none
        self.details = ""
        self.weight = 0
        self.shape = ItemShape.none
        self.shape_details = ItemShapeDetails.none
        self.dim1 = 0
        self.dim2 = 0
        self.image = None
        self.image_calibration = None

    def getItemsFromFilename(self, path: str, filename: str, image_id: int):
        self.id = image_id
        self.filename = filename
        self.image = im.open(path + "/" + filename)
        if os.path.isfile(path + "/c_" + filename):
            self.image_calibration = im.open(path + "/c_" + filename)

        words = re.split('_.', filename)
        if words[0] in itemDictionary:
            self.type = itemDictionary[words[0]]

        if words[1].isnumeric():
            self.weight = words[1]

        if self.type == ItemType.food_tray:
            if words[3] in foodTrayDictionary:
                self.details = foodTrayDictionary[words[3]]
        elif self.type == ItemType.hand_any:
            if words[3] in handTypeDictionary:
                self.type = handTypeDictionary[words[3]]
        elif words[3].startswith("r"):
            if words[3][1:].isnumeric():
                self.shape = ItemShape.round
                self.dim1 = words[3][1:]
                if words[4] and words[4] in itemShapeDetailsDictionary:
                    self.shape_details = itemShapeDetailsDictionary[words[4]]
        elif "x" in words[3]:
            subwords = words[3].split("x")
            if subwords[0].isnumeric() and subwords[1].isnumeric():
                self.dim1 = subwords[0]
                self.dim2 = subwords[1]
