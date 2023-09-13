from item.item import ItemType, ItemPlacement

itemTranslationDict = {
        ItemType.book:        "Book",
        ItemType.food_tray:   "Food tray",
        ItemType.mug_full:    "Full mug",
        ItemType.mug_empty:   "Empty mug",
        ItemType.plate_full:  "Full plate",
        ItemType.plate_empty: "Empty plate",
        ItemType.phone:       "Phone",
        ItemType.drug:        "Drugs",
        ItemType.hand_any:    "Hand",
        ItemType.hand_light:  "Light touch",
        ItemType.hand_mid:    "Normal touch",
        ItemType.hand_hard:   "Hard touch",
        ItemType.unknown:     "Nothing",
}

itemPlacementTranslationDict = {
        ItemPlacement.center:        "Center of table",
        ItemPlacement.side:          "Side of table",
        ItemPlacement.edge:          "Edge of table",
        ItemPlacement.partially_out: "Out of table",
        ItemPlacement.unknown:       "Nothing",
}
