from item.item import ItemType, ItemPlacement
from nodes.table import NodeStatus

itemTranslationDict = {
        ItemType.book:        "Book",
        ItemType.food_tray:   "Food tray",
        ItemType.mug_any:     "Mug",
        ItemType.mug_full:    "Full mug",
        ItemType.mug_empty:   "Empty mug",
        ItemType.plate_any:   "Plate",
        ItemType.plate_full:  "Full plate",
        ItemType.plate_empty: "Empty plate",
        ItemType.phone:       "Phone",
        ItemType.drug:        "Drugs",
        ItemType.hand_any:    "Hand",
        ItemType.hand_light:  "Light touch",
        ItemType.hand_mid:    "Normal touch",
        ItemType.hand_hard:   "Hard touch",
        ItemType.none:        "Nothing",
        ItemType.unknown:     "Unknown",
}

itemPlacementTranslationDict = {
        ItemPlacement.center:        "Center of table",
        ItemPlacement.side:          "Side of table",
        ItemPlacement.edge:          "Edge of table",
        ItemPlacement.partially_out: "Out of table",
        ItemPlacement.unknown:       "Nothing",
}

nodeStatusTranslationDictionary = {
        NodeStatus.unknown:                 "Unknown",
        NodeStatus.not_connected:           "Initialized but not connected to controller",
        NodeStatus.connected:               "Initialized, connected to controller but turned off",
        NodeStatus.connection_crashed:      "Connection with controller is interrupted",
        NodeStatus.connection_bad_messages: "Controller sends unrecognized data",
        NodeStatus.working:                 "Node working well",
        NodeStatus.calibrating:             "Node calibrate itself",
}
