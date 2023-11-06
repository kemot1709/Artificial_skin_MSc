from item.item import ItemType, ItemPlacement
from nodes.node_core import NodeStatus
from nodes.table import TableStatus

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
        NodeStatus.unknown:            "Unknown",
        NodeStatus.initializing:       "Initializing",
        NodeStatus.working:            "Working",

        NodeStatus.crashed:            "Crashed unknown",
        NodeStatus.crashed_internal:   "Crashed internal",
        NodeStatus.crashed_connection: "Crashed communication",
        NodeStatus.crashed_ros:        "Crashed ROS",

        TableStatus.table_off:         "Working off",
        TableStatus.table_working:     "Working on",
        TableStatus.table_calibrating: "Working calibrating",
}
