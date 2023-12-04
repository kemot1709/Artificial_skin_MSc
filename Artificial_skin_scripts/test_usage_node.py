import time
import os

# Suppress tensorflow noncritical warnings
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

from std_msgs.msg import Bool, String, Int32
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult

from nodes.node_core import Topic, Node
from nodes.table import TableStatus
from nodes.messages import prepare_string_msg

from debug.debug import debug, DBGLevel

# Global variables - Have to do it better but later
g_move_status = 0
g_item_placed_status = False
g_item_weight = 0
g_item_prediction = ""
g_item_location = ""
g_table_status = TableStatus.unknown
g_command_arrived = False
g_command = ""


# TODO properly implement this shit
# dialogflow
# language_processor


def item_placed_callback(data=None):
    global g_item_placed_status
    global g_item_weight
    global g_item_prediction
    global g_item_location
    if type(data) is Bool:
        g_item_placed_status = data.data
    else:
        g_item_placed_status = False
        g_item_weight = 0
        g_item_prediction = ""
        g_item_location = ""


def item_weight_callback(data=None):
    global g_item_weight
    if type(data) is Int32:
        g_item_weight = data.data
    else:
        g_item_weight = 0


def item_predicted_callback(data=None):
    global g_item_prediction
    if type(data) is String:
        g_item_prediction = data.data
    else:
        g_item_prediction = ""


def item_location_callback(data=None):
    global g_item_location
    if type(data) is String:
        g_item_location = data.data
    else:
        g_item_location = ""


def move_status_callback(data=None):
    # MoveBaseActionResult.status.status (Enum)
    # uint8 PENDING=0
    # uint8 ACTIVE=1
    # uint8 PREEMPTED=2
    # uint8 SUCCEEDED=3
    # uint8 ABORTED=4
    # uint8 REJECTED=5
    # uint8 PREEMPTING=6
    # uint8 RECALLING=7
    # uint8 RECALLED=8
    # uint8 LOST=9

    global g_move_status
    if type(data) is MoveBaseActionResult:
        g_move_status = data.status.status
    else:
        g_move_status = 0


def rico_heard_callback(data=None):
    global g_command_arrived
    global g_command
    if type(data) is String:
        g_command = data.data
        g_command_arrived = True
    else:
        g_command = ""
        g_command_arrived = False


def list_of_subscribed_topics():
    topics = []

    ret = Topic("/table/is_placed", Bool, callback=item_placed_callback)
    topics.append(ret)
    ret = Topic("/table/weight", Int32, callback=item_weight_callback)
    topics.append(ret)
    ret = Topic("/table/predicted_item", String, callback=item_predicted_callback)
    topics.append(ret)
    ret = Topic("/table/location", String, callback=item_location_callback)
    topics.append(ret)
    ret = Topic("/move_base/result", MoveBaseActionResult, callback=move_status_callback)
    topics.append(ret)
    ret = Topic("/txt_send", String, callback=rico_heard_callback)  # Rico heard that thing
    topics.append(ret)

    return topics


def list_of_published_topics(topic_prefix):
    topics = []

    # TODO publish status of this node
    # ret = Topic(topic_prefix + "/status", String)
    # topics.append(ret)
    ret = Topic("/move_base_simple/goal", PoseStamped)
    topics.append(ret)
    ret = Topic("/txt_msg", String)  # Rico say that thing
    topics.append(ret)

    return topics


def get_pose_kitchen():
    # TODO get coordinates of locations
    # TODO positions to json or xml
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = 2.901
    pose.pose.position.y = -0.402
    pose.pose.orientation.z = -1.532
    return pose


def get_pose_docker():
    pass


def get_pose_table():
    pass


def go_to_position(node, position):
    global g_move_status

    # TODO enum with possible Poses
    if position == "kitchen":
        goal = get_pose_kitchen()
    else:
        goal = get_pose_kitchen()

    node.publish_msg_on_topic("/move_base_simple/goal", goal)

    while 1:
        # Success
        if g_move_status == 3:
            say_sth(node, "Arrived")
            return 0
        time.sleep(0.1)


def say_sth(node, text_to_say):
    node.publish_msg_on_topic("/txt_send", prepare_string_msg(text_to_say))
    return 0


def wait_for_item_placed(node, item):
    say_sth(node, "Dej przedmiot")
    # TODO check weight and item type
    while 1:
        if g_item_placed_status:
            if g_item_location == "Center of table" or g_item_location == "Side of table":
                return 0
            elif g_item_location == "Edge of table":
                debug(DBGLevel.WARN, "Item placed on edge")
                say_sth(node, "Proszę popraw położenie przedmiotu")
                time.sleep(5)
                continue
            else:
                pass
        time.sleep(1)


def wait_for_item_taken(node, item):
    say_sth(node, "Weź przedmiot")
    while 1:
        if g_item_placed_status is False:
            return 0
        time.sleep(1)


def handle_give_tea_command(node):
    # Go to kitchen
    debug(DBGLevel.INFO, "I go to the kitchen")
    if go_to_position(node, "kitchen") != 0:
        debug(DBGLevel.CRITICAL, "Cannot drive to kitchen")
        say_sth(node, "Dupa")
        return -1

    # Get mug of tea
    debug(DBGLevel.INFO, "I wait for fullmug to be placed")
    if not wait_for_item_placed(node, "fullmug"):
        debug(DBGLevel.CRITICAL, "Cannot get item fullmug")
        say_sth(node, "Alohomora")
        return -1

    # Go to task giver
    debug(DBGLevel.INFO, "I go to the table")
    if go_to_position(node, "table") != 0:
        debug(DBGLevel.CRITICAL, "Cannot drive to table")
        say_sth(node, "Dupa")
        return -1

    # Acknowledge tea take off
    debug(DBGLevel.INFO, "I wait for fullmug to be taken")
    if not wait_for_item_taken(node, "fullmug"):
        debug(DBGLevel.CRITICAL, "Cannot get rid of item fullmug")
        say_sth(node, "Alohomora")
        return -1
    return 0


def handle_drop_mug_command(node):
    # Take mug from task giver
    debug(DBGLevel.INFO, "I wait for emptymug to be placed")
    if not wait_for_item_placed(node, "emptymug"):
        debug(DBGLevel.CRITICAL, "Cannot get item emptymug")
        say_sth(node, "Nie")
        return -1

    # Go to kitchen
    debug(DBGLevel.INFO, "I go to the kitchen")
    if go_to_position(node, "kitchen") != 0:
        debug(DBGLevel.CRITICAL, "Cannot drive to kitchen")
        say_sth(node, "Hahaha")
        return -1

    # Acknowledge mug take off
    debug(DBGLevel.INFO, "I wait for emptymug to be taken")
    if not wait_for_item_taken(node, "emptymug"):
        debug(DBGLevel.CRITICAL, "Cannot get rid of item emptymug")
        say_sth(node, "Nie")
        return -1
    return 0


if __name__ == "__main__":
    subscribed_topics = list_of_subscribed_topics()
    published_topics = list_of_published_topics('/test_usage')
    usage_node = Node('table_usage', subscribed_topics, published_topics)

    while 1:
        # Wait for commands
        if g_command_arrived:
            g_command_arrived = False
            if g_command != "":
                debug(DBGLevel.DETAILS, g_command)

            if g_command == "A":
                handle_give_tea_command(usage_node)
            elif g_command == "B":
                handle_drop_mug_command(usage_node)
            else:
                debug(DBGLevel.WARN, g_command)

        time.sleep(1)
