import time
import os
import string

# Suppress tensorflow noncritical warnings
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

from std_msgs.msg import Bool, String, Int32
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from tiago_msgs.msg import SaySentenceActionGoal

from nodes.node_core import Topic, Node
from nodes.table import TableStatus
from nodes.messages import prepare_string_msg, prepare_pose_stamped_msg, prepare_sentence_action_goal

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
        debug(DBGLevel.INFO, "Rico heard: " + data.data)
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
    ret = Topic("/rico_hear", String, callback=rico_heard_callback)  # Rico heard that thing
    topics.append(ret)

    return topics


def list_of_published_topics(topic_prefix):
    topics = []

    # TODO publish status of this node
    # nie status, tylko opublikować wiadomość raz, po wykonaniu/failu zadania
    # ret = Topic(topic_prefix + "/status", String)
    # topics.append(ret)
    ret = Topic("/move_base_simple/goal", PoseStamped)
    topics.append(ret)
    ret = Topic("/rico_says/goal", SaySentenceActionGoal)  # Rico will say that thing
    topics.append(ret)

    return topics


# TODO positions to json or xml
def get_pose_kitchen():
    pose = prepare_pose_stamped_msg(pos_x=16.85, pos_y=8.05, rot_z=-0.1, rot_w=0.0)
    return pose


def get_pose_docker():
    pose = prepare_pose_stamped_msg(pos_x=11.28, pos_y=6.9, rot_z=-0.7, rot_w=0.71)
    return pose


def get_pose_table():
    pose = prepare_pose_stamped_msg(pos_x=9.91, pos_y=8.14, rot_z=-0.54, rot_w=0.84)
    return pose


def get_pose_default():
    pose = prepare_pose_stamped_msg(pos_x=11.28, pos_y=6.9, rot_z=0.68, rot_w=0.75)
    return pose


def go_to_position(node, position):
    global g_move_status

    # TODO enum with possible Poses
    if position == "kitchen":
        goal = get_pose_kitchen()
        pl_position = "kuchni"
    elif position == "table":
        pl_position = "stolika"
        goal = get_pose_table()
    elif position == "dock":
        pl_position = "stacji dokującej"
        goal = get_pose_docker()
    elif position == "default":
        pl_position = "pozycji domyślnej"
        goal = get_pose_default()
    else:
        pl_position = "chuj wie gdzie"
        goal = get_pose_default()

    debug(DBGLevel.INFO, "I go to the " + position)
    # robot_say_sth(node, "Jadę do " + pl_position)
    robot_say_sth(node, "I go to the " + position)

    node.publish_msg_on_topic("/move_base_simple/goal", goal)
    g_move_status = 0  # Have to reset move status after publishing message

    while 1:
        # Move ended
        if g_move_status != 0:
            # Success
            if g_move_status == 3:
                debug(DBGLevel.INFO, "I arrived to the " + position)
                return 0
            # Failure
            else:
                # robot_say_sth(node, "Nie jestem w stanie dojechać do " + pl_position)
                robot_say_sth(node, "Cannot arrive to the " + position + " Code: " + str(g_move_status))
                debug(DBGLevel.ERROR, "Cannot arrive to the " + position + " Code: " + str(g_move_status))
                return 1

        time.sleep(1)


def task_completed_behaviour(node, message):
    robot_say_sth(node, message)
    if go_to_position(node, "default"):
        return -1
    return 0


def robot_say_sth(node, text_to_say):
    node.publish_msg_on_topic("/rico_says/goal", prepare_sentence_action_goal(text_to_say))
    return 0


def check_item_inside_table(node, item):
    if g_item_location == "Center of table" or g_item_location == "Side of table":
        debug(DBGLevel.INFO, "Item " + item + " has been placed")
        return 0
    elif g_item_location == "Edge of table":
        debug(DBGLevel.WARN,
              "Item " + item + " has been placed on the edge of the table - correct its position")
        robot_say_sth(node, "Please correct the position of the object")
        # robot_say_sth(node, "Proszę popraw położenie przedmiotu")
        time.sleep(10)
        return 1
    else:
        debug(DBGLevel.ERROR, "Item " + item + " couldn't be placed od the table")
        # robot_say_sth(node, "Przedmiot nie został położony na stoliku")
        robot_say_sth(node, "The object was not placed on the table")
        return 2


def check_item_in_weight_range(node, item):
    if item == "tea":
        expected_weight = 800
    elif item == "empty dish":
        expected_weight = 500
    else:
        return 0

    if expected_weight * 0.6 < g_item_weight < expected_weight * 1.5:
        return 0
    else:
        # robot_say_sth(node, "Przedmiot ma nieprawidłową wagę, czy to jest prawidłowy przedmiot?")
        robot_say_sth(node, "The item has an incorrect weight, is it the correct item?")
        debug(DBGLevel.ERROR, "Item " + item + " have wrong weight: " + str(g_item_weight))
        time.sleep(10)
        return 1


def wait_for_item_placed(node, item):
    if item == "tea":
        pl_item = "herbatę"
    elif item == "empty dish":
        pl_item = "puste naczynie"
    else:
        pl_item = "coś do przewiezienia"

    debug(DBGLevel.INFO, "I wait for " + item + " to be placed")
    # robot_say_sth(node, "Podaj proszę " + pl_item)
    robot_say_sth(node, "Could you give me " + pl_item)

    # TODO check weight and item type
    i = 0
    while 1:
        time.sleep(1)
        if g_item_placed_status:
            # Drop first measurement
            if i == 0:
                i = 1
                continue
            i = 0
            if check_item_inside_table(node, item) != 0:
                continue
            if check_item_in_weight_range(node, item) != 0:
                continue

            robot_say_sth(node, "Thank you")
            time.sleep(2)
            return 0


def wait_for_item_taken(node, item):
    if item == "tea":
        pl_item = "herbatę"
    elif item == "empty dish":
        pl_item = "puste naczynie"
    else:
        pl_item = "coś do przewiezienia"

    debug(DBGLevel.INFO, "I wait for " + item + " to be taken")
    # robot_say_sth(node, "Odbierz proszę " + pl_item)
    robot_say_sth(node, "Please take " + pl_item)

    while 1:
        if g_item_placed_status is False:
            debug(DBGLevel.INFO, "Item " + item + " has been taken")

            robot_say_sth(node, "Thank you")
            time.sleep(2)
            return 0
        time.sleep(1)


def handle_give_tea_command(node):
    # Go to kitchen
    if go_to_position(node, "kitchen") != 0:
        return -1

    # Get mug of tea
    if wait_for_item_placed(node, "tea") != 0:
        return -1

    # Go to task giver
    if go_to_position(node, "table") != 0:
        return -1

    # Acknowledge tea take off
    if wait_for_item_taken(node, "tea") != 0:
        return -1

    # End task
    # if task_completed_behaviour(node, "Herbata została dostarczona") != 0:
    if task_completed_behaviour(node, "The tea has been delivered") != 0:
        return -1
    return 0


def handle_drop_mug_command(node):
    # Go to task giver
    if go_to_position(node, "table") != 0:
        return -1

    # Take mug from task giver
    if not wait_for_item_placed(node, "empty dish"):
        return -1

    # Go to kitchen
    if go_to_position(node, "kitchen") != 0:
        return -1

    # Acknowledge mug take off
    if not wait_for_item_taken(node, "empty dish"):
        return -1

    # End task
    # if task_completed_behaviour(node, "Naczynie zostało odwiezione") != 0:
    if task_completed_behaviour(node, "The dish has been taken away") != 0:
        return -1
    return 0


def ignore_punctuation_marks(sentence):
    result = sentence.translate(str.maketrans('', '', string.punctuation))
    return result


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
            g_command = ignore_punctuation_marks(g_command)

            if g_command == "Przywieź mi herbatę" or g_command == "Bring me a cup of tea" or "tea" in g_command:
                if handle_give_tea_command(usage_node) != 0:
                    # task_completed_behaviour(usage_node, "Zadanie przerwano")
                    task_completed_behaviour(usage_node, "Task aborted")
            elif g_command == "Odwieź kubek do kuchni" or "mug" in g_command or "kubek" in g_command:
                if handle_drop_mug_command(usage_node) != 0:
                    # task_completed_behaviour(usage_node, "Zadanie przerwano")
                    task_completed_behaviour(usage_node, "Task aborted")
            else:
                debug(DBGLevel.WARN, g_command)

        time.sleep(1)
