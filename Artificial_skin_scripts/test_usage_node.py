import time
import os

# Suppress tensorflow noncritical warnings
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult

from nodes.node_core import Topic, Node
from nodes.table import TableStatus

# Global variables - Have to do it better but later
g_move_status = 0
g_item_placed_status = False
g_table_status = TableStatus.unknown
g_command_arrived = False
g_command = ""


# dialogflow
# language_processor


def item_placed_callback(data=None):
    global g_item_placed_status
    if type(data) is Bool:
        g_item_placed_status = data.data
    else:
        g_item_placed_status = False


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


def rico_heard(data=None):
    pass


def list_of_subscribed_topics():
    subscribed_topics = []

    ret = Topic("/table/is_placed", Bool, callback=item_placed_callback())
    subscribed_topics.append(ret)
    ret = Topic("/move_base/result", MoveBaseActionResult, callback=move_status_callback())
    subscribed_topics.append(ret)

    return subscribed_topics


def list_of_published_topics(topic_prefix):
    published_topics = []

    ret = Topic(topic_prefix + "/status", String)
    published_topics.append(ret)
    ret = Topic("/move_base_simple/goal", PoseStamped)
    published_topics.append(ret)

    return published_topics


def get_pose_kitchen():
    # TODO get coordinates of locations
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = 2.901
    pose.pose.position.y = -0.402
    pose.pose.orientation.z = -1.532
    return pose


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
            say_sth("Arrived")
            return 3
        time.sleep(0.1)


def say_sth(text_to_say):
    pass


def wait_for_item_placed(node, item):
    pass


def wait_for_item_taken(node, item):
    pass


def handle_give_tea_command(node):
    # Go to kitchen
    if go_to_position(node, "kitchen") != 3:
        say_sth("Dupa")
        return -1
    # Get mug of tea
    if not wait_for_item_placed(node, "fullmug"):
        say_sth("Alohomora")
        return -1
    # Go to task giver
    if go_to_position(node, "table") != 3:
        say_sth("Dupa")
        return -1
    # Acknowledge tea take off
    if not wait_for_item_taken(node, "fullmug"):
        say_sth("Alohomora")
        return -1
    return 0


def handle_drop_mug_command(node):
    # Take mug from task giver
    # Go to kitchen
    go_to_position(node, "kitchen")
    # Acknowledge mug take off
    pass


if __name__ == "__main__":
    subscribed_topics = list_of_subscribed_topics()
    published_topics = list_of_published_topics('/test_usage')
    usage_node = Node('table_usage', subscribed_topics, published_topics)

    while 1:
        # Wait for commands
        if g_command_arrived:
            g_command_arrived = False
            if g_command != "":
                print(g_command)

        time.sleep(1)
