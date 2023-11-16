import time

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult

from nodes.node_core import Topic, NodeStatus, Node


def item_placed_callback():
    pass


def move_status_callback():
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


def go_to_position(node):
    goal = PoseStamped()
    # TODO enum with possible Poses

    node.publish_msg_on_topic("/move_base_simple/goal", goal)


def say_sth(text_to_say):
    pass


if __name__ == "__main__":
    subscribed_topics = list_of_subscribed_topics()
    published_topics = list_of_published_topics('/test_usage')
    node = Node('table_usage', subscribed_topics, published_topics)

    while 1:
        time.sleep(60)
