import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Topic:
    def __init__(self, name, msg_type, callback=None, queue_size=10):
        self.name = name
        self.msg_type = msg_type
        self.callback = callback
        self.queue_size = queue_size


def xxx_subscribtion_callback():
    pass


def xxx_subscribtion_callback2():
    pass


default_subscribed_topics = []
ret = Topic("/sgn_on", 0, callback=xxx_subscribtion_callback())
default_subscribed_topics.append(ret)
ret = Topic("/sgn_calibrate", 0, callback=xxx_subscribtion_callback2())
default_subscribed_topics.append(ret)

default_published_topics = []
ret = Topic("/raw_image", 0)
default_published_topics.append(ret)
ret = Topic("/status", 0)
default_published_topics.append(ret)
ret = Topic("/is_placed", 0)
default_published_topics.append(ret)
ret = Topic("/weight", 0)
default_published_topics.append(ret)
ret = Topic("/predicted_item", 0)
default_published_topics.append(ret)
ret = Topic("/location", 0)
default_published_topics.append(ret)


class TableNode:
    subscribed_topics = []
    published_topics = []
    subscribers = []
    publishers = []

    def __init__(self, node_name="IntelligentTable", subscribed_topics=None, published_topics=None):
        if subscribed_topics is None:
            self.subscribed_topics = default_subscribed_topics
        else:
            self.subscribed_topics = subscribed_topics

        if published_topics is None:
            self.published_topics = default_published_topics
        else:
            self.published_topics = published_topics

        rospy.init_node(node_name)

        for topic in self.subscribed_topics:
            sub = self.Subscriber(topic)
            self.subscribers.append(sub)
        for topic in self.published_topics:
            pub = self.Subscriber(topic)
            self.publishers.append(pub)
            pass

    class Subscriber:
        def __init__(self, topic):
            self.topic = topic
            self.callback_function = topic.callback
            self.sub = rospy.Subscriber(self.topic.name, self.topic.msg_type, self.subscriber_callback())

        def subscriber_callback(self):
            self.callback_function()

    class Publisher:
        def __init__(self, topic):
            self.pub = rospy.Publisher(+ topic)

        def publish(self, message):
            # TODO check if message have proper type
            self.pub.publish(message)
