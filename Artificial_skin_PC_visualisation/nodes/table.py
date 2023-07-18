import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String, Int32


class Topic:
    def __init__(self, name, msg_type, callback=None, queue_size=10):
        self.name = name
        self.msg_type = msg_type
        self.callback = callback
        self.queue_size = queue_size


class TableNode:
    subscribed_topics = []
    published_topics = []
    subscribers = []
    publishers = []

    on_flag = False
    calibrate_flag = False

    def __init__(self, node_name="IntelligentTable", subscribed_topics=None, published_topics=None):
        if subscribed_topics is None:
            default_subscribed_topics = []
            ret = Topic("/sgn_on", Bool, callback=self.sgn_on_callback())
            default_subscribed_topics.append(ret)
            ret = Topic("/sgn_calibrate", Bool, callback=self.sgn_calibrate_callback())
            default_subscribed_topics.append(ret)

            self.subscribed_topics = default_subscribed_topics
        else:
            self.subscribed_topics = subscribed_topics

        if published_topics is None:
            default_published_topics = []
            ret = Topic("/raw_image", Image)
            default_published_topics.append(ret)
            ret = Topic("/status", String)
            default_published_topics.append(ret)
            ret = Topic("/is_placed", Bool)
            default_published_topics.append(ret)
            ret = Topic("/weight", Int32)
            default_published_topics.append(ret)
            ret = Topic("/predicted_item", String)
            default_published_topics.append(ret)
            ret = Topic("/location", String)
            default_published_topics.append(ret)

            self.published_topics = default_published_topics
        else:
            self.published_topics = published_topics

        rospy.init_node(node_name)

        for topic in self.subscribed_topics:
            sub = self.Subscriber(topic)
            self.subscribers.append(sub)
        for topic in self.published_topics:
            pub = self.Publisher(topic)
            self.publishers.append(pub)
            pass

    def sgn_on_callback(self, data=None):
        if type(data) is Bool:
            self.on_flag = data

    def sgn_calibrate_callback(self, data=None):
        if type(data) is Bool:
            self.calibrate_flag = data

    def publish_msg_on_topic(self, topic_name, msg):
        for pub in self.publishers:
            if pub.topic.name == topic_name:
                pub.publish(msg)

    class Subscriber:
        topic = None

        def __init__(self, topic):
            self.topic = topic
            self.callback_function = topic.callback
            self.sub = rospy.Subscriber(self.topic.name, self.topic.msg_type, self.subscriber_callback())

        def subscriber_callback(self):
            self.callback_function()

    class Publisher:
        topic = None

        def __init__(self, topic):
            self.topic = topic
            self.pub = rospy.Publisher(topic.name)

        def publish(self, message):
            if type(message) is self.topic.msg_type:
                self.pub.publish(message)
            else:
                print("Invalid publish message type, expected: " + self.topic.msg_type +
                      ", received: " + message + ".")
