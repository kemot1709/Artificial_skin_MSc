import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String, Int32

from PyQt5 import QtCore

from nodes.messages import prepare_bool_msg, prepare_image_msg, prepare_string_msg, prepare_int32_msg
from sensor.sensor import Sensor
from debug import *


# TODO make some defines for names of topics


class Topic:
    def __init__(self, name, msg_type, callback=None, queue_size=10):
        self.name = name
        self.msg_type = msg_type
        self.callback = callback
        self.queue_size = queue_size


class TableNode(QtCore.QThread):
    subscribed_topics = []
    published_topics = []
    subscribers = []
    publishers = []

    sensor = None

    on_flag = False
    calibrate_flag = False
    new_image_flag = False

    def __init__(self, node_name="IntelligentTable", subscribed_topics=None, published_topics=None):
        super(TableNode, self).__init__()
        rospy.init_node(node_name)

        if subscribed_topics is None:
            default_subscribed_topics = []
            ret = Topic("/sgn_on", Bool, callback=self.sgn_on_callback)
            default_subscribed_topics.append(ret)
            ret = Topic("/sgn_calibrate", Bool, callback=self.sgn_calibrate_callback)
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

        for topic in self.subscribed_topics:
            sub = self.Subscriber(topic)
            self.subscribers.append(sub)
        for topic in self.published_topics:
            pub = self.Publisher(topic)
            self.publishers.append(pub)

        self.exitFlag = False
        self.start()

    def set_sensor(self, sensor):
        self.sensor = sensor
        self.sensor.set_parent_node(self)

    def sgn_on_callback(self, data=None):
        if type(data) is Bool:
            self.on_flag = data.data
            self.new_image_flag = False

    def sgn_calibrate_callback(self, data=None):
        if type(data) is Bool:
            self.calibrate_flag = data.data

    def get_calibration_flag(self):
        return self.calibrate_flag

    def get_on_flag(self):
        return self.on_flag

    def publish_is_placed(self, boolean):
        self.publish_msg_on_topic("/is_placed", prepare_bool_msg(boolean))

    def publish_status(self, string):
        self.publish_msg_on_topic("/status", prepare_string_msg(string))

    def publish_predicted_item(self, string):
        self.publish_msg_on_topic("/predicted_item", prepare_string_msg(string))

    def publish_location(self, string):
        self.publish_msg_on_topic("/location", prepare_string_msg(string))

    def publish_weight(self, int32):
        self.publish_msg_on_topic("/weight", prepare_int32_msg(int32))

    def publish_image(self, image):
        self.publish_msg_on_topic("/raw_image", prepare_image_msg("Intelligent table node", image))

    def publish_msg_on_topic(self, topic_name, msg):
        for pub in self.publishers:
            if pub.topic.name == topic_name:
                pub.publish(msg)

    def new_image_from_sensor(self, image):
        self.new_image_flag = True
        debug(message="ELO")

    def run(self):
        while not self.exitFlag:
            if self.on_flag and self.new_image_flag:
                if self.calibrate_flag:
                    self.sensor.calibrate_sensor(self.sensor.image_actual)

                self.publish_image(self.sensor.image_actual)
                self.new_image_flag = False

    class Subscriber:
        topic = None
        callback_function = None

        def __init__(self, topic):
            self.topic = topic
            self.callback_function = topic.callback
            self.sub = rospy.Subscriber(self.topic.name, self.topic.msg_type, self.subscriber_callback)

        def subscriber_callback(self, data):
            self.callback_function(data)

    class Publisher:
        topic = None

        def __init__(self, topic):
            self.topic = topic
            self.pub = rospy.Publisher(topic.name, topic.msg_type, queue_size=topic.queue_size)

        def publish(self, message):
            if type(message) is self.topic.msg_type:
                self.pub.publish(message)
            else:
                print("Invalid publish message type, expected: " + str(self.topic.msg_type) +
                      ", received: " + str(message) + ".")
