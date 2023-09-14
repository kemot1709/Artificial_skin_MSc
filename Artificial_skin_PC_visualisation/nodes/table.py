import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String, Int32

from PyQt5 import QtCore
from enum import Enum

from nodes.messages import prepare_bool_msg, prepare_image_msg, prepare_string_msg, prepare_int32_msg
from sensor.sensor import Sensor
from sensor.params import ImageMask
from item.item import Item, ItemPlacement, ItemType
from classifier.position_recognition import recognise_position
from classifier.weight_estimation import estimate_weight
from classifier.image_recognition import Classifier
from debug import *


# TODO make some defines for names of topics


class Topic:
    def __init__(self, name, msg_type, callback=None, queue_size=10):
        self.name = name
        self.msg_type = msg_type
        self.callback = callback
        self.queue_size = queue_size


class NodeStatus(Enum):
    unknown = 0
    not_connected = 1
    connected = 2
    connection_crashed = 3
    connection_bad_messages = 4
    working = 10
    calibrating = 11


class TableNode(QtCore.QThread):
    language = "en"
    translation = None

    subscribed_topics = []
    published_topics = []
    subscribers = []
    publishers = []

    sensor = None

    on_flag = False
    calibrate_flag = False
    new_image_flag = False
    node_status = NodeStatus.unknown

    mask = ImageMask()
    actual_item = None
    item_cnt = 1

    item_classifier = None
    classifier_model_path = None

    def __init__(self, node_name="IntelligentTable", subscribed_topics=None, published_topics=None, language="en",
                 model_path="image_model.h5"):
        super(TableNode, self).__init__()
        rospy.init_node(node_name)

        # Place where you should import all translations
        self.language = language
        if self.language is "en":
            from languages import en as translation
            self.translation = translation
        else:
            from languages import en as translation
            self.translation = translation

        # Item classifier model initialization section
        self.classifier_model_path = model_path
        self.item_classifier = Classifier()
        self.item_classifier.import_model(self.classifier_model_path)

        # Setup subscribed topics
        # Not tested on outside given topics but should work
        if subscribed_topics is None:
            default_subscribed_topics = []
            ret = Topic("/sgn_on", Bool, callback=self.sgn_on_callback)
            default_subscribed_topics.append(ret)
            ret = Topic("/sgn_calibrate", Bool, callback=self.sgn_calibrate_callback)
            default_subscribed_topics.append(ret)

            self.subscribed_topics = default_subscribed_topics
        else:
            self.subscribed_topics = subscribed_topics

        # Setup published topics
        # Not tested on outside given topics but should work
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

        # Initialize all topics
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

    def new_image_from_sensor(self):
        self.new_image_flag = True

    def is_item_placed(self):
        if all(i <= 10 for i in self.actual_item.getExtractedImage()):
            return False
        else:
            return True

    def get_predicted_item(self):
        if self.actual_item.type is not ItemType.none:
            return self.translation.itemTranslationDict[self.actual_item.type]
        else:
            return self.translation.itemTranslationDict[ItemType.none]

    def get_predicted_location(self):
        if self.actual_item.placement is not ItemPlacement.unknown:
            return self.translation.itemPlacementTranslationDict[self.actual_item.placement]
        else:
            return self.translation.itemPlacementTranslationDict[ItemPlacement.unknown]

    def get_node_status(self):
        # TODO describe and use this statuses in code
        return self.translation.nodeStatusTranslationDictionary[self.node_status]

    def get_predicted_weight(self):
        if self.actual_item.weight > 0.0:
            return self.actual_item.weight
        else:
            return 0

    def exstract_image_from_sensor_data(self):
        # Calibration image is not nessecarry, because sensor calibrated this data on its own
        self.actual_item = Item(self.mask.getMask())
        self.actual_item.image = self.sensor.image_actual_calibrated
        self.actual_item.setExtractedImage()
        self.actual_item.id = self.item_cnt
        self.item_cnt += 1

    def make_recognition_of_image(self):
        if self.is_item_placed():
            self.actual_item.placement = recognise_position(self.actual_item.getExtractedImage(), self.mask.getMask(),
                                                            [1.5, 2.5])
            self.actual_item.weight = estimate_weight(self.actual_item.getExtractedImage())
            if self.item_classifier is not None:
                self.actual_item.type = self.item_classifier.predict(self.actual_item.getExtractedImage())
            else:
                self.actual_item.type = ItemType.unknown

    def run(self):
        while not self.exitFlag:
            if self.on_flag and self.new_image_flag:
                if self.calibrate_flag:
                    self.sensor.calibrate_sensor(self.sensor.image_actual)
                self.exstract_image_from_sensor_data()
                self.make_recognition_of_image()

                #####
                self.publish_image(self.sensor.image_actual)
                self.publish_is_placed(self.is_item_placed())
                self.publish_predicted_item(self.get_predicted_item())
                self.publish_location(self.get_predicted_location())
                self.publish_status(self.get_node_status())
                self.publish_weight(self.get_predicted_weight())
                #####

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
