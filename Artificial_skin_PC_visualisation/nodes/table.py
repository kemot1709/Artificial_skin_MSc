import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String, Int32

from PyQt5 import QtCore
from enum import Enum

from nodes.messages import prepare_bool_msg, prepare_image_msg, prepare_string_msg, prepare_int32_msg
from nodes.node_core import NodeStatus, Topic, Node
from sensor.sensor import Sensor
from sensor.params import ImageMask
from sensor.data_parsing import flatten
from item.item import Item, ItemPlacement, ItemType
from classifier.position_recognition import recognise_position
from classifier.weight_estimation import estimate_weight
from classifier.image_recognition import Classifier
from debug.debug import *


class TableNode(Node):
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

    def __init__(self, node_name="IntelligentTable", language="en", model_path="classifier/models/test_model.keras"):
        # Setup subscribed topics
        subscribed_topics = []
        ret = Topic("/sgn_on", Bool, callback=self.sgn_on_callback)
        subscribed_topics.append(ret)
        ret = Topic("/sgn_calibrate", Bool, callback=self.sgn_calibrate_callback)
        subscribed_topics.append(ret)

        # Setup published topics
        published_topics = []
        ret = Topic("/raw_image", Image)
        published_topics.append(ret)
        ret = Topic("/status", String)
        published_topics.append(ret)
        ret = Topic("/is_placed", Bool)
        published_topics.append(ret)
        ret = Topic("/weight", Int32)
        published_topics.append(ret)
        ret = Topic("/predicted_item", String)
        published_topics.append(ret)
        ret = Topic("/location", String)
        published_topics.append(ret)

        # Item classifier model initialization section
        self.classifier_model_path = model_path
        self.item_classifier = Classifier()
        self.item_classifier.import_model(self.classifier_model_path)

        # Run node
        super(TableNode, self).__init__(node_name, subscribed_topics, published_topics, language=language)

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

    def new_image_from_sensor(self):
        self.new_image_flag = True

    def is_item_placed(self):
        for i in flatten(self.actual_item.getExtractedImage()):
            if i > 10:
                return True
        else:
            return False

    def get_predicted_item(self):
        if self.actual_item.type is not ItemType.none:
            return self.translation.itemTranslationDict[self.actual_item.type[0]]
        else:
            return self.translation.itemTranslationDict[ItemType.none]

    def get_predicted_location(self):
        if self.actual_item.placement is not ItemPlacement.unknown:
            return self.translation.itemPlacementTranslationDict[self.actual_item.placement]
        else:
            return self.translation.itemPlacementTranslationDict[ItemPlacement.unknown]

    def get_predicted_weight(self):
        if self.actual_item.weight > 0.0:
            return int(round(self.actual_item.weight))
        else:
            return 0

    def exstract_image_from_sensor_data(self):
        # Calibration image is not nessecarry, because sensor calibrated this data on its own
        self.actual_item = Item(self.mask.getMask())
        self.actual_item.image = self.sensor.image_actual_calibrated
        self.actual_item.image_extracted_raw = self.sensor.image_actual_calibrated_raw
        self.actual_item.setExtractedImage()
        self.actual_item.id = self.item_cnt
        self.item_cnt += 1

    def make_recognition_of_image(self):
        if self.is_item_placed():
            self.actual_item.placement = recognise_position(self.actual_item.getExtractedImage(), self.mask.getMask(),
                                                            [1.5, 2.5])

            self.actual_item.weight = estimate_weight(self.actual_item.image_extracted_raw)

            if self.item_classifier is not None:
                prediction = self.item_classifier.predict_items_with_confidence(self.actual_item.getExtractedImage(),
                                                                                0.8)
                self.actual_item.type = prediction[0]
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
