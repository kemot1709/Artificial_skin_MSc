from connection.connection import Serial
import copy

from sensor.data_parsing import parse_data_to_np_image, parse_np_image_to_msg, cast_data_to_uint8, \
    compensate_raw_image
from debug.debug import *


class Sensor:
    ser = None
    usb_port = ""
    usb_connected = False

    parent_node = None

    image_calibrated = None
    image_calibrated_raw = None
    image_actual = None
    image_actual_raw = None
    image_actual_calibrated = None
    image_actual_calibrated_raw = None

    def __init__(self, usb_port="/dev/ttyUSB0"):
        self.usb_port = usb_port
        self.ser = Serial()
        self.ser.set_data_receiver(self.new_data_received)

    def set_usb_port(self, usb_port):
        self.usb_port = usb_port

    def connect_to_controller(self):
        self.ser.connect_to_controller(self.usb_port)

    def set_parent_node(self, node):
        self.parent_node = node

    def new_data_received(self, n_rows, n_columns, new_pressure_map):
        # TODO sometimes incoming data are corrupted (values like 4). It has to be filtered out
        self.image_actual_raw = new_pressure_map
        self.image_actual = parse_data_to_np_image(n_rows, n_columns, new_pressure_map)

        if self.image_calibrated is None:
            self.image_calibrated = self.image_actual.copy()
            self.image_calibrated_raw = copy.deepcopy(self.image_actual_raw)
            return
        self.image_actual_calibrated_raw = compensate_raw_image(n_rows, n_columns, self.image_actual_raw,
                                                                self.image_calibrated_raw)
        self.image_actual_calibrated = parse_data_to_np_image(n_rows, n_columns, self.image_actual_calibrated_raw)
        debug(DBGLevel.INFO, "New data received from controller")

        if self.parent_node is not None:
            self.parent_node.new_image_from_sensor()

    def calibrate_sensor(self, raw_data):
        debug(DBGLevel.INFO, "Calibrate sensor")
        self.image_calibrated = raw_data
