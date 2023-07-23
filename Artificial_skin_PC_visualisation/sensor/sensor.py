from connection.connection import Serial

from data_parsing import parse_data_to_np_image, parse_np_image_to_msg, cast_data_to_uint8, \
    process_raw_image_through_calibration


class Sensor:
    ser = None
    usb_port = ""
    usb_connected = False

    parent_node = None

    image_calibrated = None
    image_actual = None
    image_actual_calibrated = None

    def __init__(self, usb_port="/dev/ttyUSB0"):
        self.usb_port = usb_port
        self.ser = Serial()
        self.ser.set_data_receiver(self.new_data_received)

    def set_usb_port(self, usb_port):
        self.usb_port = usb_port

    def connect_to_sensor(self):
        self.ser.connect_to_controller(self.usb_port)

    def set_parent_node(self, node):
        self.parent_node = node
        pass

    def new_data_received(self, n_rows, n_columns, new_pressure_map):
        if self.parent_node is not None and self.parent_node.get_calibration_flag():
            # TODO new_pressure_map is 4095, calition should be 255
            # self.calibrate_sensor(new_pressure_map)
            pass
        self.image_actual = parse_data_to_np_image(n_rows, n_columns, new_pressure_map)
        # TODO image_actual is 255, calition takes 4095
        # if self.image_calibrated is not None:
        #     self.image_actual_calibrated = process_raw_image_through_calibration(n_rows, n_columns, self.image_actual,
        #                                                                          self.image_calibrated)

        if self.parent_node is not None:
            self.parent_node.new_image_from_sensor(self.image_actual)

    def calibrate_sensor(self, raw_data):
        self.image_calibrated = raw_data
