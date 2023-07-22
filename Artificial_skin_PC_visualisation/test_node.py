import time
from sys import platform

from nodes.table import TableNode
from nodes.messages import prepare_bool_msg, prepare_image_msg, prepare_string_msg

from connection.connection import Serial


class Receiver:
    table_node = None

    def set_table_node(self, table_node):
        self.table_node = table_node

    def data_receiver(self, n_rows, n_columns, new_pressure_map):
        if self.table_node is not None:
            self.table_node.new_image_from_sensor(new_pressure_map)


if __name__ == "__main__":
    node = TableNode()

    data_receiver = Receiver()
    data_receiver.set_table_node(node)

    ser = Serial()
    ser.set_data_receiver(data_receiver)
    # ser.connect_to_controller("COM3")
    # ser.connect_to_controller("/dev/ttyUSB0")

    if platform == "linux" or platform == "linux2":
        # os.chmod('/dev/ttyUSB0', 0o666)
        ser.connect_to_controller('/dev/ttyUSB1')
    elif platform == "darwin":
        print("Change your computer")
        exit()
    elif platform == "win32":
        ser.connect_to_controller('COM3')
    else:
        print("Unknown operating system")
        exit()

    weight = 0
    while 1:
        node.publish_is_placed(False)
        weight += 1
        node.publish_weight(weight)
        node.publish_predicted_item("dupa")
        node.publish_status("dupa")
        node.publish_location("dupa")
        node.publish_raw_image("dupa")
        time.sleep(1)
        node.publish_is_placed(True)
        time.sleep(1)
