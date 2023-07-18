import time

from nodes.table import TableNode
from nodes.messages import prepare_bool_msg, prepare_image_msg, prepare_string_msg

from connection.connection import Serial


def data_receiver(n_rows, n_columns, new_pressure_map):
    print(new_pressure_map[8][8])


if __name__ == "__main__":
    node = TableNode()

    ser = Serial()
    ser.set_data_receiver(data_receiver)
    ser.connect_to_controller("COM3")

    while 1:
        node.publish_msg_on_topic("/is_placed", prepare_bool_msg(True))
        time.sleep(1)
        node.publish_msg_on_topic("/is_placed", prepare_bool_msg(False))
        time.sleep(1)
