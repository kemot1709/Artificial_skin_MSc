import time
from sys import platform

from nodes.table import TableNode
from nodes.messages import prepare_bool_msg, prepare_image_msg, prepare_string_msg

from sensor.sensor import Sensor


def default_port_name():
    if platform == "linux" or platform == "linux2":
        # os.chmod('/dev/ttyUSB0', 0o666)
        return '/dev/ttyUSB0'
    elif platform == "darwin":
        print("Change your computer")
        exit()
    elif platform == "win32":
        return 'COM3'
    else:
        print("Unknown operating system")
        exit()


if __name__ == "__main__":
    sensor = Sensor(default_port_name())
    sensor.connect_to_sensor()
    node = TableNode()
    node.set_sensor(sensor)

    weight = 0
    while 1:
        node.publish_is_placed(False)
        weight += 1
        node.publish_weight(weight)
        node.publish_predicted_item("dupa")
        node.publish_status("dupa")
        node.publish_location("dupa")
        # node.publish_image("dupa")
        time.sleep(1)
        node.publish_is_placed(True)
        time.sleep(1)
