import time
import os
from sys import platform

# Suppress tensorflow noncritical warnings
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

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
    sensor.connect_to_controller()
    node = TableNode()
    node.set_sensor(sensor)

    while 1:
        pass
