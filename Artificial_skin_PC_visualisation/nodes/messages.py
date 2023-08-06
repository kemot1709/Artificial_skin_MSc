import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String, Header, Int32
from sensor.data_parsing import parse_np_image_to_msg


# TODO dictionary of possible values of strings

def prepare_bool_msg(val):
    if type(val) is bool:
        msg = Bool()
        msg.data = val
        return msg


def prepare_string_msg(val):
    if type(val) is str:
        msg = String()
        msg.data = val
        return msg


def prepare_int32_msg(val):
    if type(val) is Int32 or type(val) is int:
        msg = Int32()
        msg.data = val
        return msg


def prepare_image_msg(header_str, val):
    if type(header_str) is str and type(val) is np.ndarray:
        try:
            msg = parse_np_image_to_msg(val)
            msg.header = Header()
            msg.header.frame_id = header_str
        except:
            print("Unsuccessfull image parsing")
            return None
        return msg
