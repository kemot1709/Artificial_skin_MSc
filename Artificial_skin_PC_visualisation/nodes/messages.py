from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String, Header


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


def prepare_image_msg(header_str, val):
    if type(header_str) is str:  # TODO
        msg = Image()
        msg.header = Header()
        msg.header.frame_id = header_str
        # TODO handling of image
        return msg
