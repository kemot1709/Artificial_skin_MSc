from enum import Enum


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
