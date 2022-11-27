from sys import platform
import os
import time
import math as m

# import rospy
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import LaserScan

import serial
import serial.tools.list_ports as list_ports

MSG_PREFIX = 0x02
MSG_SUFFIX = 0x03
MSG_TYPE_TACTILE = 0x10

MSG_ERASE = b'\x02\x01\x00\x00\x00\x00\x00\x00\x00\x03'
MSG_CONFIRM = b'\x02\x02\x00\x00\x00\x00\x00\x00\x00\x03'

SCANNER_ANGLE_MIN = -1.57
SCANNER_ANGLE_MAX = 1.57
SCANNER_MIN_DIST = 0.4
scanner_dist_flag = 0

# config_keys = 4
# config_channels = 4
# config_angles = [
#     135, 105, 75, 45,
#     165, 135, 45, 15,
#     195, 225, 315, 345,
#     225, 255, 285, 315
# ]
# # config_distances = [
# #     200, 150, 150, 200,
# #     150, 100, 100, 150,
# #     150, 100, 100, 150,
# #     200, 150, 150, 200
# # ]
# config_distances = [
#     100, 100, 100, 100,
#     100, 100, 100, 100,
#     100, 100, 100, 100,
#     100, 100, 100, 100
# ]

# config_keys = 1
# config_channels = 1
# config_angles = [0]
# config_distances = [100]


config_keys = 5
config_channels = 8
config_angles = [
    81, 72, 63, 54, 45, 36, 27, 18,
    153, 144, 135, 126, 117, 108, 99, 90,
    225, 216, 207, 198, 189, 180, 171, 162,
    9, 0, 351, 342, 333, 324, 315, 306,
    297, 288, 279, 270, 261, 252, 243, 234,
]
config_distances = [
    100, 100, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100, 100, 100,
]


def python_nie_ma_makr_xd(value, out_bytes):
    return value.to_bytes(out_bytes, byteorder='little')


def connect_to_board():
    list_of_ports = list_ports.comports()
    for port in list_of_ports:
        print(port)

    try:
        if platform == "linux" or platform == "linux2":
            # os.chmod('/dev/ttyUSB0', 0o666)
            ser = serial.Serial('/dev/ttyUSB0')
        elif platform == "darwin":
            print("Change your computer")
            return 0
        elif platform == "win32":
            ser = serial.Serial('COM3')
        else:
            print("Unknown operating system")
            return 0

        ser.baudrate = 115200
        ser.timeout = 0.050
        ser.parity = serial.PARITY_NONE
        ser.stopbits = serial.STOPBITS_ONE
        ser.bytesize = serial.EIGHTBITS

        print("connected to: " + ser.portstr)
        return ser
    except serial.serialutil.SerialException:
        return 0


def configure_sensor(ser):
    # Check for valid config (number of items in list)
    if len(config_angles) != len(config_distances) or len(
            config_distances) != config_keys * config_channels:
        return 1

    res = config_channels * config_keys
    key = 0
    channel = 0

    ser.write(MSG_ERASE)

    for i in range(0, res):
        # Prepare message
        msg = python_nie_ma_makr_xd(MSG_PREFIX, 1) + \
              python_nie_ma_makr_xd(MSG_TYPE_TACTILE, 1) + \
              python_nie_ma_makr_xd(res - i - 1, 1) + \
              python_nie_ma_makr_xd(key, 1) + \
              python_nie_ma_makr_xd(channel, 1) + \
              python_nie_ma_makr_xd(config_angles[i], 2) + \
              python_nie_ma_makr_xd(config_distances[i], 2) + \
              python_nie_ma_makr_xd(MSG_SUFFIX, 1)

        print(msg)
        length = len(msg)
        if length != 10:
            return 1

        time.sleep(0.1)
        ser.write(msg)

        channel += 1
        if channel >= config_channels:
            channel = 0
            key += 1

    time.sleep(0.1)
    ser.write(MSG_CONFIRM)
    return 0


def read_sensor():
    value = ser.readline().decode('utf-8')
    if value and value != '\r':
        if value[0] == '\r':
            value = value[1:]
        force = int(value[5:9])
        angle = int(value[0:4])
        return force, angle
    return 0, 0


def setup_ROS():
    # pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
    pub = rospy.Publisher('/key_vel', Twist, queue_size=10)
    rospy.init_node('artificial_skin')
    rospy.Subscriber('scan_raw', LaserScan, laserCallback)
    return pub


def laserCallback(data_raw):
    first_el = int(round((SCANNER_ANGLE_MIN - data_raw.angle_min) / data_raw.angle_increment))
    nr_of_el = int(round((SCANNER_ANGLE_MAX - SCANNER_ANGLE_MIN) / data_raw.angle_increment))
    global scanner_dist_flag

    for i in range(first_el, first_el + nr_of_el + 1):
        if data_raw.ranges[i] < SCANNER_MIN_DIST:
            scanner_dist_flag = 1
            return
    scanner_dist_flag = 0
    return


def deg_to_rad(degree):
    return degree * m.pi / 180


if __name__ == "__main__":
    ser = connect_to_board()

    # ros_setup_flag = 0
    # pub = setup_ROS()
    # if pub:
    #     ros_setup_flag = 1

    # Check if we connected to board
    if not ser:
        print("Board not detected or busy")
    else:
        if configure_sensor(ser):
            print("Configuration unsuccessful")

        try:
            count = 0
            while True:
                force, angle = read_sensor()
                if force:
                    print(str(count) + str(': ') + str(angle) + str(', ') + str(force) + str(';'))

                    if ros_setup_flag:
                        linear = m.cos(deg_to_rad(angle))
                        angular = m.sin(deg_to_rad(angle))

                        if force < 100:
                            divider = 0
                        else:
                            divider = -1 * force / 10000.0
                        if divider > 1:
                            divider = 1

                        # print(linear)
                        # print(angular)
                        # print(divider)

                        twist = Twist()
                        if scanner_dist_flag and linear < 0:
                            twist.linear.x = 0
                            twist.angular.z = angular * divider * 4
                        else:
                            twist.linear.x = linear * divider
                            twist.angular.z = angular * divider * 2

                        # Safety from too big steering
                        if twist.angular.z > 1:
                            twist.angular.z = 1
                        if twist.linear.x > 1:
                            twist.linear.x = 1

                        pub.publish(twist)
                        # print(twist)

                    count = count + 1
        except serial.SerialException:
            print("Serial connection error, IGHT IMMA HEAD OUT")
            pass

    # ser.close()
    print("Adios")
