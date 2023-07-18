from sys import platform

import serial
import serial.tools.list_ports as list_ports

from PyQt5 import QtCore


class Serial(QtCore.QThread):
    rows = 16
    columns = 16
    pressure_map = None
    ser = None
    ui = None
    data_receiver = None

    pressureMapUpdated = QtCore.pyqtSignal(int, int, list)

    def __init__(self):
        super(QtCore.QThread, self).__init__()
        self.pressure_map = [[0 for x in range(self.columns)] for y in range(self.rows)]

        self.exitFlag = False

    def set_ui(self, ui):
        self.ui = ui
        self.start_communication_with_ui()

    def set_data_receiver(self, receiver):
        self.data_receiver = receiver

    def start_communication_with_ui(self):
        if self.data_receiver is not None and self.ser is not None:
            self.pressureMapUpdated.connect(self.data_receiver)
            super().start()
            return

        if self.ui is not None and self.ser is not None:
            self.pressureMapUpdated.connect(self.ui.updateMap)
            super().start()

    @staticmethod
    def get_list_of_ports():
        list_of_ports = []
        for port in list_ports.comports():
            list_of_ports.append(port.device)
        return list_of_ports

    def connect_to_controller(self, port):
        try:
            # if platform == "linux" or platform == "linux2":
            #     # os.chmod('/dev/ttyUSB0', 0o666)
            #     self.ser = serial.Serial('/dev/ttyUSB0')
            # elif platform == "darwin":
            #     print("Change your computer")
            #     return 0
            # elif platform == "win32":
            #     self.ser = serial.Serial('COM3')
            # else:
            #     print("Unknown operating system")
            #     return 0

            self.ser = serial.Serial(port)
            self.ser.baudrate = 115200
            self.ser.timeout = 0.050
            self.ser.parity = serial.PARITY_NONE
            self.ser.stopbits = serial.STOPBITS_ONE
            self.ser.bytesize = serial.EIGHTBITS

            print("Connected to: " + self.ser.portstr)
            self.start_communication_with_ui()

        # Communication crashed
        except serial.serialutil.SerialException:
            return 0

    def run(self):
        if self.ser is not None:
            self.ser.flush()
            try:
                while not self.exitFlag:
                    # Read values from serial and make sure it's not garbage
                    try:
                        input_msg = self.ser.readline().decode('utf-8')
                    except:
                        input_msg = str()
                        return

                    if len(input_msg) > 0 and input_msg[0] != '\r' and input_msg[0] != '\n' and input_msg[0] != '\0':
                        lines = input_msg.strip().split('|')

                        i = 0
                        for line in lines:
                            if i >= self.rows:
                                continue
                            list_of_values = line.strip().split(',')

                            j = 0
                            for value in list_of_values:
                                if j >= self.columns:
                                    break

                                try:
                                    self.pressure_map[j][i] = float(value)
                                except:
                                    print("Kurwaaaaaaaaaaaaa", i, j, int(value))
                                    pass
                                j = j + 1
                            i = i + 1
                        self.pressureMapUpdated.emit(self.rows, self.columns, self.pressure_map)
            except Exception as e:
                print(e)
