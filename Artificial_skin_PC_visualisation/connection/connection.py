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
        super(Serial, self).__init__()
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
            self.start()
            return

        if self.ui is not None and self.ser is not None:
            self.pressureMapUpdated.connect(self.ui.updateMap)
            self.start()
            return

    def send_data_explicitely(self):
        if self.data_receiver is not None and self.ser is not None:
            self.data_receiver(self.rows, self.columns, self.pressure_map)
            return

        if self.ui is not None and self.ser is not None:
            self.ui.updateMap(self.rows, self.columns, self.pressure_map)
            return

    @staticmethod
    def get_list_of_ports():
        list_of_ports = []
        for port in list_ports.comports():
            list_of_ports.append(port.device)
        return list_of_ports

    def connect_to_controller(self, port):
        try:
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
                        continue

                    if len(input_msg) > 0 and input_msg[0] != '\r' and input_msg[0] != '\n' and input_msg[0] != '\0':
                        lines = input_msg.strip().split('|')

                        i = 0
                        for line in lines:
                            if i >= self.rows:
                                break
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
                            # TODO multithreating on python 2
                        # self.pressureMapUpdated.emit(self.rows, self.columns, self.pressure_map)
                        self.send_data_explicitely()
            except Exception as e:
                print(e)
                exc_type, exc_obj, exc_tb = sys.exc_info()
                fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
                print(exc_type, fname, exc_tb.tb_lineno)
