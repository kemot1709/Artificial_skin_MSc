import sys
from sys import platform
import time
from datetime import datetime, timedelta

from PyQt5 import QtCore, QtGui, QtWidgets

import serial
import serial.tools.list_ports as list_ports

import numpy as np
from PIL import Image as im

HYP_A = 35108.0
HYP_X0 = 0.0
HYP_Y = 0.42
RES = 470
FILE_OUT_WEIGHT = "weight_test"


def weight_of_items(map_of_pressure, max_value):
    weight = 0

    for row in map_of_pressure:
        for tile in row:
            # Have to be rounded because otherwise have calculation errors with small weight
            normalized_pressure = round(tile * (4095 / max_value), 1)
            try:
                if normalized_pressure >= 4095.0:
                    continue
                resistance = (RES * normalized_pressure) / (4095 - normalized_pressure)
                weight += (HYP_X0 + (HYP_A / (resistance - HYP_Y)))
            except ZeroDivisionError:
                continue

    if weight < 0.0:
        weight = 0.0
    return weight


class Ui_MainWindow(object):
    rows = 16
    columns = 16
    value = 100
    map = None
    map_calibrated = None
    last_timestamp = datetime.now()
    values = [0 for x in range(10)]

    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1300, 500)
        self.app_screen = QtWidgets.QWidget(MainWindow)
        self.app_screen.setObjectName("app_screen")
        MainWindow.setCentralWidget(self.app_screen)

        self.graphics_view = QtWidgets.QGraphicsView(self.app_screen)
        self.graphics_view.setGeometry(QtCore.QRect(0, 0, 1000, 500))
        self.graphics_view.setObjectName("graphics_view")

        self.graphics_scene = QtWidgets.QGraphicsScene()
        self.graphics_view.scale(1, -1)
        self.graphics_view.setScene(self.graphics_scene)
        self.graphics_view.fitInView(self.graphics_scene.sceneRect(), QtCore.Qt.KeepAspectRatio)

        self.weight_1 = QtWidgets.QLabel(self.app_screen)
        self.weight_1.move(1000, 50)
        self.weight_1.setFont(QtGui.QFont('Times', 15))
        self.weight_1.setText("M1:")
        self.weight_1_value = QtWidgets.QLabel(self.app_screen)
        self.weight_1_value.move(1100, 50)
        self.weight_1_value.setFont(QtGui.QFont('Times', 15))
        self.weight_1_value.resize(QtCore.QSize(170, 50))
        self.weight_1_value.setText("      ")

        self.weight_2 = QtWidgets.QLabel(self.app_screen)
        self.weight_2.move(1000, 100)
        self.weight_2.setFont(QtGui.QFont('Times', 15))
        self.weight_2.setText("M2:")
        self.weight_2_value = QtWidgets.QLabel(self.app_screen)
        self.weight_2_value.move(1100, 100)
        self.weight_2_value.setFont(QtGui.QFont('Times', 15))
        self.weight_2_value.resize(QtCore.QSize(170, 50))
        self.weight_2_value.setText("      ")

        self.weight_3 = QtWidgets.QLabel(self.app_screen)
        self.weight_3.move(1000, 150)
        self.weight_3.setFont(QtGui.QFont('Times', 15))
        self.weight_3.setText("M3:")
        self.weight_3_value = QtWidgets.QLabel(self.app_screen)
        self.weight_3_value.move(1100, 150)
        self.weight_3_value.setFont(QtGui.QFont('Times', 15))
        self.weight_3_value.resize(QtCore.QSize(170, 50))
        self.weight_3_value.setText("      ")

        self.weight_4 = QtWidgets.QLabel(self.app_screen)
        self.weight_4.move(1000, 200)
        self.weight_4.setFont(QtGui.QFont('Times', 15))
        self.weight_4.setText("M4:")
        self.weight_4_value = QtWidgets.QLabel(self.app_screen)
        self.weight_4_value.move(1100, 200)
        self.weight_4_value.setFont(QtGui.QFont('Times', 15))
        self.weight_4_value.resize(QtCore.QSize(170, 50))
        self.weight_4_value.setText("      ")

        self.weight_5 = QtWidgets.QLabel(self.app_screen)
        self.weight_5.move(1000, 250)
        self.weight_5.setFont(QtGui.QFont('Times', 15))
        self.weight_5.setText("M5A:")
        self.weight_5_value = QtWidgets.QLabel(self.app_screen)
        self.weight_5_value.move(1100, 250)
        self.weight_5_value.setFont(QtGui.QFont('Times', 15))
        self.weight_5_value.resize(QtCore.QSize(170, 50))
        self.weight_5_value.setText("      ")

        self.weight_6 = QtWidgets.QLabel(self.app_screen)
        self.weight_6.move(1000, 300)
        self.weight_6.setFont(QtGui.QFont('Times', 15))
        self.weight_6.setText("M5B:")
        self.weight_6_value = QtWidgets.QLabel(self.app_screen)
        self.weight_6_value.move(1100, 300)
        self.weight_6_value.setFont(QtGui.QFont('Times', 15))
        self.weight_6_value.resize(QtCore.QSize(170, 50))
        self.weight_6_value.setText("      ")

        self.weight_7 = QtWidgets.QLabel(self.app_screen)
        self.weight_7.move(1000, 350)
        self.weight_7.setFont(QtGui.QFont('Times', 15))
        self.weight_7.setText("M5C:")
        self.weight_7_value = QtWidgets.QLabel(self.app_screen)
        self.weight_7_value.move(1100, 350)
        self.weight_7_value.setFont(QtGui.QFont('Times', 15))
        self.weight_7_value.resize(QtCore.QSize(170, 50))
        self.weight_7_value.setText("      ")

        self.weight_8 = QtWidgets.QLabel(self.app_screen)
        self.weight_8.move(1000, 400)
        self.weight_8.setFont(QtGui.QFont('Times', 15))
        self.weight_8.setText("M6A:")
        self.weight_8_value = QtWidgets.QLabel(self.app_screen)
        self.weight_8_value.move(1100, 400)
        self.weight_8_value.setFont(QtGui.QFont('Times', 15))
        self.weight_8_value.resize(QtCore.QSize(170, 50))
        self.weight_8_value.setText("      ")

        self.weight_9 = QtWidgets.QLabel(self.app_screen)
        self.weight_9.move(1000, 450)
        self.weight_9.setFont(QtGui.QFont('Times', 15))
        self.weight_9.setText("M6B:")
        self.weight_9_value = QtWidgets.QLabel(self.app_screen)
        self.weight_9_value.move(1100, 450)
        self.weight_9_value.setFont(QtGui.QFont('Times', 15))
        self.weight_9_value.resize(QtCore.QSize(170, 50))
        self.weight_9_value.setText("      ")

        self.button_save = QtWidgets.QPushButton(self.app_screen)
        self.button_save.move(1100, 10)
        self.button_save.resize(200, 30)
        self.button_save.setText("Save data to file")
        self.button_save.clicked.connect(self.buttonSaveToFileHandler)

        self.button_calibrate = QtWidgets.QPushButton(self.app_screen)
        self.button_calibrate.move(1010, 10)
        self.button_calibrate.resize(90, 30)
        self.button_calibrate.setText("Calibrate sensors")
        self.button_calibrate.clicked.connect(self.buttonCalibrateHandler)

        self.map_method_1 = [[0 for x in range(self.columns)] for y in range(self.rows)]
        self.map_method_2 = [[0 for x in range(self.columns)] for y in range(self.rows)]
        self.map_method_3 = [[0 for x in range(self.columns)] for y in range(self.rows)]
        self.map_method_4 = [[0 for x in range(self.columns)] for y in range(self.rows)]
        self.map_method_5 = [[0 for x in range(self.columns)] for y in range(self.rows)]
        self.map_method_6 = [[0 for x in range(self.columns)] for y in range(self.rows)]
        self.map_method_7 = [[0 for x in range(self.columns)] for y in range(self.rows)]
        self.map_method_8 = [[0 for x in range(self.columns)] for y in range(self.rows)]
        self.map_method_9 = [[0 for x in range(self.columns)] for y in range(self.rows)]

        # self.statusbar = QtWidgets.QStatusBar(MainWindow)
        # self.statusbar.setObjectName("statusbar")
        # MainWindow.setStatusBar(self.statusbar)

        # TODO Co to robi????
        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))

    def updateMap(self, n_rows, n_columns, new_pressure_map):
        # TODO coś z tymi rzędami i kolumnami zrobić
        # może wyjebać?
        self.map = new_pressure_map

        if self.map_calibrated is None:
            self.recalibrateMap(new_pressure_map)

        self.repaintMap()

    def recalibrateMap(self, new_map):
        self.map_calibrated = new_map

    def repaintMap(self):
        height = self.graphics_view.size().height()
        width = self.graphics_view.size().width()
        self.graphics_scene.clear()
        self.graphics_scene.setSceneRect(0.0, 0.0, width - 10, height - 10)

        fieldWidth = width / self.columns
        fieldHeight = height / self.rows

        map_255 = [[0 for x in range(self.columns)] for y in range(self.rows)]
        map_with_cabration_diff = [[0 for x in range(self.columns)] for y in range(self.rows)]

        map_max_value = max(max(self.map))
        filterA = 4000
        filterB = 3950
        filterC = 3900
        multA = 0.99
        multB = 0.98

        for i in range(self.columns):
            x0 = i * fieldWidth
            x1 = x0 + fieldWidth

            for j in range(self.rows):
                y0 = j * fieldHeight
                y1 = y0 + fieldHeight

                self.map_method_1[i][j] = self.map[i][j]
                self.map_method_2[i][j] = self.map[i][j] + 4095 - map_max_value
                self.map_method_3[i][j] = self.map[i][j] + 4095 - self.map_calibrated[i][j]
                self.map_method_4[i][j] = self.map[i][j] * (4095 / self.map_calibrated[i][j])

                self.map_method_5[i][j] = self.map[i][j]
                if self.map_method_5[i][j] > filterA:
                    self.map_method_5[i][j] = filterA
                self.map_method_6[i][j] = self.map[i][j]
                if self.map_method_6[i][j] > filterB:
                    self.map_method_6[i][j] = filterB
                self.map_method_7[i][j] = self.map[i][j]
                if self.map_method_7[i][j] > filterC:
                    self.map_method_7[i][j] = filterC

                self.map_method_8[i][j] = self.map[i][j]
                if self.map_method_8[i][j] > multA * self.map_calibrated[i][j]:
                    self.map_method_8[i][j] = multA * self.map_calibrated[i][j]
                self.map_method_8[i][j] = self.map_method_8[i][j] * 4095 / (multA * self.map_calibrated[i][j])

                self.map_method_9[i][j] = self.map[i][j]
                if self.map_method_9[i][j] > multB * self.map_calibrated[i][j]:
                    self.map_method_9[i][j] = multB * self.map_calibrated[i][j]
                self.map_method_9[i][j] = self.map_method_9[i][j] * 4095 / (multB * self.map_calibrated[i][j])

                # Calculating by previously saved calibration at empty table of every field
                # self.value = int(255 - self.map[i][j] * 255 / self.map_calibrated[i][j])

                # Calculating by using filter at level, every value lower than filter is considered as valid
                # filter = 3850
                # if self.map[i][j] > filter:
                #     self.map[i][j] = filter
                # self.value = int(255 - self.map[i][j] * 255 / filter)

                # Calculating for perfect sensor, dont need validity check
                self.value = int(255 - self.map[i][j] * 255 / 4095)
                self.value *= 10

                if self.value > 255:
                    self.value = 255
                if self.value < 0:
                    self.value = 0

                map_255[i][j] = self.value
                brush = QtGui.QBrush(QtGui.QColor(self.value, self.value, self.value))
                pen = QtGui.QPen(QtGui.QColor(self.value, self.value, self.value), 1.0)
                self.graphics_scene.addRect(x0, y0, x1, y1, pen, brush)

        self.values[0] = str(weight_of_items(self.map_method_1, 4095))
        self.values[1] = str(weight_of_items(self.map_method_2, 4095))
        self.values[2] = str(weight_of_items(self.map_method_3, 4095))
        self.values[3] = str(weight_of_items(self.map_method_4, 4095))
        self.values[4] = str(weight_of_items(self.map_method_5, filterA))
        self.values[5] = str(weight_of_items(self.map_method_6, filterB))
        self.values[6] = str(weight_of_items(self.map_method_7, filterC))
        self.values[7] = str(weight_of_items(self.map_method_8, 4095))
        self.values[8] = str(weight_of_items(self.map_method_9, 4095))

        self.weight_1_value.setText(self.values[0])
        self.weight_2_value.setText(self.values[1])
        self.weight_3_value.setText(self.values[2])
        self.weight_4_value.setText(self.values[3])
        self.weight_5_value.setText(self.values[4])
        self.weight_6_value.setText(self.values[5])
        self.weight_7_value.setText(self.values[6])
        self.weight_8_value.setText(self.values[7])
        self.weight_9_value.setText(self.values[8])

        t = datetime.now()
        if (t - self.last_timestamp).seconds > 5.0:
            self.last_timestamp = t

            image = im.fromarray(np.array(map_255, dtype=np.uint8))
            stamp = t.strftime('%H_%M_%S')
            # image.save(r'img\image_' + stamp + '.png')
            # print('image saved')

    def buttonSaveToFileHandler(self):
        with open(FILE_OUT_WEIGHT, "a") as file:
            file.write(
                    "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (
                            self.values[0], self.values[1], self.values[2], self.values[3],
                            self.values[4], self.values[5], self.values[6], self.values[7],
                            self.values[8]))

    def buttonCalibrateHandler(self):
        self.recalibrateMap(self.map)


ui = Ui_MainWindow()


class Serial(QtCore.QThread):
    rows = 16
    columns = 16
    pressure_map = None
    ser = None

    pressureMapUpdated = QtCore.pyqtSignal(int, int, list)

    def __init__(self):
        super().__init__()
        self.pressure_map = [[0 for x in range(self.columns)] for y in range(self.rows)]
        self.connect_to_board()

        self.exitFlag = False

    def connect_to_board(self):
        list_of_ports = list_ports.comports()
        for port in list_of_ports:
            print(port)

        try:
            if platform == "linux" or platform == "linux2":
                # os.chmod('/dev/ttyUSB0', 0o666)
                self.ser = serial.Serial('/dev/ttyUSB0')
            elif platform == "darwin":
                print("Change your computer")
                return 0
            elif platform == "win32":
                self.ser = serial.Serial('COM3')
            else:
                print("Unknown operating system")
                return 0

            self.ser.baudrate = 115200
            self.ser.timeout = 0.050
            self.ser.parity = serial.PARITY_NONE
            self.ser.stopbits = serial.STOPBITS_ONE
            self.ser.bytesize = serial.EIGHTBITS

            print("connected to: " + self.ser.portstr)
            self.pressureMapUpdated.connect(ui.updateMap)
            super().start()

        # Wywaliło komunikację
        except serial.serialutil.SerialException:
            return 0

    def run(self):
        self.ser.flush()
        try:
            while not self.exitFlag:
                # Read values from serial and make sure it's not garbage
                try:
                    input_msg = self.ser.readline().decode('utf-8')
                except:
                    input_msg = str()

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


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)

    ser = Serial()
    if not ser:
        print("Board not detected or busy")
        sys.exit()
    else:
        MainWindow = QtWidgets.QMainWindow()
        ui.setupUi(MainWindow)

    MainWindow.show()
    sys.exit(app.exec_())
