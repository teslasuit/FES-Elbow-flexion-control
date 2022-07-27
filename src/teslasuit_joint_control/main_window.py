import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore, QtGui, QtWidgets
from ctypes import *
from PyQt5.QtWidgets import QSlider, QLabel, QRadioButton, QButtonGroup


class AngleBiomechanicsPlotter():
    def __init__(self):
        # Liveplot drawers initialization
        self.x = np.arange(0, 1000)
        self.angle_drawer = np.zeros(1000)
        self.desired_angle_drawer = np.zeros(1000)
        self.left_pid_output = np.zeros(1000)
# Radiobuttons for control arm choise


class ControlTypeRadioButton():
    def __init__(self):
        print("Buttons inited")
        self.controlled_arm_choise_bg = QButtonGroup()
        self.left_arm_chose_button = QRadioButton("Left arm")
        self.left_arm_chose_button.setChecked(True)
        self.right_arm_chose_button = QRadioButton("Right arm")

        self.controlled_arm_choise_bg.addButton(self.left_arm_chose_button)
        self.controlled_arm_choise_bg.addButton(self.right_arm_chose_button)

        self.controlling_method_choise_bg = QButtonGroup()
        self.manual_control_choise_button = QRadioButton("Manual control")
        self.manual_control_choise_button.setChecked(True)

        self.opposite_arm_choise_button = QRadioButton("Opposite arm control")

        self.controlling_method_choise_bg.addButton(
            self.manual_control_choise_button)
        self.controlling_method_choise_bg.addButton(
            self.opposite_arm_choise_button)

        self.pause_button = QtWidgets.QPushButton('calibrate muscle response')
        self.reset_button = QtWidgets.QPushButton('Reset PID')


class DesiredAngleSlider():
    def __init__(self):
        # Angle slider initialization
        self.desired_angle = 0
        self.Angleslider = QSlider(QtCore.Qt.Horizontal, self)
        self.Angleslider.setRange(0, 200)
        self.Angleslider.setMinimumWidth(600)
        self.Angleslider.setValue(0)
        self.Angleslider_label = QLabel('Angle ' + str(0))
        self.Angleslider_label.setAlignment(
            QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

 # PID sliders initializtion


class PIDSlider():
    def __init__(self):
        self.slider_P = QSlider(QtCore.Qt.Horizontal, self)
        self.slider_P.setRange(0, 550)
        self.slider_P.setMinimumWidth(600)
        self.slider_P.setValue(0)
        self.slider_P.valueChanged.connect(self.change_P)
        self.slider_label_P = QLabel('PID Proportional ' + str(0))
        self.slider_label_P.setAlignment(
            QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

        self.slider_I = QSlider(QtCore.Qt.Horizontal, self)
        self.slider_I.setRange(0, 1500)
        self.slider_I.setMinimumWidth(600)
        self.slider_I.setValue(0)
        self.slider_I.valueChanged.connect(self.change_I)
        self.slider_label_I = QLabel('PID Integral ' + str(0))
        self.slider_label_I.setAlignment(
            QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

        self.slider_D = QSlider(QtCore.Qt.Horizontal, self)
        self.slider_D.setRange(0, 150)
        self.slider_D.setMinimumWidth(600)
        self.slider_D.setValue(0)
        self.slider_D.valueChanged.connect(self.change_D)
        self.slider_label_D = QLabel('PID Differential ' + str(0))
        self.slider_label_D.setAlignment(
            QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

    # PID sliders functions
    def change_P(self):
        P = self.slider_P.value()
        self.pid_parameters_list[0] = P / 100
        self.slider_label_P.setText('PID Proportional ' + str(P / 100))

    def change_I(self):
        I = self.slider_I.value()
        self.pid_parameters_list[1] = I / 100
        self.slider_label_I.setText('PID Integral ' + str(I / 100))

    def change_D(self):
        D = self.slider_D.value()
        self.pid_parameters_list[2] = D / 100
        self.slider_label_D.setText('PID Differential ' + str(D / 100))


class PIDMainWindow(
        QtWidgets.QMainWindow,
        ControlTypeRadioButton,
        DesiredAngleSlider,
        PIDSlider,
        AngleBiomechanicsPlotter):
    def __init__(self, parent=None):
        super(QtWidgets.QMainWindow, self).__init__(parent)
        super(ControlTypeRadioButton, self).__init__()
        super(DesiredAngleSlider, self).__init__()
        super(PIDSlider, self).__init__()

        # GUI Spaces Initialization
        self.mainbox = QtWidgets.QWidget()
        self.setCentralWidget(self.mainbox)
        self.mainbox.setLayout(QtWidgets.QVBoxLayout())

        self.Angle_canvas = pg.GraphicsLayoutWidget()
        self.Angle_plot = self.Angle_canvas.addPlot()
        self.Angle_plot.addLegend(offset=(3, 3))
        self.mainbox.layout().addWidget(self.Angle_canvas)

        self.SliderBox = QtWidgets.QVBoxLayout()
        self.mainbox.layout().addLayout(self.SliderBox)

        self.PlotBox = QtWidgets.QHBoxLayout()
        self.mainbox.layout().addLayout(self.PlotBox)

        self.Leftbox = QtWidgets.QVBoxLayout()
        self.PlotBox.layout().addLayout(self.Leftbox)

        self.Rightbox = QtWidgets.QVBoxLayout()
        self.PlotBox.layout().addLayout(self.Rightbox)

        self.Left_data_status = QtWidgets.QVBoxLayout()
        self.Leftbox.addLayout(self.Left_data_status)

        self.Left_haptic_canvas = pg.GraphicsLayoutWidget()
        self.Leftbox.layout().addWidget(self.Left_haptic_canvas)
        self.Left_Haptic_plot1 = self.Left_haptic_canvas.addPlot(row=0, col=0)
        self.Left_Haptic_plot1.addLegend(offset=(3, 3))
        self.Left_Haptic_plot1.addLine(y=100)
        self.Left_Haptic_plot1.addLine(y=-100)
        self.Angle_plot.addLegend(offset=(3, 3))
        self.Left_Haptic_plot1.addLegend(offset=(3, 3))
        super(AngleBiomechanicsPlotter, self).__init__()
        self.angle_widget = self.Angle_plot.plot(
            x=self.x,
            y=self.angle_drawer,
            pen=QtGui.QColor('papayawhip'),
            name='Сurrent angle')
        self.desired_angle_widget = self.Angle_plot.plot(
            x=self.x,
            y=self.desired_angle_drawer,
            pen=QtGui.QColor('red'),
            name='Desired angle')

        self.left_pid = self.Left_Haptic_plot1.plot(
            x=self.x,
            y=self.left_pid_output,
            pen=QtGui.QColor('wheat'),
            name='1-st PID response')

        self.Left_haptic_status = QtWidgets.QVBoxLayout()
        self.Leftbox.addLayout(self.Left_haptic_status)

        self.controlled_arm_choise_box = QtWidgets.QHBoxLayout()
        self.Rightbox.layout().addLayout(self.controlled_arm_choise_box)

        self.controlling_method_choise_box = QtWidgets.QHBoxLayout()
        self.Rightbox.layout().addLayout(self.controlling_method_choise_box)

        self.PlotBox.layout().addWidget(self.pause_button)
        self.controlling_method_choise_box.layout().addWidget(
            self.manual_control_choise_button)
        self.controlled_arm_choise_box.layout().addWidget(self.left_arm_chose_button)
        self.controlled_arm_choise_box.layout().addWidget(self.right_arm_chose_button)
        self.controlling_method_choise_box.layout().addWidget(
            self.opposite_arm_choise_button)
        self.PlotBox.layout().addWidget(self.reset_button)

        self.SliderBox.addWidget(self.Angleslider_label)
        self.SliderBox.layout().addWidget(self.Angleslider)

        self.Leftbox.addWidget(self.slider_label_P)
        self.Leftbox.layout().addWidget(self.slider_P)
        self.Leftbox.addWidget(self.slider_label_I)
        self.Leftbox.layout().addWidget(self.slider_I)
        self.Leftbox.addWidget(self.slider_label_D)
        self.Leftbox.layout().addWidget(self.slider_D)
