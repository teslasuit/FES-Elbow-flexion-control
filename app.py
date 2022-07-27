
""" This is an example  of single-PID control over the elbow angle.
In this example user have an asses to single PID controlled biceps stimulation with
possibility to calibrate controller manually and both manual and opposite-side control modes.
"""
import os
import sys
import time
import numpy as np
from scipy.fft import rfft, rfftfreq
from PyQt5 import QtCore, QtWidgets
from src.teslasuit_joint_control.suit_handler import Teslasuit
from src.teslasuit_joint_control.control_system import *
from src.teslasuit_joint_control.main_window import PIDMainWindow

ts_api_path = os.environ['TESLASUIT_PYTHON_API_PATH']
sys.path.append(ts_api_path)


class App(PIDMainWindow):
    def __init__(self, parent=None):
        super(App, self).__init__(parent)
        # Suit initialization
        self.suit = Teslasuit()
        self.suit.connect_suit()

        self.model = SkeletalModel()

        self.model_updater = SkeletalModelUpdater(self.suit, self.model)
        self.model_updater.calibrate()

        self.controller = PIDSkeletalModelController(self.suit, self.model)
        self.pid_parameters_list = [0, 0, 0]

        # Timers initialization
        self.view_update_timer = QtCore.QTimer()
        self.view_update_timer.timeout.connect(lambda: self.view_update())
        self.view_update_timer.start(50)

        self.PID_update = QtCore.QTimer()
        self.PID_update.timeout.connect(lambda: self.update_pid())
        self.PID_update.start(50)

        self.Haptic_update = QtCore.QTimer()
        self.Haptic_update.timeout.connect(lambda: self.send_haptic_touch())
        self.Haptic_update.start(50)

        self.SF_calibration = QtCore.QTimer()
        self.SF_calibration.timeout.connect(lambda: self.SF_calibration_loop())
        self.oscilation_calibration = QtCore.QTimer()

        self.oscilation_calibration.timeout.connect(
            lambda: self.integral_part_calbration())

        # buttons connection
        self.pause_button.clicked.connect(self.calibrate_PID)
        self.reset_button.clicked.connect(self.reset_PID)

        # self.left_arm_chose_button.toggled.connect(self.change_controlled_arm)
        # self.right_arm_chose_button.toggled.connect(self.change_controlled_arm)
        self.Angleslider.valueChanged.connect(self.change_angle)

    def update_pid(self):
        if self.left_arm_chose_button.isChecked():
            self.current_controlled_side = 'Left'
            self.controlled_joint = self.model.left_elbow_joint
            self.opposite_joint = self.model.right_elbow_joint
        elif self.right_arm_chose_button.isChecked():
            self.current_controlled_side = 'Right'
            self.controlled_joint = self.model.right_elbow_joint
            self.opposite_joint = self.model.left_elbow_joint
        self.controlled_joint.controller.saggital_plane_controller.agonist_controller.parameters_list = self.pid_parameters_list
        self.controlled_joint.controller.saggital_plane_controller.agonist_controller.change_parameters()
        self.controlled_joint.controller.update_haptic_output()
        print(self.pid_parameters_list)

    def send_haptic_touch(self):
        self.controlled_joint.controller.saggital_plane_controller.agonist_controller.send_haptic_output()

    def reset_PID(self):
        self.pid_parameters_list = [0, 0, 0]
        self.slider_P.setValue(0)
        self.slider_I.setValue(0)
        self.slider_D.setValue(0)

    def SF_calibration_loop(self):
        self.pid_parameters_list[0] += 0.3
        if self.controlled_joint.saggital_plane.angle >= self.desired_angle or self.pid_parameters_list[
                0] >= 3:
            self.Proportional = self.pid_parameters_list[0]
            self.slider_P.setValue(int(self.Proportional / 100))
            print('Proportional calibrated', self.Proportional)
            self.SF_calibration.stop()
            self.start_oscilation_time = time.time()
            self.oscilation_calibration.start()

    def integral_part_calbration(self):
        self.oscilation_dict['timestamp'].append(
            time.time() - self.start_oscilation_time)
        self.oscilation_dict['angle'].append(
            self.controlled_joint.saggital_plane.angle)
        print(self.oscilation_dict['timestamp'][-1])
        if (time.time() - self.start_oscilation_time) >= 10:
            self.oscilation_calibration.stop()
            self.max_oscilation_harm = np.argmax(
                abs(rfft(self.oscilation_dict['angle'])[1:]))
            self.oscilation_freq_array = rfftfreq(
                len(self.oscilation_dict['angle']), 5 / len(self.oscilation_dict['angle']))
            self.oscilation_freq = self.oscilation_freq_array[self.max_oscilation_harm]
            self.Integral = 0.5 * 1 / self.oscilation_freq
            self.Derivative = 0.125 * 1 / self.oscilation_freq
            self.slider_P.setValue(int(self.Proportional * 100))
            print('Proportional calibrated', self.Proportional)
            self.slider_I.setValue(int(self.Integral * 100))
            print('Integral calibrated', self.Integral)
            self.slider_D.setValue(int(self.Derivative * 100))
            print('Derivative calibrated', self.Derivative)

    def calibrate_PID(self):
        self.Angleslider.setValue(int(100))
        self.desired_angle = 100
        self.oscilation_dict = {}
        self.oscilation_dict['timestamp'] = []
        self.oscilation_dict['angle'] = []
        self.SF_calibration.start(500)

    def change_angle(self):
        self.desired_angle = self.Angleslider.value()
        self.Angleslider_label.setText('Angle ' + str(self.desired_angle))

    def view_update(self):
        self.model_updater.update_angles()
        self.tracking_angle = self.controlled_joint.saggital_plane.angle
        if self.opposite_arm_choise_button.isChecked():
            self.desired_angle = self.opposite_joint.saggital_plane.angle
            self.controlled_joint.saggital_plane.desired_angle = self.desired_angle
        else:
            self.desired_angle = self.Angleslider.value()
            self.controlled_joint.saggital_plane.desired_angle = self.desired_angle
        self.angle_drawer[0:-1] = self.angle_drawer[1:]
        self.angle_drawer[-1:] = self.tracking_angle
        self.desired_angle_drawer[0:-1] = self.desired_angle_drawer[1:]
        self.desired_angle_drawer[-1:] = self.desired_angle
        self.left_pid_output[0:-1] = self.left_pid_output[1:]
        self.left_pid_output[-1:] = self.controlled_joint.controller.saggital_plane_controller.agonist_controller.output
        self.x = np.arange(0, 1000)
        self.angle_widget.setData(x=self.x, y=self.angle_drawer)
        self.desired_angle_widget.setData(
            x=self.x, y=self.desired_angle_drawer)
        self.left_pid.setData(x=self.x, y=self.left_pid_output)


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    thisapp = App()
    thisapp.showMaximized()
    sys.exit(app.exec_())
