from PyQt6 import QtCore, QtGui, QtWidgets
import sys
import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
import RsInstrument
from skuggsja_config import Config
from robodk_functions import RDK_KUKA
import robodk.robomath as robomath
import numpy as np
import threading
import functools
import datetime
import time

save_on_scan_end = False

def synchronized(f):
    @functools.wraps(f)
    def new_function(self, *args, **kw):
        lock = self.movement_lock
        if not lock.locked():
            lock.acquire()
            self.statusbar.setStyleSheet("background-color: red")
            try:
               return f(self, *args, **kw)
            finally:
                lock.release()
                self.statusbar.setStyleSheet("background-color: green")
        else:
            print("Another function is already controlling the robot")
    return new_function

def msgtoarr(s):
    return np.fromstring(s, sep=',')
class QLabelFramed(QtWidgets.QLabel):
    def __init__(self, parent=None):
        super(QLabelFramed, self).__init__(parent)
        self.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.setFrameShadow(QtWidgets.QFrame.Shadow.Plain)
        self.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)

class RobotStopButton(QtWidgets.QPushButton):

    group = QtWidgets.QButtonGroup()
    def __init__(self, parent=None, robot_IP = "255.255.0.0"):
        super(RobotStopButton, self).__init__(parent)
        self.setText("STOP")
        self.setMaximumWidth(100)
        self.setStyleSheet("font-weight: bold")
        # self.clicked.connect(lambda: self.robot_stop_button(robot_IP))
        RobotStopButton.group.addButton(self)

    def robot_stop_button(self, robot_IP):
        print(f"Stop command sent to robot arm at {robot_IP}")
        # print(self.group.buttons())

class ConnectionWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(ConnectionWidget, self).__init__(parent)
        self.con_params_gridLayout = QtWidgets.QGridLayout(self)

        self.robot_port_lineEdit = QtWidgets.QLineEdit()
        self.con_vna_pushButton = QtWidgets.QPushButton()
        self.ping_robot_pushButton = QtWidgets.QPushButton()
        self.robot_ip_label = QtWidgets.QLabel()
        self.con_robot_pushButton = QtWidgets.QPushButton()
        self.con_stop_pushButton = RobotStopButton()
        self.vna_port_lineEdit = QtWidgets.QLineEdit()
        self.vna_ip_label = QtWidgets.QLabel()
        self.vna_port_label = QtWidgets.QLabel()
        self.robot_ip_lineEdit = QtWidgets.QLineEdit()
        self.ping_vna_pushButton = QtWidgets.QPushButton()
        self.disconnect_pushButton = QtWidgets.QPushButton()
        self.vna_ip_lineEdit = QtWidgets.QLineEdit()
        self.robot_port_label = QtWidgets.QLabel()

        # self.vna_ip_lineEdit.setEchoMode(QtWidgets.QLineEdit.EchoMode.NoEcho)

        self.robot_ip_label.setText("Robot IP")
        self.robot_port_label.setText("Robot Port")
        self.vna_ip_label.setText("VNA IP")
        self.vna_port_label.setText("VNA Port")
        self.ping_robot_pushButton.setText("Ping Robot")
        self.ping_vna_pushButton.setText("Ping VNA")
        self.con_vna_pushButton.setText("Connect VNA")
        self.con_robot_pushButton.setText("Run RoboDK")
        self.disconnect_pushButton.setText("Disconnect both")

        self.con_params_gridLayout.addWidget(self.robot_ip_label, 0, 0, 1, 1)
        self.con_params_gridLayout.addWidget(self.robot_port_lineEdit, 1, 2, 1, 1)
        self.con_params_gridLayout.addWidget(self.vna_ip_label, 2, 0, 1, 1)
        self.con_params_gridLayout.addWidget(self.vna_port_label, 3, 0, 1, 1)
        self.con_params_gridLayout.addWidget(self.vna_ip_lineEdit, 2, 2, 1, 1)
        self.con_params_gridLayout.addWidget(self.vna_port_lineEdit, 3, 2, 1, 1)
        self.con_params_gridLayout.addWidget(self.robot_ip_lineEdit, 0, 2, 1, 1)
        self.con_params_gridLayout.addWidget(self.robot_port_label, 1, 0, 1, 1)

        self.con_params_gridLayout.addWidget(self.ping_robot_pushButton, 4, 0, 1, 1)
        self.con_params_gridLayout.addWidget(self.ping_vna_pushButton, 5, 0, 1, 1)
        self.con_params_gridLayout.addWidget(self.con_stop_pushButton, 6, 0, 1, 1)
        self.con_params_gridLayout.addWidget(self.con_robot_pushButton, 4, 1, 1, 1)
        self.con_params_gridLayout.addWidget(self.con_vna_pushButton, 5, 1, 1, 1)
        self.con_params_gridLayout.addWidget(self.disconnect_pushButton, 6, 1, 1, 1)

        self.con_params_gridLayout.addItem(QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Policy.Minimum,
                                                                 QtWidgets.QSizePolicy.Policy.Expanding), 7, 1, 1, 1)

class ManualControlWidget(QtWidgets.QWidget):
    group_minus = QtWidgets.QButtonGroup()
    group_plus = QtWidgets.QButtonGroup()

    def __init__(self, parent=None, coordinate = '_'):
        super(ManualControlWidget, self).__init__(parent)
        layout = QtWidgets.QGridLayout()
        self.coordinate = coordinate

        self.radioButton = QtWidgets.QRadioButton()
        self.step_label = QtWidgets.QLabel()
        self.step_lineEdit = QtWidgets.QLineEdit()
        self.step_lineEdit.setMaximumSize(QtCore.QSize(40, 30))
        self.horizontalSlider = QtWidgets.QSlider()
        self.horizontalSlider.setMaximum(100)
        self.horizontalSlider.setProperty("value", 50)
        self.horizontalSlider.setOrientation(QtCore.Qt.Orientation.Horizontal)
        self.horizontalSlider.setTickPosition(QtWidgets.QSlider.TickPosition.TicksBelow)
        
        self.minus_pushButton = QtWidgets.QPushButton()
        self.plus_pushButton = QtWidgets.QPushButton()
        ManualControlWidget.group_minus.addButton(self.minus_pushButton)
        ManualControlWidget.group_plus.addButton(self.plus_pushButton)

        self.man_label = QtWidgets.QLabel()
        self.man_lineEdit = QtWidgets.QLineEdit()
        self.man_lineEdit.setMaximumSize(QtCore.QSize(60, 30))
        self.pos_fdbk_label = QLabelFramed()
        self.dlabel = QtWidgets.QLabel()
        self.dfdbk_label = QLabelFramed()

        self.step_label.setText(f"s{coordinate}")
        self.man_label.setText(f"{coordinate}man")
        self.pos_fdbk_label.setText("TextLabel")
        self.dfdbk_label.setText("TextLabel")
        self.radioButton.setText(coordinate)
        self.dlabel.setText("d"+coordinate)
        self.minus_pushButton.setText("-")
        self.minus_pushButton.setStyleSheet("font-weight: bold;")
        self.plus_pushButton.setText("+")
        self.plus_pushButton.setStyleSheet("font-weight: bold")

        layout.addWidget(self.radioButton, 0, 0, 1, 1)
        layout.addWidget(self.step_label, 0, 1, 1, 1)
        layout.addWidget(self.step_lineEdit, 0, 2, 1, 1)
        layout.addWidget(self.horizontalSlider, 0, 3, 1, 1)
        layout.addWidget(self.man_label, 0, 4, 1, 1)
        layout.addWidget(self.man_lineEdit, 0, 5, 1, 1)
        layout.addWidget(self.pos_fdbk_label, 0, 6, 1, 1)
        layout.addWidget(self.dlabel, 0, 7, 1, 1)
        layout.addWidget(self.dfdbk_label, 0, 8, 1, 1)

        self.radioButton.toggled.connect(self.radio_button_check)
        self.man_lineEdit.returnPressed.connect(lambda: self.set_slider(self.man_lineEdit, self.horizontalSlider))
        self.horizontalSlider.valueChanged.connect(self.slider_changed)
        # self.man_lineEdit.textChanged.connect(lambda : print(self.man_lineEdit.text()))

        self.setLayout(layout)

        self.widget_touple = (self.radioButton,
        self.step_label,
        self.step_lineEdit,
        self.horizontalSlider,
        self.minus_pushButton,
        self.plus_pushButton,
        self.man_label,
        self.man_lineEdit,
        self.pos_fdbk_label,
        self.dlabel,
        self.dfdbk_label)

        for w in self.widget_touple[1:]:
            w.setEnabled(False)
        
    def getwidgets(self):
        return self.widget_touple

    def radio_button_check(self):
        for w in self.widget_touple[1:]:
            w.setEnabled(self.widget_touple[0].isChecked())

    def set_slider(self, textfield, slider):
        try:
            value = int(float(textfield.text()) * 10)

            if value < slider.minimum() or value > slider.maximum():
                print(f"Value out of bounds, snapped to {'minimum' if value < slider.minimum() else 'maximum'}")
            slider.setValue(value)
        except ValueError:
            print('Inputted value is not a floating point number')

    def slider_changed(self):
        self.new_val = self.horizontalSlider.value() / 10
        self.man_lineEdit.setText(str(self.new_val))
        
        
class RobotControlsWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(RobotControlsWidget, self).__init__(parent)
        self.VLayout = QtWidgets.QVBoxLayout(self)
        gridLayout = QtWidgets.QGridLayout()
        self.stop_pushButton = RobotStopButton()

        self.man_step_label = QtWidgets.QLabel()
        self.man_step_lineEdit = QtWidgets.QLineEdit()
        self.robot_joint_speed_label = QtWidgets.QLabel()
        self.robot_joint_speed_lineEdit = QtWidgets.QLineEdit()
        self.robot_joint_accel_label = QtWidgets.QLabel()
        self.robot_joint_accel_lineEdit = QtWidgets.QLineEdit()
        self.position_reset_pushButton = QtWidgets.QPushButton()
        self.position_reset_cross_pushButton = QtWidgets.QPushButton()
        self.set_scan_init_pushButton = QtWidgets.QPushButton()
        self.set_robot_speed_pushButton = QtWidgets.QPushButton()
        self.robot_joint_speed_lineEdit.setMaximumWidth(40)
        self.robot_joint_accel_lineEdit.setMaximumWidth(40)

        self.man_step_label.setText("Manual movement step")
        self.man_step_lineEdit.setText("5")
        self.robot_joint_speed_label.setText("Joints speed")
        self.robot_joint_accel_label.setText("Joints acceleration")
        self.robot_joint_speed_lineEdit.setText("5")
        self.robot_joint_accel_lineEdit.setText("5")
        self.position_reset_pushButton.setText("Reset robot position")
        self.position_reset_cross_pushButton.setText("Reset robot position (cross)")
        self.set_scan_init_pushButton.setText("Set new scan origin point")
        self.set_robot_speed_pushButton.setText("Set robot speed")

        gridLayout.addWidget(self.man_step_label,0,0,1,1)
        gridLayout.addWidget(self.man_step_lineEdit, 0, 1, 1, 1)
        gridLayout.addWidget(self.robot_joint_speed_label, 0, 2, 1, 1)
        gridLayout.addWidget(self.robot_joint_speed_lineEdit, 0, 3, 1, 1)
        gridLayout.addWidget(self.robot_joint_accel_label, 0, 4, 1, 1)
        gridLayout.addWidget(self.robot_joint_accel_lineEdit, 0, 5, 1, 1)
        gridLayout.addWidget(self.set_robot_speed_pushButton, 1, 3, 1, 2)
        gridLayout.addWidget(self.position_reset_pushButton, 1, 0, 1, 1)
        gridLayout.addWidget(self.position_reset_cross_pushButton, 2, 0, 1, 1)
        gridLayout.addWidget(self.set_scan_init_pushButton, 1, 1, 1, 1)
        self.VLayout.addLayout(gridLayout)
        self.VLayout.addWidget(self.stop_pushButton)

        size_policy = QtWidgets.QSizePolicy()
        self.setSizePolicy(size_policy)
        
        

class ScanParametersWidget(QtWidgets.QWidget):
    def __init__(self, parent=None, type = "XYZ scan"):
        super(ScanParametersWidget, self).__init__(parent)
        self.scan_params_VLayout = QtWidgets.QVBoxLayout(self)
        self.scan_params_label = QtWidgets.QLabel()
        self.scan_params_label.setMaximumHeight(30)
        self.scan_params_label.setStyleSheet("font-weight: bold")
        self.scan_params_label.setText("Scan Parameters")
        # self.scan_params_label.setSizePolicy(QtWidgets.QSizePolicy.)

        self.x0_label = QtWidgets.QLabel()
        self.y0_label = QtWidgets.QLabel()
        self.z0_label = QtWidgets.QLabel()
        self.x0_lineEdit = QtWidgets.QLineEdit()
        self.y0_lineEdit = QtWidgets.QLineEdit()
        self.z0_lineEdit = QtWidgets.QLineEdit()

        self.a0_label = QtWidgets.QLabel()
        self.b0_label = QtWidgets.QLabel()
        self.c0_label = QtWidgets.QLabel()
        self.a0_lineEdit = QtWidgets.QLineEdit()
        self.b0_lineEdit = QtWidgets.QLineEdit()
        self.c0_lineEdit = QtWidgets.QLineEdit()

        self.x1min_label = QtWidgets.QLabel()
        self.x1max_label = QtWidgets.QLabel()
        self.x1step_label = QtWidgets.QLabel()
        self.x1min_lineEdit = QtWidgets.QLineEdit()
        self.x1max_lineEdit = QtWidgets.QLineEdit()
        self.x1step_lineEdit = QtWidgets.QLineEdit()

        self.x2min_label = QtWidgets.QLabel()
        self.x2max_label = QtWidgets.QLabel()
        self.x2step_label = QtWidgets.QLabel()
        self.x2min_lineEdit = QtWidgets.QLineEdit()
        self.x2max_lineEdit = QtWidgets.QLineEdit()
        self.x2step_lineEdit = QtWidgets.QLineEdit()

        self.x3min_label = QtWidgets.QLabel()
        self.x3max_label = QtWidgets.QLabel()
        self.x3step_label = QtWidgets.QLabel()
        self.x3min_lineEdit = QtWidgets.QLineEdit()
        self.x3max_lineEdit = QtWidgets.QLineEdit()
        self.x3step_lineEdit = QtWidgets.QLineEdit()

        self.origin_params_w_list = [self.x0_label,self.x0_lineEdit,
                                     self.y0_label,self.y0_lineEdit,
                                     self.z0_label,self.z0_lineEdit]
        self.orientation_params_w_list = [self.a0_label, self.a0_lineEdit,
                                     self.b0_label, self.b0_lineEdit,
                                     self.c0_label, self.c0_lineEdit]
        self.x1_params_w_list = [self.x1min_label, self.x1min_lineEdit,
                                     self.x1max_label, self.x1max_lineEdit,
                                     self.x1step_label, self.x1step_lineEdit]
        self.x2_params_w_list = [self.x2min_label, self.x2min_lineEdit,
                                     self.x2max_label, self.x2max_lineEdit,
                                     self.x2step_label, self.x2step_lineEdit]
        self.x3_params_w_list = [self.x3min_label, self.x3min_lineEdit,
                                     self.x3max_label, self.x3max_lineEdit,
                                     self.x3step_label, self.x3step_lineEdit]

        self.params_rows = [self.origin_params_w_list,self.orientation_params_w_list,
                            self.x1_params_w_list,self.x2_params_w_list,self.x3_params_w_list]

        self.scan_params_gridLayout = QtWidgets.QGridLayout()
        for ir, r in enumerate(self.params_rows):
            for iw, w in enumerate(r):
                self.scan_params_gridLayout.addWidget(w, ir, iw, 1, 1)

        self.sttl_label = QtWidgets.QLabel()
        self.sttl_label.setText("Settling time, s")
        self.sttl_lineEdit = QtWidgets.QLineEdit()
        self.sttl_lineEdit.setText("0")
        self.sttl_label.setSizePolicy(QtWidgets.QSizePolicy.Policy.Fixed,QtWidgets.QSizePolicy.Policy.Fixed)
        self.sttl_lineEdit.setMaximumWidth(50)
        sttl_layout = QtWidgets.QHBoxLayout()
        sttl_layout.setAlignment(QtCore.Qt.AlignmentFlag.AlignLeft)
        sttl_layout.addWidget(self.sttl_label)
        sttl_layout.addWidget(self.sttl_lineEdit)

        self.scan_start_pushButton = QtWidgets.QPushButton()
        self.scan_start_pushButton.setText("Start scan")

        self.scan_params_VLayout.addWidget(self.scan_params_label)
        self.scan_params_VLayout.addLayout(self.scan_params_gridLayout)
        self.scan_params_VLayout.addLayout(sttl_layout)
        self.scan_params_VLayout.addWidget(self.scan_start_pushButton)

        size_policy = QtWidgets.QSizePolicy()
        self.setSizePolicy(size_policy)

        self.set_scan(type)

    def set_scan(self,scan_type):
        coordinates = ["X1", "X2", "X3"]
        if scan_type == "XYZ scan":
            for iw, w in enumerate(self.origin_params_w_list[::2]):
                w.setText(["X0","Y0","Z0"][iw])
                coordinates = ["X", "Y", "Z"]
        if scan_type == "Spherical scan":
            for iw, w in enumerate(self.origin_params_w_list[::2]):
                w.setText(["Xc","Yc","Zc"][iw])
                coordinates = ["R", "φ", "θ"]

        for iw, w in enumerate(self.params_rows[1][::2]):
            w.setText(["A0", "B0", "C0"][iw])
        
        for ir, row in enumerate(self.params_rows[2:]):
            for iw, w in enumerate(row[::2]):
                w.setText(["{}min", "{}max", "{}step"][iw].format(coordinates[ir]))

            for iw, w in enumerate(row[1::2]):
                side = 10
                steps = 1
                w.setText([f"{-side}", f"{side}", f"{steps}"][iw])

        if scan_type == "Spherical scan":
            self.params_rows[2][0].setText(coordinates[0])
            for w in self.params_rows[2][2:]:
                w.setVisible(False)
                w.sizePolicy().setRetainSizeWhenHidden(True)
        else:
            for w in self.params_rows[2][2:]:
                w.setVisible(True)

class VNAParametersWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(VNAParametersWidget, self).__init__(parent)

        self.VLayout = QtWidgets.QVBoxLayout(self)
        self.label = QtWidgets.QLabel()
        self.label.setStyleSheet("font-weight: bold")
        self.label.setMaximumHeight(30)
        self.label.setText("VNA Parameters")
        self.VLayout.addWidget(self.label)

        self.gridLayout = QtWidgets.QGridLayout()
        self.ifbw_label = QtWidgets.QLabel()
        self.ifbw_label.setText("Filter bandwidth, Hz")
        self.ifbw_lineEdit = QtWidgets.QLineEdit()
        self.fmin_label = QtWidgets.QLabel()
        self.fmin_label.setText("Min frequency, Hz")
        self.fmin_lineEdit = QtWidgets.QLineEdit()
        self.fmax_label = QtWidgets.QLabel()
        self.fmax_label.setText("Max frequency, Hz")
        self.fmax_lineEdit = QtWidgets.QLineEdit()
        self.fstp_label = QtWidgets.QLabel()
        self.fstp_label.setText("Frequency points")
        self.fstp_lineEdit = QtWidgets.QLineEdit()
        self.pwr_label = QtWidgets.QLabel()
        self.pwr_label.setText("Power, dBm")
        self.pwr_lineEdit = QtWidgets.QLineEdit()
        self.avg_num_label = QtWidgets.QLabel()
        self.avg_num_label.setText("Averaging samples")
        self.avg_num_lineEdit = QtWidgets.QLineEdit()
        self.ab_checkBox = QtWidgets.QCheckBox()
        self.ab_checkBox.setText("A/B")
        self.fwbw_checkBox = QtWidgets.QCheckBox()
        self.fwbw_checkBox.setText("Frwd/Bckwd")
        self.avg_checkBox = QtWidgets.QCheckBox()
        self.avg_checkBox.setText("Averaging")
        self.pushButton = QtWidgets.QPushButton()
        self.pushButton.setText("Send to VNA")

        self.gridLayout.addWidget(self.ifbw_label, 0, 0, 1, 1)
        self.gridLayout.addWidget(self.ifbw_lineEdit, 0, 1, 1, 1)
        self.gridLayout.addWidget(self.fmin_label, 0, 2, 1, 1)
        self.gridLayout.addWidget(self.fmin_lineEdit, 0, 3, 1, 1)
        self.gridLayout.addWidget(self.fmax_label, 0, 4, 1, 1)
        self.gridLayout.addWidget(self.fmax_lineEdit, 0, 5, 1, 1)
        self.gridLayout.addWidget(self.fstp_label, 0, 6, 1, 1)
        self.gridLayout.addWidget(self.fstp_lineEdit, 0, 7, 1, 1)
        self.gridLayout.addWidget(self.pwr_label, 1, 0, 1, 1)
        self.gridLayout.addWidget(self.pwr_lineEdit, 1, 1, 1, 1)
        self.gridLayout.addWidget(self.avg_num_label, 1, 2, 1, 1)
        self.gridLayout.addWidget(self.avg_num_lineEdit, 1, 3, 1, 1)
        self.gridLayout.addWidget(self.ab_checkBox, 1, 4, 1, 1)
        self.gridLayout.addWidget(self.fwbw_checkBox, 1, 5, 1, 1)
        self.gridLayout.addWidget(self.avg_checkBox, 1, 6, 1, 1)
        self.gridLayout.addWidget(self.pushButton, 1, 7, 1, 1)

        self.VLayout.addLayout(self.gridLayout)
        size_policy = QtWidgets.QSizePolicy()
        self.setSizePolicy(size_policy)

class RobotStartWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(RobotStartWidget, self).__init__(parent)

        self.VLayout = QtWidgets.QVBoxLayout(self)
        self.label = QtWidgets.QLabel()
        self.label.setStyleSheet("font-weight: bold")
        self.label.setMaximumHeight(30)
        self.label.setText("Start/Stop")
        self.VLayout.addWidget(self.label)

        self.run_pushButton = QtWidgets.QPushButton()
        self.run_pushButton.setText("Run")
        self.run_pushButton.setMaximumWidth(100)
        self.simulate_pushButton = QtWidgets.QPushButton()
        self.simulate_pushButton.setText("Simulate")
        self.simulate_pushButton.setMaximumWidth(100)
        self.stop_pushButton = RobotStopButton()

        self.VLayout.addWidget(self.run_pushButton)
        self.VLayout.addWidget(self.simulate_pushButton)
        self.VLayout.addWidget(self.stop_pushButton)

        size_policy = QtWidgets.QSizePolicy()
        self.setSizePolicy(size_policy)

class FeedbackWidget(QtWidgets.QWidget):
    def __init__(self, parent=None, type = "XYZ scan"):
        super(FeedbackWidget, self).__init__(parent)

        self.VLayout = QtWidgets.QVBoxLayout(self)
        self.label = QtWidgets.QLabel()
        self.label.setStyleSheet("font-weight: bold")
        self.label.setMaximumHeight(30)
        self.label.setText("Live position feedback")
        self.VLayout.addWidget(self.label)
        
        self.x_label = QtWidgets.QLabel()
        self.y_label = QtWidgets.QLabel()
        self.z_label = QtWidgets.QLabel()
        self.x_val = QtWidgets.QLabel()
        self.y_val = QtWidgets.QLabel()
        self.z_val = QtWidgets.QLabel()

        self.a_label = QtWidgets.QLabel()
        self.b_label = QtWidgets.QLabel()
        self.c_label = QtWidgets.QLabel()
        self.a_val = QtWidgets.QLabel()
        self.b_val = QtWidgets.QLabel()
        self.c_val = QtWidgets.QLabel()

        self.x1_label = QtWidgets.QLabel()
        self.x2_label = QtWidgets.QLabel()
        self.x3_label = QtWidgets.QLabel()
        self.x1_val = QtWidgets.QLabel()
        self.x2_val = QtWidgets.QLabel()
        self.x3_val = QtWidgets.QLabel()

        self.xyz_w_list = [self.x_label, self.x_val,
                                     self.y_label, self.y_val,
                                     self.z_label, self.z_val]
        self.abc_w_list = [self.a_label, self.a_val,
                                          self.b_label, self.b_val,
                                          self.c_label, self.c_val]
        self.x1x2x3_w_list = [self.x1_label, self.x1_val,
                           self.x2_label, self.x2_val,
                           self.x3_label, self.x3_val]

        self.params_rows = [self.xyz_w_list,self.abc_w_list,self.x1x2x3_w_list]
        self.gridLayout = QtWidgets.QGridLayout()
        for ir, r in enumerate(self.params_rows):
            for iw, w in enumerate(r):
                self.gridLayout.addWidget(w, ir, iw, 1, 1)


        for r in self.params_rows:
            for w in r[1::2]:
                w.setMaximumWidth(40)
                w.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
                w.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)

        self.VLayout.addLayout(self.gridLayout)
        size_policy = QtWidgets.QSizePolicy()
        self.setSizePolicy(size_policy)

        self.set_scan(type)

    def set_scan(self,scan_type):
        coordinates = [["X", "Y", "Z"],["A", "B", "C"]]
        if scan_type == "Spherical scan":
            coordinates.append(["R", "φ", "θ"])
        else:
            coordinates.append(["X1", "X2", "X3"])
        rows = len(self.params_rows)
        if scan_type == "XYZ scan":
            rows -= 1
            for w in self.params_rows[2]:
                w.setVisible(False)
        else:
            for w in self.params_rows[2]:
                w.setVisible(True)

        for ir, r in enumerate(self.params_rows[:rows]):
            for iw, w in enumerate(r[::2]):
                w.setText(coordinates[ir][iw])

class LogWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(LogWidget, self).__init__(parent)

        self.VLayout = QtWidgets.QVBoxLayout(self)
        self.label = QtWidgets.QLabel()
        self.label.setStyleSheet("font-weight: bold")
        self.label.setMaximumHeight(30)
        self.label.setText("Log")
        self.VLayout.addWidget(self.label)
        self.textBrowser = QtWidgets.QTextBrowser()
        # self.textBrowser.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
        self.VLayout.addWidget(self.textBrowser)

class PlotWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(PlotWidget, self).__init__(parent)

        self.VLayout = QtWidgets.QVBoxLayout(self)
        self.figure = Figure(figsize=(5, 4), dpi=100)
        self.canvas = FigureCanvas(self.figure)
        self.toolbar = NavigationToolbar(self.canvas, self)
        self.VLayout.addWidget(self.toolbar)
        self.VLayout.addWidget(self.canvas)
        self.plot_formats = {"Magnitude": abs, "Phase": np.angle, "Mag dB": lambda x: 20 * np.log10(np.abs(x)),
                             "Real": np.real, "Imaginary": np.imag}
        self.plot_format_units = {"Magnitude": "mW", "Phase": "degrees", "Mag dB": "dBm",
                                  "Real": "mW", "Imaginary": "mW"}

class VNAPlotWidget(PlotWidget):
    def __init__(self, parent=None):
        super(VNAPlotWidget, self).__init__(parent)

        self.gridLayout = QtWidgets.QGridLayout()
        self.button_update_continuous = QtWidgets.QPushButton()
        self.button_step = QtWidgets.QPushButton()
        self.traces_to_show = QtWidgets.QComboBox()

        self.button_update_continuous.setText("Plot")
        self.button_step.setText("Step")

        self.gridLayout.addWidget(self.button_update_continuous, 3, 0, 1, 1)
        self.gridLayout.addWidget(self.button_step, 3, 1, 1, 1)
        self.gridLayout.addWidget(self.traces_to_show, 1, 0, 1, 2)
        self.gridLayout.addWidget(self.canvas, 0, 0, 1, 2)

        self.VLayout.addLayout(self.gridLayout)

class ScanPlotWidget(PlotWidget):
    def __init__(self, parent=None):
        super(ScanPlotWidget, self).__init__(parent)
        self.gridLayout = QtWidgets.QGridLayout()

        self.gridLayout = QtWidgets.QGridLayout()
        self.plot_format_combobox = QtWidgets.QComboBox()
        self.slice_direction_combobox = QtWidgets.QComboBox()
        self.checkbox_scatter = QtWidgets.QRadioButton()
        self.coordinate_label = QtWidgets.QLabel()
        self.coordinate_textfield = QtWidgets.QLineEdit()
        self.coordinate_slider = QtWidgets.QSlider()
        self.coordinate_slider.setOrientation(QtCore.Qt.Orientation.Horizontal)
        self.frequency_label = QtWidgets.QLabel()
        self.frequency_textfield = QtWidgets.QLineEdit()
        self.frequency_slider = QtWidgets.QSlider()
        self.frequency_slider.setOrientation(QtCore.Qt.Orientation.Horizontal)

        self.checkbox_scatter.setText("Plot Format: line/scatter")
        self.coordinate_label.setText("Change coordinate to (mm):")
        self.frequency_label.setText("Change frequency to (GHz):")
        self.plot_format_combobox.addItems(self.plot_formats.keys())
        self.slice_directions = {"XY": 2, "XZ": 1, "YZ": 0}
        self.slice_direction_combobox.addItems(self.slice_directions.keys())

        self.gridLayout.addWidget(self.plot_format_combobox, 1, 0, 1, 2)
        self.gridLayout.addWidget(self.checkbox_scatter, 1, 2, 1, 1)
        self.gridLayout.addWidget(self.slice_direction_combobox, 2, 0, 1, 2)
        self.gridLayout.addWidget(self.coordinate_label, 4, 0, 1, 1)
        self.gridLayout.addWidget(self.coordinate_textfield, 4, 1, 1, 1)
        self.gridLayout.addWidget(self.coordinate_slider, 4, 2, 1, 1)
        self.gridLayout.addWidget(self.frequency_label, 3, 0, 1, 1)
        self.gridLayout.addWidget(self.frequency_textfield, 3, 1, 1, 1)
        self.gridLayout.addWidget(self.frequency_slider, 3, 2, 1, 1)

        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.button_save_current_points = QtWidgets.QPushButton()
        self.button_save_all_points_at_frequency = QtWidgets.QPushButton()
        self.button_save_all_points = QtWidgets.QPushButton()

        self.button_save_current_points.setText("Save current slice as .txt")
        self.button_save_all_points_at_frequency.setText("Save all points at current frequency as .txt")
        self.button_save_all_points.setText("Save all points as .txt")

        self.horizontalLayout.addWidget(self.button_save_current_points)
        self.horizontalLayout.addWidget(self.button_save_all_points_at_frequency)
        self.horizontalLayout.addWidget(self.button_save_all_points)

        self.VLayout.addLayout(self.gridLayout)
        self.VLayout.addLayout(self.horizontalLayout)

        self.ax_scan = self.figure.add_subplot(111)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, *args, obj=None, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.configs = Config("aaa.toml")
        self.setupUi(self)
        self.vna_connected = False
        self.vna_parameters_widget.setEnabled(False)
        self.vna_plot_w.setEnabled(False)

        self.robot_connected = False
        for w in (self.manualCTab,self.scan_parameters_widget, self.scan_plot_w):
            w.setEnabled(False)

        self.connection_widget.vna_ip_lineEdit.setText(self.configs.VNA_settings["ip"])
        self.connection_widget.con_vna_pushButton.clicked.connect(self.vna_connect_button_clicked)
        self.connection_widget.con_robot_pushButton.clicked.connect(self.robot_connect_button_clicked)

        RobotStopButton.group.buttonClicked.connect(self.stop_button_clicked)

        self.timer = QtCore.QTimer()
        self.timer.setInterval(10)
        self.timer.stop()
        self.timer.timeout.connect(self.plot_update)
        self.toggle_var = False
        self.vna_parameters_widget.pushButton.clicked.connect(self.send_to_vna_button_clicked)
        self.vna_plot_w.button_update_continuous.clicked.connect(self.toggle)
        self.vna_plot_w.button_step.clicked.connect(self.step_plot)

        self.move_timer = QtCore.QTimer()
        self.move_timer.setInterval(1)
        self.move_timer.timeout.connect(self.move_timer_func)
        self.manual_direction = "+Z"
        ManualControlWidget.group_minus.buttonPressed.connect(self.minus_button_pressed)
        ManualControlWidget.group_plus.buttonPressed.connect(self.plus_button_pressed)
        ManualControlWidget.group_minus.buttonReleased.connect(self.manual_button_released)
        ManualControlWidget.group_plus.buttonReleased.connect(self.manual_button_released)

        self.coord_update_timer = QtCore.QTimer()
        self.coord_update_timer.setInterval(1)
        self.coord_update_timer.timeout.connect(self.coord_update_func)

        self.robot_controls_widget.position_reset_pushButton.clicked.connect(self.position_reset_button_clicked)
        self.robot_controls_widget.position_reset_cross_pushButton.clicked.connect(self.position_reset_cross_button_clicked)
        self.robot_controls_widget.set_scan_init_pushButton.clicked.connect(self.set_scan_init_button_clicked)
        self.robot_controls_widget.set_robot_speed_pushButton.clicked.connect(self.set_robot_speed)

        self.scan_parameters_widget.scan_start_pushButton.clicked.connect(self.begin_scan)

        self.run_on_robot = False
        self.robot_start_widget.simulate_pushButton.clicked.connect(self.run_program_in_sim)
        self.robot_start_widget.run_pushButton.clicked.connect(self.run_program_on_robot)

        self.movement_lock = threading.Lock()

        self.scan_plot_w.frequency_slider.valueChanged.connect(self.frequency_slider_changed)
        self.scan_plot_w.frequency_slider.sliderPressed.connect(lambda: self.freq_select_vline.set_visible(True))
        self.scan_plot_w.frequency_slider.sliderReleased.connect(lambda: self.freq_select_vline.set_visible(False))
        self.scan_plot_w.slice_direction_combobox.currentTextChanged.connect(self.update_coord_slider)
        self.scan_plot_w.coordinate_slider.valueChanged.connect(self.coordinate_slider_changed)
    def minus_button_pressed(self):
        a = [x.radioButton.text() for x in self.coordinates_widgets_list]
        b = [x.radioButton.isChecked() for x in self.coordinates_widgets_list].index(True)
        self.manual_direction = ("-" + a[b])
        self.move_timer.start()

    def plus_button_pressed(self):
        a = [x.radioButton.text() for x in self.coordinates_widgets_list]
        b = [x.radioButton.isChecked() for x in self.coordinates_widgets_list].index(True)
        self.manual_direction = ("+" + a[b])
        self.move_timer.start()

    def manual_button_released(self):
        self.move_timer.stop()

    def movement_function_decorator(func):
        @functools.wraps(func)
        def _wrapper(*args, **kwargs):

            print("moved")
            lock = args[0].movement_lock
            if lock.locked():
                return None
            else:
                return func(*args, **kwargs)

        return _wrapper

    @synchronized
    def move_timer_func(self):
        # with self.movement_lock:
            # print(self.manual_direction)
            try:
                dict = {"+X":(1,0,0),"+Y":(0,1,0),"+Z":(0,0,1),
                        "-X":(-1,0,0),"-Y":(0,-1,0),"-Z":(0,0,-1),
                        "+A": (0, 0, 0, 1, 0, 0), "+B": (0, 0, 0, 0, 1, 0), "+C": (0, 0, 0, 0, 0, 1),
                        "-A": (0, 0, 0, -1, 0, 0), "-B": (0, 0, 0, 0, -1, 0), "-C": (0, 0, 0, 0, 0, -1)}
                speed = float(self.robot_controls_widget.man_step_lineEdit.text())
                self.current_robot_pose = self.robot_rdk.move_relative([x*speed for x in dict[self.manual_direction]])
                self.update_current_robot_pose()
            except ValueError:
                print('Inputted value is invalid')

    def coord_update_func(self):
        current_coords = robomath.Pose_2_KUKA(self.robot_rdk.robot.Pose())
        i = 0
        for r in self.feedback_widget.params_rows[:2]:
            for w in r[1::2]:
                w.setText(f"{current_coords[i]:.2f}")
                i+=1

    def update_current_robot_pose(self):
        for i,w in enumerate(self.coordinates_widgets_list):
            w.pos_fdbk_label.setText(f"{self.current_robot_pose[i]:.0f}")
        for i, x in enumerate(self.current_robot_pose):
            self.feedback_widget.params_rows[i//3][(i%3)*2+1].setText(f"{x:.0f}")


    def position_reset_button_clicked(self):
        self.send_robot_to_initial("H")

    def position_reset_cross_button_clicked(self):
        self.send_robot_to_initial("V")

    @synchronized
    def send_robot_to_initial(self, pol ="H"):
        if pol == "H":
            self.robot_rdk.robot.MoveJ(self.robot_rdk.target_init.Joints())
            self.robot_rdk.target_rel.setPose(self.robot_rdk.target_init.Pose())
        elif pol == "V":
            self.robot_rdk.robot.MoveJ(self.robot_rdk.target_init_cross.Joints())
            self.robot_rdk.target_rel.setPose(self.robot_rdk.target_init_cross.Pose())
        else:
            pass

    def set_scan_init_button_clicked(self):
        self.scan_init = self.robot_rdk.set_scan_initial()
        w_list = [self.scan_parameters_widget.origin_params_w_list,
                  self.scan_parameters_widget.orientation_params_w_list]
        for i, x in enumerate(self.scan_init):
            w_list[i//3][(i%3)*2+1].setText(f"{x:.3f}")

    def begin_scan(self):
        x = threading.Thread(target=self.scan_xyz)
        x = QtCore.QThread
        x.start()
        # self.scan_xyz()

    @synchronized
    def scan_xyz(self):
        can_plot = self.vna_connected
        try:
            points_axes = []
            shape = []
            for param in self.scan_parameters_widget.params_rows[2:]:
                min = float(param[1].text())
                max = float(param[3].text())
                steps = int(param[5].text())
                shape.append(steps)
                points_axes.append(np.linspace(min,max,steps))
            dir_x = 1
            dir_y = 1
            dir_z = 1
            data = np.empty(shape)
            X_grid, Y_grid = np.meshgrid(points_axes[0],points_axes[1])
            data[:] = np.nan
            for iz, z in enumerate(points_axes[2]):
                for iy, y in enumerate(points_axes[1][::dir_y]):
                    for ix, x in enumerate(points_axes[0][::dir_x]):
                        # print(x,y,z)
                        # self.robot_rdk.robot.setPoseFrame(self.robot_rdk.frame_scan_init)
                        # self.robot_rdk.move_target(self.robot_rdk.target_scan, (x,y,z))
                        # self.robot_rdk.target_scan.setPose(robomath.Offset(self.robot_rdk.frame_scan_init,x,y,z))
                        new_pose = self.robot_rdk.target_scan_init.Pose() * (robomath.eye().Offset(x, y, z))
                        self.robot_rdk.target_scan.setPose(new_pose)
                        self.robot_rdk.robot.MoveJ(self.robot_rdk.target_scan.Pose())

                    # self.robot_rdk.target_scan.setPose(robomath.Offset(self.robot_rdk.frame_scan_init.PoseWrt(self.robot_rdk.robot.Parent()),x,y,z))
                    # self.robot_rdk.robot.setPoseFrame(self.robot_rdk.robot.Parent())

                        if can_plot:
                            time.sleep(float(self.scan_parameters_widget.sttl_lineEdit.text()))
                            #time.sleep(0)
                            #data_point = self.query_data(1)[0]
                            data_point = self.query_data(1)[601//2]

                            data[ix*dir_x-(1 if dir_x<0 else 0),
                                 iy*dir_y-(1 if dir_y<0 else 0),
                                 iz] = data_point

                            # print(data)
                            self.scan_plot_w.ax_scan.clear()
                            self.scan_plot_w.ax_scan.pcolor(Y_grid,X_grid,data[:, :, 0].T,shading='nearest')
                            self.scan_plot_w.canvas.draw()

                    dir_x *= -1
                dir_y *= -1
            self.robot_rdk.robot.MoveJ(self.robot_rdk.target_scan_init.Pose())
            print(data)
            current_time = datetime.datetime.now().strftime("%Y_%m_%d_%H-%M-%S")


            if save_on_scan_end == True:
                np.savetxt(f"{current_time}.txt", np.squeeze(self.data))
                np.save(f"{current_time}.npy", self.data)
                np.savetxt(f"{current_time}_coords_x.txt", np.array(scan_points[0]))
                np.savetxt(f"{current_time}_coords_y.txt", np.array(scan_points[1]))
                np.savetxt(f"{current_time}_coords_z.txt", np.array(scan_points[2]))
            # self.scan_plot_w.ax_scan.clear()
            # self.scan_plot_w.ax_scan.pcolor(data[:, :, 0])
            # self.scan_plot_w.canvas.draw()
        except ValueError:
            print('Inputted value is invalid')

    def run_program_on_robot(self):
        self.run_on_robot = True
#        self.robot_rdk.RunMode()
        self.robot_rdk.run_on_robot(self.configs.robot_settings['ip'],
                                                 self.configs.robot_settings['port'])
        self.set_robot_speed()


    def run_program_in_sim(self):
        self.run_on_robot = False
        self.robot_rdk.setRunMode(1)  #  RUNMODE_SIMULATE = 1

    def set_robot_speed(self):
        try:
            j_speed = float(self.robot_controls_widget.robot_joint_speed_lineEdit.text())
            j_accel = float(self.robot_controls_widget.robot_joint_accel_lineEdit.text())
            self.robot_rdk.robot.setSpeed(speed_linear=-1,speed_joints=j_speed,accel_joints=j_accel)
        except ValueError:
            print('Inputted value is invalid')


    def append_log(self, text):
        self.connection_tab_log_widget.textBrowser.append(text)
        self.log.textBrowser.append(text)

    def robot_connect_button_clicked(self):
        if not self.robot_connected:
            try:
                self.robot_rdk = RDK_KUKA(quit_on_close = True, joints=self.configs.robot_settings["joints2"])
                self.append_log("Succesfully connected to RoboDK")
                self.robot_connected = True
                self.connection_widget.con_robot_pushButton.setText("Disconnect robot")
                self.connection_widget.con_robot_pushButton.setStyleSheet("background-color: red")
                self.coord_update_timer.start()
                self.statusbar.setStyleSheet("background-color: green")
                self.set_robot_speed()
            finally:
                pass
            # except RsInstrument.RsInstrException:
            #     print("no")
            #     self.append_log("Connection Failed")
            #     self.vna_connected = False
        elif self.robot_connected:
            self.robot_rdk.Disconnect()
            self.append_log("RoboDK session closed")
            self.robot_connected = False
            self.connection_widget.con_robot_pushButton.setText("Connect robot")
            self.connection_widget.con_robot_pushButton.setStyleSheet("background-color: light gray")
            self.coord_update_timer.stop()
            self.statusbar.setStyleSheet("background-color: light gray")
        for w in (self.manualCTab,self.scan_parameters_widget):
            w.setEnabled(self.robot_connected)



    def vna_connect_button_clicked(self):
        if not self.vna_connected:
            try:
                self.instr = RsInstrument.RsInstrument(f"TCPIP::{self.connection_widget.vna_ip_lineEdit.text()}::hislip0",True,False)
                self.append_log("Succesfully connected to "+self.instr.query_str("*IDN?"))
                print(self.instr.query_str("*IDN?"))
                self.vna_connected = True
                self.connection_widget.con_vna_pushButton.setText("Disconnect VNA")
                self.connection_widget.con_vna_pushButton.setStyleSheet("background-color: red")
                self.update_VNA_settings()
                self.plot_initialize()
            except RsInstrument.RsInstrException:
                print("no")
                self.append_log("Connection Failed")
                self.vna_connected = False
        elif self.vna_connected:
            self.instr.close()
            self.append_log("VNA session closed")
            self.vna_connected = False
            self.connection_widget.con_vna_pushButton.setText("Connect VNA")
            self.connection_widget.con_vna_pushButton.setStyleSheet("background-color: light gray")
        self.vna_parameters_widget.setEnabled(self.vna_connected)
        self.vna_plot_w.setEnabled(self.vna_connected)
        self.scan_plot_w.setEnabled(self.vna_connected)
        if self.toggle_var:
            self.toggle()

    def stop_button_clicked(self):
        self.append_log(f"Stop command sent to the VNA at {self.connection_widget.vna_ip_lineEdit.text()}")
        if self.vna_connected:
            self.vna_connect_button_clicked()
        if self.robot_connected:
            self.robot_connect_button_clicked()

    def send_to_vna_button_clicked(self):
        self.set_VNA_settings()


    def query_data(self, ch, raw=False):
        """Returns the array of datapoints from the VNA.
        Formatted real values if raw is True, unformatted complex if false"""
        data = msgtoarr(self.instr.query(f'CALC1:DATA:TRAC? "Trc{ch}", {["F", "S"][raw]}DAT'))
        if raw:
            data = data[::2] + 1j * data[1::2]
        return data
    def plot_initialize(self):

        self.vna_plot_w.figure.clear()

        self.ax = self.vna_plot_w.figure.add_subplot(111)
        self.ax.set_xlabel('Frequency, GHz')
        self.ax.set_ylabel('S-parameters, dB')
        list_of_traces = self.instr.query_str_stripped("CONFigure:TRACe:CATalog?").split(',')
        self.configs.traces_to_show = self.instr.query_str_stripped("CONFigure:TRACe:CATalog?").split(',')[::2]
        self.traces = []
        for tr_n in self.configs.traces_to_show:
            self.traces.append(self.ax.plot(self.freq_arr, self.query_data(tr_n), label=f"trace {tr_n}")[0])
        self.ax.legend()
        self.vna_plot_w.figure.tight_layout()

        self.vna_plot_w.traces_to_show.clear()       # delete all items from comboBox
        self.vna_plot_w.traces_to_show.addItems(list_of_traces[1::2])

        self.freq_select_vline = self.ax.axvline(self.freq_arr[self.scan_plot_w.frequency_slider.value()],
                                                 visible = 1, color = "k")

        # refresh canvas
        self.vna_plot_w.canvas.draw()

    def plot_update(self):
        """Update the data trace shown on the plot with the one currently on the VNA screen"""
        for i, trace in enumerate(self.traces):
            trace.set_ydata(self.query_data(self.configs.traces_to_show[i]))
        self.vna_plot_w.canvas.draw()

    def set_VNA_settings(self):
        try:
            new_bandwidth = self.vna_parameters_widget.ifbw_lineEdit.text()
            new_fmin = self.vna_parameters_widget.fmin_lineEdit.text()
            new_fmax = self.vna_parameters_widget.fmax_lineEdit.text()
            new_points = self.vna_parameters_widget.fstp_lineEdit.text()
            new_power= self.vna_parameters_widget.pwr_lineEdit.text()
            sweep_reverse = self.vna_parameters_widget.fwbw_checkBox.isChecked()
            new_averaged_samples = self.vna_parameters_widget.avg_num_lineEdit.text()
            averaging = self.vna_parameters_widget.avg_checkBox.isChecked()
            # if len(self.point_coordinates):
            #     dlg = FrequencyResolutionDialog()
            #     if dlg.exec():
            #         self.frequency_reset()
            #         self.scan_plot_update()
            #         pass
            #     else:
            #         return
            if new_bandwidth != '':
                self.instr.write(f'SENSe:BAND {float(new_bandwidth)}')
            if new_fmin != '':
                if float(new_fmin) >= self.configs.VNA_settings["min_fmin"]:
                    self.instr.write(f'FREQuency:STARt {float(new_fmin)}')
                else:
                    self.append_log("Min frequency lower than the edge of the band")
            if new_fmax != '':
                if float(new_fmax) <= self.configs.VNA_settings["max_fmax"]:
                    self.instr.write(f'FREQuency:STOP {float(new_fmax)}')
                else:
                    self.append_log("Max frequency higher than the edge of the band")
            if new_points != '':
                self.instr.write(f'SWEep:POINts {int(new_points)}')
            if new_power != '':
                if int(new_power) <= 8:
                    self.instr.write(f'SOUR:POW {int(new_power)}')
                else:
                    self.append_log("Requested power is too high")
            self.instr.write(f'SWEep:REVerse {["OFF","ON"][sweep_reverse]}')
            self.instr.write(f'AVERage {["OFF", "ON"][averaging]}')
            self.instr.write(f'AVERage:CLEar')
            if new_averaged_samples != '':
                self.instr.write(f'AVERage:COUNt {int(new_averaged_samples)}')
            self.update_VNA_settings()
            self.freq_arr = msgtoarr((self.instr.query('CALC:DATA:STIM?'))) / 1e9
            self.plot_initialize()
            self.plot_initialize()
        except ValueError:
            print('Inputted value is invalid')

    def update_VNA_settings(self):
        """Queries the VNA for the current settings and updates the text in the widget.
        Also saves the current settings to the latest_settings.toml file."""
        self.configs.VNA_settings["bandwidth"] = self.instr.query_float('SENSe:BAND?')
        self.configs.VNA_settings["point_number"] = self.instr.query_int('SWEep:POINts?')
        self.configs.VNA_settings["time_per_sweep"] = self.instr.query_float('SWEep:TIMe?')
        self.configs.VNA_settings["averaged_samples"] = self.instr.query_int('AVERage:COUNt?')
        self.configs.VNA_settings["minimum_frequency"] = self.instr.query_int('FREQuency:STARt?')
        self.configs.VNA_settings["maximum_frequency"] = self.instr.query_int('FREQuency:STOP?')
        self.configs.VNA_settings["power"] = self.instr.query_int('SOURce:POWer?')
        self.configs.VNA_settings["reversed_sweep"] = self.instr.query_bool('SWEep:REVerse?')
        self.configs.VNA_settings['averaged_samples'] = self.instr.query_int("AVERage:COUNt?")
        self.configs.VNA_settings["averaging_on/off"] = self.instr.query_bool('AVERage?')
        # self.VNA_settings.setText(
        #     f"Bandwidth: {str(self.configs.VNA_settings['bandwidth']) + ' Hz' if float(self.configs.VNA_settings['bandwidth']) < 1000 else '{:.0f} kHz'.format(self.configs.VNA_settings['bandwidth'] / 1e3)}  "
        #     f"\nNumber of points: {self.configs.VNA_settings['point_number']} "
        #     f"\nAveraged samples: {self.configs.VNA_settings['averaged_samples']} "
        #     f"\nSeconds per sweep: {self.configs.VNA_settings['time_per_sweep']}")
        self.vna_parameters_widget.ifbw_lineEdit.setText(f'{self.configs.VNA_settings["bandwidth"]:.4g}')
        self.vna_parameters_widget.fmin_lineEdit.setText(f'{self.configs.VNA_settings["minimum_frequency"]:.4g}')
        self.vna_parameters_widget.fmax_lineEdit.setText(f'{self.configs.VNA_settings["maximum_frequency"]:.4g}')
        self.vna_parameters_widget.fstp_lineEdit.setText(f'{self.configs.VNA_settings["point_number"]:.4g}')
        self.vna_parameters_widget.pwr_lineEdit.setText(str(self.configs.VNA_settings["power"]))
        self.vna_parameters_widget.fwbw_checkBox.setChecked(self.configs.VNA_settings["reversed_sweep"])
        self.vna_parameters_widget.avg_checkBox.setChecked(self.configs.VNA_settings["averaging_on/off"])
        self.vna_parameters_widget.avg_num_lineEdit.setText(str(self.configs.VNA_settings["averaged_samples"]))
        self.configs.save_toml("latest_settings.toml")

        self.freq_arr = msgtoarr((self.instr.query('CALC:DATA:STIM?'))) / 1e9
        self.scan_plot_w.frequency_slider.setRange(0,self.configs.VNA_settings["point_number"]-1)

    def button_text_update(self):
        if self.toggle_var:
            self.vna_plot_w.button_update_continuous.setText("Plot (push to stop)")
            self.vna_plot_w.button_step.setText("Step (push to stop and update)")
        else:
            self.vna_plot_w.button_update_continuous.setText("Plot")
            self.vna_plot_w.button_step.setText("Step")

    def step_plot(self):
        self.plot_update()
        if self.toggle_var:
            self.timer.stop()
            self.toggle_var = not self.toggle_var
        self.button_text_update()

    def toggle(self):
        """Toggles the plot continuously refreshing"""
        if self.toggle_var:
            self.timer.stop()
        else:
            self.timer.start()
        self.toggle_var = not self.toggle_var
        self.button_text_update()

    def scan_plot_update(self):
        X_grid, Y_grid = np.meshgrid(self.scan_coords[0], self.scan_coords[1])
        self.scan_plot_w.ax_scan.clear()
        self.scan_plot_w.ax_scan.pcolor(Y_grid, X_grid, self.data[:, :, self.scan_plot_w.coordinate_slider.value()].T, shading='nearest')
        self.scan_plot_w.canvas.draw()

    def set_slider(self, value, slider):
        try:
            if value < slider.minimum() or value > slider.maximum():
                print(f"Value out of bounds, snapped to {'minimum' if value < slider.minimum() else 'maximum'}")
            slider.setValue(value)
        except ValueError:
            print('Inputted value is not a floating point number')

    def frequency_slider_changed(self):
        # print(self.textfield_frequency_slider.value())
        self.new_frequency = self.freq_arr[self.scan_plot_w.frequency_slider.value()]
        self.scan_plot_w.frequency_textfield.setText(f"{self.new_frequency:.2f}")
        self.freq_select_vline.set_xdata([self.new_frequency, self.new_frequency])
        # self.new_frequency_index = self.find_nearest_frequency_point(self.new_frequency)
        self.vna_plot_w.canvas.draw()
        # self.scan_plot_update()

    def find_nearest_frequency_point(self, value):
        return np.argmin(np.abs(self.freq_arr - value))

    def coordinate_slider_changed(self):
        # print(self.textfield_frequency_slider.value())
        coord_arr = self.scan_coords[self.scan_plot_w.slice_directions[self.scan_plot_w.slice_direction_combobox.currentText()]]
        self.new_coordinate = coord_arr[self.scan_plot_w.coordinate_slider.value()]
        self.scan_plot_w.coordinate_textfield.setText(str(self.new_coordinate))
        # self.coord_select_vline.set_xdata([self.new_coordinate, self.new_coordinate])
        self.scan_plot_update()

    def update_coord_slider(self):
        self.scan_plot_w.coordinate_slider.setRange(0,
            len(self.scan_coords[self.scan_plot_w.slice_directions[self.scan_plot_w.slice_direction_combobox.currentText()]])-1)
        self.scan_plot_update()

    def initialize_connectionTab(self):
        connectionTab = QtWidgets.QWidget()

        self.gridLayout_3 = QtWidgets.QGridLayout(connectionTab)
        self.connection_main_HLayout = QtWidgets.QHBoxLayout(connectionTab)
        self.gridLayout_3.addLayout(self.connection_main_HLayout, 0, 0, 1, 1)

        self.connection_widget = ConnectionWidget()

        self.connection_tab_log_widget = LogWidget()
        # self.connection_tab_log_widget.textBrowser.setText("test")

        self.connection_main_HLayout.addWidget(self.connection_widget)
        self.connection_main_HLayout.addWidget(self.connection_tab_log_widget)

        return connectionTab

    def initialize_manualCTab(self):
        manualCTab = QtWidgets.QWidget()
        self.gridLayout_4 = QtWidgets.QGridLayout(manualCTab)
        self.man_main_VLayout = QtWidgets.QVBoxLayout()
        self.man_move_gridLayout = QtWidgets.QGridLayout()

        coordinates_list = ["X", "Y", "Z", "A", "B", "C"]
        self.coordinates_widgets_list = []
        for ic, coord in enumerate(coordinates_list):
            setattr(self, f"manual_control_{coord}", ManualControlWidget(parent = manualCTab, coordinate= coord))
            self.coordinates_widgets_list.append(eval(f"self.manual_control_{coord}"))

        for ic, coord in enumerate(coordinates_list):
            for iw, widget in enumerate(self.coordinates_widgets_list[ic].getwidgets()):
                self.man_move_gridLayout.addWidget(widget, 1+ic,iw,1,1)

        self.man_select_label = QtWidgets.QLabel()
        self.man_select_label.setStyleSheet("font-weight: bold")
        self.man_move_gridLayout.addWidget(self.man_select_label, 0, 0, 1, 1)
        self.man_select_label.setMaximumHeight(30)
        self.man_step_label = QtWidgets.QLabel()
        self.man_step_label.setStyleSheet("font-weight: bold")
        self.man_move_gridLayout.addWidget(self.man_step_label, 0, 2, 1, 1)
        self.man_pos_label = QtWidgets.QLabel()
        self.man_pos_label.setStyleSheet("font-weight: bold")
        self.man_move_gridLayout.addWidget(self.man_pos_label, 0, 3, 1, 1)
        self.man_input_label = QtWidgets.QLabel()
        self.man_input_label.setStyleSheet("font-weight: bold")
        self.man_move_gridLayout.addWidget(self.man_input_label, 0, 7, 1, 1)
        self.man_pos_fdbk_label = QtWidgets.QLabel()
        self.man_pos_fdbk_label.setStyleSheet("font-weight: bold")
        self.man_move_gridLayout.addWidget(self.man_pos_fdbk_label, 0, 8, 1, 1)
        self.man_pos_real_fdbk_label = QtWidgets.QLabel()
        self.man_pos_real_fdbk_label.setStyleSheet("font-weight: bold")
        self.man_move_gridLayout.addWidget(self.man_pos_real_fdbk_label, 0, 10, 1, 1)

        self.man_step_label.setText("Step size")
        self.man_pos_label.setText("Position Selection")
        self.man_select_label.setText("Movement Selection")
        self.man_pos_fdbk_label.setText("Position feedback")
        self.man_pos_real_fdbk_label.setText("Position accuracy")
        self.man_input_label.setText("Manual In")


        self.man_main_VLayout.addLayout(self.man_move_gridLayout)

        self.robot_controls_widget = RobotControlsWidget()
        self.man_main_VLayout.addWidget(self.robot_controls_widget)

        self.gridLayout_4.addLayout(self.man_main_VLayout, 0, 0, 1, 1)
        spacerItem1 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Expanding)
        self.gridLayout_4.addItem(spacerItem1, 1, 0, 1, 1)

        return manualCTab

    def initialize_scanTab(self):
        scan_tab = QtWidgets.QWidget()
        self.gridLayout_6 = QtWidgets.QGridLayout(scan_tab)
        self.xyzS_VLayout = QtWidgets.QVBoxLayout()
        self.scan_parameters_widget = ScanParametersWidget()
        self.xyzS_VLayout.addWidget(self.scan_parameters_widget)

        self.scan_types = ["XYZ scan","Spherical scan","None"]
        self.scan_type_combobox = QtWidgets.QComboBox(parent=self)
        self.scan_type_combobox.addItems(self.scan_types)
        self.scan_type_combobox.currentTextChanged.connect(lambda: update_scan_type(self.scan_type_combobox.currentText()))

        self.xyzS_VLayout.addWidget(self.scan_type_combobox)
        spacerItem3 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Policy.Minimum,
                                            QtWidgets.QSizePolicy.Policy.Expanding)
        self.xyzS_VLayout.addItem(spacerItem3)

        self.vna_parameters_widget = VNAParametersWidget()
        self.xyzS_VLayout.addWidget(self.vna_parameters_widget)

        self.xyzS_HLayout = QtWidgets.QHBoxLayout()

        self.robot_start_widget = RobotStartWidget()
        self.feedback_widget = FeedbackWidget()
        self.log = LogWidget()

        self.xyzS_HLayout.addWidget(self.robot_start_widget)
        self.xyzS_HLayout.addWidget(self.feedback_widget)
        self.xyzS_HLayout.addWidget(self.log)
        self.xyzS_VLayout.addLayout(self.xyzS_HLayout)


        self.gridLayout_6.addLayout(self.xyzS_VLayout,0,0,1,1)
        
        def update_scan_type(s_type):
            self.scan_parameters_widget.set_scan(s_type)
            self.feedback_widget.set_scan(s_type)

        return scan_tab
        
    def setupUi(self, MainWindow):
        # MainWindow.resize(1624, 791)
        MainWindow.setDockOptions(QtWidgets.QMainWindow.DockOption.AllowTabbedDocks|QtWidgets.QMainWindow.DockOption.AnimatedDocks|QtWidgets.QMainWindow.DockOption.VerticalTabs)
        self.centralwidget = QtWidgets.QWidget(parent=MainWindow)

        self.gridLayout_2 = QtWidgets.QGridLayout(self.centralwidget)
        self.mainGridLayout = QtWidgets.QGridLayout()

        self.mainTabWidget = QtWidgets.QTabWidget(parent=self.centralwidget)

        self.connectionTab = self.initialize_connectionTab()
        self.mainTabWidget.addTab(self.connectionTab, "")

        self.manualCTab = self.initialize_manualCTab()
        self.mainTabWidget.addTab(self.manualCTab, "")

        self.scanTab = self.initialize_scanTab()
        self.mainTabWidget.addTab(self.scanTab, "")

        self.mainTabWidget.setTabText(self.mainTabWidget.indexOf(self.connectionTab), "Connection")
        self.mainTabWidget.setTabText(self.mainTabWidget.indexOf(self.manualCTab),"Manual Control")
        self.mainTabWidget.setTabText(self.mainTabWidget.indexOf(self.scanTab), "Scanning")

        self.mainGridLayout.addWidget(self.mainTabWidget, 0, 0, 1, 1)
        self.gridLayout_2.addLayout(self.mainGridLayout, 0, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)

        self.vna_plot_w = VNAPlotWidget()
        self.tracesDockWidget = QtWidgets.QDockWidget(parent=MainWindow)
        self.tracesDockWidget.setWidget(self.vna_plot_w)
        MainWindow.addDockWidget(QtCore.Qt.DockWidgetArea(2), self.tracesDockWidget)
        self.tracesDockWidget.setWindowTitle("VNA traces")

        self.scan_plot_w = ScanPlotWidget()
        self.scanDockWidget = QtWidgets.QDockWidget(parent=MainWindow)
        self.scanDockWidget.setWidget(self.scan_plot_w)
        MainWindow.addDockWidget(QtCore.Qt.DockWidgetArea(2), self.scanDockWidget)
        self.scanDockWidget.setWindowTitle("Scanned points")

        MainWindow.tabifyDockWidget(self.tracesDockWidget,self.scanDockWidget)
        self.tracesDockWidget.raise_()

        self.menubar = QtWidgets.QMenuBar(parent=MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1624, 22))
        self.menuSettings = QtWidgets.QMenu(parent=self.menubar)
        self.menuHelp = QtWidgets.QMenu(parent=self.menubar)
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(parent=MainWindow)
        MainWindow.setStatusBar(self.statusbar)
        self.menuSettings.setTitle("Settings")
        self.menuHelp.setTitle("Help")

        self.actionReset = QtGui.QAction(parent=MainWindow)
        self.actionClose = QtGui.QAction(parent=MainWindow)
        self.actionAbout = QtGui.QAction(parent=MainWindow)

        self.actionReset.setText("Reset")
        self.actionClose.setText("Close")
        self.actionAbout.setText("About")

        # self.indicatorLed = QtWidgets.QLabel()
        # self.statusbar.setAutoFillBackground(True)
        # self.statusbar.setStyleSheet("border-radius: 20px; min-height: 40px; min-width: 40px, background-color: red")
        self.statusbar.setStyleSheet("background-color: light gray")
        # self.indicatorLed.setText("None")

        self.menuSettings.addAction(self.actionReset)
        self.menuSettings.addAction(self.actionClose)
        self.menuHelp.addAction(self.actionAbout)
        self.menubar.addAction(self.menuSettings.menuAction())
        self.menubar.addAction(self.menuHelp.menuAction())
        # self.statusbar.addWidget(self.indicatorLed)

        MainWindow.setWindowTitle("Main Window")
        self.mainTabWidget.setCurrentIndex(0)

    def closeEvent(self, event):
        self.stop_button_clicked()
        super(MainWindow, self).closeEvent(event)


if __name__ == '__main__':
    if not QtWidgets.QApplication.instance():
        app = QtWidgets.QApplication(sys.argv)
    else:
        app = QtWidgets.QApplication.instance()
    window = MainWindow()
    window.show()
    sys.exit(app.exec())