from PyQt6 import QtCore, QtGui, QtWidgets
import sys
import numpy as np
import scipy as sp
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
import RsInstrument
from skuggsja_config import Config
from robodk_functions import RDK_KUKA
import robodk.robomath as robomath
import robodk.robolinkutils as robolinkutils
from robodk.robolink import TargetReachError, StoppedError
# from robodk.robomath import *
import threading
import functools
import datetime
import time
import warnings
import pickle
import glob
import matplotlib.tri as mtri
import scipy.constants as const
warnings.filterwarnings("ignore", category=DeprecationWarning)

SAVE_ON_SCAN_END = False

def movement_wrapper(f, mute = False):
    @functools.wraps(f)
    def new_function(self, *args, **kw):
        # lock = self.movement_lock
        if not self.robot_busy:
            # lock.acquire()
            # self.statusbar.setStyleSheet("background-color: red")
            # try:
            return f(self, *args, **kw)
            # finally:
            #     lock.release()
            #     self.statusbar.setStyleSheet("background-color: green")
        else:
            if not mute:
                print("Another function is already controlling the robot")
    return new_function

movement_wrapper_mute = functools.partial(movement_wrapper, mute = True)

class RobotMovementMonitorObject(QtCore.QObject):
    kill_signal = QtCore.pyqtSignal()
    restart_signal = QtCore.pyqtSignal()
    def __init__(self):
        super(RobotMovementMonitorObject, self).__init__()
        self.rdk_instance = RDK_KUKA(quit_on_close=True)
        # self.timer = QtCore.QTimer()
        # self.timer.setInterval(10)
        # self.timer.timeout.connect(self.print_coords_check)
        # self.timer.start()
        self.robot_in_box = True

    def coords_check_loop(self):
        try:
            while self.robot_in_box:
                    self.robot_in_box = self.coords_check()
                # if self.robot_in_box == False:
                #     self.rdk_instance.robot.Stop()
                #     self.kill_signal.emit()
                #     self.worker_thread().exit()
                #     sys.exit()
            else:
                self.rdk_instance.robot.Stop()
                self.kill_signal.emit()
                self.block_loop()
                print("Collision detected at ",robomath.Pose_2_KUKA(self.rdk_instance.robot.SolveFK(self.rdk_instance.robot.Joints(),tool=self.rdk_instance.robot.PoseTool())))
                # self.rdk_instance.Disconnect()
        except (ConnectionError, OSError):
            print("Command unsuccessful since connection has been closed")
            self.rdk_instance.Disconnect()

    def block_loop(self):
        for i in range(30):
            if not self.robot_in_box:
                self.rdk_instance.robot.Stop()

    def coords_check(self):
        joints = self.rdk_instance.robot.Joints()
        if len(joints.rows) == 6:
            coords = robomath.Pose_2_KUKA(self.rdk_instance.robot.SolveFK(joints,tool=self.rdk_instance.robot.PoseTool()))
            x, y, z = coords[:3] #520,0,502
            in_box = (420. < (x - 446.14) < 620.) and (-200. < y < 200.) and (300. < z < 700.)
            # in_box = (420. < (x - 446.14) < 620.) and (-20. < y < 20.) and (300. < z < 700.)
            if in_box and not self.robot_in_box:
                self.restart_signal.emit()
        else:
            in_box = self.robot_in_box
        return in_box


def movement_wrapper_robot_internal(f, ):
    @functools.wraps(f)
    def new_function(self, *args, **kw):
        try:
            if self.can_move:
                return f(self, *args, **kw)
            else:
                print("Robot busy or stopped")
        except StoppedError as err:
            print(f"Command {f} unsuccessful since robot has been stopped :\n", err)
        except (ConnectionError, ConnectionResetError, OSError) as err:
            print(f"Command {f} unsuccessful since connection has been closed :\n", err)
    return new_function


class RobotMovementObject(QtCore.QObject):
    arrived_at_point = QtCore.pyqtSignal(tuple,tuple)
    began_movement = QtCore.pyqtSignal()
    finished_movement = QtCore.pyqtSignal()
    finished_scan = QtCore.pyqtSignal()
    ready_for_acquisition = QtCore.pyqtSignal()
    def __init__(self):
        super(RobotMovementObject, self).__init__()
        # self.movement_lock = threading.Lock()
        self.rdk_instance = RDK_KUKA(quit_on_close = True)
        self.monitor = RobotMovementMonitorObject()
        self.monitor_thread = QtCore.QThread()
        # self.monitor.moveToThread(self.monitor_thread)
        # self.monitor_thread.started.connect(self.monitor.coords_check_loop)
        # self.monitor_thread.start()
        self.can_move = True
        # self.monitor.kill_signal.connect(self.killswitch_recieved)
        # self.monitor.kill_signal.connect(self.monitor_thread.quit)
        self.settle_time = 0
        self.acquisition_time = 0
        self.E_plane = 0

    @QtCore.pyqtSlot(bool)
    def set_plane(self,E_plane = False):
        if E_plane:
            self.rdk_instance.robot.setPoseTool(self.rdk_instance.default_pose_tool*robomath.rotz(np.pi/2))
            self.E_plane = 1
        else:
            self.rdk_instance.robot.setPoseTool(self.rdk_instance.default_pose_tool)
            self.E_plane = 0

    @QtCore.pyqtSlot(float,float,float,float)
    def rotate_pose_tool(self,phi,az,el,z_phc):
        if self.E_plane:
            pose_tool = self.rdk_instance.default_pose_tool*robomath.rotz(np.pi/2)
        else:
            pose_tool = self.rdk_instance.default_pose_tool
        self.rdk_instance.robot.setPoseTool(pose_tool.Offset(0,0,z_phc)*robomath.rotz(np.deg2rad(phi))*robomath.roty(np.deg2rad(az))*robomath.rotx(np.deg2rad(el)))

    @QtCore.pyqtSlot(tuple,tuple,tuple)
    @movement_wrapper_robot_internal
    def move_robot_to_point_scan(self,coord_tuple=None, index_tuple=None,
                                 dir_tuple=None):
        if coord_tuple != None:
            self.began_movement.emit()
            # coord_tuple, index_tuple, dir_tuple, settle_time
            x, y, z = coord_tuple
            new_pose = self.rdk_instance.target_scan_init.Pose() * (robomath.eye().Offset(x, y, z))
            self.rdk_instance.target_scan.setPose(new_pose)
            self.moveJointsSafe(new_pose)
            # print(coord_tuple)
            time.sleep(self.settle_time)
            self.ready_for_acquisition.emit()
            time.sleep(self.acquisition_time)
            self.arrived_at_point.emit(index_tuple, dir_tuple)
            self.finished_movement.emit()

    @QtCore.pyqtSlot(tuple,tuple,tuple)
    @movement_wrapper_robot_internal
    def move_robot_to_point_scan_sph(self,coord_tuple=None, index_tuple=None,
                                 dir_tuple=None):
        if coord_tuple != None:
            self.began_movement.emit()
            # coord_tuple, index_tuple, dir_tuple, settle_time
            phi, theta, r = coord_tuple
            # new_pose = self.rdk_instance.target_scan_init.Pose() * (robomath.eye().Offset(x, y, z))
            new_pose = self.rdk_instance.target_scan_init.Pose() * (robomath.eye().Offset(x =0, y =0, z=0,rx=theta,ry=phi))
            self.rdk_instance.target_scan.setPose(new_pose)
            self.moveJointsSafe(new_pose)
            # print(coord_tuple)
            time.sleep(self.settle_time)
            self.ready_for_acquisition.emit()
            time.sleep(self.acquisition_time)
            self.arrived_at_point.emit(index_tuple, dir_tuple)
            self.finished_movement.emit()

    def moveJointsSafe(self, new_pose):
        if len(self.rdk_instance.robot.SolveIK(new_pose, tool=self.rdk_instance.robot.PoseTool()).tolist()) == 6:
            config = np.squeeze(
                self.rdk_instance.robot.JointsConfig(self.rdk_instance.target_init.Joints())[:3]).tolist()
            joints = robolinkutils.SolveIK_Conf(self.rdk_instance.robot, new_pose,
                                                toolpose=self.rdk_instance.robot.PoseTool(), joint_config=config)
            print(self.rdk_instance.robot.JointsConfig(self.rdk_instance.target_init.Joints()))
            # self.rdk_instance.robot.MoveJ(self.rdk_instance.target_scan.Pose())
            if len(joints) > 0:
                diff = np.array(joints)[:, :-2] - np.tile(np.array(self.rdk_instance.target_init.Joints())[:, :6],
                                                          (len(joints), 1))
                print(diff)
                print(np.linalg.norm(diff, axis=1))
                print(diff[np.linalg.norm(diff, axis=1).argmin()])
                best_config = diff[np.linalg.norm(diff, axis=1).argmin()]
                self.rdk_instance.robot.MoveJ(joints[np.linalg.norm(diff, axis=1).argmin()])
            else:
                print("No suitable configuration found")
        else:
            print("Target coordinates are inaccessible")

    @QtCore.pyqtSlot(tuple)
    @movement_wrapper_robot_internal
    def move_robot_to_coordinate(self, coord_tuple):
        self.began_movement.emit()
        pose_to_move_to = robomath.KUKA_2_Pose(coord_tuple)
        # if len(self.rdk_instance.robot.SolveIK(pose_to_move_to).tolist()) == 6:
        self.rdk_instance.target_rel.setPose(pose_to_move_to)
        self.moveJointsSafe(pose_to_move_to)
        self.finished_movement.emit()

    @QtCore.pyqtSlot(tuple)
    @movement_wrapper_robot_internal
    def move_robot_relative(self, coord_tuple):
        current = robomath.Pose_2_KUKA(self.rdk_instance.target_rel.Pose())
        new = [current[i] + x for i, x in enumerate(coord_tuple)]
        self.move_robot_to_coordinate(new)

    @QtCore.pyqtSlot()
    @movement_wrapper_robot_internal
    def move_robot_to_init_scan(self):
        self.began_movement.emit()
        self.rdk_instance.robot.MoveJ(self.rdk_instance.target_scan_init.Pose())
        self.finished_movement.emit()
        self.finished_scan.emit()

    @QtCore.pyqtSlot()
    @movement_wrapper_robot_internal
    def move_robot_to_init(self):
        self.began_movement.emit()
        self.rdk_instance.robot.MoveJ(self.rdk_instance.target_init.Joints())
        self.rdk_instance.target_rel.setPose(self.rdk_instance.target_init.Pose())
        self.finished_movement.emit()

    @QtCore.pyqtSlot()
    def killswitch_recieved(self):
        print("robot thread processed hit")
        # self.rdk_instance.Disconnect()
        #
        # raise TargetReachError

    @QtCore.pyqtSlot()
    def restart_after_killswitch(self):
        if self.monitor.coords_check():
            self.can_move = True
            self.monitor.robot_in_box=True
            self.monitor_thread.start()

    @QtCore.pyqtSlot(float)
    def set_settle_time(self,time):
        self.settle_time = time

    @QtCore.pyqtSlot(float)
    def set_acquisition_time(self,time):
        self.acquisition_time = time

    @QtCore.pyqtSlot()
    def talk(self):
        print('yooooooo')

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
        gridLayout2 = QtWidgets.QGridLayout()
        self.stop_pushButton = RobotStopButton()

        self.man_step_label = QtWidgets.QLabel()
        self.man_step_lineEdit = QtWidgets.QLineEdit()
        self.robot_joint_speed_label = QtWidgets.QLabel()
        self.robot_joint_speed_lineEdit = QtWidgets.QLineEdit()
        self.robot_joint_accel_label = QtWidgets.QLabel()
        self.robot_joint_accel_lineEdit = QtWidgets.QLineEdit()
        self.position_reset_pushButton = QtWidgets.QPushButton()
        self.set_H_plane_pushButton = QtWidgets.QPushButton()
        self.set_E_plane_pushButton = QtWidgets.QPushButton()
        self.set_tool_rotation_pushButton = QtWidgets.QPushButton()
        self.set_scan_init_pushButton = QtWidgets.QPushButton()
        self.set_robot_speed_pushButton = QtWidgets.QPushButton()
        self.robot_joint_speed_lineEdit.setMaximumWidth(40)
        self.robot_joint_accel_lineEdit.setMaximumWidth(40)

        self.tool_phi_label = QtWidgets.QLabel()
        self.tool_phi_lineEdit = QtWidgets.QLineEdit()
        self.tool_az_label = QtWidgets.QLabel()
        self.tool_az_lineEdit = QtWidgets.QLineEdit()
        self.tool_el_label = QtWidgets.QLabel()
        self.tool_el_lineEdit = QtWidgets.QLineEdit()
        self.tool_z_label = QtWidgets.QLabel()
        self.tool_z_lineEdit = QtWidgets.QLineEdit()

        self.man_step_label.setText("Manual movement step")
        self.man_step_lineEdit.setText("5")
        self.robot_joint_speed_label.setText("Joints speed")
        self.robot_joint_accel_label.setText("Joints acceleration")
        self.robot_joint_speed_lineEdit.setText("5")
        self.robot_joint_accel_lineEdit.setText("5")
        self.position_reset_pushButton.setText("Reset robot position")
        self.set_H_plane_pushButton.setText("Set H plane")
        self.set_E_plane_pushButton.setText("Set E plane")
        self.set_tool_rotation_pushButton.setText("Set tool rotation")
        self.set_scan_init_pushButton.setText("Set new scan origin point")
        self.set_robot_speed_pushButton.setText("Set robot speed")
        self.tool_phi_label.setText("phi")
        self.tool_az_label.setText("az")
        self.tool_el_label.setText("el")
        self.tool_z_label.setText("z phase center")
        self.tool_phi_lineEdit.setText("0")
        self.tool_az_lineEdit.setText("0")
        self.tool_el_lineEdit.setText("0")
        self.tool_z_lineEdit.setText("0")

        gridLayout.addWidget(self.man_step_label,0,0,1,1)
        gridLayout.addWidget(self.man_step_lineEdit, 0, 1, 1, 1)
        gridLayout.addWidget(self.robot_joint_speed_label, 0, 2, 1, 1)
        gridLayout.addWidget(self.robot_joint_speed_lineEdit, 0, 3, 1, 1)
        gridLayout.addWidget(self.robot_joint_accel_label, 0, 4, 1, 1)
        gridLayout.addWidget(self.robot_joint_accel_lineEdit, 0, 5, 1, 1)
        gridLayout.addWidget(self.set_robot_speed_pushButton, 1, 3, 1, 2)
        gridLayout.addWidget(self.position_reset_pushButton, 1, 0, 1, 1)
        # gridLayout.addWidget(self.position_reset_cross_pushButton, 2, 0, 1, 1)
        gridLayout.addWidget(self.set_scan_init_pushButton, 1, 1, 1, 1)
        gridLayout2.addWidget(self.set_H_plane_pushButton,0,0,1,1)
        gridLayout2.addWidget(self.set_E_plane_pushButton, 0, 1, 1, 1)
        gridLayout2.addWidget(self.set_tool_rotation_pushButton, 0, 2, 1, 1)
        gridLayout2.addWidget(self.tool_phi_label, 0, 3, 1, 1)
        gridLayout2.addWidget(self.tool_phi_lineEdit, 0, 4, 1, 1)
        gridLayout2.addWidget(self.tool_az_label, 0, 5, 1, 1)
        gridLayout2.addWidget(self.tool_az_lineEdit, 0, 6, 1, 1)
        gridLayout2.addWidget(self.tool_el_label, 0, 7, 1, 1)
        gridLayout2.addWidget(self.tool_el_lineEdit, 0, 8, 1, 1)
        gridLayout2.addWidget(self.tool_z_label, 0, 9, 1, 1)
        gridLayout2.addWidget(self.tool_z_lineEdit, 0, 10, 1, 1)

        self.VLayout.addLayout(gridLayout)
        self.VLayout.addLayout(gridLayout2)
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
        self.acq_label = QtWidgets.QLabel()
        self.acq_label.setText("Acquisition time, s")
        self.acq_labelFramed = QLabelFramed()
        self.acq_labelFramed.setText("0")
        self.total_label = QtWidgets.QLabel()
        self.total_label.setText("Total idling time, m")
        self.total_labelFramed = QLabelFramed()
        self.total_labelFramed.setText("0")
        self.sttl_label.setSizePolicy(QtWidgets.QSizePolicy.Policy.Fixed,QtWidgets.QSizePolicy.Policy.Fixed)
        self.sttl_lineEdit.setMaximumWidth(50)
        self.acq_label.setSizePolicy(QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Fixed)
        self.acq_labelFramed.setMaximumWidth(50)
        self.total_label.setSizePolicy(QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Fixed)
        self.total_labelFramed.setMaximumWidth(50)
        time_layout = QtWidgets.QHBoxLayout()
        time_layout.setAlignment(QtCore.Qt.AlignmentFlag.AlignLeft)
        time_layout.addWidget(self.sttl_label)
        time_layout.addWidget(self.sttl_lineEdit)
        time_layout.addWidget(self.acq_label)
        time_layout.addWidget(self.acq_labelFramed)
        time_layout.addWidget(self.total_label)
        time_layout.addWidget(self.total_labelFramed)

        self.scan_start_pushButton = QtWidgets.QPushButton()
        self.scan_start_pushButton.setText("Start scan")

        self.message_label = QtWidgets.QLabel()
        self.message_label.setWordWrap(True)
        self.scan_progressbar = QtWidgets.QProgressBar()
        self.scan_progressbar.setMaximum(100)
        self.scan_progressbar.setMinimum(0)
        progress_layout = QtWidgets.QHBoxLayout()
        progress_layout.addWidget(self.message_label)
        progress_layout.addWidget(self.scan_progressbar)

        self.scan_params_VLayout.addWidget(self.scan_params_label)
        self.scan_params_VLayout.addLayout(self.scan_params_gridLayout)
        self.scan_params_VLayout.addLayout(time_layout)
        self.scan_params_VLayout.addWidget(self.scan_start_pushButton)
        self.scan_params_VLayout.addLayout(progress_layout)

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

class DataFileWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(DataFileWidget, self).__init__(parent)
        self.gridLayout = QtWidgets.QGridLayout(self)

        self.filename_textbox = QtWidgets.QLineEdit()
        self.button_save_points = QtWidgets.QPushButton()
        self.button_load_points = QtWidgets.QPushButton()
        # TODO: saving files as dataframes
        self.sim_textbox = QtWidgets.QLineEdit()
        self.button_load_points_sim = QtWidgets.QPushButton()

        self.button_save_points.setText("Save points as .nb")
        self.button_load_points.setText("Load points from .nb")
        self.sim_textbox.setText("Farfield_w_band")
        self.button_load_points_sim.setText("Load simulation data")

        self.gridLayout.addWidget(self.filename_textbox,0,0,1,1)
        self.gridLayout.addWidget(self.button_save_points,0,1,1,1)
        self.gridLayout.addWidget(self.button_load_points,0,2,1,1)
        self.gridLayout.addWidget(self.sim_textbox,1 , 0, 1, 1)
        self.gridLayout.addWidget(self.button_load_points_sim, 1, 2, 1, 1)

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
        size_policy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Policy.MinimumExpanding, QtWidgets.QSizePolicy.Policy.MinimumExpanding)
        size_policy.setRetainSizeWhenHidden(True)
        # self.canvas.setMinimumWidth(40)
        self.canvas.setSizePolicy(size_policy)
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
        self.traces_toolbutton = QtWidgets.QToolButton(self)
        self.traces_toolmenu = QtWidgets.QMenu(self)

        self.traces_toolbutton.setText('Traces to show')
        self.button_update_continuous.setText("Plot")
        self.button_step.setText("Step")
        self.traces_toolbutton.setMenu(self.traces_toolmenu)
        self.traces_toolbutton.setPopupMode(QtWidgets.QToolButton.ToolButtonPopupMode.InstantPopup)
        size_policy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Policy.Expanding,QtWidgets.QSizePolicy.Policy.Fixed)
        self.traces_toolbutton.setSizePolicy(size_policy)
        # self.traces_toolbutton.setMaximumWidth(50)

        self.gridLayout.addWidget(self.button_update_continuous, 3, 0, 1, 1)
        self.gridLayout.addWidget(self.button_step, 3, 1, 1, 1)
        self.gridLayout.addWidget(self.traces_toolbutton, 1, 0, 1, 2)
        self.gridLayout.addWidget(self.canvas, 0, 0, 1, 2)

        self.VLayout.addLayout(self.gridLayout)

class ScanPlotWidget(PlotWidget):
    def __init__(self, parent=None):
        super(ScanPlotWidget, self).__init__(parent)
        self.gridLayout = QtWidgets.QGridLayout()

        self.plot_format_combobox = QtWidgets.QComboBox()
        self.slice_direction_combobox = QtWidgets.QComboBox()
        self.checkbox_fft = QtWidgets.QRadioButton()
        self.checkbox_backpropagation = QtWidgets.QCheckBox()
        self.backpropagation_distance_label = QtWidgets.QLabel()
        self.backpropagation_distance_textfield = QtWidgets.QLineEdit()
        self.backpropagation_distance_slider = QtWidgets.QSlider()
        self.backpropagation_distance_slider.setOrientation(QtCore.Qt.Orientation.Horizontal)
        self.coordinate_label = QtWidgets.QLabel()
        self.coordinate_textfield = QtWidgets.QLineEdit()
        self.coordinate_slider = QtWidgets.QSlider()
        self.coordinate_slider.setOrientation(QtCore.Qt.Orientation.Horizontal)
        self.frequency_label = QtWidgets.QLabel()
        self.frequency_textfield = QtWidgets.QLineEdit()
        self.frequency_slider = QtWidgets.QSlider()
        self.frequency_slider.setOrientation(QtCore.Qt.Orientation.Horizontal)

        self.checkbox_fft.setText("2D XY FFT")
        self.checkbox_fft.setEnabled(False)
        self.checkbox_backpropagation.setText("Backpropagation")
        # self.checkbox_backpropagation.setEnabled(False)
        self.backpropagation_distance_label.setText("Distance (mm)")
        self.coordinate_label.setText("Change coordinate to (mm):")
        self.frequency_label.setText("Change frequency to (GHz):")
        self.plot_format_combobox.addItems(self.plot_formats.keys())
        self.slice_directions = {"XY": 2, "XZ": 1, "YZ": 0}
        self.slice_direction_combobox.addItems(self.slice_directions.keys())

        self.gridLayout.addWidget(self.plot_format_combobox, 1, 0, 1, 2)
        self.gridLayout.addWidget(self.checkbox_fft, 1, 2, 1, 1)
        self.gridLayout.addWidget(self.slice_direction_combobox, 2, 0, 1, 2)
        self.gridLayout.addWidget(self.coordinate_label, 4, 0, 1, 1)
        self.gridLayout.addWidget(self.coordinate_textfield, 4, 1, 1, 1)
        self.gridLayout.addWidget(self.coordinate_slider, 4, 2, 1, 1)
        self.gridLayout.addWidget(self.frequency_label, 3, 0, 1, 1)
        self.gridLayout.addWidget(self.frequency_textfield, 3, 1, 1, 1)
        self.gridLayout.addWidget(self.frequency_slider, 3, 2, 1, 1)

        self.backpropagation_distance_slider.setMaximum(3000)
        self.backpropagation_distance_slider.valueChanged.connect(lambda: self.backpropagation_distance_textfield.setText(str(self.backpropagation_distance_slider.value())))
        fft_hLayout = QtWidgets.QHBoxLayout()
        fft_hLayout.addWidget(self.checkbox_backpropagation)
        fft_hLayout.addWidget(self.backpropagation_distance_label)
        fft_hLayout.addWidget(self.backpropagation_distance_textfield)
        fft_hLayout.addWidget(self.backpropagation_distance_slider)

        self.VLayout.addLayout(self.gridLayout)
        self.VLayout.addLayout(fft_hLayout)

        self.ax_scan = self.figure.add_subplot(111)


class SlicePlotWidget(PlotWidget):
    def __init__(self, parent=None):
        super(SlicePlotWidget, self).__init__(parent)
        self.gridLayout = QtWidgets.QGridLayout()
        self.slice_direction_combobox = QtWidgets.QComboBox()
        self.coordinate_label = QtWidgets.QLabel()
        self.coordinate_textfield = QtWidgets.QLineEdit()
        self.coordinate_slider = QtWidgets.QSlider()
        self.coordinate_slider.setOrientation(QtCore.Qt.Orientation.Horizontal)

        self.slice_directions = {"Horizontal": 0, "Vertical": 1}
        self.slice_direction_combobox.addItems(self.slice_directions.keys())

        self.coordinate_label.setText("Change coordinate to (mm):")
        self.gridLayout.addWidget(self.slice_direction_combobox, 1, 0, 1, 3)
        self.gridLayout.addWidget(self.coordinate_label, 2, 0, 1, 1)
        self.gridLayout.addWidget(self.coordinate_textfield, 2, 1, 1, 1)
        self.gridLayout.addWidget(self.coordinate_slider, 2, 2, 1, 1)
        self.ax_slice = self.figure.add_subplot(111)


        self.VLayout.addLayout(self.gridLayout)


class MainWindow(QtWidgets.QMainWindow):
    send_movement_coords_scan = QtCore.pyqtSignal(tuple,tuple,tuple)
    send_movement_coords_scan_sph = QtCore.pyqtSignal(tuple, tuple, tuple)
    send_movement_coords_rel = QtCore.pyqtSignal(tuple)
    send_movement_coords_abs = QtCore.pyqtSignal(tuple)
    send_robot_to_scan_init = QtCore.pyqtSignal()
    send_robot_to_init = QtCore.pyqtSignal()
    send_settle_time = QtCore.pyqtSignal(float)
    send_acquisition_time = QtCore.pyqtSignal(float)
    send_tool_plane = QtCore.pyqtSignal(bool)
    send_tool_rotation = QtCore.pyqtSignal(float,float,float,float)
    def __init__(self, *args, obj=None, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.configs = Config("aaa.toml")
        self.setupUi(self)
        self.vna_connected = False
        self.sim_data_available = False
        self.vna_parameters_widget.setEnabled(False)
        self.vna_plot_w.setEnabled(False)

        self.movement_lock = threading.Lock()

        self.previous_time = 0
        self.coord_dict = {"X": 0, "Y": 1, "Z": 2}
        self.fft_padding = 1

        self.robot_connected = False
        self.scan_finished = False
        for w in (self.manualCTab,self.scan_parameters_widget, self.scan_plot_w, self.slice_plot_w):
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

        for lineeditwidget in [x[5] for x in self.scan_parameters_widget.params_rows[2:]]+\
                              [self.scan_parameters_widget.sttl_lineEdit]:
            lineeditwidget.textChanged.connect(self.update_estimated_scan_time)

        self.move_timer = QtCore.QTimer()
        self.move_timer.setInterval(10)
        self.move_timer.timeout.connect(self.move_timer_func)
        self.manual_direction = "+Z"
        ManualControlWidget.group_minus.buttonPressed.connect(self.minus_button_pressed)
        ManualControlWidget.group_plus.buttonPressed.connect(self.plus_button_pressed)
        ManualControlWidget.group_minus.buttonReleased.connect(self.manual_button_released)
        ManualControlWidget.group_plus.buttonReleased.connect(self.manual_button_released)

        self.coord_update_timer = QtCore.QTimer()
        self.coord_update_timer.setInterval(10)
        self.coord_update_timer.timeout.connect(self.coord_update_func)

        self.robot_controls_widget.position_reset_pushButton.clicked.connect(self.position_reset_button_clicked)
        self.robot_controls_widget.set_scan_init_pushButton.clicked.connect(self.set_scan_init_button_clicked)
        self.robot_controls_widget.set_robot_speed_pushButton.clicked.connect(self.set_robot_speed)

        self.robot_controls_widget.set_E_plane_pushButton.clicked.connect(lambda: self.send_tool_plane.emit(1))
        self.robot_controls_widget.set_H_plane_pushButton.clicked.connect(lambda: self.send_tool_plane.emit(0))
        self.robot_controls_widget.set_tool_rotation_pushButton.clicked.connect(self.set_tool_rotation)

        self.scan_parameters_widget.scan_start_pushButton.clicked.connect(self.begin_scan)

        self.file_widget.button_save_points.clicked.connect(lambda: self.save_data_file(self.file_widget.filename_textbox.text()))
        self.file_widget.button_load_points.clicked.connect(lambda: self.load_data_file(self.file_widget.filename_textbox.text()))
        self.file_widget.button_load_points_sim.clicked.connect(self.load_sim_data_file)

        self.run_on_robot = False
        self.robot_start_widget.simulate_pushButton.clicked.connect(self.run_program_in_sim)
        self.robot_start_widget.run_pushButton.clicked.connect(self.run_program_on_robot)

        self.scan_plot_w.frequency_slider.valueChanged.connect(self.frequency_slider_changed)
        if self.vna_connected:
            self.scan_plot_w.frequency_slider.sliderPressed.connect(lambda: self.freq_select_vline.set_visible(True))
            self.scan_plot_w.frequency_slider.sliderReleased.connect(lambda: self.freq_select_vline.set_visible(False))
        self.scan_plot_w.slice_direction_combobox.currentTextChanged.connect(self.update_coord_slider)
        self.scan_plot_w.plot_format_combobox.currentTextChanged.connect(self.scan_plot_update)
        self.scan_plot_w.checkbox_fft.clicked.connect(self.scan_plot_update)
        self.scan_plot_w.checkbox_backpropagation.clicked.connect(self.scan_plot_update)
        self.scan_plot_w.coordinate_slider.valueChanged.connect(self.coordinate_slider_changed)
        self.scan_plot_w.backpropagation_distance_slider.valueChanged.connect(self.scan_plot_update)

        self.slice_plot_w.slice_direction_combobox.currentTextChanged.connect(self.update_coord_slider)
        self.slice_plot_w.coordinate_slider.valueChanged.connect(self.coordinate_slider_changed_slice)


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

    @movement_wrapper_mute
    #TODO: Move to arbitrary coordinate
    def move_timer_func(self):
        # with self.movement_lock:
            # print(self.manual_direction)
            try:
                if not self.robot_busy:
                    dict = {"+X": (1, 0, 0, 0, 0, 0),"+Y": (0,1,0, 0, 0, 0),"+Z": (0,0,1, 0, 0, 0),
                            "-X": (-1,0,0, 0, 0, 0),"-Y": (0,-1,0, 0, 0, 0),"-Z": (0,0,-1, 0, 0, 0),
                            "+A": (0, 0, 0, 1, 0, 0), "+B": (0, 0, 0, 0, 1, 0), "+C": (0, 0, 0, 0, 0, 1),
                            "-A": (0, 0, 0, -1, 0, 0), "-B": (0, 0, 0, 0, -1, 0), "-C": (0, 0, 0, 0, 0, -1)}
                    step = float(self.robot_controls_widget.man_step_lineEdit.text())
                    # self.current_robot_pose = self.robot_rdk.move_relative([x*speed for x in dict[self.manual_direction]])
                    # self.update_current_robot_pose()
                    self.send_movement_coords_rel.emit(tuple([x*step for x in dict[self.manual_direction]]))
            except ValueError:
                print('Inputted value is invalid')



    def coord_update_func(self):
        current_coords = robomath.Pose_2_KUKA(self.robot_rdk.robot.Pose())
        i = 0
        # current_time = time.time_ns()/1e9
        for r in self.feedback_widget.params_rows[:2]:
            for w in r[1::2]:
                w.setText(f"{current_coords[i]:.2f}")
                i+=1
        for i,w in enumerate(self.coordinates_widgets_list):
            w.pos_fdbk_label.setText(f"{current_coords[i]:.2f}")
        # if self.robot_busy and self.previous_coords != current_coords:
        #     delta = np.linalg.norm(np.array(current_coords[:3])-np.array(self.previous_coords[:3]))
        #     current_speed = (delta)/(current_time-self.previous_time)
        #     print(current_speed)
        # self.previous_coords = current_coords
        # self.previous_time = current_time

    @QtCore.pyqtSlot()
    @movement_wrapper
    def position_reset_button_clicked(self):
        self.send_robot_to_init.emit()

    def set_tool_rotation(self):
        try:
            phi,az,el,z_phc = [float(x.text()) for x in [self.robot_controls_widget.tool_phi_lineEdit,
                                                   self.robot_controls_widget.tool_az_lineEdit,
                                                   self.robot_controls_widget.tool_el_lineEdit,
                                                         self.robot_controls_widget.tool_z_lineEdit]]
            self.send_tool_rotation.emit(phi,az,el,z_phc)
        except ValueError:
            print("Value is not a float")

    def set_scan_init_button_clicked(self):
        self.scan_init = self.robot_rdk.set_scan_initial()
        w_list = [self.scan_parameters_widget.origin_params_w_list,
                  self.scan_parameters_widget.orientation_params_w_list]
        for i, x in enumerate(self.scan_init):
            w_list[i//3][(i%3)*2+1].setText(f"{x:.3f}")

    @QtCore.pyqtSlot()
    @movement_wrapper
    def begin_scan(self):
        # x = threading.Thread(target=self.test_scan)
        # x = QtCore.QThread
        # x.start()
        self.scan_started = 1
        if self.scan_type_combobox.currentText() == "XYZ scan":
            self.scan_xyz()
        elif self.scan_type_combobox.currentText() == "Spherical scan":
            self.scan_sph()
        else:
            pass
        # self.test_scan()

    @movement_wrapper
    def scan_xyz(self):
        try:
            points_axes = []
            shape = []

            self.update_estimated_scan_time()
            for param in self.scan_parameters_widget.params_rows[2:]:
                min = float(param[1].text())
                max = float(param[3].text())
                steps = int(param[5].text())
                shape.append(steps)
                points_axes.append(np.linspace(min,max,steps))
            self.scan_coords = points_axes
            self.delta=[]
            if self.vna_connected:
                shape.append(len(self.freq_arr))
                self.instr.write_bool("INITiate:CONTinuous:ALL",False)
            dir_x = 1
            dir_y = 1
            dir_z = 1
            self.data = np.empty(shape, dtype= complex)
            # X_grid, Y_grid = np.meshgrid(points_axes[0],points_axes[1])
            self.data[:] = np.nan
            if self.vna_connected:
                self.scan_plot_initialize()
                self.slice_plot_initialize()
                self.update_coord_slider()

            self.scanpoint_data = []
            self.scanpoint_index = 0

            settle_time = float(self.scan_parameters_widget.sttl_lineEdit.text())
            self.send_settle_time.emit(settle_time)
            if self.vna_connected:
                acq_time = float(self.scan_parameters_widget.acq_labelFramed.text())
                self.send_acquisition_time.emit(acq_time)
        # except ValueError:
        #     print('Inputted value is invalid')
        # try:
            for iz, z in enumerate(points_axes[2][::dir_z]):
                for iy, y in enumerate(points_axes[1][::dir_y]):
                    for ix, x in enumerate(points_axes[0][::dir_x]):
                        self.scanpoint_data.append({'c':(x,y,z),'i':(ix,iy,iz),'d':(dir_x,dir_y,dir_z)})
                    dir_x *= -1
                dir_y *= -1
            self.send_movement_coords_scan.emit(self.scanpoint_data[0]['c'],self.scanpoint_data[0]['i'],self.scanpoint_data[0]['d'])
            # print(self.data)
        except ValueError as err:
            print('Inputted value is invalid\n' + str(err))

    @movement_wrapper
    def scan_sph(self):
        try:
            points_axes = []
            shape = []
            self.update_estimated_scan_time()
            for param in self.scan_parameters_widget.params_rows[3:]:
                min = float(param[1].text())
                max = float(param[3].text())
                steps = int(param[5].text())
                shape.append(steps)
                points_axes.append(np.linspace(min,max,steps))
            points_axes.append(np.array((float(self.scan_parameters_widget.params_rows[2][1].text()),)))
            shape.append(1)
            self.scan_coords = points_axes
            self.delta=[]
            if self.vna_connected:
                shape.append(len(self.freq_arr))
                self.instr.write_bool("INITiate:CONTinuous:ALL",False)
            dir_phi = 1
            dir_theta = 1
            dir_r = 1
            self.data = np.empty(shape, dtype= complex)
            # X_grid, Y_grid = np.meshgrid(points_axes[0],points_axes[1])
            self.data[:] = np.nan
            if self.vna_connected:
                self.scan_plot_initialize()
                self.slice_plot_initialize()
                self.update_coord_slider()

            settle_time = float(self.scan_parameters_widget.sttl_lineEdit.text())
            self.send_settle_time.emit(settle_time)
            if self.vna_connected:
                acq_time = float(self.scan_parameters_widget.acq_labelFramed.text())
                self.send_acquisition_time.emit(acq_time)

            self.scanpoint_data = []
            self.scanpoint_index = 0
        # except ValueError:
        #     print('Inputted value is invalid')
        # try:
            for ir, r in enumerate(points_axes[2][::dir_r]):
                for itheta, theta in enumerate(points_axes[1][::dir_theta]):
                    for iphi, phi in enumerate(points_axes[0][::dir_phi]):
                        self.scanpoint_data.append({'c':(phi,theta,r),'i':(iphi,itheta,ir),'d':(dir_phi,dir_theta,dir_r)})
                        # self.send_movement_coords_scan_sph.emit((phi,theta,r),(iphi,itheta,ir),(dir_phi,dir_theta,dir_r))
                    dir_phi *= -1
                dir_theta *= -1
            print(self.scanpoint_data)
            self.send_movement_coords_scan_sph.emit(self.scanpoint_data[0]['c'],self.scanpoint_data[0]['i'],self.scanpoint_data[0]['d'])
            # print(self.data)
        except ValueError as err:
            print('Inputted value is invalid\n' + str(err))

    @QtCore.pyqtSlot(tuple, tuple)
    def scan_data_add_point(self, index_tuple, dir_tuple):
        ix, iy, iz = index_tuple
        dir_x, dir_y, dir_z = dir_tuple
        if self.vna_connected:
            if not self.instr.query_bool("*OPC?"):
                print("WARNING: data has been read before a sweep finished")
            data_point = self.query_data(1,raw=True)
            self.data[ix * dir_x - (1 if dir_x < 0 else 0),
            iy * dir_y - (1 if dir_y < 0 else 0),
            iz* dir_z - (1 if dir_z < 0 else 0),:] = data_point
            self.scan_plot_update()
        current_time = time.time_ns() / 1e9
        index_current = (index_tuple[2])*self.data.shape[0]*self.data.shape[1]+ \
                        (index_tuple[1]) * self.data.shape[0] + \
                        (index_tuple[0])
        index_total =np.prod(self.data.shape[:3])
        if self.scanpoint_index != index_current:
            print("something went wrong, index mismatched")
        else:
            self.scanpoint_index +=1
            if self.scanpoint_index < index_total:
                if self.scan_type_combobox.currentText() == "XYZ scan":
                    movement_signal = self.send_movement_coords_scan
                elif self.scan_type_combobox.currentText() == "Spherical scan":
                    movement_signal = self.send_movement_coords_scan_sph
                movement_signal.emit(self.scanpoint_data[self.scanpoint_index]['c'],
                                                        self.scanpoint_data[self.scanpoint_index]['i'],
                                                        self.scanpoint_data[self.scanpoint_index]['d'])
            else:
                self.send_robot_to_scan_init.emit()
        self.scan_parameters_widget.scan_progressbar.setMaximum(index_total)
        self.scan_parameters_widget.scan_progressbar.setValue(index_current)
        message = f"Point {index_current + 1} out of {index_total}\n"
        if index_current !=0:
            self.delta.append(current_time-self.previous_time)
        if index_current+1 == index_total:
            self.scan_parameters_widget.scan_progressbar.reset()
            message += "Scan finished!"
        elif index_current > 1:
            self.average_time_per_move = np.average(self.delta[1:])
            time_left = (index_total - index_current)*self.average_time_per_move
            message += f"Time left: {time_left//(60*60):.0f} h {time_left//60:.0f} m {time_left%60:.1f} s"
        self.scan_parameters_widget.message_label.setText(message)
        self.previous_time = current_time

    @QtCore.pyqtSlot()
    def robot_scan_finished(self):
        self.statusbar.setStyleSheet("background-color: green")
        self.robot_busy = False
        self.scan_finished = True

        current_time = datetime.datetime.now().strftime("%Y_%m_%d_%H-%M-%S")

        if self.vna_connected:
            self.slice_plot_initialize()
            self.instr.write_bool("INITiate:CONTinuous:ALL", True)
            self.data_fft = sp.fft.fftshift(
                sp.fft.fft2(self.data, s=(self.data.shape[0] * self.fft_padding, self.data.shape[1] * self.fft_padding), axes=(0, 1)),
                axes=(0, 1))
            self.scan_plot_w.checkbox_fft.setEnabled(True)

        if SAVE_ON_SCAN_END:
            self.save_data_file(current_time)


    @QtCore.pyqtSlot()
    def trigger_VNA(self):
        if self.vna_connected:
            self.instr.write("INITiate:IMMediate:ALL")

    def save_data_file(self,filename):
        np.save(f"{filename}.npy", self.data)
        pickle.dump(self.scan_coords+[self.freq_arr],open(f"{filename}.pkl","wb"))

    def load_data_file(self,filename):
        # filename = "2024_03_26_21-48-18"
        try:
            self.data = np.load(f"{filename}.npy")

            # self.data[:, 1::2] = self.data[::-1, 1::2]
            # self.data[:,::2] = np.roll(self.data[:,::2],1,axis=0)
            # self.data[:, 1::2] = np.roll(self.data[:, 1::2], -1, axis=0)
            # print(self.data[:,:,0,np.argmin(abs(temp_list[3]-91.0))])
            # np.savetxt("slice.csv",self.data[:, :, 0, np.argmin(abs(temp_list[3] - 91.0))])

            # self.data = sp.fft.fftshift(sp.fft.fft(self.data, axis=3),axes=3)
            temp_list = pickle.load(open(f"{filename}.pkl","rb"))
            print(temp_list)

            self.scan_plot_w.checkbox_fft.setChecked(False)
            self.scan_coords = temp_list[:-1]
            self.freq_arr = np.array(temp_list[-1])
            window1d1 = sp.signal.windows.hamming(self.data.shape[0])
            window1d2 = sp.signal.windows.hamming(self.data.shape[1])
            # window2d = np.moveaxis(np.tile(np.moveaxis(np.tile(np.sqrt(np.outer(window1d1, window1d2)),(1,1,self.data.shape[2])),0,-1),(1,1,1,self.data.shape[3])),0,-1)
            window2d = np.sqrt(np.outer(window1d1, window1d2))
            data_temp = np.einsum("ij,ijkm->ijkm", window2d, self.data)
            data_temp = self.data
            self.data_fft = sp.fft.fftshift(
                sp.fft.fft2(data_temp, s=(self.data.shape[0] * self.fft_padding, self.data.shape[1] * self.fft_padding), axes=(0, 1)),
                axes=(0, 1))
            self.scan_plot_w.checkbox_fft.setEnabled(True)
            self.scan_plot_initialize()
            self.slice_plot_initialize()
            self.update_coord_slider()
            self.scan_plot_w.frequency_slider.setRange(0, len(self.freq_arr) - 1)
            self.scan_plot_w.setEnabled(True)
            self.slice_plot_w.setEnabled(True)
        except FileNotFoundError as err:
            print(err)

    def load_sim_data_file(self,folder="Farfield_w_band"):
        path = r"C:\Users\giguv\Desktop\skuggsja-scan\\"
        folder = self.file_widget.sim_textbox.text()
        files = glob.glob(path + folder + "/" + "*].txt")
        if len(files):
            self.sim_data_available = True
            files.sort(key=lambda x: float(x.split(" ")[1].split("=")[1][:-1]))
            self.sim_freq_list = [float(file.split(" ")[1].split("=")[1][:-1]) for file in files]
            self.sim_interp_list = []
            for file in files:
                farfield = np.loadtxt(file, unpack=1, skiprows=2)
                azimuth = np.rad2deg(np.arctan(np.cos(np.deg2rad(farfield[1])) * np.tan(np.deg2rad(farfield[0]))))
                elevation = np.rad2deg(np.arcsin(np.sin(np.deg2rad(farfield[1])) * np.sin(np.deg2rad(farfield[0]))))
                # azimuth = farfield[0]+90
                # elevation = farfield[1]
                data_mag = farfield[5]
                data_im = farfield[6]
                f_ind = np.argwhere((abs(farfield[0]) < 90) & (abs(farfield[1] < 90)))
                triang = mtri.Triangulation(azimuth[f_ind][:, 0], elevation[f_ind][:, 0])
                interp_cubic_geom_mag = mtri.CubicTriInterpolator(triang, data_mag[f_ind][:, 0], kind='geom')
                interp_cubic_geom_im = mtri.CubicTriInterpolator(triang, data_im[f_ind][:, 0], kind='geom')
                interp_cubic_geom = (lambda x,y,mag=interp_cubic_geom_mag,im=interp_cubic_geom_im:mag(x,y) * np.exp(np.deg2rad(im(x,y)) *1j))
                self.sim_interp_list.append(interp_cubic_geom)

            self.slice_plot_initialize()




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
                self.robot_busy = False
                self.connection_widget.con_robot_pushButton.setText("Disconnect robot")
                self.connection_widget.con_robot_pushButton.setStyleSheet("background-color: red")
                self.coord_update_timer.start()
                self.statusbar.setStyleSheet("background-color: green")
                self.set_robot_speed()

                self.robot_establish_connection()

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
            # self.worker.de
        for w in (self.manualCTab,self.scan_parameters_widget):
            w.setEnabled(self.robot_connected)

    def robot_establish_connection(self):
        self.worker_thread = QtCore.QThread()
        self.worker = RobotMovementObject()
        self.worker.moveToThread(self.worker_thread)
        self.connect_robot_signals()
        self.worker_thread.start()


    def connect_robot_signals(self):
        self.send_movement_coords_scan.connect(self.worker.move_robot_to_point_scan)
        self.send_movement_coords_scan_sph.connect(self.worker.move_robot_to_point_scan_sph)
        self.send_movement_coords_rel.connect(self.worker.move_robot_relative)
        self.send_movement_coords_abs.connect(self.worker.move_robot_to_coordinate)
        self.send_robot_to_scan_init.connect(self.worker.move_robot_to_init_scan)
        self.send_robot_to_init.connect(self.worker.move_robot_to_init)
        self.send_tool_plane.connect(self.worker.set_plane)
        self.send_tool_rotation.connect(self.worker.rotate_pose_tool)
        self.worker.arrived_at_point.connect(self.scan_data_add_point)
        self.worker.began_movement.connect(self.robot_movement_started)
        self.worker.finished_movement.connect(self.robot_movement_finished)
        self.worker.ready_for_acquisition.connect(self.trigger_VNA)
        self.worker.finished_scan.connect(self.robot_scan_finished)
        self.worker.monitor.kill_signal.connect(self.process_boundary_hit)
        # self.worker.monitor.kill_signal.connect(self.worker_thread.quit)
        self.send_settle_time.connect(self.worker.set_settle_time)
        self.send_acquisition_time.connect(self.worker.set_acquisition_time)
        self.send_robot_to_init.connect(self.worker.restart_after_killswitch)
        self.worker.monitor.restart_signal.connect(self.restart_after_hit)


    @QtCore.pyqtSlot()
    def robot_movement_started(self):
        self.statusbar.setStyleSheet("background-color: red")
        # print(self.instr.query_bool("*OPC?"))
        self.robot_busy = True
    @QtCore.pyqtSlot()
    def robot_movement_finished(self):
        self.statusbar.setStyleSheet("background-color: green")
        self.robot_busy = False

    @QtCore.pyqtSlot()
    def process_boundary_hit(self):
        print("main thread processed hit")
        self.worker.can_move = False
        self.robot_controls_widget.position_reset_pushButton.setText("Attempt to reconnect")
        self.scan_parameters_widget.scan_progressbar.setMaximum(0)
        self.statusbar.setStyleSheet("background-color: darkviolet")
        self.robot_busy = False
        # self.worker_thread.quit()
        # if self.robot_connected:
        #     self.robot_connect_button_clicked()

    @QtCore.pyqtSlot()
    def restart_after_hit(self):
        self.robot_controls_widget.position_reset_pushButton.setText("Reset robot position")
        self.statusbar.setStyleSheet("background-color: green")
        pass


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
        self.slice_plot_w.setEnabled(self.vna_connected)
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
    def plot_initialize(self, reset_trace_list = True):

        self.vna_plot_w.figure.clear()

        self.ax = self.vna_plot_w.figure.add_subplot(111)
        self.ax.set_xlabel('Frequency, GHz')
        self.ax.set_ylabel('S-parameters, dB')
        list_of_traces = self.instr.query_str_stripped("CONFigure:TRACe:CATalog?").split(',')
        list_of_trace_names = self.instr.query_str_stripped("CALCulate:PARameter:CATalog?").split(',')
        self.dict_of_trace_nums = dict(zip(list_of_traces[1::2],list_of_traces[::2]))
        self.dict_of_trace_meas = dict(zip(list_of_trace_names[::2], list_of_trace_names[1::2]))
        if reset_trace_list:
            self.configs.traces_to_show = list(self.dict_of_trace_nums.keys())
        # print(self.configs.traces_to_show)
        self.traces = []
        for trace in self.configs.traces_to_show:
            self.traces.append(self.ax.plot(self.freq_arr, self.query_data(self.dict_of_trace_nums[trace]), label=f"{self.dict_of_trace_meas[trace]}")[0])
        self.ax.legend()
        self.vna_plot_w.figure.tight_layout()

        if reset_trace_list:
            self.vna_plot_w.traces_toolmenu.clear()       # delete all items from comboBox
            for trace in list(self.dict_of_trace_meas):
                action = self.vna_plot_w.traces_toolmenu.addAction(trace+": "+self.dict_of_trace_meas[trace])
                action.setCheckable(True)
                action.setChecked(True)
                action.toggled.connect(self.change_traces_shown)

        self.freq_select_vline = self.ax.axvline(self.freq_arr[self.scan_plot_w.frequency_slider.value()],
                                                 visible = 1, color = "k")

        # refresh canvas
        self.vna_plot_w.canvas.draw()

    def plot_update(self):
        """Update the data trace shown on the plot with the one currently on the VNA screen"""
        for i, trace in enumerate(self.traces):
            trace.set_ydata(self.query_data(self.dict_of_trace_nums[self.configs.traces_to_show[i]]))

        self.vna_plot_w.canvas.draw()

    def change_traces_shown(self):
        self.configs.traces_to_show = []
        print("aaaa")
        for action in self.vna_plot_w.traces_toolmenu.actions():
            if action.isChecked():
                print(action)
                self.configs.traces_to_show.append(action.text().split(":")[0])
        self.plot_initialize(reset_trace_list=False)

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
            self.plot_initialize(reset_trace_list = False)
            self.plot_initialize(reset_trace_list = False)
        except ValueError as err:
            print('Inputted value is invalid\n'+err)

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
        self.update_estimated_scan_time()
        self.configs.save_toml("latest_settings.toml")

        self.freq_arr = msgtoarr((self.instr.query('CALC:DATA:STIM?'))) / 1e9
        self.scan_plot_w.frequency_slider.setRange(0,self.configs.VNA_settings["point_number"]-1)

    def update_estimated_scan_time(self):
        try:
            sweep_time = 0
            if self.vna_connected:
                sweep_time = self.instr.query_float('SWEep:TIMe?')
                self.scan_parameters_widget.acq_labelFramed.setText(f"{sweep_time:.2f}")
            points = np.prod([int(x[5].text()) for x in self.scan_parameters_widget.params_rows[2:]])
            total_time = (float(self.scan_parameters_widget.sttl_lineEdit.text())+sweep_time)*points
            self.scan_parameters_widget.total_labelFramed.setText(f"{total_time/60:.2f}")
        except ValueError:
            pass
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

    def scan_plot_initialize(self):
        if self.scan_plot_w.checkbox_fft.isChecked():
            data = self.data_fft
        else:
            data = self.data
        format = self.scan_plot_w.plot_format_combobox.currentText()
        formatted_data = self.scan_plot_w.plot_formats[format](data)
        self.scan_plot_w.ax_scan.clear()
        if hasattr(self,"meshplot"):
            self.meshplot = None
        slice_plane = self.scan_plot_w.slice_direction_combobox.currentText()
        coord_1 = self.scan_coords[self.coord_dict[slice_plane[0]]]
        coord_2 = self.scan_coords[self.coord_dict[slice_plane[1]]]
        self.scan_plot_w.ax_scan.set_xlabel(f"Position along ${slice_plane[0].lower()}$, mm")
        self.scan_plot_w.ax_scan.set_ylabel(f"Position along ${slice_plane[1].lower()}$, mm")
        if self.scan_type_combobox.currentText() == "Spherical scan":
            self.scan_plot_w.ax_scan.set_xlabel(f"Azimuth, degrees")
            self.scan_plot_w.ax_scan.set_ylabel(f"Elevation, degrees")
        scan_slice = self.return_scan_slice(slice_plane,self.scan_plot_w.coordinate_slider.value(),self.scan_plot_w.frequency_slider.value())
        self.meshplot = self.scan_plot_w.ax_scan.imshow(formatted_data[scan_slice].T, origin="lower",cmap = ("viridis" if format != "Phase" else "hsv"))
        # self.meshplot = self.scan_plot_w.ax_scan.imshow(formatted_data[scan_slice].T,origin ="lower",cmap="jet",vmax=0,vmin=-60)
        d1 = (coord_1.max()-coord_1.min())/(len(coord_1)-1)/2
        d2 = (coord_2.max() - coord_2.min()) / (len(coord_2)-1)/2
        extent = [coord_1.min()-d1,coord_1.max()+d1,coord_2.min()-d2,coord_2.max()+d2]
        self.meshplot.set_extent(extent)

        self.scan_plot_w.hline = self.scan_plot_w.ax_scan.axhline(0, visible=1, color="k")
        self.scan_plot_w.vline = self.scan_plot_w.ax_scan.axvline(0, visible=1, color="k")
        self.scan_plot_w.line_dict = {0:self.scan_plot_w.hline,1:self.scan_plot_w.vline}

        self.scan_plot_w.figure.tight_layout()
        self.scan_plot_w.canvas.draw()
        self.scan_plot_update()

    def return_scan_slice(self,slice_plane,c,f):
        if slice_plane == "XY":
            slice = np.index_exp[:,:,c,f]
        elif slice_plane == "XZ":
            slice = np.index_exp[:,c,:,f]
        elif slice_plane == "YZ":
            slice = np.index_exp[c,:,:,f]
        return slice

    def return_scan_slice_2(self, slice_plane, c):
        if slice_plane == "Horizontal":
            slice = np.index_exp[:, c]
        elif slice_plane == "Vertical":
            slice = np.index_exp[c, :]
        return slice

    def scan_plot_update(self):
        # formatted_data = self.scan_plot_w.plot_formats[self.scan_plot_w.plot_format_combobox.currentText()](self.data)
        if (self.scan_plot_w.checkbox_fft.isChecked()) or (self.scan_plot_w.checkbox_backpropagation.isChecked()):
            data = self.data_fft
        else:
            data = self.data
        scan_slice = self.return_scan_slice(self.scan_plot_w.slice_direction_combobox.currentText(),self.scan_plot_w.coordinate_slider.value(),self.scan_plot_w.frequency_slider.value())
        if self.scan_plot_w.checkbox_backpropagation.isChecked():
            k0 = self.freq_arr[self.scan_plot_w.frequency_slider.value()] * 1e9 * 2 * np.pi / const.speed_of_light
            kx = 2*np.pi*self.scan_coords[0]/(self.scan_coords[0].size)
            ky = 2*np.pi*self.scan_coords[1]/(self.scan_coords[1].size)
            kx, ky = np.meshgrid(kx,ky)
            kz = np.sqrt(k0**2-kx**2-ky**2)
            data_slice = data[scan_slice] * np.exp(1j * kz.T * self.scan_plot_w.backpropagation_distance_slider.value())\
                         # *\
                         # np.exp(-1j * (phase_x.T+phase_y.T))
            if not self.scan_plot_w.checkbox_fft.isChecked():
                data_slice = sp.fft.ifft2(sp.fft.fftshift(data_slice))
        else:
            data_slice = data[scan_slice]
        formatted_data = self.scan_plot_w.plot_formats[self.scan_plot_w.plot_format_combobox.currentText()](data_slice)
        self.meshplot.set_data(formatted_data.T)
        self.meshplot.autoscale()
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
        if self.vna_connected:
            self.freq_select_vline.set_xdata([self.new_frequency, self.new_frequency])
        # self.new_frequency_index = self.find_nearest_frequency_point(self.new_frequency)
        self.vna_plot_w.canvas.draw()
        self.scan_plot_update()
        self.slice_plot_update()

    def find_nearest_frequency_point(self, value):
        return np.argmin(np.abs(self.freq_arr - value))

    def coordinate_slider_changed(self):
        # print(self.textfield_frequency_slider.value())
        coord_arr = self.scan_coords[self.scan_plot_w.slice_directions[self.scan_plot_w.slice_direction_combobox.currentText()]]
        self.new_coordinate = coord_arr[self.scan_plot_w.coordinate_slider.value()]
        self.scan_plot_w.coordinate_textfield.setText(str(self.new_coordinate))
        # self.coord_select_vline.set_xdata([self.new_coordinate, self.new_coordinate])
        self.scan_plot_update()
        self.slice_plot_update()

    def coordinate_slider_changed_slice(self):
        dir = self.slice_plot_w.slice_directions[
                    self.slice_plot_w.slice_direction_combobox.currentText()]
        coord_arr = self.scan_coords[self.coord_dict[
            self.scan_plot_w.slice_direction_combobox.currentText()[not dir]]]
        self.new_coordinate = coord_arr[self.slice_plot_w.coordinate_slider.value()]
        self.slice_plot_w.coordinate_textfield.setText(str(self.new_coordinate))
        scan_line = self.scan_plot_w.line_dict[dir]
        (scan_line.set_ydata,scan_line.set_xdata)[dir]((self.new_coordinate,self.new_coordinate))
        self.slice_plot_update()
        self.scan_plot_w.canvas.draw()

    def update_coord_slider(self):
        self.scan_plot_w.coordinate_slider.setRange(0,
            len(self.scan_coords[self.scan_plot_w.slice_directions[self.scan_plot_w.slice_direction_combobox.currentText()]])-1)
        slice_range = len(self.scan_coords[self.coord_dict[
                self.scan_plot_w.slice_direction_combobox.currentText()[not
                    self.slice_plot_w.slice_directions[
                        self.slice_plot_w.slice_direction_combobox.currentText()]]]])-1
        self.slice_plot_w.coordinate_slider.setRange(0,slice_range)
        if self.vna_connected:
            self.scan_plot_initialize()
            self.slice_plot_initialize()

    def slice_plot_initialize(self):
        self.slice_plot_w.ax_slice.clear()
        self.slice_plot_w.ax_slice.set_xlabel('Coord')
        self.slice_plot_w.ax_slice.set_ylabel(self.scan_plot_w.plot_format_combobox.currentText())
        coords = self.scan_coords[self.coord_dict[
            self.scan_plot_w.slice_direction_combobox.currentText()[
            self.slice_plot_w.slice_directions[
                self.slice_plot_w.slice_direction_combobox.currentText()]]]]
        scan_slice = self.return_scan_slice(self.scan_plot_w.slice_direction_combobox.currentText(),
                                            self.scan_plot_w.coordinate_slider.value(),
                                            self.scan_plot_w.frequency_slider.value())
        scan_slice_2 = self.return_scan_slice_2(self.slice_plot_w.slice_direction_combobox.currentText(),
                                                min(self.slice_plot_w.coordinate_slider.value(),len(coords)-1))
        if self.scan_plot_w.checkbox_fft.isChecked():
            data = self.data_fft
            coords = np.linspace(coords.min(),coords.max(),coords.size*self.fft_padding)
        else:
            data = self.data
        format_function = self.scan_plot_w.plot_formats[self.scan_plot_w.plot_format_combobox.currentText()]
        formatted_data = format_function(data[scan_slice])
        self.sliceplot, = self.slice_plot_w.ax_slice.plot(coords,formatted_data[scan_slice_2],label = "Experimental")
        self.slice_plot_w.ax_slice.set_xlim(coords.min(),coords.max())
        if self.sim_data_available:
            self.interpolator = self.sim_interp_list[np.argmin(abs(self.sim_freq_list-self.freq_arr[scan_slice[3]]))]
            #TODO: fix for cartesian
            self.int_grid = np.meshgrid(self.scan_coords[1],self.scan_coords[0])
            data_sim = self.interpolator(self.int_grid[0][scan_slice_2],self.int_grid[1][scan_slice_2])
            self.int_norm = np.abs(self.interpolator(self.int_grid[1],self.int_grid[0])).max()
            data_sim = data_sim*np.abs(data[scan_slice]).max()/self.int_norm
            self.int_sliceplot, = self.slice_plot_w.ax_slice.plot(coords,format_function(data_sim),label = "Simulated")
        self.slice_plot_w.ax_slice.legend()


    def slice_plot_update(self):
        scan_slice = self.return_scan_slice(self.scan_plot_w.slice_direction_combobox.currentText(),
                                            self.scan_plot_w.coordinate_slider.value(),
                                            self.scan_plot_w.frequency_slider.value())
        scan_slice_2 = self.return_scan_slice_2(self.slice_plot_w.slice_direction_combobox.currentText(),
                                                self.slice_plot_w.coordinate_slider.value())
        if self.scan_plot_w.checkbox_fft.isChecked():
            data = self.data_fft
        else:
            data = self.data
        format_function = self.scan_plot_w.plot_formats[self.scan_plot_w.plot_format_combobox.currentText()]
        formatted_data = format_function(data[scan_slice])
        if len(formatted_data[scan_slice_2]) == len(self.sliceplot.get_ydata()):
            self.sliceplot.set_ydata(formatted_data[scan_slice_2])
            if self.sim_data_available:
                self.interpolator = self.sim_interp_list[
                    np.argmin(abs(self.sim_freq_list - self.freq_arr[scan_slice[3]]))]
                self.int_norm = np.abs(self.interpolator(self.int_grid[1], self.int_grid[0])).max()
                self.int_sliceplot.set_ydata(format_function(self.interpolator(self.int_grid[1][scan_slice_2],self.int_grid[0][scan_slice_2])*(np.abs(data[scan_slice]).max()/self.int_norm)))
            try:
                self.slice_plot_w.ax_slice.set_ylim(np.nanmin(formatted_data), np.nanmax(formatted_data))
            except ValueError:
                pass
            self.slice_plot_w.canvas.draw()
        else:
            self.slice_plot_initialize()


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

        self.scan_types = ["XYZ scan","Spherical scan"]
        self.scan_type_combobox = QtWidgets.QComboBox(parent=self)
        self.scan_type_combobox.addItems(self.scan_types)
        self.scan_type_combobox.currentTextChanged.connect(lambda: update_scan_type(self.scan_type_combobox.currentText()))

        self.xyzS_VLayout.addWidget(self.scan_type_combobox)
        spacerItem3 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Policy.Minimum,
                                            QtWidgets.QSizePolicy.Policy.Expanding)
        self.file_widget = DataFileWidget()
        self.xyzS_VLayout.addWidget(self.file_widget)
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

        self.slice_plot_w = SlicePlotWidget()
        self.sliceDockWidget = QtWidgets.QDockWidget(parent=MainWindow)
        self.sliceDockWidget.setWidget(self.slice_plot_w)
        MainWindow.addDockWidget(QtCore.Qt.DockWidgetArea(2), self.sliceDockWidget)
        self.sliceDockWidget.setWindowTitle("Field map slice")

        MainWindow.tabifyDockWidget(self.tracesDockWidget,self.scanDockWidget)
        MainWindow.tabifyDockWidget(self.scanDockWidget,self.sliceDockWidget)
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