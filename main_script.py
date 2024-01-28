from PyQt6 import QtCore, QtGui, QtWidgets
import sys
import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure

class QLabelFramed(QtWidgets.QLabel):
    def __init__(self, parent=None):
        super(QLabelFramed, self).__init__(parent)
        self.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.setFrameShadow(QtWidgets.QFrame.Shadow.Plain)
        self.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)

class RobotStopButton(QtWidgets.QPushButton):
    def __init__(self, parent=None, robot_IP = "255.255.0.0"):
        super(RobotStopButton, self).__init__(parent)
        self.setText("STOP")
        self.setMaximumWidth(100)
        self.setStyleSheet("font-weight: bold")
        self.clicked.connect(lambda: self.robot_stop_button(robot_IP))

    def robot_stop_button(self, robot_IP):
        print(f"Stop command sent to robot arm at {robot_IP}")

class ConnectionWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(ConnectionWidget, self).__init__(parent)
        self.con_params_gridLayout = QtWidgets.QGridLayout(self)

        self.robot_port_lineEdit = QtWidgets.QLineEdit()
        self.cont_vna_pushButton = QtWidgets.QPushButton()
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

        self.robot_ip_label.setText("Robot IP")
        self.robot_port_label.setText("Robot Port")
        self.vna_ip_label.setText("VNA IP")
        self.vna_port_label.setText("VNA Port")
        self.ping_robot_pushButton.setText("Ping Robot")
        self.ping_vna_pushButton.setText("Ping VNA")
        self.cont_vna_pushButton.setText("Connect VNA")
        self.con_robot_pushButton.setText("Connect Robot")
        self.disconnect_pushButton.setText("Disconnect")

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
        self.con_params_gridLayout.addWidget(self.cont_vna_pushButton, 5, 1, 1, 1)
        self.con_params_gridLayout.addWidget(self.disconnect_pushButton, 6, 1, 1, 1)

        self.con_params_gridLayout.addItem(QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Policy.Minimum,
                                                                 QtWidgets.QSizePolicy.Policy.Expanding), 7, 1, 1, 1)

class ManualControlWidget(QtWidgets.QWidget):
    def __init__(self, parent=None, coordinate = '_'):
        super(ManualControlWidget, self).__init__(parent)
        layout = QtWidgets.QGridLayout()

        self.radioButton = QtWidgets.QRadioButton(parent=parent)
        self.step_label = QtWidgets.QLabel(parent=parent)
        self.step_textEdit = QtWidgets.QLineEdit(parent=parent)
        self.step_textEdit.setMaximumSize(QtCore.QSize(40, 30))
        self.horizontalSlider = QtWidgets.QSlider(parent=parent)
        self.horizontalSlider.setMaximum(100)
        self.horizontalSlider.setProperty("value", 50)
        self.horizontalSlider.setOrientation(QtCore.Qt.Orientation.Horizontal)
        self.horizontalSlider.setTickPosition(QtWidgets.QSlider.TickPosition.TicksBelow)
        self.man_label = QtWidgets.QLabel(parent=parent)
        self.man_textEdit = QtWidgets.QLineEdit(parent=parent)
        self.man_textEdit.setMaximumSize(QtCore.QSize(60, 30))
        self.pos_fdbk_label = QLabelFramed(parent=parent)
        self.dlabel = QtWidgets.QLabel(parent=parent)
        self.dfdbk_label = QtWidgets.QLabel(parent=parent)
        self.dfdbk_label.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.dfdbk_label.setFrameShadow(QtWidgets.QFrame.Shadow.Plain)
        self.dfdbk_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)

        self.step_label.setText(f"s{coordinate}")
        self.man_label.setText(f"{coordinate}man")
        self.pos_fdbk_label.setText("TextLabel")
        self.dfdbk_label.setText("TextLabel")
        self.radioButton.setText(coordinate)
        self.dlabel.setText("d"+coordinate)

        layout.addWidget(self.radioButton, 0, 0, 1, 1)
        layout.addWidget(self.step_label, 0, 1, 1, 1)
        layout.addWidget(self.step_textEdit, 0, 2, 1, 1)
        layout.addWidget(self.horizontalSlider, 0, 3, 1, 1)
        layout.addWidget(self.man_label, 0, 4, 1, 1)
        layout.addWidget(self.man_textEdit, 0, 5, 1, 1)
        layout.addWidget(self.pos_fdbk_label, 0, 6, 1, 1)
        layout.addWidget(self.dlabel, 0, 7, 1, 1)
        layout.addWidget(self.dfdbk_label, 0, 8, 1, 1)

        self.radioButton.toggled.connect(self.radio_button_check)
        self.man_textEdit.returnPressed.connect(lambda: self.set_slider(self.man_textEdit, self.horizontalSlider))
        self.horizontalSlider.valueChanged.connect(self.slider_changed)
        # self.man_textEdit.textChanged.connect(lambda : print(self.man_textEdit.text()))

        self.setLayout(layout)

        self.widget_touple = (self.radioButton,
        self.step_label,
        self.step_textEdit,
        self.horizontalSlider,
        self.man_label,
        self.man_textEdit,
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
        self.man_textEdit.setText(str(self.new_val))

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

        self.scan_params_VLayout.addWidget(self.scan_params_label)
        self.scan_params_VLayout.addLayout(self.scan_params_gridLayout)
        size_policy = QtWidgets.QSizePolicy()
        self.setSizePolicy(size_policy)

        self.set_scan(type)

    def set_scan(self,scan_type):
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
                w.setText(["{}max", "{}min", "{}step"][iw].format(coordinates[ir]))

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
        self.ifbw_label.setText("IF BW")
        self.ifbw_lineEdit = QtWidgets.QLineEdit()
        self.fmin_label = QtWidgets.QLabel()
        self.fmin_label.setText("f min")
        self.fmin_lineEdit = QtWidgets.QLineEdit()
        self.fmax_label = QtWidgets.QLabel()
        self.fmax_label.setText("f max")
        self.fmax_lineEdit = QtWidgets.QLineEdit()
        self.fstp_label = QtWidgets.QLabel()
        self.fstp_label.setText("f step")
        self.fstp_lineEdit = QtWidgets.QLineEdit()
        self.attn_label = QtWidgets.QLabel()
        self.attn_label.setText("Attenuation")
        self.attn_lineEdit = QtWidgets.QLineEdit()
        self.sttl_label = QtWidgets.QLabel()
        self.sttl_label.setText("Settling time")
        self.sttl_lineEdit = QtWidgets.QLineEdit()
        self.ab_checkBox = QtWidgets.QCheckBox()
        self.ab_checkBox.setText("A/B")
        self.fwbw_checkBox = QtWidgets.QCheckBox()
        self.fwbw_checkBox.setText("Frwd/Bckwd")
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
        self.gridLayout.addWidget(self.attn_label, 1, 0, 1, 1)
        self.gridLayout.addWidget(self.attn_lineEdit, 1, 1, 1, 1)
        self.gridLayout.addWidget(self.sttl_label, 1, 2, 1, 1)
        self.gridLayout.addWidget(self.sttl_lineEdit, 1, 3, 1, 1)
        self.gridLayout.addWidget(self.ab_checkBox, 1, 4, 1, 1)
        self.gridLayout.addWidget(self.fwbw_checkBox, 1, 5, 1, 1)
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
        self.combobox_plot_format = QtWidgets.QComboBox()
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

        self.gridLayout.addWidget(self.combobox_plot_format, 1, 0, 1, 2)
        self.gridLayout.addWidget(self.checkbox_scatter, 1, 2, 1, 1)
        self.gridLayout.addWidget(self.coordinate_label, 3, 0, 1, 1)
        self.gridLayout.addWidget(self.coordinate_textfield, 3, 1, 1, 1)
        self.gridLayout.addWidget(self.coordinate_slider, 3, 2, 1, 1)
        self.gridLayout.addWidget(self.frequency_label, 2, 0, 1, 1)
        self.gridLayout.addWidget(self.frequency_textfield, 2, 1, 1, 1)
        self.gridLayout.addWidget(self.frequency_slider, 2, 2, 1, 1)

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


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, *args, obj=None, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.setupUi(self)

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

        coordinates_list = ["X", "Y", "Z", "A", "B", "C", "T"]
        self.coordinates_widgets_list = []
        for ic, coord in enumerate(coordinates_list):
            setattr(self, f"manual_control_{coord}", ManualControlWidget(parent = manualCTab, coordinate= coord))
            self.coordinates_widgets_list.append(eval(f"self.manual_control_{coord}"))

        for ic, coord in enumerate(coordinates_list):
            for iw, widget in enumerate(self.coordinates_widgets_list[ic].getwidgets()):
                self.man_move_gridLayout.addWidget(widget, 1+ic,iw,1,1)

        self.man_select_label = QtWidgets.QLabel(parent=manualCTab)
        self.man_select_label.setStyleSheet("font-weight: bold")
        self.man_move_gridLayout.addWidget(self.man_select_label, 0, 0, 1, 1)
        self.man_select_label.setMaximumHeight(30)
        self.man_step_label = QtWidgets.QLabel(parent=manualCTab)
        self.man_step_label.setStyleSheet("font-weight: bold")
        self.man_move_gridLayout.addWidget(self.man_step_label, 0, 2, 1, 1)
        self.man_pos_label = QtWidgets.QLabel(parent=manualCTab)
        self.man_pos_label.setStyleSheet("font-weight: bold")
        self.man_move_gridLayout.addWidget(self.man_pos_label, 0, 3, 1, 1)
        self.man_input_label = QtWidgets.QLabel(parent=manualCTab)
        self.man_input_label.setStyleSheet("font-weight: bold")
        self.man_move_gridLayout.addWidget(self.man_input_label, 0, 5, 1, 1)
        self.man_pos_fdbk_label = QtWidgets.QLabel(parent=manualCTab)
        self.man_pos_fdbk_label.setStyleSheet("font-weight: bold")
        self.man_move_gridLayout.addWidget(self.man_pos_fdbk_label, 0, 6, 1, 1)
        self.man_pos_real_fdbk_label = QtWidgets.QLabel(parent=manualCTab)
        self.man_pos_real_fdbk_label.setStyleSheet("font-weight: bold")
        self.man_move_gridLayout.addWidget(self.man_pos_real_fdbk_label, 0, 8, 1, 1)

        self.man_step_label.setText("Step size")
        self.man_pos_label.setText("Position Selection")
        self.man_select_label.setText("Movement Selection")
        self.man_pos_fdbk_label.setText("Position feedback")
        self.man_pos_real_fdbk_label.setText("Position accuracy")
        self.man_input_label.setText("Manual In")


        self.man_main_VLayout.addLayout(self.man_move_gridLayout)
        self.man_stop_pushButton = RobotStopButton(parent=manualCTab)
        self.man_main_VLayout.addWidget(self.man_stop_pushButton)

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


        self.tracesDockWidget = QtWidgets.QDockWidget(parent=MainWindow)
        self.tracesDockWidget.setWidget(VNAPlotWidget())
        MainWindow.addDockWidget(QtCore.Qt.DockWidgetArea(2), self.tracesDockWidget)
        self.tracesDockWidget.setWindowTitle("VNA traces")

        self.scanDockWidget = QtWidgets.QDockWidget(parent=MainWindow)
        self.scanDockWidget.setWidget(ScanPlotWidget())
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

        self.menuSettings.addAction(self.actionReset)
        self.menuSettings.addAction(self.actionClose)
        self.menuHelp.addAction(self.actionAbout)
        self.menubar.addAction(self.menuSettings.menuAction())
        self.menubar.addAction(self.menuHelp.menuAction())

        MainWindow.setWindowTitle("Main Window")
        self.mainTabWidget.setCurrentIndex(0)


if __name__ == '__main__':
    if not QtWidgets.QApplication.instance():
        app = QtWidgets.QApplication(sys.argv)
    else:
        app = QtWidgets.QApplication.instance()
    window = MainWindow()
    window.show()
    sys.exit(app.exec())