import robodk.robolink
from robodk.robolink import *  # API to communicate with RoboDK
from robodk.robomath import *  # basic matrix operations
from PyQt6.QtWidgets import *
from PyQt6.QtCore import QTimer
import numpy as np
import threading
class RDK_KUKA(Robolink):
    def __init__(self, coordinates = None, joints = None, *args, **kargs):
        super(RDK_KUKA, self).__init__(*args, **kargs)
        self.AddFile("KUKA-KR-6-R900-2.robot")
        self.robot = self.ItemUserPick('KUKA KR 6 R900 2', ITEM_TYPE_ROBOT)
        self.robot.setSpeed(-1,20)  # Set linear speed in mm/s, joints speed in deg/s
        if coordinates != None or joints != None:
            if joints != None:
                self.robot.setJoints(joints)
            else:
                self.robot.setPose(KUKA_2_Pose(coordinates))
        self.Command("FitAll")
        self.AddTarget('Target initial')
        self.AddTarget('Target initial cross')
        self.target_init = self.Item('Target initial')
        self.target_init_cross = self.Item('Target initial cross')
        self.target_init_cross.setAsJointTarget()
        self.target_init.setAsJointTarget()
        if coordinates != None or joints != None:
            if joints != None:
                self.target_init.setJoints(joints)
            else:
                self.target_init.setPose(KUKA_2_Pose(coordinates))
        joints_cross = self.target_init.Joints()
        joints_cross[5,0] = joints_cross[5,0]+90
        self.target_init_cross.setJoints(joints_cross)
        self.AddTarget('Target manual')
        self.target_rel = self.Item('Target manual')

        # self.AddFrame('Frame scan initial',self.robot.Parent())
        # self.frame_scan_init = self.Item('Frame scan initial')
        # self.frame_scan_init.setPose(self.target_init.Pose())
        # self.robot.setPoseFrame(self.robot.Parent())
        # self.robot.setPoseTool(self.robot.PoseTool())

        self.AddTarget('Target scan initial',self.robot.Parent())
        self.target_scan_init = self.Item('Target scan initial')
        # self.target_scan_init.setPose(self.frame_scan_init.Pose())
        self.AddTarget('Target scan',self.robot.Parent())
        self.target_scan = self.Item('Target scan')
        # self.target_scan.setPose(self.frame_scan_init.Pose())

        self.robot.MoveJ(self.target_init)


    def move_target(self, target, coordinate_tuple):
        if len(coordinate_tuple) == 3:
            target.setPose(target.Pose().setPos(coordinate_tuple))
        elif len(coordinate_tuple) == 6:
            target.setPose(KUKA_2_Pose(coordinate_tuple))

    def set_initial(self,coordinate_tuple):
        self.move_target(self.target_init,coordinate_tuple)

    def move_to_initial(self):
        self.robot.MoveJ(self.target_init.Pose())
        self.target_rel.setPose(self.target_init.Pose())

    def move_relative(self, coordinate_tuple):
        current = Pose_2_KUKA(self.target_rel.Pose())
        self.move_target(self.target_rel,[current[i] + x for i, x in enumerate(coordinate_tuple)])
        if len(self.robot.SolveIK(self.target_rel.Pose()).tolist()) < 6:
            self.move_target(self.target_rel, current)
        else:
            self.robot.MoveJ(self.target_rel.Pose())
        return Pose_2_KUKA(self.robot.Pose())

    def set_scan_initial(self):
        # self.frame_scan_init.setPose(self.target_rel.Pose())
        self.target_scan_init.setPose(self.target_rel.Pose())
        return Pose_2_KUKA(self.target_scan_init.Pose())

    def move_scan_target(self, target, coordinate_tuple):
        self.robot.setPoseFrame(self.frame_scan_init)
        self.move_target(target, coordinate_tuple)
        self.robot.setPoseFrame(self.robot.Parent())

        
    def move_line(self,x1,x2):
        None
        # self.robot.MoveJ(target)

    def run_on_robot(self,ip,port):
        if self.RunMode() != RUNMODE_RUN_ROBOT:
            # Update connection parameters if required:
            self.robot.setConnectionParams(ip,port,'/', 'anonymous','')

            # Connect to the robot using default IP
            success = self.robot.Connect()  # Try to connect once
            # success robot.ConnectSafe() # Try to connect multiple times
            status, status_msg = self.robot.ConnectedState()
            if status != ROBOTCOM_READY:
                # Stop if the connection did not succeed
                print(status_msg)
                print("Failed to connect: " + status_msg)
                # raise Exception("Failed to connect: " + status_msg)

            # This will set to run the API programs on the robot and the simulator (online programming)
            self.setRunMode(RUNMODE_RUN_ROBOT)


class x_slider(QWidget):
    def __init__(self, rdk):
        super(x_slider, self).__init__()
        layout = QHBoxLayout()
        self.slider = QSlider(self)
        self.button = QPushButton(self)
        layout.addWidget(self.slider)
        layout.addWidget(self.button)
        self.button.setText("-->")
        self.setLayout(layout)
        self.r = rdk
        self.slider.valueChanged.connect(self.slider_changed)
        self.timer = QTimer()
        self.timer.setInterval(10)
        self.timer.timeout.connect(lambda: self.r.move_relative((0,0,0,1,0,0)))
        self.button.pressed.connect(self.timer.start)
        self.button.released.connect(self.timer.stop)

    def slider_changed(self):
        new = self.r.xyz_ref
        new[2] = self.slider.value()+700
        target_pose = self.r.Item('Target 1').Pose()
        target_pose.setPos(new)
        self.r.robot.MoveJ(target_pose)

if __name__ == '__main__':
    # rob = RDK_KUKA((550,-300,780))
    rob = RDK_KUKA()
    # rob.set_initial((550,-300,880))
    # rob.robot.MoveJ(rob.Item('Target 1'))

    if not QApplication.instance():
        app = QApplication(sys.argv)
    else:
        app = QApplication.instance()
    window = x_slider(rob)
    window.show()

    sys.exit(app.exec())
