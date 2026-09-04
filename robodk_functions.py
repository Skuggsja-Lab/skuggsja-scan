import robodk.robolink
from robodk.robolink import *  # API to communicate with RoboDK
from robodk.robomath import *  # basic matrix operations
from PyQt6.QtWidgets import *
from PyQt6.QtCore import QTimer
import numpy as np
import threading
class RDK_KUKA(Robolink):
    TIMEOUT = 60
    # NODELAY = True
    def __init__(self, coordinates = None, joints = None, obstacles=None,sample_holder_args = None, *args, **kargs):
        super(RDK_KUKA, self).__init__(*args, **kargs)
        self._setTimeout(60)
        self.sample_holder_in_workpace = False
        self.robot = self.Item('KUKA KR 6 R900-2', ITEM_TYPE_ROBOT)
        if not self.robot.Valid():
            self.AddFile("KUKA-KR-6-R900-2.robot")
            # self.AddFile("KUKA-KR-6-R700-sixx.robot")
            self.AddFile("w_band_mount.tool")
            self.robot = self.Item('KUKA KR 6 R900-2', ITEM_TYPE_ROBOT)
            # self.robot = self.ItemUserPick('KUKA KR 6 R700 sixx', ITEM_TYPE_ROBOT)
            # self.robot.setPoseTool(self.robot.PoseTool()*rotz(pi))
            # self.robot.setPoseTool(self.robot.PoseTool() * rotz(-pi/2))
            self.robot.setVisible(1, VISIBLE_ROBOT_DEFAULT and not VISIBLE_ROBOT_FLANGE)
            self.robot.setSpeed(-1,20)  # Set linear speed in mm/s, joints speed in deg/s
            if coordinates != None or joints != None:
                if joints != None:
                    self.robot.setJoints(joints)
                else:
                    self.robot.setPose(KUKA_2_Pose(coordinates))
            self.Command("FitAll")
            self.AddTarget('Target initial')
            self.AddTarget('Target initial cross')
            self.AddTarget('Target manual')
            self.AddTarget('Target scan initial',self.robot.Parent())
            self.AddTarget('Target scan',self.robot.Parent())

            if obstacles is not None:
                for obstacle in obstacles.keys():
                    new_item = self.AddFile(obstacle)
                    new_item.setPose(KUKA_2_Pose(obstacles[obstacle]))

        self.default_pose_tool = self.robot.PoseTool()
        self.tool = self.Item("w_band_mount", ITEM_TYPE_TOOL)
        self.robot.setPoseFrame(self.robot.Parent())
        self.setCollisionActivePair(COLLISION_OFF, self.tool, self.robot.ObjectLink(6))
        # self.setCollisionActive(COLLISION_ON)
        # self.AddFile("opticbench.STEP")

        self.target_init = self.Item('Target initial')
        self.target_init_cross = self.Item('Target initial cross')
        if coordinates != None or joints != None:
            if joints != None:
                self.target_init.setJoints(joints)
            else:
                self.target_init.setPose(KUKA_2_Pose(coordinates))
        joints_cross = self.target_init.Joints()
        joints_cross[5,0] = joints_cross[5,0]+90
        self.target_init_cross.setJoints(joints_cross)
        self.target_rel = self.Item('Target manual')

        self.target_init_cross.setAsJointTarget()
        self.target_init.setAsJointTarget()
        # self.AddFrame('Frame scan initial',self.robot.Parent())
        # self.frame_scan_init = self.Item('Frame scan initial')
        # self.frame_scan_init.setPose(self.target_init.Pose())
        # self.robot.setPoseFrame(self.robot.Parent())
        # self.robot.setPoseTool(self.robot.PoseTool())

        self.target_scan_init = self.Item('Target scan initial')
        # self.target_scan_init.setPose(self.frame_scan_init.Pose())
        self.target_scan = self.Item('Target scan')
        for target in [self.target_init,self.target_init_cross,
                       self.target_rel,
                       self.target_scan_init,self.target_scan]:
            target.setRobot(self.robot)
        # self.target_scan.setPose(self.frame_scan_init.Pose())
        if sample_holder_args != None:
            self.sample_holder_args = sample_holder_args
            if sample_holder_args["on_init"]:
                self.add_sample_holder(**sample_holder_args)
        # self.robot.MoveJ(self.target_init)
    def add_sample_holder(self,rel_position,robot = "Mecademic-Meca500-R3.robot", sample = None, joints = None,joints_away = None, **kargs):
        self.sample_holder = self.Item(" ".join(robot.split(".")[0].split("-")), ITEM_TYPE_ROBOT)
        if not self.sample_holder.Valid():
            self.AddFile(robot)
            if sample != None:
                self.AddFile(sample)
            self.sample_holder = self.Item(" ".join(robot.split(".")[0].split("-")), ITEM_TYPE_ROBOT)
            # self.sample_holder.setPoseTool(self.robot.PoseTool() * rotz(pi))
            self.sample_holder.setVisible(1, VISIBLE_ROBOT_DEFAULT and not VISIBLE_ROBOT_FLANGE)
            self.sample_holder.Parent().setPose(Staubli_2_Pose(rel_position))
            self.AddFile("frame_tool.tool",self.sample_holder)
            self.sample_holder.setPoseFrame(self.sample_holder.Parent())

            if joints != None:
                self.sample_holder.setJoints(joints)

            self.AddTarget('Target holder initial',self.sample_holder.Parent())
            self.AddTarget('Target holder manual', self.sample_holder.Parent())
            self.AddTarget('Target holder away', self.sample_holder.Parent())
            self.AddTarget('Target holder rotation', self.sample_holder.Parent())
            self.AddTarget('Target holder rotation initial', self.sample_holder.Parent())

        self.target_holder_init = self.Item('Target holder initial')
        self.target_holder_init.setRobot(self.sample_holder)
        self.target_holder_init.setAsJointTarget()
        self.target_holder_init.setJoints(joints)
        self.target_holder_init.setPose(self.sample_holder.SolveFK(joints))
        self.target_holder_away = self.Item('Target holder away')
        self.target_holder_away.setRobot(self.sample_holder)
        self.target_holder_away.setAsJointTarget()
        self.target_holder_away.setJoints(joints_away)
        self.target_holder_away.setPose(self.sample_holder.SolveFK(joints_away))
        self.target_holder_rel = self.Item('Target holder manual')
        self.target_holder_rel.setRobot(self.sample_holder)
        self.target_holder_rel.setPose(self.sample_holder.Pose())
        self.target_holder_scan = self.Item('Target holder rotation')
        self.target_holder_scan.setRobot(self.sample_holder)
        self.target_holder_scan.setPose(self.sample_holder.Pose())
        self.target_holder_scan_init = self.Item('Target holder rotation initial')
        self.target_holder_scan_init.setRobot(self.sample_holder)
        self.target_holder_scan_init.setPose(self.sample_holder.Pose())
        self.sample_holder_in_workpace = True



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
            if self.sample_holder_in_workpace:
                self.sample_holder.setConnectionParams(self.sample_holder_args["ip"],port,'/', 'anonymous','')
                success_sh = self.sample_holder.Connect()
                status_sh, status_sh_msg = self.robot.ConnectedState()
                if status_sh != ROBOTCOM_READY:
                    # Stop if the connection did not succeed
                    print(status_sh_msg)
                    print("Failed to connect: " + status_sh_msg)
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
