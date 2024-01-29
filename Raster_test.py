# Draw a hexagon around Target 1
from robodk.robolink import *  # API to communicate with RoboDK
from robodk.robomath import *  # basic matrix operations
import numpy as np
from datetime import datetime
import os

# Start the RoboDK API:
RDK = Robolink()

# Get the robot (first robot found):
robot = RDK.Item('KUKA KR 6 R900-2', ITEM_TYPE_ROBOT)
if not robot.Valid():
    raise Exception('No robot selected or available')

# Get the reference target by name:
target = RDK.Item('Target 1')
target_pose = target.Pose()
xyz_ref = target_pose.Pos()

# Provide the reference and the tool frames 
robot.setPoseFrame(robot.PoseFrame())
robot.setPoseTool(robot.PoseTool())
robot.setSpeed(-1,20)  # Set linear speed in mm/s, joints speed in deg/s

# Move the robot to the reference point:
robot.MoveJ(target)

# Shift down the TCP
Z_shift = -300
x = xyz_ref[0]
y = xyz_ref[1]
z = xyz_ref[2] + Z_shift

target_pose.setPos([x,y,z])
new_target_pose = target.Pose()
robot.MoveJ(target_pose)

joints = robot.Joints()
pose = robot.Pose()

# Compute error on position
target_pose_K = Pose_2_KUKA(target_pose)
target_pose_arr = np.asarray(target_pose_K)
pose = Pose_2_KUKA(pose)
pose_arr = np.asarray(pose)
delta_pose_arr = target_pose_arr - pose_arr

jlst = joints.list()
now = datetime.now()
current_time = now.strftime("%d%m%Y_%H:%M:%S:%f")
RDK.ShowMessage(str(jlst) + os.linesep + str(pose) + os.linesep + str(delta_pose_arr), popup = False)

# Reset ref pose for scanning
xyz_new_ref = target_pose.Pos()

# Raster scanning
N_line = 10
N_points = 10

# X_world = Horizontal axis
# Y_world = Transverse Horizontal
# Z_world = Transverse Vertical

Y_half_span = 450
dY = 2*Y_half_span/N_line
Z_half_span = 450
dZ = 2*Z_half_span/N_points

Y_add = np.arange(-Y_half_span, Y_half_span + dY, dY)
Z_add = np.arange(-Z_half_span, Z_half_span + dZ, dZ)

for n_l in range(N_line+1):
    for n_p in range(N_points+1):
        # Calculate the new position:
        x = xyz_new_ref[0]
        y = xyz_new_ref[1] + Y_add[n_l]
        z = xyz_new_ref[2] + ((-1)**n_l)*Z_add[n_p]
        target_pose.setPos([x,y,z])

        # Move to the new target:
        robot.MoveJ(target_pose)
        joints = robot.Joints()
        pose = robot.Pose()
        
        # Compute error on position
        target_pose_K = Pose_2_KUKA(target_pose)
        target_pose_arr = np.asarray(target_pose_K)
        pose = Pose_2_KUKA(pose)
        pose_arr = np.asarray(pose)
        delta_pose_arr = target_pose_arr - pose_arr
        
        jlst = joints.list()
        now = datetime.now()
        current_time = now.strftime("%d%m%Y_%H:%M:%S:%f")
        RDK.ShowMessage(str(jlst) + os.linesep + str(pose) + os.linesep + str(delta_pose_arr), popup = False)

# Trigger a program call at the end of the movement
# robot.RunInstruction('Program_Done')

# Move back to the reference target:
robot.MoveJ(target)
joints = robot.Joints()
pose = robot.Pose()

# Compute error on position
target_pose_K = Pose_2_KUKA(target_pose)
target_pose_arr = np.asarray(target_pose_K)
pose = Pose_2_KUKA(pose)
pose_arr = np.asarray(pose)
delta_pose_arr = target_pose_arr - pose_arr

jlst = joints.list()
now = datetime.now()
current_time = now.strftime("%d%m%Y_%H:%M:%S:%f")
RDK.ShowMessage(str(jlst) + os.linesep + str(pose) + os.linesep + str(delta_pose_arr), popup = False)

RDK.ShowMessage("Done!", popup = True)
