#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String


def move_group_python_interface_tutorial():
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)
  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  group = moveit_commander.MoveGroupCommander("arm_1")
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)

  ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
  print "============ Robot Initial Joint Values: %s" % group.get_current_joint_values()
  print "============ Robot Initial Pose [position orientation]: %s" % group.get_current_pose().pose
  #print "============ Waiting for RVIZ..."
  rospy.sleep(10)

  print "============ MOTION 3 Planning to: set_random_target()  "
  group.clear_pose_targets()
  pose_target = geometry_msgs.msg.Pose()
  group.set_random_target()
  plan_random = group.plan()
  group.go(wait=True)
  rospy.sleep(20)
  print "============ Robot reached Joint Values: %s" % group.get_current_joint_values()
  print "============ Robot reached Pose [position orientation]: %s" % group.get_current_pose().pose
  
  print "============  FOR Cycle Planning to: set_random_target()  "
  for i in range(6):
  	group.clear_pose_targets()
  	pose_target = geometry_msgs.msg.Pose()
  	group.set_random_target()
  	plan_random = group.plan()
        print "============ la linea gialla stampata prima di questa? ok per simulare su robot"
        group.go(wait=True)
        print "============ la linea gialla stampata prima di questa? male per simulare su robot"
  	rospy.sleep(20)
  	print "============ Ciclo numero: %d Robot reached Joint Values: %s" % ( i , group.get_current_joint_values())
  	print "============ Ciclo numero: %d Robot reached Pose [position orientation]: %s" % ( i , group.get_current_pose().pose )   
  else:
  	## When finished shut down moveit_commander.
  	moveit_commander.roscpp_shutdown()
        ## END_TUTORIAL
	print "============ STOPPING"




if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass
