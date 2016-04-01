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
  #print "============ Robot Initial Pose [position orientation]: %s" % group.get_current_pose().pose
  #print "============ Waiting for RVIZ..."
  rospy.sleep(10)

  print "============ MOTION 1 Planning to JOINT-space goal: DEFAULT POSITION  "
  group_variable_values = group.get_current_joint_values()
  group_variable_values = [3.0662607633948866, 0.5601688277178765, -0.901226487118465, 2.4373301489729045, 2.891868967060404]
  group.set_joint_value_target(group_variable_values)
  plan_default_position = group.plan()
  print "============ Waiting while RVIZ displays MOTION 1, DEFAULT POSITION "
  group.go(wait=True)
  rospy.sleep(5)
  print "============ Waiting while Gazebo execute MOTION 1, DEFAULT POSITION"
  rospy.sleep(40)
  print "============ Robot reached Joint Values: %s" % group.get_current_joint_values()
  print "============ Robot reached Pose [position orientation]: %s" % group.get_current_pose().pose
  #Prendo la default_position_target per avere un target di planning to a Pose Goal
  default_position_target = group.get_current_pose().pose

  print "============ MOTION 2 Planning to JOINT-space goal: [1,1.5,-2.0,0.0,4.0]"
  #group_variable_values = group.get_current_joint_values()
  group_variable_values = [1,1.5,-2.0,0.0,4.0]
  group.set_joint_value_target(group_variable_values)
  plan_motion2 = group.plan()
  print "============ Waiting while RVIZ displays MOTION 2, [1,1.5,-2.0,0.0,4.0]"
  rospy.sleep(5)
  ## You can ask RVIZ to visualize a plan (aka trajectory) for you.  But the group.plan() method does this automatically so this is not that useful here (it just displays the same trajectory again).
  print "============ VISUALIZING MOTION 2, [1,1.5,-2.0,0.0,4.0]"
  display_trajectory = moveit_msgs.msg.DisplayTrajectory()
  display_trajectory.trajectory_start = robot.get_current_state()
  display_trajectory.trajectory.append(plan_motion2)
  display_trajectory_publisher.publish(display_trajectory);
  print "============ Waiting while MOTION 2, [1,1.5,-2.0,0.0,4.0] is visualized (again)..."
  rospy.sleep(5)
  group.go(wait=True)
  print "============ Waiting while Gazebo execute MOTION 2, [1,1.5,-2.0,0.0,4.0]"
  rospy.sleep(40)
  print "============ Robot reached Joint Values: %s" % group.get_current_joint_values()
  #print "============ Robot reached Pose [position orientation]: %s" % group.get_current_pose().pose

  print "============ MOTION 3 Planning to CARTESIAN PATHS"
  waypoints = []
  waypoints.append(group.get_current_pose().pose)
  wpose = geometry_msgs.msg.Pose()
  wpose.orientation.w = 1.0
  wpose.position.x = waypoints[0].position.x + 0.1
  wpose.position.y = waypoints[0].position.y
  wpose.position.z = waypoints[0].position.z
  waypoints.append(copy.deepcopy(wpose))
  # second move down
  wpose.position.z -= 0.10
  waypoints.append(copy.deepcopy(wpose))
  # third move to the side
  wpose.position.y += 0.05
  waypoints.append(copy.deepcopy(wpose))
  ## We want the cartesian path to be interpolated at a resolution of 1 cm which is why we will specify 0.01 as the eef_step in cartesian translation.  We will specify the jump threshold as 0.0, effectively disabling it.
  (plan_cartesian_path, fraction) = group.compute_cartesian_path(
                               waypoints,   # waypoints to follow
                               0.01,        # eef_step
                               0.0)         # jump_threshold
  print "============ Waiting while RVIZ displays MOTION 3, CARTESIAN PATHS"
  rospy.sleep(5)
  group.go(wait=True)
  print "============ Waiting while Gazebo execute MOTION 3, CARTESIAN PATHS"
  rospy.sleep(40)
  print "============ Robot reached Joint Values: %s" % group.get_current_joint_values()
  #print "============ Robot reached Pose [position orientation]: %s" % group.get_current_pose().pose

  print "============ MOTION 4 Planning to a POSE Goal: set_random_target()  "
  group.clear_pose_targets()
  pose_target = geometry_msgs.msg.Pose()
  group.set_random_target()
  plan_random = group.plan()
  print "============ Waiting while RVIZ displays MOTION 4, POSE Goal: set_random_target()"
  rospy.sleep(5)
  group.go(wait=True)
  print "============ Waiting while Gazebo execute MOTION 4, CARTESIAN PATHS"
  rospy.sleep(40)
  
  print "============  FOR Cycle Planning to a POSE Goal: set_random_target()  "
  for i in range(3):
  	group.clear_pose_targets()
  	pose_target = geometry_msgs.msg.Pose()
  	group.set_random_target()
  	plan_random = group.plan()
        group.go(wait=True)
  	rospy.sleep(20)
  	print "============ Ciclo numero: %d Robot reached Joint Values: %s" % ( i , group.get_current_joint_values())
  	#print "============ Ciclo numero: %d Robot reached Pose [position orientation]: %s" % ( i , group.get_current_pose().pose )   
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
