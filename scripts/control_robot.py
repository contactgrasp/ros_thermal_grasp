#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import moveit_msgs.msg as moveit_msg
import geometry_msgs.msg as geom_msg
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from tf import transformations as tx
from tf import TransformListener, TransformBroadcaster
import tf2_ros
import numpy as np
import os
from copy import deepcopy
from collections import OrderedDict
from IPython.core.debugger import set_trace
import argparse
osp = os.path


class RobotController(object):
  def __init__(self, models_dir=osp.join('~', 'deepgrasp_data', 'models'),
      object_frame='object',
      object_trans=(-0.5, 0, 0.7),
      scale_factor=1.15,
      object_rot=tx.quaternion_about_axis(0, (0, 0, 1))):
    super(RobotController, self).__init__()
    rospy.init_node('thermal_grasp_controller', anonymous=True)

    self.models_dir = osp.expanduser(models_dir)
    self.scale_factor = scale_factor
    self.tros = TransformListener()
    self.tb = TransformBroadcaster()

    moveit_commander.roscpp_initialize(sys.argv)
    self.robot = moveit_commander.RobotCommander()
    self.scene = moveit_commander.PlanningSceneInterface()
    self.scene.remove_world_object()

    # reference frames
    self.world_frame = self.robot.get_planning_frame().split('/')[-1]
    self.object_frame = object_frame
    self.palm_frame = 'hand_palm'

    # commanders for various groups 
    # order of fingers is important while reading from the grasp files!
    self.fingers = OrderedDict()
    for finger_name in ['index_finger', 'mid_finger',
      'ring_finger', 'pinky_finger', 'thumb_finger']:
      self.fingers[finger_name] =\
          moveit_commander.MoveGroupCommander(finger_name)
      # send fingers to home position
      self.move_finger(finger_name, np.zeros(4))
    self.arm = moveit_commander.MoveGroupCommander('arm')
    # send arm to home position
    joint_goal = self.arm.get_current_joint_values()
    for idx in range(len(joint_goal)):
      joint_goal[idx] = 0
    self.arm.go(joint_goal, wait=True)
    self.arm.stop()

    # static transform of the object w.r.t. world
    self.broadcaster = tf2_ros.StaticTransformBroadcaster()
    t = geom_msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = self.world_frame
    t.child_frame_id = self.object_frame
    t.transform.translation.x = object_trans[0]
    t.transform.translation.y = object_trans[1]
    t.transform.translation.z = object_trans[2]
    t.transform.rotation.x    = object_rot[0]
    t.transform.rotation.y    = object_rot[1]
    t.transform.rotation.z    = object_rot[2]
    t.transform.rotation.w    = object_rot[3]
    self.broadcaster.sendTransform(t)

    # publisher to planning scene for disabling collisions w/ object
    self.pub_planning_scene = rospy.Publisher('planning_scene', PlanningScene)


  def add_object(self, object_name):
    rospy.sleep(0.5)
    self.scene.remove_world_object()
    
    pose = geom_msg.PoseStamped()
    pose.header.frame_id = self.object_frame
    
    mesh_filename = osp.join(self.models_dir, '{:s}.ply'.format(object_name))
    self.scene.add_mesh(self.object_frame, pose, mesh_filename, [self.scale_factor]*3)

    # disable collisions with the object. See
    # https://groups.google.com/d/msg/moveit-users/EI73skgnGVk/nBLgzjXTjW8J and
    # https://answers.ros.org/question/257795/collision-checking-between-collision-object-and-object-attached-to-the-robot/?answer=257954#post-id-257954
    rospy.wait_for_service('/get_planning_scene', 10.0)
    get_planning_scene = rospy.ServiceProxy('/get_planning_scene',
        GetPlanningScene)
    request = PlanningSceneComponents(
        components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
    response = get_planning_scene(request)
    acm = response.scene.allowed_collision_matrix
    if not self.object_frame in acm.default_entry_names:
      acm.default_entry_names  += [self.object_frame]
      acm.default_entry_values += [True]
    planning_scene_diff = PlanningScene(is_diff=True,
        allowed_collision_matrix=acm)
    self.pub_planning_scene.publish(planning_scene_diff)
    rospy.sleep(0.5)

    rospy.loginfo('Added {:s} to the scene'.format(object_name))


  def move_finger(self, finger_name, joint_values):
    rospy.sleep(0.1)
    joint_goal = self.fingers[finger_name].get_current_joint_values()
    joint_goal[:] = joint_values[:]
    self.fingers[finger_name].go(joint_goal, wait=True)
    self.fingers[finger_name].stop()

    
  def grasp(self, grasp_filename):
    rospy.sleep(0.5)

    grasp = np.loadtxt(osp.expanduser(grasp_filename), delimiter=',')
    palm_pose, joints = grasp[:12], grasp[12:]
  
    # move the arm to get the hand in position
    # construct pose of palm w.r.t. object from grasp file
    palm_pose = palm_pose.reshape((3, 4))
    palm_pose = np.vstack((palm_pose, [0, 0, 0, 1]))
    q = tx.quaternion_from_matrix(palm_pose)
    palm_goal_to_obj = geom_msg.PoseStamped()
    palm_goal_to_obj.header.frame_id = self.object_frame
    palm_goal_to_obj.pose.position.x = palm_pose[0, 3] 
    palm_goal_to_obj.pose.position.y = palm_pose[1, 3] 
    palm_goal_to_obj.pose.position.z = palm_pose[2, 3] 
    palm_goal_to_obj.pose.orientation.x = q[0]
    palm_goal_to_obj.pose.orientation.y = q[1]
    palm_goal_to_obj.pose.orientation.z = q[2]
    palm_goal_to_obj.pose.orientation.w = q[3]

    palm_goal_to_palm = self.tros.transformPose(self.palm_frame,
        palm_goal_to_obj)
    palm_goal_to_world = self.tros.transformPose(self.world_frame,
        palm_goal_to_palm)

    if False:  # debug: show goal frame for palm
      pose = palm_goal_to_world
      t = [
          pose.pose.position.x,
          pose.pose.position.y,
          pose.pose.position.z]
      q = [
          pose.pose.orientation.x,
          pose.pose.orientation.y,
          pose.pose.orientation.z,
          pose.pose.orientation.w]

      r = rospy.Rate(10)
      while not rospy.is_shutdown():
        self.tb.sendTransform(t, q, rospy.Time.now(), "palm_goal", pose.header.frame_id)
        r.sleep()

    # moveit!
    self.arm.set_pose_target(palm_goal_to_world)
    plan = self.arm.go(wait=True)
    self.arm.stop()
    self.arm.clear_pose_targets()
    
    # move the fingers to grasp pose
    idx = 0
    for finger_name in self.fingers.keys():
      rospy.loginfo('Moving {:s}...'.format(finger_name))
      self.move_finger(finger_name, joints[4*idx : 4*(idx+1)])
      rospy.loginfo('Done.')
      idx += 1


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--object_name', required=True)
  parser.add_argument('--session_name', required=True)
  args = parser.parse_args()

  rc = RobotController()
  grasp_filename = osp.join('..', 'data', 'grasps',
      '{:s}_{:s}_gt_hand_pose.txt'.format(args.session_name, args.object_name))
  rc.grasp(grasp_filename)
  rc.add_object(args.object_name)
