#!/usr/bin/env python3

# Students: Vincent Ying 
# Project: Aloha Wash
# Date: Dec 13, 2024
# Acknowledgements: None

from __future__ import print_function

import sys
import copy

import rospy
import moveit_msgs.msg
from six.moves import input
import moveit_commander
import geometry_msgs.msg

try:
    from math import pi
    from math import cos
    from math import tau
    from math import dist
    from math import fabs
except:  # For Python 2 compatibility
    from math import pi
    from math import cos
    from math import fabs
    from math import sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class AlohaWash(object):
    """AlohaWash"""

    def __init__(self):
        super(AlohaWash, self).__init__()

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        self.robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        self.scene = moveit_commander.PlanningSceneInterface()

        ## We can get a list of all the groups in the robot:
        self.group_names = self.robot.get_group_names()
        print("============ Available Planning Groups:", self.robot.get_group_names())

        ## Sometimes for debugging it is useful to print the entire state of the
        ## robot:
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")


class AlohaGripper(object):
    """AlohaGripper"""

    def __init__(self, group_name):
        super(AlohaGripper, self).__init__()

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Aloha robot, so we set the group's name to "fr_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        print("============ Move group: %s" % group_name)
        
        ## We can get the name of the reference frame for this robot:
        self.planning_frame = self.move_group.get_planning_frame()
        print("============ Planning frame: %s" % self.planning_frame)

    def open_gripper(self, extension):
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal = list(extension)

        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()

        # current_joints = self.move_group.get_current_joint_values()
        # return all_close(joint_goal, current_joints, 0.01)


class AlohaArm(object):
    """AlohaArm"""

    def __init__(self, group_name):
        super(AlohaArm, self).__init__()

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        print("============ Move group: %s" % group_name)

        self.move_group.set_goal_tolerance(0.05)

        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        self.planning_frame = self.move_group.get_planning_frame()
        print("============ Planning frame: %s" % self.planning_frame)

        self.eef_link = self.move_group.get_end_effector_link()
        print("============ End effector link: %s\n" % self.eef_link)

        # Misc variables
        self.box_name = ""
        
    def go_to_start_state(self):
        start_pose = (0, 0, 0, 0, 0, 0)
        joint_goal = list(start_pose)
        self.go_to_joint_state(start_pose)
        
        # Try once more
        current_joints = self.move_group.get_current_joint_values()
        if (not all_close(joint_goal, current_joints, 0.05)):
            self.go_to_joint_state(start_pose)

    def go_to_joint_state(self, joint_angles):
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Aloha's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal = list(map(lambda deg: deg / 360 * tau, joint_angles))

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()

        # For testing:
        # current_joints = self.move_group.get_current_joint_values()
        # return all_close(joint_goal, current_joints, 0.05)
    
    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        ## Ensuring Collision Updates Are Received
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node was just created (https://github.com/ros/ros_comm/issues/176),
        ## or dies before actually publishing the scene update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed.
        ## To avoid waiting for scene updates like this at all, initialize the
        ## planning scene interface with  ``synchronous = True``.
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([self.box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = self.box_name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False

    def detach_box(self, timeout=4):
        self.scene.remove_attached_object(self.eef_link, name=self.box_name)
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box(self, timeout=4):
        self.scene.remove_world_object(self.box_name)


def main():
    try:
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_aloha_wash", anonymous=True)

        print("")
        print("----------------------------------------------------------")
        print("Welcome to the Lab MoveGroup Python Interface")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
        input("============ Press `Enter` to setup moveit_commander ...")
        frGripper = AlohaGripper("fr_gripper")
        flGripper = AlohaGripper("fl_gripper")
        rrGripper = AlohaGripper("rr_gripper")
        lrGripper = AlohaGripper("lr_gripper")
        frArm = AlohaArm("fr_arm")
        flArm = AlohaArm("fl_arm")
        rrArm = AlohaArm("rr_arm")
        lrArm = AlohaArm("lr_arm")

        input("============ Press `Enter` to reset arm positions ...")
        frArm.go_to_start_state()
        flArm.go_to_start_state()
        rrArm.go_to_start_state()
        lrArm.go_to_start_state()

        input("============ Press `Enter` to detach boxes from Aloha robot ...")
        frArm.detach_box()
        flArm.detach_box()
        rrArm.detach_box()
        lrArm.detach_box()

        input("============ Press `Enter` to remove boxes from planning scene ...")
        frArm.remove_box()
        flArm.remove_box()
        rrArm.remove_box()
        lrArm.remove_box()

        input("============ Press `Enter` to close grippers ...")
        frGripper.open_gripper((0.0, 0.0))
        flGripper.open_gripper((0.0, 0.0))
        rrGripper.open_gripper((0.0, 0.0))
        lrGripper.open_gripper((0.0, 0.0))

        print("============ Teardown complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()

