#!/usr/bin/env python3
"""Sort the cubes the camera sees onto the pad of their colour, using MoveIt.

For every detected cube (that is not already on its pad):
  pre-grasp above it -> straight down -> close gripper -> attach -> lift
  -> above its pad -> straight down -> detach -> open -> lift
"""
import math
import sys

import moveit_commander
import numpy as np
import rospy
import tf.transformations as tft
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class PickPlace:
    def __init__(self):
        # move_group takes a while to come up when everything starts from one launch file
        self.arm = moveit_commander.MoveGroupCommander("arm", wait_for_servers=60.0)
        self.gripper = moveit_commander.MoveGroupCommander("gripper", wait_for_servers=60.0)
        self.arm.set_max_velocity_scaling_factor(rospy.get_param("~velocity_scale", 0.6))
        self.arm.set_max_acceleration_scaling_factor(rospy.get_param("~velocity_scale", 0.6))
        self.arm.set_planning_time(5.0)
        self.arm.set_num_planning_attempts(5)

        # Grasp point = centre of the two finger pads when closed, in the joint6_flange frame.
        # Measured in Gazebo: the pads hang 88-115 mm below the flange and sit 8 mm off its axis.
        # Calibration knob: re-measure for a different gripper.
        self.tcp = np.array(rospy.get_param("~tcp_offset", [0.0, 0.008, 0.102]))
        self.cube_size = rospy.get_param("~cube_size", 0.03)
        self.approach = rospy.get_param("~approach_height", 0.06)
        self.grip_open = rospy.get_param("~gripper_open", 0.1)
        self.grip_closed = rospy.get_param("~gripper_closed", -0.45)  # pad gap ~30 mm
        self.observe_joints = rospy.get_param("~observe_joints", [-1.57, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.pads = rospy.get_param("~pads", {"red": [0.0, 0.20], "green": [0.0, -0.20], "blue": [0.10, -0.20]})

        self.attach_pub = rospy.Publisher("vision_pick/attach", String, queue_size=1)
        self.add_collision_objects()

    def add_collision_objects(self):
        """Tell MoveIt about the floor and the real size of the hand.

        Without the floor the planner routes the arm through it. The upstream model gives
        the gripper only tiny collision cylinders, so a box covering the whole hand
        (down to the finger pads) is attached to the flange as well.
        """
        scene = moveit_commander.PlanningSceneInterface(synchronous=True)
        ground = PoseStamped()
        ground.header.frame_id = self.arm.get_planning_frame()
        ground.pose.orientation.w = 1.0
        ground.pose.position.z = -0.011  # 2 cm slab, top 1 mm below z = 0 so the robot base is not "in collision"
        scene.add_box("ground", ground, size=(2.0, 2.0, 0.02))

        hand = PoseStamped()
        hand.header.frame_id = "joint6_flange"
        hand.pose.orientation.w = 1.0
        hand.pose.position.z = 0.0675  # box spans 20-115 mm below the flange
        touch = moveit_commander.RobotCommander().get_link_names(group="gripper") + ["joint6_flange", "joint6"]
        scene.attach_box("joint6_flange", "hand", hand, size=(0.10, 0.04, 0.095), touch_links=touch)

    # ---------- motion helpers ----------
    def flange_pose(self, x, y, z, yaw):
        """Flange pose that puts the grasp point at (x, y, z), fingers pointing down, rotated by yaw."""
        T = tft.euler_matrix(0, 0, yaw) @ tft.euler_matrix(math.pi, 0, 0)
        pos = np.array([x, y, z]) - T[:3, :3] @ self.tcp
        q = tft.quaternion_from_matrix(T)
        p = Pose()
        p.position.x, p.position.y, p.position.z = pos
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
        return p

    def move_to(self, pose):
        self.arm.set_pose_target(pose)
        ok = self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        return ok

    def straight_to(self, pose):
        """Straight-line move (for approach/retreat); falls back to a free move if it cannot be planned."""
        plan, fraction = self.arm.compute_cartesian_path([pose], 0.005, True)  # 5 mm steps, collision-checked
        if fraction > 0.95:
            return self.arm.execute(plan, wait=True)
        rospy.logwarn("cartesian path only %.0f%% feasible, using free motion", fraction * 100)
        return self.move_to(pose)

    def set_gripper(self, value):
        self.gripper.set_joint_value_target({"gripper_controller": value})
        self.gripper.go(wait=True)  # may report failure when fingers stall on the cube: that's fine
        self.gripper.stop()

    def home(self):
        """Swing the arm behind the base so it does not block the overhead camera."""
        self.arm.set_joint_value_target(self.observe_joints)
        self.arm.go(wait=True)
        self.arm.stop()

    # ---------- task ----------
    def wait_for_cubes(self, timeout=30.0):
        """Return [(colour, x, y, yaw)] from the detector (arm must be out of the camera view)."""
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            msg = rospy.wait_for_message("vision_pick/cubes", MarkerArray, timeout=timeout)
            cubes = []
            for m in msg.markers:
                if m.action != Marker.ADD:
                    continue
                q = m.pose.orientation
                yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
                cubes.append((m.ns, m.pose.position.x, m.pose.position.y, yaw))
            if cubes:
                return cubes
        return []

    def pick(self, name, x, y, yaw):
        z = self.cube_size / 2
        ok = self.move_to(self.flange_pose(x, y, z + self.approach, yaw))
        ok = ok and self.straight_to(self.flange_pose(x, y, z, yaw))
        if not ok:
            rospy.logerr("could not reach %s at (%.3f, %.3f)", name, x, y)
            self.straight_to(self.flange_pose(x, y, z + self.approach, yaw))
            return False
        self.set_gripper(self.grip_closed)
        self.attach_pub.publish(String("cube_" + name))
        rospy.sleep(0.5)
        self.straight_to(self.flange_pose(x, y, z + self.approach, yaw))
        return True

    def place(self, name):
        px, py = self.pads[name]
        z = self.cube_size / 2 + 0.003
        self.move_to(self.flange_pose(px, py, z + self.approach, 0.0))
        self.straight_to(self.flange_pose(px, py, z, 0.0))
        self.attach_pub.publish(String(""))
        rospy.sleep(0.5)
        self.set_gripper(self.grip_open)
        self.straight_to(self.flange_pose(px, py, z + self.approach, 0.0))

    def run(self):
        self.set_gripper(self.grip_open)
        self.home()
        rospy.sleep(1.0)  # let the camera deliver a frame taken after the arm stopped
        cubes = self.wait_for_cubes()
        rospy.loginfo("detected: %s", ", ".join("%s (%.3f, %.3f, %+.0fdeg)" % (n, x, y, math.degrees(a)) for n, x, y, a in cubes))
        done = 0
        for name, x, y, yaw in cubes:
            if name not in self.pads:
                continue
            if math.hypot(x - self.pads[name][0], y - self.pads[name][1]) < 0.04:
                rospy.loginfo("%s is already on its pad", name)
                continue
            rospy.loginfo("picking %s", name)
            if self.pick(name, x, y, yaw):
                self.place(name)
                done += 1
        self.home()
        rospy.loginfo("pick_place: sorted %d of %d cubes", done, len(cubes))


if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("pick_place")
    PickPlace().run()
    moveit_commander.roscpp_shutdown()
