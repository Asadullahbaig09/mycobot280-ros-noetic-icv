#!/usr/bin/env python3
"""Detect coloured cubes in the overhead camera and localise them in the world.

Pipeline per frame:
  BGR -> HSV -> colour threshold -> morphological open -> contours
  -> minAreaRect (pixel centre + edge direction)
  -> back-project through the camera intrinsics onto the plane z = cube top
  -> world (x, y, yaw)

Publishes
  /vision_pick/cubes        visualization_msgs/MarkerArray  (ns = colour, pose = cube centre in world)
  /vision_pick/debug_image  sensor_msgs/Image               (annotated camera image)
"""
import cv2
import numpy as np
import rospy
import tf.transformations as tft
import tf2_ros
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import Marker, MarkerArray

from vision_geometry import pixel_to_plane, square_yaw

# HSV ranges (OpenCV: H 0-180). Saturation >= 120 rejects the pale drop-off pads.
COLOURS = {
    "red": [((0, 120, 60), (10, 255, 255)), ((170, 120, 60), (180, 255, 255))],
    "green": [((40, 120, 60), (85, 255, 255))],
    "blue": [((100, 120, 60), (130, 255, 255))],
}
DRAW = {"red": (0, 0, 255), "green": (0, 200, 0), "blue": (255, 0, 0)}
RGBA = {"red": (1, 0, 0), "green": (0, 1, 0), "blue": (0, 0, 1)}


class CubeDetector:
    def __init__(self):
        self.cube_size = rospy.get_param("~cube_size", 0.03)
        self.min_area = rospy.get_param("~min_area_px", 60)
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)

        info = rospy.wait_for_message("camera/camera_info", CameraInfo)
        self.K = np.array(info.K).reshape(3, 3)
        self.R, self.t = self.camera_pose(info.header.frame_id)
        rospy.loginfo("cube_detector: camera at %s", np.round(self.t, 3))

        self.pub = rospy.Publisher("vision_pick/cubes", MarkerArray, queue_size=1)
        self.dbg = rospy.Publisher("vision_pick/debug_image", Image, queue_size=1)
        rospy.Subscriber("camera/image_raw", Image, self.on_image, queue_size=1, buff_size=2**24)

    def camera_pose(self, frame):
        # ponytail: camera is fixed in the world, so its pose is looked up once. Re-query per frame for eye-in-hand.
        tr = self.tf_buffer.lookup_transform(self.world_frame, frame, rospy.Time(0), rospy.Duration(10)).transform
        q = tr.rotation
        R = tft.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        return R, np.array([tr.translation.x, tr.translation.y, tr.translation.z])

    def detect(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        kernel = np.ones((3, 3), np.uint8)
        found = []
        for name, ranges in COLOURS.items():
            mask = np.zeros(hsv.shape[:2], np.uint8)
            for lo, hi in ranges:  # red wraps around H=0, so it needs two ranges
                mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                if cv2.contourArea(c) < self.min_area:
                    continue
                rect = cv2.minAreaRect(c)
                box = cv2.boxPoints(rect)
                # top face centre and one edge -> world position and yaw
                centre = pixel_to_plane(rect[0][0], rect[0][1], self.K, self.R, self.t, self.cube_size)
                e0 = pixel_to_plane(box[0][0], box[0][1], self.K, self.R, self.t, self.cube_size)
                e1 = pixel_to_plane(box[1][0], box[1][1], self.K, self.R, self.t, self.cube_size)
                found.append((name, centre, square_yaw(e0, e1), box))
        return found

    def on_image(self, msg):
        bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cubes = self.detect(bgr)

        markers = MarkerArray()
        markers.markers.append(Marker(action=Marker.DELETEALL))
        for i, (name, p, yaw, box) in enumerate(cubes):
            m = Marker()
            m.header.frame_id = self.world_frame
            m.header.stamp = msg.header.stamp
            m.ns, m.id, m.type = name, i, Marker.CUBE
            m.pose.position.x, m.pose.position.y, m.pose.position.z = p[0], p[1], self.cube_size / 2
            qx, qy, qz, qw = tft.quaternion_from_euler(0, 0, yaw)
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = qx, qy, qz, qw
            m.scale.x = m.scale.y = m.scale.z = self.cube_size
            m.color.r, m.color.g, m.color.b = RGBA[name]
            m.color.a = 0.6
            m.lifetime = rospy.Duration(1.0)
            markers.markers.append(m)

            cv2.drawContours(bgr, [np.int0(box)], 0, DRAW[name], 2)
            label = "%s (%.3f, %.3f) %+.0fdeg" % (name, p[0], p[1], np.degrees(yaw))
            org = tuple(np.int0(box.min(axis=0)) - [0, 6])
            cv2.putText(bgr, label, org, cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 2)
            cv2.putText(bgr, label, org, cv2.FONT_HERSHEY_SIMPLEX, 0.4, DRAW[name], 1)

        self.pub.publish(markers)
        self.dbg.publish(self.bridge.cv2_to_imgmsg(bgr, "bgr8"))


if __name__ == "__main__":
    rospy.init_node("cube_detector")
    CubeDetector()
    rospy.spin()
