#!/usr/bin/env python
"""
The ROS data publishers for UI
"""
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


def publish_camera(index):
    """Camera publisher"""
    pub = rospy.Publisher("/camera" + str(index) + "/image_raw", Image, queue_size=10)
    rospy.init_node("image" + str(index), anonymous=False)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            break
        if (frame.shape == (376, 1344, 3)):
            frame = frame[0:376, 0:672]
        msg = bridge.cv2_to_imgmsg(frame, "bgr8")
        pub.publish(msg)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        if rospy.is_shutdown():
            cap.release()


if __name__ == "__main__":
    try:
        camera_id = int(sys.argv[1])
        cap = cv2.VideoCapture(camera_id)
        bridge = CvBridge()
        publish_camera(camera_id)
    except rospy.ROSInterruptException:
        pass
