#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv_bridge
import cv2
from geometry_msgs.msg import Twist
import gc
import imutils
from std_msgs.msg import String


upperBound = (123, 123, 252)
lowerBound = (1, 0, 176)

keep_going = False


def move_to_object(image_message, publisher):
    global keep_going
    bridge = cv_bridge.CvBridge()
    image = None
    try:
        image = bridge.imgmsg_to_cv2(image_message, "bgr8")  # convert image message to OpenCV image matrix
    except cv_bridge.CvBridgeError, e:
        rospy.logerr(e.message)
        print e.message

    if image is not None:
        image = imutils.resize(image, width=600)  # resize the image for displaying onscreen
        (height, width) = image.shape[:2]
        mask = cv2.inRange(image, lowerBound, upperBound)  # create a mask layer based on color bounds
        mask = cv2.erode(mask, None, iterations=2)  # make the selection closer to a
        mask = cv2.dilate(mask, None, iterations=2)  # regular polygon, if possible

        # find contours in the masked image and keep the largest one
        if cv2.__version__ == "3.1.0":  # Because the return tuple changed in version 3.1.0
            (_, contours, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        else:
            (contours, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours is not None and len(contours) > 0:  # if a contour is found...
            c = max(contours, key=cv2.contourArea)  # let c be the largest contour

            # approximate the centroid of the contour
            moments = cv2.moments(c)
            centroid_x = int(moments["m10"] / moments["m00"])  # x coord of centroid of object
            centroid_y = int(moments["m01"] / moments["m00"])  # y coord of centroid of object
            object_centroid = (centroid_x, centroid_y)

            # draw the contour and centroid of the shape on the image
            cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
            cv2.circle(image, (centroid_x, centroid_y), 7, (255, 255, 255), -1)
            cv2.putText(image, "center", (centroid_x - 20, centroid_y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # determine if the object is centered horizontally, plus/minus 10 px, and if so, publish value of 0
            if ((width / 2) - 10) < object_centroid[0] < ((width / 2) + 10):
                rospy.loginfo("## Object centered! ##")
                object_offset = 0
            else:
                # otherwise, publish difference between x coords of center of object and center of frame
                object_offset = (width / 2) - object_centroid[0]

            if height - 10 < object_centroid[1]:  # used to determine if object went off bottom of frame
                keep_going = True

            gc.collect()  # force garbage collection; the list of contours potentially is very large
            vel = Twist()
            if object_offset < 0:  # object is to right of center; rotate left
                vel.angular.z = -0.6
                if object_offset > -40:  # slow down speed as we get closer
                    vel.angular.z = -0.4
                    if object_offset > -20:  # slow down more as we get even closer
                        vel.angular.z = -0.2
                vel.linear.x = 0
            elif object_offset > 0:  # object is to left of center; rotate right
                vel.angular.z = 0.6
                if object_offset < 40:  # slow down speed as we get closer
                    vel.angular.z = 0.4
                    if object_offset < 20:  # slow down more as we get even closer
                        vel.angular.z = -0.2
                vel.linear.x = 0
            else:  # object is centered; go forward
                vel.angular.z = 0
                vel.linear.x = 0.4
            rospy.loginfo("optical_center_finder reported: " + str(object_offset))
            publisher.publish(vel)  # publish the velocity commands as a Twist message
        elif keep_going:  # the object has gone off the bottom of the frame; continue forward until the tag is found
            vel = Twist()
            vel.angular.z = 0
            vel.linear.x = 0.4
            publisher.publish(vel)

        cv2.imshow("Camera Feed", image)  # show the image
        cv2.waitKey(1)  # refresh contents of image frame


def on_rfid_found(string_msg):
    global keep_going
    tag_id = string_msg.data
    rospy.loginfo("Found RFID tag with ID: " + tag_id)
    keep_going = False  # we've found the tag; stop


if __name__ == "__main__":
    rospy.init_node("find_object")  # initialize the node
    robot = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)  # set up a publisher to control Turtlebot
    rospy.Subscriber("camera/rgb/image_raw", Image, move_to_object, callback_args=robot)  # camera subscriber
    rospy.Subscriber("rfid_data", String, on_rfid_found, queue_size=10)  # rfid data subscriber
    rospy.loginfo("Node `find_object` started...")  # loginfo that the node has been set up
    rospy.spin()  # keeps the script from exiting until the node is killed
