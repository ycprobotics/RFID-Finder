# RFID-Finder

# 3. Install Python libraries
For our scripts, we'll use a few libraries in addition to OpenCV; `numpy`, `chardet`, and `pyserial`. You can install these packages by running `pip install numpy` 
and `pip install pyserial`. If any of these fail to install, try re-running the `pip install` command as `sudo` (e.g. `sudo pip install` the package). 
Here's an explanation of what each of these packages are for:

1. numpy - A standardized Python math library; used heavily by OpenCV (images are represented as numpy matricies)
2. pyserial - A library for reading/writing data over a serial connection

# 4. Setting the permissions of the serial device
In order for `pyserial` to be able to read data from the RFID reader over a serial connection, you need to set the permissions of the serial device. First, run the 
command `dmesg | grep tty` to see a list of all connected serial devices. Then, connect the RFID reader using the USB cable and run `dmesg | grep tty` again; the device 
that was not listed the first time is your RFID reader. This will tell you which serial port your device is connected to. For me, it was ttyUSB0:
```bash 
$ dmesg | grep tty
[    0.000000] console [tty0] enabled
[ 6290.333239] usb 1-3: FTDI USB Serial Device converter now attached to ttyUSB0
```
Once you've identified the serial port ID (e.g. ttyUSB0 in my case), you can change the permissions for the device using the `chmod` command. Since we won't need to 
write to the device (only read from it), we'll just add the "read" permission by running the command `sudo chmod +r /dev/ttyUSB0` (if your serial ID was different, 
replace `ttyUSB0` with your serial ID). Here's my entire terminal session for this part of the tutorial:
```bash
$ dmesg | grep tty
[    0.000000] console [tty0] enabled
$ dmesg | grep tty
[    0.000000] console [tty0] enabled
[ 6290.333239] usb 1-3: FTDI USB Serial Device converter now attached to ttyUSB0
$ sudo chmod 777 /dev/ttyUSB0
[sudo] password for root:
$
```
# 5. ROS-ify the RFID reader
In order to fully utilize the RFID reader, we need to ROS-ify it (e.g. write a ROS publisher node to publish data read from the device). Create a new file in 
the `scripts` subdirectory of the package we created earlier called `rfid_reader_node.py` and open the file in your favorite text editor; I'm using 
Jetbrains PyCharm Professional Edition 2016.

First, let's get our required imports set up:
```python 
#!/usr/bin/env python
import serial
import rospy
from std_msgs.msg import String
import argparse
import sys
```

1. `serial` is required to read from the serial device.
2. `rospy` is required to ROS-ify the script
5. `String` from `std_msgs.msg` is required because its the message type we'll use to publish the data.
4. `argparse` is required to parse command line arguments.
5. `sys` is required to terminate the script properly.

First, let's parse our command line arguments using `argparse`:
```python
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-b', '--baud', help="The baud rate of the serial device. Default: 9600.", required=False)
    parser.add_argument('serialID', help="The serial device to read from. Example: ttyUSB0")
    args = parser.parse_args()

    serial_device = args.serialID
    baud_rate = 9600
    if args.baud:
        baud_rate = args.baud
```

Next, we need to initialize the ROS node and create a ROS topic and publisher. We'll give our node the name "rfid_reader", and our topic the name "rfid_data":
```python
rospy.init_node("rfid_reader")
pub = rospy.Publisher("rfid_data", String, queue_size=10)
```
Then, all that's left is to actually read and publish the data. Since this will be a standalone node, all we need is a loop that constantly checks for 
data coming over the serial port, and when there is, decodes and publishes the data.
```python
ser = serial.Serial('/dev/' + serial_device, baud_rate)
while True:
    try:
        data = ser.readline()
        pub.publish(data)
    except Exception, e:
        sys.exit(0)
```

That's it! The full script should look like this:
```python
#!/usr/bin/env python
import serial
import rospy
from std_msgs.msg import String
import argparse
import sys


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-b', '--baud', help="The baud rate of the serial device. Default: 9600.", required=False)
    parser.add_argument('serialID', help="The serial device to read from. Example: ttyUSB0")
    args = parser.parse_args()

    serial_device = args.serialID
    baud_rate = 9600
    if args.baud:
        baud_rate = args.baud

    rospy.init_node("rfid_reader")
    pub = rospy.Publisher("rfid_data", String, queue_size=10)

    ser = serial.Serial('/dev/' + serial_device, baud_rate)
    rospy.loginfo("Node `rfid_reader` started.")
    while True:
        try:
            data = ser.readline()
            pub.publish(data)
        except Exception, e:
            sys.exit(0)
```
Now we can build the package by running the following commands:
```bash
$ roscd rfid_finder
~/ROS-RFID-Finder/rfid_finder$ rosmake
```
This will generate all the required message and build files. Now, we need to make our Python script executable by running the following commands:
```bash
$ roscd rfid_finder
~/ROS-RFID-Finder/rfid_finder$ cd scripts
~/ROS-RFID-Finder/rfid_finder/scripts$ chmod +x rfid_reader_node.py
```
Now, you should be able to start your node (assuming `roscore` is running) by running the following command at a terminal:
```bash
$ rosrun rfid_finder rfid_reader_node.py ttyUSB0
```
While the script is running, if you run `rostopic list` in a terminal, the list of topics should now include `/rfid_data`. 
You can test the script by running `rostopic echo /rfid_data`, and then holding an RFID tag up to the reader. The topic should receive a message with 
contents similar to `data: 7F001B607F7B`. You can press `ctrl+C` to terminate the script.

# 6. Write the script to optically navigate to the object
In your `scripts` folder, create a new file named `find_object.py`. First, let's go through our required imports.

```python
#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv_bridge
import cv2
from geometry_msgs.msg import Twist
import gc
import imutils
from std_msgs.msg import String
```

1. `rospy` is needed to interface with ROS nodes.
2. `Image` from `sensor_msgs.msg` is required to get image messages from the Turtlebot's camera.
3. `cv2_bridge` is needed to convert from an Image ROS message to an OpenCV image object.
4. `cv2` is needed to process the received images using OpenCV (identify object, find object's center, determine if object is centered in frame).
5. `Twist` from `geometry_msgs.msg` is the message type used to send velocity commands to the Turtlebot
6. `gc` is the Python garbage collection module; we need this so we can force garbage collection for optimization purposes
7. `imutils` is a module of utility functions for working with OpenCV images; it can be easily installed via Pip by running the command `sudo pip install imutils`
8. `String` from `std_msgs.msg` is the message type for the RFID reader node

Next, you need to determine an appropriate `upperBound` and `lowerBound` color vectors, in the BGR color space (**not RGB**). If you're not sure of what these should be, you can use
another script I've prepared called [BGRColorTool.py](https://github.com/YorkCpE/YCPRobotics/blob/master/tools/OpenCV-Color-Tools/BGRColorTool.py) to figure out 
what these values should be. Essentially, it allows you to click on an object, and each time you click, it adds the clicked color to a running average; then, when 
the script terminates, it should print an appropriate `upperBound` and `lowerBound` value for the object you clicked on. Make sure you take enough samples to get an 
accurate range (in my experience, 30-50 clicks should suffice, depending on lighting). Make sure you rotate the object to several angles as you're clicking, 
and get samples from both shadowed and illuminated parts of the object.

Another thing to keep in mind: if the Turtlebot's camera has not been configured (e.g. the OpenNI2 driver cannot find a configuration file for it), and depending on the camera, 
the colors may not be accurate, which will make it harder to track the object properly. One possible solution would be to use the BGRColorTool.py script using the Turtlebot's 
camera as the input device.

For the object I'm using (which is bright red), my values are: <br />
```python
upperBound = (123, 123, 252)
lowerBound = (1, 0, 176)
```

We'll also need a global variable to help us determine when the object goes off the bottom of the frame of the image, because the camera can only see objects that are a 
certain distance from the Turtlebot:
```python
keep_going = False
```

Next, let's design a function that will be our callback for our ROS publisher. Before we write the code, let's think about the general procedure **for each frame**:
1. Identify the target object using the `upperBound` and `lowerBound` color values.
2. Get the contours that exist in the frame for that color range and discard all but the largest contour; this will be your actual object.
3. Calculate the centroid of this contour.
4. Determine if the centroid is horizontally centered in the frame and publish a Twist message to the Turtlebot based on that information
5. Determine whether the object has drifted off the bottom of the frame, and if so, keep moving until the RFID tag is read.
6. Force garbage collection; our list of contours is *potentially* **a lot** of data.

This leads to the following callback method:
```python
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
```

Next, let's define a callback for a subscriber to the `rfid_data` topic that just reports the RFID tag ID that was read and stops the Turtlebot:
```python
def on_rfid_found(string_msg):
    global keep_going
    tag_id = string_msg.data
    rospy.loginfo("Found RFID tag with ID: " + tag_id)
    keep_going = False  # we've found the tag; stop
```

Now, all that's left to do is to set up the controller node itself:

```python
if __name__ == "__main__":
    rospy.init_node("find_object")  # initialize the node
    robot = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)  # set up a publisher to control Turtlebot
    rospy.Subscriber("camera/rgb/image_raw", Image, move_to_object, callback_args=robot)  # camera subscriber
    rospy.Subscriber("rfid_data", String, on_rfid_found, queue_size=10)  # rfid data subscriber
    rospy.loginfo("Node `find_object` started...")  # loginfo that the node has been set up
    rospy.spin()  # keeps the script from exiting until the node is killed
```

The reason for the line `if __name__ == "__main__":` is so that this module can be included in other scripts without this code actually executing.

So, the complete script for this section is:
```python
#!/usr/bin/env python
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

```


# 8. Launching the system
The first we need to do is install our new package on the Turtlebot's computer. To do this, copy the **entire project directory** (in this case, `ROS-RFID-Finder`) to the 
Turtlebot's computer; alternatively, if using git, you can use git to clone the project onto the Turtlebot's computer. Then, this directory needs to be added to the 
`ROS_PACKAGE_PATH` environment variable in the `~/.bashrc` file. Then, restart your terminal to apply changes. Once you have the project, you need to build and install it 
 the same way as in section 7, but on the Turtlebot's computer.
 
Then, launch all the dependencies. On the **Turtlebot's computer**, connect the Turtlebot and its onboard camera using the USB cables and run each of the 
following commands in a terminal tab:
1. `roscore`
2. `roslaunch openni2_launch openni2.launch`
3. `roslaunch turtlebot_bringup minimal.launch`
4. `rosrun rfid_finder rfid_reader_node.py`

Next, **on your main computer**, run the following command in a terminal window:
`rosrun rfid_finder find_object.py`

This should open a window that mirrors the Turtlebot's camera input overlayed with data from our node (like the object contours and centroid) so you can watch as images 
are received and processed to help the Turtlebot navigate to the object.


Congratulations! You've now implemented simple robotic optical navigation using ROS and Python 2! <br /><br /><br />
**Note:** The main script (`find_object_.py`) might need some tinkering depending on how you attached the RFID reader to the Turtlebot and the size and shape of your 
RFID tagged object.
