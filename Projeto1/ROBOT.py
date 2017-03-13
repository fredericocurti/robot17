import roslib
import rospy
import sys
import smach
import smach_ros
import rospy
import numpy
from numpy import linalg
import transformations
from tf import TransformerROS
import tf2_ros
import math
import time
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import cv2
import cv2.cv as cv
# from matplotlib import pyplot as plt

#
# cap = cv2.VideoCapture(0)
# cap.set(cv.CV_CAP_PROP_FRAME_WIDTH, 640)
# cap.set(cv.CV_CAP_PROP_FRAME_HEIGHT, 480)
# font = cv2.FONT_HERSHEY_SIMPLEX
# lower = 0
# upper = 1
#
# import roslib
# import rospy
# import sys
# import smach
# import smach_ros
# import rospy
# import numpy
# from numpy import linalg
# import transformations
# from tf import TransformerROS
# import tf2_ros
# import math
# import time
# from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
# from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import Image
# from std_msgs.msg import Header
# import cv2
# import cv2.cv as cv
# from matplotlib import pyplot as plt


cap = cv2.VideoCapture(0)
cap.set(cv.CV_CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv.CV_CAP_PROP_FRAME_HEIGHT, 480)
font = cv2.FONT_HERSHEY_SIMPLEX
lower = 0
upper = 1

while(True):
    # Capture frame-by-frame
    # print("New frame")
    ret, frame = cap.read()

    cv2.imshow('amazing',frame)
    # print("No circles were found")
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
