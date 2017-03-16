import roslib
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
from matplotlib import pyplot as plt
# 
# tfl = 0
buffer = tf2_ros.Buffer()

cap = cv2.VideoCapture(0)
cap.set(cv.CV_CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv.CV_CAP_PROP_FRAME_HEIGHT, 480)
font = cv2.FONT_HERSHEY_SIMPLEX
lower = 0
upper = 1

while(True):
    # Capture frame-by-frame
    # print("New frame")
    print(ar_pose_marker())
    ret, frame = cap.read()
    # arframe = individualMarkers.camera_image(frame)
    cv2.imshow('amazing',arframe)
    # print("No circles were found")
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

if __name__=="__main__":
	global tfl
	global buffer

	rospy.init_node("robot") # Como nosso programa declara seu nome para o sistema ROS

	recebedor = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, recebe) # Para recebermos notificacoes de que marcadores foram vistos
    tfl = tf2_ros.TransformListener(buffer) # Para fazer conversao de sistemas de coordenadas - usado para calcular angulo
    print(recebedor,recebe)

cap.release()
cv2.destroyAllWindows()
