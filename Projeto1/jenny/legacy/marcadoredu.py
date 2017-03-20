#! /usr/bin/env python
# -*- coding:utf-8 -*-
__author__ = ["Rachel P. B. Moraes", "Fabio Miranda"]

import rospy
import numpy
from numpy import linalg
#import transformations
from tf import TransformerROS
import tf2_ros
import math
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Bump
from std_msgs.msg import Header

x = 0
y = 0
z = 0
id = 0
tfl = 0
global survivor_status
survivor_status = False
		#x = round(marker.pose.pose.position.x, 2) # para arredondar casas decimais e impressao ficar ok
		#y = round(marker.pose.pose.position.y, 2)
		#z = round(marker.pose.pose.position.z, 2)
		x = numpy.longdouble(marker.pose.pose.position.x)
		y = numpy.longdouble(marker.pose.pose.position.y)
		z = numpy.longdouble(marker.pose.pose.position.z)
		print(type(x))

		#print(x)
		id = marker.id
		#print(marker.pose.pose)
		# if id == 5:
		# 	print(buffer.can_transform("base_link", "ar_marker_5", rospy.Time(0)))
		# 	header = Header(frame_id= "ar_marker_5")
		# 	# Procura a transformacao em sistema de coordenadas entre a base do robo e o marcador numero 5
		# 	# Note que para seu projeto 1 voce nao vai precisar de nada que tem abaixo, a
		# 	# Nao ser que queira levar angulos em conta
		# 	trans = buffer.lookup_transform("base_link", "ar_marker_5", rospy.Time(0))
		# 	# Separa as translacoes das rotacoes
		# 	t = transformations.translation_matrix([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
		# 	# Encontra as rotacoes
		# 	r = transformations.quaternion_matrix([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
		# 	m = numpy.dot(r,t)
		# 	v2 = numpy.dot(m,[0,0,1,0])
		# 	v2_n = v2[0:-1]
		# 	n2 = v2_n/linalg.norm(v2_n)
		# 	cosa = numpy.dot(n2,[1,0,0])
		# 	angulo_marcador_robo = math.degrees(math.acos(cosa))
		# 	print("Angulo entre marcador e robo", angulo_marcador_robo)


def recebe2(msg):
	global x # O global impede a recriacao de uma variavel local, para podermos usar o x global ja'  declarado
	global y
	global z
	global id
	if msg.markers == []:
		print("Marker not found!")
	for marker in msg.markers:
		x = round(marker.pose.pose.position.x * 1e308 *100,2)
		y = round(marker.pose.pose.position.y * 1e308 *100,2)
		z = round(marker.pose.pose.position.z * 1e308 *1000000000,2)
		#print(x)
		print(marker.id, "x:", x, " y:", y, " z:", z)

		# z = '{0:.315f}'.format(x)
		# print(z[311:325])
		# print('y',y * 1e308 *100)

		if marker.id == 8 and not survivor_status:
			if z < 10:
				print('andando pra tras')
				pub.publish(Twist(Vector3(-1,-1,0), Vector3(0,0,0)))

		elif marker.id == 5 and not survivor_status:
			if x > 3.5 and z > 5:
				print('andando pra frente!')
				pub.publish(Twist(Vector3(0.5,0.5,0), Vector3(10,0,0)))
			if x < 3.5 and z > 5:
				pub.publish(Twist(Vector3(0.5,0.5,5), Vector3(0,10,20)))
		elif survivor_status:
			print('bateu')
			pub.publish(Twist(Vector3(-1,-1,0), Vector3(0,0,0)))
		else:
			pub.publish(Twist(Vector3(0,0,0), Vector3(0,0,0)))


		#print(marker.pose.pose)
def recebedor_survivor(msg):
	if msg.leftFront == 1 or msg.rightFront == 1 or msg.leftSide == 1 or msg.rightSide == 1:
		survivor_status = True
	else:
		survivor_status = False


#numpy.longdouble


if __name__=="__main__":
	global tfl
	global buffer

	rospy.init_node("marcador") # Como nosso programa declara seu nome para o sistema ROS
	survivor = rospy.Subscriber("/bump", Bump, recebedor_survivor,queue_size=1)
	recebedor = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, recebe2) # Para recebermos notificacoes de que marcadores foram vistos
	pub = rospy.Publisher("cmd_vel", Twist, queue_size=3)
	# velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1) # Para podermos controlar o robo

	tfl = tf2_ros.TransformListener(buffer) # Para fazer conversao de sistemas de coordenadas - usado para calcular angulo


	try:
		# Loop principal - todo programa ROS deve ter um
		while not rospy.is_shutdown():
			a = None

			rospy.sleep(1)

	except rospy.ROSInterruptException:
	    print("   programa encerrado")
