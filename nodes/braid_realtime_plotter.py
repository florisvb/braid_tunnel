#!/usr/bin/env python
import matplotlib.pyplot as plt
import matplotlib.animation as animate
from mpl_toolkits.mplot3d import Axes3D
import roslib
import numpy as np
import rospy
from std_msgs.msg import Float32, Float32MultiArray
from ros_flydra.msg import *
import time

fig = plt.figure()
ax = plt.axes(projection = '3d')
x_vec = []
y_vec = []
z_vec = []

tcall = time.time()
def trigger_callback(super_packet):
        tcall = time.time()
        obj_ids = []
        for packet in super_packet.packets:
            for obj in packet.objects:
                obj_ids.append(obj.obj_id)
                x_vec.append(obj.position.x)
                y_vec.append(obj.position.y)
                z_vec.append(obj.position.z)
                #print(obj.obj_id)




def braid_sub():
	rospy.init_node("Gimme_the_data", anonymous = True)
	rospy.Subscriber("/flydra_mainbrain/super_packets",flydra_mainbrain_super_packet, trigger_callback)
	#rospy.spin()
	plt.show(block = True)
	
def animate_(i, x_vec, y_vec, z_vec):
		plt.style.use('seaborn-white')
		x_vec = x_vec[-1000:]
		y_vec = y_vec[-1000:]
		z_vec = z_vec[-1000:]
		ax.clear()
		ax.set_ylim(-.3,.3)
		ax.set_xlim(-.5,.5)
		ax.set_zlim(0,.5)
		ax.scatter(x_vec, y_vec, z_vec)	

ani = animate.FuncAnimation(fig, animate_, fargs=(x_vec, y_vec, z_vec), interval=10)
#plt.show()
if __name__ == "__main__":
	braid_sub()


