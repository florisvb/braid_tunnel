#!/usr/bin/env python
"""
This script emulates braid to create fake flydra_mainbrain_super_packets

Depends on:
https://github.com/strawlab/ros_flydra
https://github.com/florisvb/PyNumDiff

"""

import time
import numpy as np
from optparse import OptionParser

import roslib
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('std_msgs')
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import UInt32, Float32
roslib.load_manifest('ros_flydra')
from ros_flydra.msg import flydra_mainbrain_super_packet, flydra_mainbrain_packet, flydra_object

import rospy

try:
    import pynumdiff
except:
    raise ValueError('Install pynumdiff: pip install pynumdiff')


class VirtualTrajectory:
    def __init__(self, obj_id, length, x=None, y=None, z=None, P=None, fps=100, probability_of_birth=0.005):
        '''
        obj_id  -- int
        length  -- length in frames
        x, y, z -- 1D np.arrays of position in x y and z. Defaults to sinusoidal oscillations
        P       -- covariance matrices, [6,6,length] defaults to 0.1 in the diagonal
        fps     -- framerate, default = 100 fps
        probability_of_birth -- when not active, the probability that the trajectory will be "born" each time step. 1 guarantees the trajectory will always exist
        '''

        self.obj_id = obj_id
        self.length = length
        self.dt = 1/float(fps)
        self.probability_of_birth = probability_of_birth

        t = np.arange(0, length*self.dt, self.dt)

        if x is None:
            x = 0.5*np.sin(t)
        if y is None:
            y = 0.25*np.cos(3*t)
        if z is None:
            z = 0.25 + 0.25*np.sin(0.5*t)
        self.x = x
        self.y = y
        self.z = z

        _, self.xvel = pynumdiff.linear_model.savgoldiff(x, self.dt, [3, 20])
        _, self.yvel = pynumdiff.linear_model.savgoldiff(y, self.dt, [3, 20])
        _, self.zvel = pynumdiff.linear_model.savgoldiff(z, self.dt, [3, 20])

        if P is None:
            eye = 0.1*np.eye(6)
            P = np.repeat(eye[:, :, np.newaxis], length, axis=2)
        self.P = P

        self.active = False 
        self.indexnumber = 0

    def activate(self, obj_id):
        if not self.active:
            # activate
            if np.random.uniform() < self.probability_of_birth:
                self.active = True
                self.indexnumber = 0
                self.obj_id = obj_id

    def next(self):
        if self.indexnumber >= self.length-1:
            self.active = False

        if self.active:
            Ps = {'P%d%d'%(i,i): self.P[:,:,self.indexnumber][i,i] for i in range(6)}
            update_dict = {'obj_id': self.obj_id,
                           'x': self.x[self.indexnumber],
                           'y': self.y[self.indexnumber],
                           'z': self.z[self.indexnumber],
                           'xvel': self.xvel[self.indexnumber],
                           'yvel': self.yvel[self.indexnumber],
                           'zvel': self.zvel[self.indexnumber]}
            update_dict.update(Ps)

            self.indexnumber += 1
            return update_dict

        else:
            return None




class VirtualBraidProxy:
    def __init__(self, virtual_trajectories, fps=100):
        '''
        virtual_trajectories -- list of instances of VirtualTrajectory
        '''
        self.fps = fps
        self.virtual_trajectories = virtual_trajectories

        self.pub = rospy.Publisher('flydra_mainbrain/super_packets',
            flydra_mainbrain_super_packet, queue_size=100)

        self.current_obj_id = self.virtual_trajectories[-1].obj_id # so we can spawn new trajectories
        self.framenumber = -1

    def publish(self):
        self.framenumber += 1

        msg = flydra_mainbrain_super_packet()
        packet = flydra_mainbrain_packet()
        objects = []

        for vtraj in self.virtual_trajectories:
            if not vtraj.active:
                vtraj.activate(self.current_obj_id+1)
                self.current_obj_id += 1

            else:
                update_dict = vtraj.next()

                if update_dict is not None:
                    obj_id = vtraj.obj_id

                    obj = flydra_object()
                    obj.obj_id = update_dict['obj_id']
                    obj.position.x = update_dict['x']
                    obj.position.y = update_dict['y']
                    obj.position.z = update_dict['z']
                    obj.velocity.x = update_dict['xvel']
                    obj.velocity.y = update_dict['yvel']
                    obj.velocity.z = update_dict['zvel']
                    obj.posvel_covariance_diagonal = [update_dict['P%d%d'%(i,i)] for i in range(6)]
                    objects.append(obj)

        packet.framenumber = self.framenumber
        packet.reconstruction_stamp = rospy.get_rostime()
        packet.acquire_stamp = packet.reconstruction_stamp
        packet.objects = objects

        msg.packets = [packet]

        self.pub.publish(msg)

    def run(self):
        rospy.init_node('braid_emulator', anonymous=True)
        rate = rospy.Rate(self.fps) # 10h
        
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep() 

if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--num_trajecs", type="int", dest="num_trajecs", default=2,
                        help="Number of (potentially) simultaneous trajectories, default=2")
    parser.add_option("--trajec_length", type="int", dest="trajec_length", default=500,
                        help="Default trajectory length for first trajectory, in frames, default=500")
    parser.add_option("--trajec_length_min", type="int", dest="trajec_length_min", default=100,
                        help="Minimum trajectory length in frames, default=100")
    parser.add_option("--trajec_length_max", type="int", dest="trajec_length_max", default=100,
                        help="Maximum trajectory length in frames, default=800")
    parser.add_option("--fps", type="int", dest="fps", default=100,
                        help="Frames per second, default=100")
    parser.add_option("--trajec_prob_of_birth", type="float", dest="trajec_prob_of_birth", default=0.005,
                        help="probability that new trajectory is spawned, per frame, default=0.005")
    (options, args) = parser.parse_args()

    fps = options.fps

    # first trajectory
    virtual_trajectories = [VirtualTrajectory(1, options.trajec_length),]

    # additional trajectories
    if options.num_trajecs >= 2:
        for i in range(1, options.num_trajecs):
            vt = VirtualTrajectory(i+1, np.random.uniform(options.trajec_length_min, options.trajec_length_max))
            virtual_trajectories.append(vt)

    virtual_braid = VirtualBraidProxy(virtual_trajectories, fps=fps)
    virtual_braid.run()
