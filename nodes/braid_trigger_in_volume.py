#!/usr/bin/env python

# Command line arguments
from optparse import OptionParser

# ROS imports
import roslib, rospy

# numpy imports - basic math and matrix manipulation
import numpy as np
import time

import yaml

from std_msgs.msg import Float32, Float32MultiArray

# flydra message
from ros_flydra.msg import *

################################################################################

################################################################################

class BraidTrigger:
    def __init__(self, 
                 config_file,
                 print_triggers=True,
                 braid_topic="/flydra_mainbrain/super_packets"):
        '''
        config -- path to a .yaml file describing the parameters for triggering. see below for an example.

        when the parameters of the configuration are determine, a "1" is published on the braid_trigger_topic, 
        and obj_id and object position is published on braid_trigger_topic + '_objid_info'

        # example braid volume trigger configuration
        topic: "braid_trigger_topic"

        xmin: -0.3
        xmax: 0.3
        ymin: -0.1
        ymax: 0.1
        zmin: 0.1
        zmax: 0.4

        vel_xmin: 0
        vel_xmax: None
        vel_ymin: -0.5
        vel_ymax: 0.5
        vel_zmin: -0.2
        vel_zmax: 0.2

        refractory_time: 1 # seconds
        min_trajec_length: 0.25 # seconds
        '''

        # Load config
        with open(config_file) as file:
            self.config = yaml.load(file)

        # Save inputs
        self.braid_topic = braid_topic
        self.braid_trigger_topic = self.config['topic']
        self.print_triggers = print_triggers

        # Subscriber
        self.braid_sub = rospy.Subscriber(self.braid_topic, flydra_mainbrain_super_packet, self.trigger_callback)

        # Publishers
        self.braid_trigger_topic = rospy.Publisher(self.braid_trigger_topic, Float32)
        self.braid_trigger_topic_objid = rospy.Publisher(self.config['topic'] + '_objid_info', Float32MultiArray)
        
        # keep track of how old objects are, and when last trigger was
        self.obj_birth_times = {}
        self.last_trigger = time.time()

    def run(self):
        # ROS
        rospy.init_node('binary_braid_trigger', anonymous=True)
        rospy.spin()

    def trigger_callback(self, super_packet):
        tcall = time.time()
        obj_ids = []
        for packet in super_packet.packets:
            for obj in packet.objects:
                obj_ids.append(obj.obj_id)

                # if it is a new object, save birth time
                if obj.obj_id not in self.obj_birth_times.keys():
                    self.obj_birth_times[obj.obj_id] = tcall
                    continue
                # if it is an old object, check to make sure the trajectory is long enough
                elif (tcall - self.obj_birth_times[obj.obj_id]) < self.config['min_trajec_length']:
                    continue

                # If trajectory is old enough, make sure we are past refactory period
                if tcall < self.last_trigger + self.config['refractory_time']:
                    continue

                # If trajectory is old enough, AND we are past refractory period:
                # check position and volume trigger parameters
                if obj.position.x > self.config['xmin'] and obj.position.x < self.config['xmax']:
                    if obj.position.y > self.config['ymin'] and obj.position.y < self.config['ymax']:
                        if obj.position.z > self.config['zmin'] and obj.position.z < self.config['zmax']:
                            if obj.velocity.x > self.config['vel_xmin'] and obj.velocity.x < self.config['vel_xmax']:
                                if obj.velocity.y > self.config['vel_ymin'] and obj.velocity.y < self.config['vel_ymax']:
                                    if obj.velocity.z > self.config['vel_zmin'] and obj.velocity.z < self.config['vel_zmax']:
                                        # all of the checks pass, trigger away!

                                        # reset trigger time for refactory period
                                        self.last_trigger = tcall

                                        # Publish the trigger
                                        # Note: you could add some function here to publish a value that is a function of the object position/velocity, instead of 1  
                                        self.braid_trigger_topic.publish(1)
                                        
                                        # publish obj id and position info for the object that caused the trigger
                                        msg = Float32MultiArray()
                                        msg.data = [obj.obj_id, obj.position.x, obj.position.y, obj.position.z]
                                        self.braid_trigger_topic_objid.publish(msg)

                                        if self.print_triggers:
                                            print('')
                                            print('Triggered!')
                                            print(msg.data)
        
        # kill off old objects
        for obj_id in self.obj_birth_times.keys():
            if obj_id not in obj_ids:
                self.obj_birth_times.pop(obj_id, None)
################################################################################

if __name__ == '__main__':    
    parser = OptionParser()
    parser.add_option("--config", type="str", dest="config", default='',
                        help="Full path that points to a config.yaml file. See ../configs/volume_trigger_config.yaml for an example")
    (options, args) = parser.parse_args()

    braid_trigger = BraidTrigger(config_file=options.config)
    braid_trigger.run()




    
