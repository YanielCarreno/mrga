#!/usr/bin/env python
# Author: Yaniel Carreno
# All rights reserved.

from __future__ import print_function
import sys
import numpy as np
import rospy
import cv2
import time
import re
from std_msgs.msg import String
from linecache import getline
from std_srvs.srv import Empty, EmptyResponse
from mrga_msgs.srv import allocation, allocationResponse

# Import the library to read csv
import csv

class facts(object):

    def __init__(self):
        '''
        constructor
        '''
        rospy.init_node('facts_allocation')
        rospy.Service('facts_allocation', allocation, self.serviceCall)

        # Has the goal been loaded?

    def serviceCall(self, req):
        self.allocation_autorization = req.start
        self.fact_number = req.fact_number
        self.mission_constrains_directory = rospy.get_param('~mission_constrains_directory')

        if(self.allocation_autorization):
            self.allocation_data = open(str(self.mission_constrains_directory)+ 'allocation_solution.txt', 'rw')
            self.fact_line = self.allocation_data.readlines()
            self.fact_line = self.fact_line[self.fact_number]
            curr = self.fact_line.find("act")
            next = self.fact_line.find("wp")-1
            self.robot_name = str(self.fact_line[curr+4:next])
            curr = self.fact_line.find("wp")
            next = self.fact_line.find(")")
            self.wp_name = str(self.fact_line[curr:next])
            self.allocation_data.close()
            return allocationResponse(str(self.robot_name),str(self.wp_name))

        else:
            print('No autorization to open the file')



if __name__ == '__main__':
    try:
        facts = facts()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
