#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from spoon_perception.srv import *


class SpoonPerception:
    """class that subscribes to a ros image topic and does the perception
    related with the spoon
    """

    def __init__(self,image_topic_name,th_detection=0.5,number_frames=5):
        self.detect_object_srv = None
        return

    def startDetectObjectSrv(self):
        self.detect_object_srv = rospy.Service('detect_object_spoon',ObjectSpoon,self.detectObject)
        return

    def detectObject(self,req):
        response = True
        return ObjectSpoonResponse(response)

def main():

    rospy.init_node('spoon_perception',anonymous = True)

    inteligent_spoon = SpoonPerception('/camera/image_raw')

    try:
        inteligent_spoon.startDetectObjectSrv()
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting Donw')

if __name__ == '__main__':
    main()
