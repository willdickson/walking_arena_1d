#!/usr/bin/env python
from __future__ import print_function
from optparse import OptionParser
import os
import Queue
import roslib
import rospy
import numpy
import std_msgs.msg
import cv2
import time

from blob_finder import BlobFinder

from operator import attrgetter
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from led_scheduler import LedScheduler

from walking_arena_1d.msg import ArenaData
from walking_arena_1d.msg import RegionData


class PathIntegrationNode(object):

    def __init__(self,nodenum):

        self.nodenum = str(nodenum)
        rospy.init_node('arean_node')

        self.scheduler_base_params = rospy.get_param( 
                '/scheduler_base_params', 
                { 
                    'led_dev_num' : 0,
                    'led_on_value' : 255, 
                    'led_off_value' : 0, 
                    'led_on_duration' : 1.0, 
                    'led_minimum_off_duration': 9.0, 
                    }
                )

	self.food_width =  rospy.get_param('/food_width', 10)    
	self.food_height = rospy.get_param('/food_height', 20)  

        self.pretrial_duration = rospy.get_param('/pretrial_duration', 6.0)
        self.experiment_duration = rospy.get_param('/experiment_duration', 1800.0)
        self.posttrial_duration = rospy.get_param('/posttrial_duration', 600.0)

        self.tracking_threshold = rospy.get_param('/tracking/threshold/', 40)
        self.tracking_min_area = rospy.get_param('/tracking/min_area', 2)
        self.tracking_max_area = rospy.get_param('/tracking/max_area', 1000)

        self.region_param = rospy.get_param(
                '/regions', 
                [ 
                    {'roi': {'x_min': -1, 'y_max': 39, 'y_min': 1,  'x_max': -1}, 'led_pin': 13, 'food_pos': [327, 17]}, 
                    {'roi': {'x_min': -1, 'y_max': 88, 'y_min': 39, 'x_max': -1}, 'led_pin': 14, 'food_pos': [327, 66]}, 
                ]) 

        
        self.bg_image_file = rospy.get_param('bg_image_file','bg_image.npy')
        self.bg_image = numpy.load(self.bg_image_file)

        # DEVEL
        # --------------------------------------------------
        for region_dict in self.region_param:
            print(region_dict)
        # --------------------------------------------------

        self.start_time = rospy.Time.now().to_time()
        self.logger_killed = False

        self.bridge = CvBridge()
        self.image_queue = Queue.Queue()
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
         
        self.led_schedulers = []
        for region_dict in self.region_param:
            params = dict(self.scheduler_base_params)
            params['pin'] = region_dict['led_pin'] 
            self.led_schedulers.append(LedScheduler(params))

        self.data_pub = rospy.Publisher('waking_arena_1d_data', ArenaData, queue_size=10) 

    def image_callback(self,imgmsg):
        cv_image = self.bridge.imgmsg_to_cv2(imgmsg,desired_encoding='mono8')
        self.image_queue.put(cv_image)

    #def tracked_objects_callback(self,data,nodenum):
    #    number_of_objects = len(data.tracked_objects)
    #    tracked = []
    #    if number_of_objects > 0:
    #        for obj in data.tracked_objects:
    #            inside_roi = True
    #            x, y = obj.position.x, obj.position.y
    #            if self.roi_x_min != -1 and x < self.roi_x_min:
    #                inside_roi = False
    #            if self.roi_x_max != -1 and x > self.roi_x_max:
    #                inside_roi = False
    #            if self.roi_y_min != -1 and y < self.roi_y_min:
    #                inside_roi = False
    #            if self.roi_y_max != -1 and y > self.roi_y_max:
    #                inside_roi = False
    #            if inside_roi:
    #                tracked.append(obj)
    #    number_of_objects = len(tracked)
    #    if number_of_objects > 0:
    #        fly = max(tracked, key = attrgetter('size'))
    #        self.fly_queue.put(fly)

    def on_food_test(self,fly,food_position):
        food_x, food_y = food_position
        test_x = abs(fly.position.x - food_x) < self.food_width/2.0 
        test_y = abs(fly.position.y - food_y) < self.food_height/2.0 
        return test_x and test_y

    def run(self):

        blob_finder = BlobFinder(
                threshold=self.tracking_threshold,
                minArea=self.tracking_min_area, 
                maxArea=self.tracking_max_area,
                )

        while not rospy.is_shutdown():

            try:
                new_image = self.image_queue.get_nowait()
            except Queue.Empty:
                continue

            current_time = rospy.Time.now().to_time()
            elapsed_time = current_time - self.start_time 

            diff_image = cv2.absdiff(new_image,self.bg_image)
            blob_list, blob_image = blob_finder.find(diff_image)
            
            # DEVEL Temporary
            # --------------------------------
            on_food = False
            # --------------------------------

            if (elapsed_time >= self.pretrial_duration) and (elapsed_time < self.pretrial_duration + self.experiment_duration):             
                for scheduler in self.led_schedulers:
                    scheduler.update(rospy.Time.now().to_time(), on_food)

            if elapsed_time > (self.pretrial_duration + self.experiment_duration + self.posttrial_duration):
                if not self.logger_killed:
                    os.system('rosnode kill experiment_logger')
                    self.logger_killed = True

            new_image_bgr = cv2.cvtColor(new_image, cv2.COLOR_GRAY2BGR)
            for region_dict in self.region_param:
                cx, cy = region_dict['food_pos']
                x0 = int(cx - self.food_width/2.0)
                y0 = int(cy - self.food_height/2.0)
                x1 = int(cx + self.food_width/2.0)
                y1 = int(cy + self.food_height/2.0)
                cv2.rectangle(new_image_bgr, (x0,y0), (x1,y1), (0,0,255), 1)
                
            cv2.imshow('blob image', blob_image)
            cv2.imshow('walking arena 1d',new_image_bgr)
            cv2.waitKey(1)

            #while (self.fly_queue.qsize() > 0):

            #    fly = self.fly_queue.get()
            #    for scheduler, food_position in zip(self.led_schedulers, self.food_positions):
            #        fly_on_food = self.on_food_test(fly,food_position)
            #        current_time = rospy.Time.now().to_time()
            #        elapsed_time = current_time - self.start_time 
            #        
            #        if (elapsed_time >= self.pretrial_duration) and (elapsed_time < self.pretrial_duration + self.experiment_duration):
            #            scheduler.update(rospy.Time.now().to_time(), fly_on_food)

            #        header = std_msgs.msg.Header()
            #        header.stamp = rospy.Time.now()
            #        data = PathIntegrationData( 
            #                    header, 
            #                    fly, 
            #                    fly_on_food,
            #                    food_position,
            #                    scheduler.led_pin, 
            #                    scheduler.activation_count
            #                    )
            #        self.data_pub.publish(data)

            ## Dummy update - so that scheduler keeps updating when there are no fly events
            #current_time = rospy.Time.now().to_time()
            #elapsed_time = current_time - self.start_time 
            #for scheduler in self.led_schedulers:
            #    scheduler.update(current_time, False)

            #if elapsed_time > self.pretrial_duration + self.experiment_duration + self.posttrial_duration:
            #    if not self.logger_killed:
            #        os.system('rosnode kill experiment_logger')
            #        self.logger_killed = True

        

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--nodenum", type="str", dest="nodenum", default='1',
                        help="node number, for example, if running multiple tracker instances on one computer")
    (options, args) = parser.parse_args()

    node = PathIntegrationNode(options.nodenum) 
    node.run()
