#!/usr/bin/env python
from __future__ import division
from optparse import OptionParser
import roslib
import rospy
import os
import time
import threading

import numpy as np
from multi_tracker.msg import Trackedobject, Trackedobjectlist
from path_integration.msg import PathIntegrationData

import h5py

import atexit

class DataListener:
    def __init__(self, nodenum, info='data information', record_time_hrs=24):
        # self.subTrackedObjects = rospy.Subscriber('multi_tracker/' + nodenum + '/tracked_objects', Trackedobjectlist, self.tracked_object_callback, queue_size=300)
        self.subTrackedObjects = rospy.Subscriber('/' + nodenum + '/path_integration_data', PathIntegrationData, self.tracked_object_callback, queue_size=300)

        experiment_basename = rospy.get_param('/multi_tracker/' + nodenum + '/experiment_basename', 'none')
        if experiment_basename == 'none':
            experiment_basename = time.strftime("%Y%m%d_%H%M%S_N" + nodenum, time.localtime())
           
        filename = experiment_basename + '_path_trackedobjects.hdf5'
        home_directory = os.path.expanduser( rospy.get_param('/multi_tracker/' + nodenum + '/data_directory') )
        filename = os.path.join(home_directory, filename)
        print 'Saving path integration hdf5 data to: ', filename
        self.time_start = time.time()
        self.record_time_hrs = record_time_hrs
        
        self.buffer = []
        self.array_buffer = []
        # set up thread locks
        self.lockParams = threading.Lock()
        self.lockBuffer = threading.Lock()
        
        self.chunk_size = 5000
        self.hdf5 = h5py.File(filename, 'w')
        self.hdf5.swmr_mode = True # helps prevent file corruption if closed improperly
        self.hdf5.attrs.create("info", info)
        
        self.data_to_save = [   'fly.objid',
                                'header.stamp.secs',
                                'header.stamp.nsecs', 
                                'header.frame_id', 
                                'fly.position.x', 
                                'fly.position.y', 
                                'fly.position.z', 
                                'fly.velocity.x',
                                'fly.velocity.y',
                                'fly.velocity.z',
                                'fly.angle',
                                'fly.size',
                                'fly.covariance',
                                'fly.measurement.x',
                                'fly.measurement.y',
                                'activation_count',
                                'on_food',
                                ]
        self.data_format = {    'fly.objid': int,
                                'header.stamp.secs': int,
                                'header.stamp.nsecs': int, 
                                'header.frame_id':int, 
                                'fly.position.x': float, 
                                'fly.position.y': float, 
                                'fly.position.z': float, 
                                'fly.velocity.x': float,
                                'fly.velocity.y': float,
                                'fly.velocity.z': float,
                                'fly.angle': float,
                                'fly.size': float,
                                'fly.covariance': float,
                                'fly.measurement.x': float,
                                'fly.measurement.y': float,
                                'activation_count': np.uint64,
                                'on_food': bool,
                            }
                            
        self.dtype = [(data,self.data_format[data]) for data in self.data_to_save]
        rospy.init_node('save_path_data_to_hdf5_' + nodenum)
        
        self.hdf5.create_dataset('data', (self.chunk_size, 1), maxshape=(None,1), dtype=self.dtype)
        self.hdf5['data'].attrs.create('current_frame', 0)
        self.hdf5['data'].attrs.create('line', 0)
        self.hdf5['data'].attrs.create('length', self.chunk_size)
        
    def add_chunk(self):
        length = self.hdf5['data'].attrs.get('length')
        new_length = length + self.chunk_size
        self.hdf5['data'].resize(new_length, axis=0)
        self.hdf5['data'].attrs.modify('length', new_length)
            
    def save_array_data(self):
        newline = self.hdf5['data'].attrs.get('line') 
        nrows_to_add = len(self.array_buffer)
        
        self.hdf5['data'].attrs.modify('line', newline+nrows_to_add)
        if newline+nrows_to_add >= self.hdf5['data'].attrs.get('length')-50:
            self.hdf5.flush()
            self.add_chunk()
        
        self.hdf5['data'][newline:newline+nrows_to_add] = self.array_buffer
        self.array_buffer = []
                                                   
    def tracked_object_callback(self, PathIntegrationData):
        with self.lockBuffer:
            fly = PathIntegrationData.fly
            # print PathIntegrationData.header.frame_id
            a = np.array([(     fly.objid,
                                PathIntegrationData.header.stamp.secs,
                                PathIntegrationData.header.stamp.nsecs,
                                0,#PathIntegrationData.header.frame_id,
                                fly.position.x, fly.position.y, fly.position.z,
                                fly.velocity.x, fly.velocity.y, fly.velocity.z,
                                fly.angle,
                                fly.size,
                                fly.covariance,
                                fly.measurement.x, fly.measurement.y, 
                                PathIntegrationData.activation_count,
                                PathIntegrationData.on_food,
                           )], dtype=self.dtype)
            self.array_buffer.append(a)
        
            # for tracked_object in tracked_objects: #.tracked_objects:
            #    a = np.array([(     tracked_object.fly.objid,
            #                        PathIntegrationData.header.stamp.secs,
            #                        PathIntegrationData.header.stamp.nsecs,
            #                        PathIntegrationData.header.frame_id,
            #                        tracked_object.fly.position.x, tracked_object.fly.position.y, tracked_object.fly.position.z,
            #                        tracked_object.fly.velocity.x, tracked_object.fly.velocity.y, tracked_object.fly.velocity.z,
            #                        tracked_object.fly.angle,
            #                        tracked_object.fly.size,
            #                        tracked_object.fly.covariance,
            #                        tracked_object.fly.measurement.x, tracked_object.fly.measurement.y, PathIntegrationData.activation_count, PathIntegrationData.on_food,
            #                   )], dtype=self.dtype)
            #    self.array_buffer.append(a)
        
    def process_buffer(self):
        self.save_array_data()
            
    def main(self):
        atexit.register(self.stop_saving_data)
        while (not rospy.is_shutdown()):
            t = time.time() - self.time_start
            if t > self.record_time_hrs*3600:
                self.stop_saving_data()
                return
            with self.lockBuffer:
                time_now = rospy.Time.now()
                if len(self.array_buffer) > 0:
                    self.process_buffer()
                pt = (rospy.Time.now()-time_now).to_sec()
                if len(self.buffer) > 9:
                    rospy.logwarn("Data saving processing time exceeds acquisition rate. Processing time: %f, Buffer: %d", pt, len(self.buffer))
            
        
    def stop_saving_data(self):
        self.hdf5.close()
        print 'shut down nicely'
        
if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--nodenum", type="str", dest="nodenum", default='1',
                        help="node number, for example, if running multiple tracker instances on one computer")
    parser.add_option("--record-time-hrs", type="int", dest="record_time_hrs", default=24,
                        help="number of hours to record data for")
    (options, args) = parser.parse_args()
    
    datalistener = DataListener(options.nodenum, options.record_time_hrs)
    datalistener.main()
