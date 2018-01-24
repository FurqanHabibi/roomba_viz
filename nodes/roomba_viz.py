#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""

"""

# ROS
import rospy
from rospy.exceptions import ROSException
import tf

# ROS msgs
from nav_msgs.msg import (
    OccupancyGrid,
    GridCells
)
from geometry_msgs.msg import (
    Point
)

import numpy as np
from skimage.draw import circle

class RoombaViz(object):
    """
    Show visited areas as GridCells

    """
    def __init__(self):
        """
        Create KeyboardPiano object.

        """
        # wait for map
        while True:
            try:
                map_message = rospy.wait_for_message("map", OccupancyGrid, 2.0)
                break
            except ROSException:
                rospy.loginfo("Map is not published")
                continue
            except:
                raise

        # vars
        self._radius = rospy.get_param('~radius', 0.165)
        self._visited_arr, self._map_res, self._map_origin = self.create_array(map_message)

        # pubs
        self._visited_pub = rospy.Publisher('visited_area', GridCells, queue_size=1)

        # tf
        self._tf_listener = tf.TransformListener()
    
    def create_array(self, map_message):
        res = map_message.info.resolution
        origin = map_message.info.origin.position

        array = np.zeros((map_message.info.width, map_message.info.height), dtype=np.int)

        return (array, res, origin)
    
    def array_to_gridcells(self, array):
        gc = GridCells()
        gc.header.stamp = rospy.Time.now()
        gc.header.frame_id = 'map'
        gc.cell_width = self._map_res
        gc.cell_height = self._map_res
        gc.cells = []
        rr, cc = np.nonzero(array)
        rr -= array.shape[0]/2
        cc -= array.shape[1]/2
        zz = np.zeros_like(rr)
        for pp in zip(rr, cc, zz):
            point = Point()
            point.x = pp[0] * self._map_res
            point.y = pp[1] * self._map_res
            point.z = pp[2]
            gc.cells.append(point)
        
        return gc

    def run(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                (trans,rot) = self._tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            
            rr, cc = circle((trans[0]/self._map_res) + (self._visited_arr.shape[0]/2), (trans[1]/self._map_res) + (self._visited_arr.shape[1]/2), self._radius/self._map_res)

            self._visited_arr[rr, cc] = 1

            # publish grid cells
            self._visited_pub.publish(self.array_to_gridcells(self._visited_arr))

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('roomba_viz')
    RV = RoombaViz()
    RV.run()
