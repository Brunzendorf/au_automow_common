#!/usr/bin/env python

"""
This ROS node takes the field survey file and publishes a
field polygon as a geometry_msgs/PolygonStamped for use in 
other nodes and for visualization in rviz.
"""

import roslib; roslib.load_manifest('automow_maps')
import rospy

from geometry_msgs.msg import PolygonStamped, Point32, Polygon


def offset_polygon(points, offset):
    import polygon_offset
    from polygon_offset import getinsetpoint
    temp_points = []
    polygon_offset.OFFSET = -offset
    for i in range(len(points)-2):
        temp_points.append(getinsetpoint(points[i],
                                         points[i+1],
                                         points[i+2]))
    temp_points.append(getinsetpoint(points[-2],
                                     points[-1],
                                     points[0]))
    temp_points.append(getinsetpoint(points[-1],
                                     points[0],
                                     points[1]))
    result = []
    for point in temp_points:
        result.append(Point32(point[0], point[1], 0))
    return result


class FieldPublisherNode(object):
    """
    This is a ROS node that is responsible for publishing the field.
    """
    def __init__(self):
        # Setup ROS node
        rospy.init_node('field_publisher')

        # Get ROS parameters
        self.field_polygon = rospy.get_param("~field_polygon")
        self.safety_zone_offset = rospy.get_param("~safety_zone_offset", 1)
        self.cut_area_offset = rospy.get_param("~cut_area_offset", -0.5)
        self.field_frame_id = rospy.get_param("~field_frame_id", "/map")

        # Setup publishers and subscribers
        safety_pub = rospy.Publisher('/field/safety', PolygonStamped, latch=True, queue_size=1)
        boundry_pub = rospy.Publisher('/field/boundry', PolygonStamped, latch=True, queue_size=1)
        cut_area_pub = rospy.Publisher('/field/cut_area', PolygonStamped, latch=True, queue_size=1)

        # Read the field in
        if self.read_field_file():
            # Publish the msg once, it is latched so no need to repeat

            safety_pub.publish(self.safety_msg)
            boundry_pub.publish(self.boundry_msg)
            cut_area_pub.publish(self.cut_area_msg)
            # Spin
            rospy.spin()

    def read_field_file(self):
        # Setup msgs
        self.safety_msg = PolygonStamped()
        self.boundry_msg = PolygonStamped()
        self.cut_area_msg = PolygonStamped()
        self.safety_msg.header.stamp = rospy.Time.now()
        self.safety_msg.header.frame_id = self.field_frame_id
        self.boundry_msg.header = self.safety_msg.header
        self.cut_area_msg.header = self.safety_msg.header
        # Parse out the points
        polygon_points = []
        polygon_points32 = []
        point_count = 0
        for point in self.field_polygon:
            point_count += 1
            # if point['fix_type'] < 3:
            #     rospy.logwarn('Point %i has a low quality fix type of %i'
            #                   % (point_count, point['fix_type']))
            (easting, northing) = (point['xpose'], point['ypose'])
            polygon_points.append((float(easting), float(northing)))
            polygon_points32.append(Point32(float(easting), float(northing), 0))
        # Put the points into the boundry_msg
        self.boundry_msg.polygon = Polygon(polygon_points32)
        # Expand and contract the field shape for safety buffer and cut area
        safety_points = offset_polygon(polygon_points, self.safety_zone_offset)
        cut_area_points = offset_polygon(polygon_points, self.cut_area_offset)
        self.safety_msg.polygon = Polygon(safety_points)
        self.cut_area_msg.polygon = Polygon(cut_area_points)
        return True


if __name__ == '__main__':
    fpn = FieldPublisherNode()
