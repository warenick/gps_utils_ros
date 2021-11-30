#! /usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
import tf
import tf2_ros
import geometry_msgs.msg
from geonav_conversions import LLtoUTM
class GPS2UTM(object):
    
    def __init__(self,gps_topic, map_position, out_topic, utm_frame, map_frame):
        if out_topic is None:
            out_topic = gps_topic+"_in_map_utm"
        self.utm_frame = utm_frame 
        self.map_frame = map_frame
        self.utm_in_map_pub = rospy.Publisher(out_topic, PoseStamped, queue_size=2)
        self.gps_sub = rospy.Subscriber(gps_topic, NavSatFix, self.gps_callback)
        self.init_map_transform(map_position)
        self.map_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tf_listener = tf.TransformListener()

    def init_map_transform(self, map_position):
        self.static_transformStamped = geometry_msgs.msg.TransformStamped()       
        self.static_transformStamped.header.stamp = rospy.Time.now()
        self.static_transformStamped.header.frame_id = self.utm_frame
        self.static_transformStamped.child_frame_id = self.map_frame
        if "wsg" in map_position["type"]:
            North, East, zone = LLtoUTM(map_position["latitude"], map_position["longitude"]) 
            alt = map_position["altitude"]
            yaw = map_position["heading"]
        elif "utm" in map_position["type"]:
            North = map_position["North"]
            East = map_position["East"]
            alt = map_position["altitude"]
            yaw = map_position["heading"]
        # lat = -lat
        # lon = -lon
        alt = 0
        self.static_transformStamped.transform.translation.x = East
        self.static_transformStamped.transform.translation.y = North
        self.static_transformStamped.transform.translation.z = alt         
        quat = tf.transformations.quaternion_from_euler(0,0,yaw)
        self.static_transformStamped.transform.rotation.x = quat[0]
        self.static_transformStamped.transform.rotation.y = quat[1]
        self.static_transformStamped.transform.rotation.z = quat[2]
        self.static_transformStamped.transform.rotation.w = quat[3]       
    
    def static_tf_gps2map_pub(self):
        self.static_transformStamped.header.stamp = rospy.Time.now()
        self.map_broadcaster.sendTransform(self.static_transformStamped)

    def gps_callback(self, msg):
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude
        alt = 0.
        North, East, _ = LLtoUTM(lat,lon) 
        
        # x,y = self.static_transformStamped
        utm_in_map_pose = PoseStamped()
        utm_in_map_pose.header.frame_id = self.utm_frame
        # utm_in_map_pose.header.frame_id = self.map_frame
        utm_in_map_pose.header.stamp = rospy.Time.now()
        utm_in_map_pose.pose.position.x = East
        utm_in_map_pose.pose.position.y = North
        utm_in_map_pose.pose.position.z = alt
        utm_in_map_pose.pose.orientation.x = 0.
        utm_in_map_pose.pose.orientation.y = 0.
        utm_in_map_pose.pose.orientation.z = 0.
        utm_in_map_pose.pose.orientation.w = 1.
        utm_in_map_pose = self.tf_listener.transformPose(self.map_frame, utm_in_map_pose)
        
        self.utm_in_map_pub.publish(utm_in_map_pose)

if __name__=="__main__":
    
    rospy.init_node("gps2utm")
    
    utm_frame = rospy.get_param('~utm_frame', "world")
    map_frame = rospy.get_param('~map_frame', "map")
    map_position = rospy.get_param('~map_position', { 
         "type": "wsg", # "wsg" or "utm"
         "latitude": 55.738617,
         "longitude": 37.524898,
         "altitude": 160.,
         "heading": 2.0})
    gps_topic = rospy.get_param('~gps_topic', "/mavros/global_position/raw/fix")
    out_topic = rospy.get_param('~out_topic', "/utm_in_map")
    rate = rospy.get_param('~rate', 30.)
    rate = 1/rate
    gps2utm = GPS2UTM(gps_topic, map_position, out_topic, utm_frame, map_frame)
    while not rospy.is_shutdown():
        gps2utm.static_tf_gps2map_pub()
        rospy.sleep(rate)
    rospy.spin()