#! /usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped, PoseArray
from sensor_msgs.msg import NavSatFix
import tf
import tf2_ros
import geometry_msgs.msg
from geonav_conversions import LLtoUTM, UTMtoLL
class GPS2UTM(object):
    
    def __init__(self,map_position, transform, utm_frame, map_frame):
        self.utm_frame = utm_frame 
        self.map_frame = map_frame
        # self.utm_in_map_pub = rospy.Publisher(out_topic, PoseStamped, queue_size=2)
        # self.gps_sub = rospy.Subscriber(gps_topic, NavSatFix, self.gps_callback)
        self.transform = transform
        self.utm_zone = None
        self.init_transforms(self.transform)
        self.init_map_transform(map_position)
        self.map_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tf_listener = tf.TransformListener()

    def init_transforms(self, transform):
        for transf in transform:
            
            strtype = transform[transf]["to_msg_type"]
            pub_type = NavSatFix if ("NavSatFix" in strtype) else PoseStamped if ("PoseStamped" in strtype) else PoseArray if ("PoseArray" in strtype) else None
            transform[transf]["publisher"] = rospy.Publisher(transform[transf]["to"], pub_type, queue_size=1)
            
            strtype = transform[transf]["from_msg_type"]
            sub_type = NavSatFix if ("NavSatFix" in strtype) else PoseStamped if ("PoseStamped" in strtype) else PoseArray if ("PoseArray" in strtype) else None
            transform[transf]["subscriber"] = rospy.Subscriber(transform[transf]["from"], sub_type, self.transform_callback, (transform[transf]))
            
    def transform_callback(self, msg, info):
        # if "wgs" in info["from_frame"]:
        if "NavSatFix" in info["from_msg_type"] and "PoseStamped" in info["to_msg_type"]:
            transformed_msg = self.wgs2frame(msg, info["to_frame"])
        elif "PoseStamped" in info["from_msg_type"] and "NavSatFix" in info["to_msg_type"]:
            transformed_msg = self.frame2wgs(msg)
        elif "PoseArray" in info["from_msg_type"] and "PoseArray" in info["to_msg_type"] and "wgs" in info["from_frame"]:
            transformed_msg = self.arr_wgs2frame(msg, info["to_frame"])
        if transformed_msg is not None:
            info["publisher"].publish(transformed_msg)
        
    def frame2wgs(self, msg):
        if self.utm_zone is None:
            return None
        try:
            in_world_msg = self.tf_listener.transformPose("world", msg)
        except:
            rospy.logwarn("transform world to "+msg.header.frame_id + " not exist")
            return None
        lat = in_world_msg.pose.position.x
        lon = in_world_msg.pose.position.y
        (lat, lon) = UTMtoLL(lon, lat, self.utm_zone) # return (Lat, Long)
        navsat_msg = NavSatFix()
        navsat_msg.header = msg.header
        navsat_msg.status.status = 0 # FIX
        navsat_msg.status.service = 1 # GPS
        navsat_msg.latitude = lat
        navsat_msg.longitude = lon
        navsat_msg.altitude = msg.pose.position.z
        return navsat_msg
        
    def arr_wgs2frame(self, msg, frame_id):
            if self.utm_zone is None:
                rospy.logwarn("utm zone is not exist. wgs convertion is not initialized")
                return None
            posearr_msg = PoseArray()
            posearr_msg.header = msg.header
            posearr_msg.header.frame_id = frame_id
            for pose in msg.poses:
                wgs_msg = NavSatFix()
                wgs_msg.latitude = pose.position.x
                wgs_msg.longitude = pose.position.y
                # wgs_msg.header = pose.position.z
                pose_msg = self.wgs2frame(wgs_msg, frame_id)
                posearr_msg.poses.append(pose_msg.pose)
            return posearr_msg
        

        # Calculate wgs position in map frame
    def wgs2frame(self, wgs_msg, frame_id):
        lat = wgs_msg.latitude
        lon = wgs_msg.longitude
        alt = wgs_msg.altitude
        alt = 0.
        North, East, _ = LLtoUTM(lat,lon) 
        in_frame_pose = PoseStamped()
        in_frame_pose.header.frame_id = self.utm_frame
        in_frame_pose.header.stamp = rospy.Time.now()
        in_frame_pose.pose.position.x = East
        in_frame_pose.pose.position.y = North
        in_frame_pose.pose.position.z = alt
        in_frame_pose.pose.orientation.x = 0.
        in_frame_pose.pose.orientation.y = 0.
        in_frame_pose.pose.orientation.z = 0.
        in_frame_pose.pose.orientation.w = 1.
        try:
            in_frame_pose = self.tf_listener.transformPose(frame_id, in_frame_pose)
        except:
            rospy.logwarn("transform "+in_frame_pose.header.frame_id+" to "+frame_id + " not exist")
        return in_frame_pose


    def init_map_transform(self, map_position):
        self.static_transformStamped = geometry_msgs.msg.TransformStamped()       
        self.static_transformStamped.header.stamp = rospy.Time.now()
        self.static_transformStamped.header.frame_id = self.utm_frame
        self.static_transformStamped.child_frame_id = self.map_frame
        if "wsg" in map_position["type"]:
            North, East, self.utm_zone = LLtoUTM(map_position["latitude"], map_position["longitude"]) 
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
        rospy.loginfo("inited map wgs conversions")

    
    def static_tf_gps2map_pub(self):
        self.static_transformStamped.header.stamp = rospy.Time.now()
        self.map_broadcaster.sendTransform(self.static_transformStamped)

    def gps_callback(self, msg):
    #         [sensor_msgs/NavSatFix]:
    # uint8 COVARIANCE_TYPE_UNKNOWN=0
    # uint8 COVARIANCE_TYPE_APPROXIMATED=1
    # uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN=2
    # uint8 COVARIANCE_TYPE_KNOWN=3
    # std_msgs/Header header
    #   uint32 seq
    #   time stamp
    #   string frame_id
    # sensor_msgs/NavSatStatus status
    #   int8 STATUS_NO_FIX=-1
    #   int8 STATUS_FIX=0
    #   int8 STATUS_SBAS_FIX=1
    #   int8 STATUS_GBAS_FIX=2
    #   uint16 SERVICE_GPS=1
    #   uint16 SERVICE_GLONASS=2
    #   uint16 SERVICE_COMPASS=4
    #   uint16 SERVICE_GALILEO=8
    #   int8 status
    #   uint16 service
    # float64 latitude
    # float64 longitude
    # float64 altitude
    # float64[9] position_covariance
    # uint8 position_covariance_type
        
        # Calculate position in map frame
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude
        alt = 0.
        North, East, _ = LLtoUTM(lat,lon) 
        utm_in_map_pose = PoseStamped()
        utm_in_map_pose.header.frame_id = self.utm_frame
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

        # 

if __name__=="__main__":
    
    rospy.init_node("gps2utm")
    
    utm_frame = rospy.get_param('~utm_frame', "world")
    map_frame = rospy.get_param('~map_frame', "map")
    map_position = rospy.get_param('~map_position', { 
         "type": "wsg", # "wsg" or "utm"
         "latitude": 55.738585,
         "longitude": 37.52480,
         "altitude": 160.,
         "heading": 1.684})

    transform = rospy.get_param('~transform',
      {  
        "gps_mavros":{          "from":       "/mavros/global_position/raw/fix",
                                "from_frame": "wgs",
                                "to":         "/gps_in_map",
                                "to_frame":   "map"},

        "gps":{                 "from":       "/gps/fix",
                                "from_frame": "wgs",
                                "to":         "/gps/gps_in_map",
                                "to_frame":   "map"},

        "frontend_goal_click":{ "from":       "/frontend/goal_click",
                                "from_frame": "wgs",
                                "to":         "/move_base_simple/goal",
                                "to_frame":   "map"},
        "localisation": { "from":       "/localization_pose",
                                "from_frame": "map",
                                "to":         "/gps/localization_pose",
                                "to_frame":   "wgs"},
        "waypoints_wgs":{"from":"/waypoints_wgs",
                                "from_frame":           "wgs",
                                "to":                   "/waypoints",
                                "to_frame":             "map"},


    })
    # gps_topic = rospy.get_param('~gps_topic', "/mavros/global_position/raw/fix")
    # out_topic = rospy.get_param('~out_topic', "/gps_in_map")
    rate = rospy.get_param('~rate', 30.)
    rate = 1/rate
    gps2utm = GPS2UTM(map_position, transform, utm_frame, map_frame)
    while not rospy.is_shutdown():
        gps2utm.static_tf_gps2map_pub()
        rospy.sleep(rate)
    rospy.spin()