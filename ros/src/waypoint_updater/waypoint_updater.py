#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int32

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number
TARGET_SPEED = 10   # [MPH]
MAX_DECEL = 0.7
STOP_DIST = 10.0

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)
        rospy.Subscriber('/obstacle_waypoints', Lane, self.obstacle_cb, queue_size=1)
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.vehiclePose = PoseStamped()
        self.baseWayPoints = Lane()
        self.waypoint = Waypoint()
        self.nxtWayPoint = 0
        self.final_waypoints = Lane()
        self.ID = 0
        self.traffic_light_waypoint = None
        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        self.vehiclePose = msg
        self.publish_final_wp()
        pass
        
    def publish_final_wp(self):

        #if self.vehiclePose.pose is not None:

        nxtWayPoint = self.next_way_point()
        lookahead_waypoints = self.baseWayPoints.waypoints[nxtWayPoint:nxtWayPoint+LOOKAHEAD_WPS]

        if self.traffic_light_waypoint is None or self.traffic_light_waypoint < 0:
            # No RED lights, setting the velocity
            for i in range(len(lookahead_waypoints) - 1):
                # convert [MPH] to meters per sec
                self.set_waypoint_velocity(lookahead_waypoints, i, (TARGET_SPEED * 1609.34) / (60 * 60))

        else:
            tl_lookahead_index = max(0, self.traffic_light_waypoint - nxtWayPoint)
            lookahead_waypoints = self.decelerate(lookahead_waypoints, tl_lookahead_index)

        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = lookahead_waypoints  
        self.final_waypoints_pub.publish(lane)

    # def publish_final_wp(self):
    # self.nxtWayPoint = self.next_way_point()
    # if self.nxtWayPoint != -1:
    #     print(self.nxtWayPoint)
    #     waypoints = self.baseWayPoints.waypoints
    #     self.final_waypoints.header.seq = self.ID
    #     ++self.ID
    #     self.final_waypoints.header.stamp = rospy.get_rostime()
    #     if (abs(self.nxtWayPoint - len(waypoints)) > LOOKAHEAD_WPS):
    #         self.final_waypoints.waypoints = waypoints[self.nxtWayPoint: 
    #                                             self.nxtWayPoint+LOOKAHEAD_WPS]
    #     else:
    #         self.final_waypoints.waypoints = waypoints[self.nxtWayPoint:]
    #     self.final_waypoints_pub.publish(self.final_waypoints)

    def decelerate(self, waypoints, redlight_index):
        # decelerate the vehicle
        rospy.loginfo("Deceleration")
        if len(waypoints) < 1:
            return []

        first = waypoints[0]
        last = waypoints[redlight_index]

        last.twist.twist.linear.x = 0.
        total_dist = self.distance(first.pose.pose.position, last.pose.pose.position)
        start_vel = first.twist.twist.linear.x
        # start from the waypoint before last and go backwards
        for index, wp in enumerate(waypoints):
            if index > redlight_index:
                vel = 0
            else:
                dist = self.distance(wp.pose.pose.position, last.pose.pose.position)
                dist = max(0, dist - STOP_DIST)                
                vel  = math.sqrt(2 * MAX_DECEL * dist) 
                if vel < 1.:
                    vel = 0.
            wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            rospy.loginfo("wp.twist.twist.linear.x: " + str(wp.twist.twist.linear.x))

        return waypoints 


 

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.baseWayPoints = waypoints
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_light_waypoint = msg.data        
        rospy.loginfo("Detected light: " + str(msg.data))
        if self.traffic_light_waypoint > -1:
            # detected light is RED
            rospy.loginfo("RED traffic light detected")
            self.publish_final_wp()

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def distance_bw(self, w1, w2):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        return dl(w1.pose.position, w2.pose.pose.position)

    def distance(self, p1, p2):
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        dist = math.sqrt(x*x + y*y + z*z)
        return dist

    def closest_way_point(self):
        waypointslist = self.baseWayPoints.waypoints
        closestWayPoint = -1
        if len(waypointslist) > 0 and self.vehiclePose is not None:
            min_dist = self.distance_bw(self.vehiclePose, waypointslist[0])
            for i, waypoint in enumerate(waypointslist):
                dist = self.distance_bw(self.vehiclePose, waypoint)
                if dist< min_dist:
                    min_dist = dist
                    closestWayPoint = i
        return closestWayPoint

    def next_way_point(self):
        closestWayPoint = self.closest_way_point()
        if closestWayPoint!=-1:
            wp_x = self.baseWayPoints.waypoints[closestWayPoint].pose.pose.position.x
            wp_y = self.baseWayPoints.waypoints[closestWayPoint].pose.pose.position.y
            v_x = self.vehiclePose.pose.position.x
            v_y = self.vehiclePose.pose.position.y
            heading = math.atan2((wp_y - v_y), (wp_x - v_x))
            veh_quat_orient = self.vehiclePose.pose.orientation
            veh_quaternion = [veh_quat_orient.x, veh_quat_orient.y, 
                              veh_quat_orient.z, veh_quat_orient.w] 
            veh_euler = euler_from_quaternion(veh_quaternion)
            vehicle_angle = veh_euler[2]
            angle = abs(vehicle_angle - heading)
            angle = min(2*math.pi - angle, angle)
            if (angle > math.pi/4):
                ++closestWayPoint
            if(closestWayPoint == len(self.baseWayPoints.waypoints)):
                closestWayPoint = 0
   
        return closestWayPoint     
    
    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity
        #rospy.loginfo("velocity: " + str(velocity))
        #rospy.loginfo("waypoints[waypoint].twist.twist.linear.x: " + str(waypoints[waypoint].twist.twist.linear.x))

    # def distance(self, waypoints, wp1, wp2):
    #     dist = 0
    #     dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
    #     for i in range(wp1, wp2+1):
    #         dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
    #         wp1 = i
    #     return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
