#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Odometry
from scipy import interpolate
import tf2_ros
from scipy.spatial.transform import Rotation as R

class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')
        # TODO: create ROS subscribers and publishers
        self.pose_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.pose_callback, 10)
        self.waypoint_pub = self.create_publisher(MarkerArray, '/waypoints', 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.target = 0
        self.L = 1.5
        self.Kp = 1.0
        self.create_waypoints()

    def create_waypoints(self):
        #Define skeleton waypoints to interpolate between
        x_skel = [9.24423, 9.54901, 9.45486, 8.95488, 5.41726, -1.9899, -8.85176, -11.8562, -13.0861, -13.725, -13.7381, -13.3022, -12.3192, -7.07787, 0.785085, 2.83137, 8.51307, 9.71355]
        y_skel = [0.139301, 5.30634, 8.1212, 8.74803, 8.63542, 8.74653, 8.84947, 8.74803, 8.3686, 7.39398, 4.29553, 0.024683, -0.30985, -0.15942, -0.186643, -0.126611, -0.121112, 1.02859]
        self.waypoints = []

        tck, u = interpolate.splprep([x_skel, y_skel], s=0, per=True) #Interpolate the spline
        x, y = interpolate.splev(np.linspace(0, 1, 400), tck)

        for i in range(len(x)):
            self.waypoints.append([x[i], y[i]])

    def publish_waypoints(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.type = Marker.POINTS
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.id = 0

        marker.points = [Point(x=x, y=y, z=0.0) for x,y in self.waypoints]


        target = Marker()
        target.header.frame_id = 'map'
        target.type = Marker.POINTS
        target.scale.x = 0.2
        target.scale.y = 0.2
        target.color.r = 0.0
        target.color.g = 0.0
        target.color.b = 1.0
        target.color.a = 1.0
        target.id = 1

        target.points = [Point(x=self.target[0], y=self.target[1], z=0.0)]
        
        marker_array = MarkerArray()
        marker_array.markers = [marker, target]

        self.waypoint_pub.publish(marker_array)

    def find_target_waypoint(self, pose_msg):
        x = pose_msg.pose.pose.position.x
        y = pose_msg.pose.pose.position.y
        arr = self.waypoints.copy()
        arr.append(arr[0]) #Add first element again so we have a loop
        for i in range(0, len(arr)-1): #Iterate forwards so we choose a point ahead of us
            p = arr[i]
            next_p = arr[i+1]
            if (p[0] - x)**2 + (p[1] - y)**2 <= self.L**2 and (next_p[0] - x)**2 + (next_p[1] - y)**2 > self.L**2:
                self.target = [(p[0] + next_p[0]) / 2, (p[1] + next_p[1]) / 2]

    def transform_target(self, pose_msg, x, y):
        car_x = pose_msg.pose.pose.position.x
        car_y = pose_msg.pose.pose.position.y
        car_z = 0
        r = R.from_quat([pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y, 
                         pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w])
        #TODO
        rotation_matrix = r.as_matrix()
        h_matrix = np.array([[rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2], car_x],
                             [rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2], car_y],
                             [rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2], car_z],
                             [0, 0, 0, 1]])
        point = np.array([x, y, 0, 1])
        new_point = np.linalg.inv(h_matrix) @ point #Point in car's reference frame
        print(new_point)
        
        return new_point[0], new_point[1]

    def pose_callback(self, pose_msg):

        #find the current waypoint to track using methods mentioned in lecture
        self.find_target_waypoint(pose_msg)

        #transform goal point to vehicle frame of reference
        car_targetx, car_targety = self.transform_target(pose_msg, self.target[0], self.target[1])
        
        #calculate curvature/steering angle
        if car_targety >= 0: #TURN LEFT
            steering_angle = 2.0*np.abs(car_targety)/(self.L**2) * self.Kp
        else: #TURN RIGHT
            steering_angle = -2.0*np.abs(car_targety)/(self.L**2) * self.Kp
        print(steering_angle)

        #publish drive message, don't forget to limit the steering angle.
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = steering_angle
        msg.drive.speed = 4.0
        self.drive_pub.publish(msg)

        self.publish_waypoints()

def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
