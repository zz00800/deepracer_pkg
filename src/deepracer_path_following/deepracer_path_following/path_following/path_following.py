#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys,os
import rclpy
import ros2pkg
from rclpy.node import Node
from rclpy.node import QoSProfile
import numpy as np
from nav_msgs.msg import Path, Odometry
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, PoseStamped, Point, Point32
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import PointCloud
from team2_interface.msg import PoseVel
# from morai_msgs.msg import CtrlCmd
# from ackermann_msgs.msg import AckermannDrive

from lib.utils import pathReader, findLocalPath, purePursuit, pidController

from math import cos,sin,sqrt,pow,atan2,pi

class pid_planner(Node):
    def __init__(self):

        super().__init__('pid_planner')
        qos_profile = QoSProfile(depth =10)

        self.ctrl_msg = Twist()
        self.target_velocity = 0.5 # 계산 잘 해보자
        self.configure()
        
        self.is_status=False ## 차량 상태 점검

        # path data reader
        path_reader = pathReader('path_extraction') ## 경로 파일의 패키지 위치
        self.global_path = path_reader.read_txt(self.path_file_name, self.path_frame) ## 

        # ros cmd publisher
        self.ctrl_msg = Twist()
        self.twist_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            qos_profile
        )

        # path publisher
        # self.global_path_pub = rclpy.Publisher('/global_path',Path, queue_size=1) ## global_path publisher
        # self.local_path_pub = rclpy.Publisher('/local_path',Path, queue_size=1) ## local_path publisher
        # self.odometry_path_pub = rclpy.Publisher('/odom_path', Path, queue_size=1) ## odometry history
        # self.odometry_path_msg = Path()

        # debug rviz publisher
        # self.fwd_point_pub = rclpy.Publisher('/forward_point', PointCloud, queue_size=1)

        # ros subscriber
        # rclpy.Subscriber("/Ego_globalstate", Odometry, self.statusCB) ## Vehicl Status Subscriber 
        self.status_subscriber = self.create_subscription(
            PoseVel,
            '/pos_vel',
            self.statusCB,
            qos_profile
        )
        
        # class
        self.pure_pursuit = purePursuit(self.vehicle_length, self.lfd, self.min_lfd, self.max_lfd) ## purePursuit import
        self.pid = pidController(self.p_gain, self.i_gain, self.d_gain, self.control_time)
        # ref_vel = float(self.reference_velocity)/float(3.6) # m/s
        # self.vel_planner = velocityPlanning(ref_vel, self.road_friction) ## 속도 계획 (reference velocity, friciton)
        # self.vel_profile = self.vel_planner.curveBasedVelocity(self.global_path,100)

        # time var
        rate = self.create_rate(self.frequency)

        while rclpy.ok():
            if self.is_status:
                self.spin_once()
                
            rate.sleep()
        
    def configure(self): ### 파라미터 불러오기
        self.path_file_name = "deepracer_path.txt" # rclpy.get_param("~path_file_name")
        self.platform = "deepracer" # rclpy.get_param("~platform")
        self.frequency = 20 # rclpy.get_param("~frequency")
        self.path_frame = "/odom" #rclpy.get_param("~path_frame")
        self.local_path_step = 10 # rclpy.get_param("~local_path_step")

        # Steering (purePursuit)
        self.vehicle_length = 0.5 # 측정값 넣기 # rclpy.get_param("~vehicle_length")
        self.lfd = 0.5 # rclpy.get_param("~initial_lfd")
        self.min_lfd = 0.5 # rclpy.get_param("~min_lfd")
        self.max_lfd = 3.0 # rclpy.get_param("~max_lfd")
        
        # PID Controller
        self.road_friction = 0.3 # rclpy.get_param("~road_friction")
        # self.reference_velocity = rclpy.get_param("~reference_velocity")
        self.p_gain = 1.0 # rclpy.get_param("~p_gain")
        self.i_gain = 0.0 # rclpy.get_param("~i_gain")
        self.d_gain = 0.05 # rclpy.get_param("~d_gain")
        self.control_time = float(1)/float(self.frequency)

    def spin_once(self): # 중추
        ## global_path와 차량의 status_msg를 이용해 현재 waypoint와 local_path를 생성
        self.local_path, self.current_waypoint = findLocalPath(self.global_path, self.status_msg, self.path_frame, self.local_path_step)
        
        # Steering Control (steering_angle; pure pursuit control)
        self.get_steering_angle()

        # Cruise Control (control_input; velocity)
        self.get_control_velocity()

        self.twist_publisher.publish(self.ctrl_msg) ## Vehicle Control 출력
    
    def get_steering_angle(self):
        self.pure_pursuit.getPath(self.local_path) ## pure_pursuit 알고리즘에 Local path 적용
        self.pure_pursuit.getEgoStatus(self.status_msg) ## pure_pursuit 알고리즘에 차량의 status 적용

        # ego_current_velocity = self.status_msg.twist.twist.linear (msg type changed)
        velocity = self.status_msg.velocity

        steering_angle, forward_point, current_lfd = self.pure_pursuit.steering_angle()
        L = self.vehicle_length # vehicle length (m)

        if (self.platform == "deepracer"): # 매우 중요 이걸로 steering 나옴
            self.ctrl_msg.angular.z = velocity * sin(steering_angle) / L  # angular velocity
            

        # debug with rviz marker
        # self.pubDebugMarker(forward_point, current_lfd)

        return steering_angle

    # get_control_velocity는 cruise control을 이용한 target velocity를 사용하여 속도제어 --> cruise control 사용 안하므로 속도제어 x
    # 나중에 속도제어 알고리즘을 따로 만들기
    def get_control_velocity(self): 
        ego_current_velocity = self.status_msg.velocity
        target_velocity = self.target_velocity
        # target_velocity = self.cc.acc(ego_current_velocity, self.vel_profile[self.current_waypoint]) ## advanced cruise control 적용한 속도 계획
        control_input = self.pid.pid(target_velocity, ego_current_velocity) ## 속도 제어를 위한 PID 적용 (target Velocity, Status Velocity)
        if control_input > 0:
                self.ctrl_msg.linear.x = control_input # (km/h)
                self.ctrl_msg.linear.y = 0
                self.ctrl_msg.linear.z = 0
                self.ctrl_msg.angular.x = 0
                self.ctrl_msg.angular.y = 0
                self.ctrl_msg.angular.z = 0
        else :
                self.ctrl_msg.linear.x = 0
                self.ctrl_msg.linear.y = 0
                self.ctrl_msg.linear.z = 0
                self.ctrl_msg.angular.x = 0
                self.ctrl_msg.angular.y = 0
                self.ctrl_msg.angular.z = 0
        '''
        if (self.platform == "real"):
            if control_input > 0:
                self.ctrl_msg.speed = control_input # (m/s)
            else:
                self.ctrl_msg.speed = 0
                self.ctrl_msg.acceleration = 0
                self.ctrl_msg.jerk = 0
        elif (self.platform == "sim"):
            if control_input > 0 :
                self.ctrl_msg.accel= control_input # (m/s)
                self.ctrl_msg.brake= 0
            else :
                self.ctrl_msg.accel= 0
                self.ctrl_msg.brake= -control_input
        elif (self.platform == "turtlebot"): #deepracer
            if control_input > 0:
                self.ctrl_msg.linear.x = control_input # (km/h)
                self.ctrl_msg.linear.y = 0
                self.ctrl_msg.linear.z = 0
                self.ctrl_msg.angular.x = 0
                self.ctrl_msg.angular.y = 0
            else :
                self.ctrl_msg.linear.x = 0
                self.ctrl_msg.linear.y = 0
                self.ctrl_msg.linear.z = 0
                self.ctrl_msg.angular.x = 0
                self.ctrl_msg.angular.y = 0
                self.ctrl_msg.angular.z = 0
        '''
    '''
    def pubDebugMarker(self, forward_point, current_lfd):
        # publish rviz maker (Forward Point)
        fwd_point_msg = PointCloud()
        fwd_point = Point32()
        fwd_point.x = forward_point.x
        fwd_point.y = forward_point.y
        fwd_point.z = 0

        fwd_point_msg.header.frame_id = self.path_frame
        fwd_point_msg.points.append(fwd_point)
        self.fwd_point_pub.publish(fwd_point_msg)
    '''

    def statusCB(self, msg): ## Vehicle Status Subscriber 
        self.is_status=True
        
        self.status_msg = msg # (PoseVel)
        # Ego_HeadingAngle = [self.status_msg.pose.pose.orientation.x, self.status_msg.pose.pose.orientation.y, self.status_msg.pose.pose.orientation.z, self.status_msg.pose.pose.orientation.w]
        
        '''# Map -> gps TF Broadcaster
        self.TFsender = tf.TransformBroadcaster()
        self.TFsender.sendTransform((self.status_msg.pose.pose.position.x, self.status_msg.pose.pose.position.y, 0),
                        Ego_HeadingAngle,
                        self.get_clock().now(),
                        "gps", # child frame "base_link"
                        self.path_frame) # parent frame "map"'''

        # Odometry history viewer
        last_point = PoseVel()
        last_point.pose.position.x = self.status_msg.pose_x
        last_point.pose.position.y = self.status_msg.pose_y
        last_point.pose.position.z = self.status_msg.pose_z
        last_point.pose.orientation.x = 0
        last_point.pose.orientation.y = 0
        last_point.pose.orientation.z = 0
        last_point.pose.orientation.w = 1

        self.twist_publisher.publish(self.ctrl_msg)
        self.get_logger().info('Published ctrl_msg: x_vel {0}, z_Ang {1}'.format(self.ctrl_msg.linear.x, self.ctrl_msg.angular.z))
        '''self.odometry_path_msg.header.frame_id = self.path_frame
        self.odometry_path_msg.poses.append(last_point)
        self.odometry_path_pub.publish(self.odometry_path_msg)  '''

    '''def getEgoVel(self):
        vx = self.status_msg.twist.twist.linear.x
        vy = self.status_msg.twist.twist.linear.y
        return np.sqrt(np.power(vx, 2) + np.power(vy,2))'''
    '''
    def twist_publish(self, linear, angular):
        # linear: self.toy_linear_x, angular: self.get_steering_angle()
        result = Twist()
        result.angular.z = angular
        result.linear.x = linear
        result.linear.y, result.linear.z, result.angular.x, result.angular.y = 0, 0, 0, 0
        self.twist_publisher.publish(result)
    '''

def main(args = None):
        rclpy.init(args=args)
        node = pid_planner()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            node.destroy_node()
            rclpy.shutdown()
            

if __name__ == '__main__':
    main()

'''if __name__ == '__main__':
    try:
        kcity_pathtracking = pid_planner()
    except rclpy.ROSInterruptException:
        pass'''