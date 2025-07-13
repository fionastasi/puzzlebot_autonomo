#!/usr/bin/env python3 
import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Pose2D 
from geometry_msgs.msg import Twist 
from std_msgs.msg import Bool, String
import numpy as np 
import signal # To handle Ctrl+C 
import sys # To exit the program  

class OdometryControllerNode(Node):
    def __init__(self):  
        super().__init__('odometry_controller_node')  # Node name
        ###########  INIT PUBLISHERS ################ 
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10) 
        self.pub_pose_reached = self.create_publisher(String, 'pose_reached', 10)  # Publisher for pose_reached
        self.pub_restart_odometry = self.create_publisher(String, '/start', 10)
        
        signal.signal(signal.SIGINT, self.shutdown_function) 

        ############## INIT SUBSCRIBERS ##################  
        self.create_subscription(Pose2D, "pose", self.pose_cb, 10)  
        self.create_subscription(String, "zebra", self.zebra_cb, 10) 
        self.create_subscription(String, 'sign_detected', self.sign_detected_cb, 10)

        ############ ROBOT CONSTANTS ################  
        self.goal_received = False  # Initially no goal is received
        self.kv = 0.2 # Linear velocity gain
        self.kw = 0.2 # Angular velocity gain (increased for faster alignment)
        self.tolerance = 0.1 # Tolerance to reach the goal [m]
        self.angle_tolerance = 0.1 # Tolerance for angular alignment [rad]
        self.cmd_vel = Twist() 

        # Robot pose
        self.xr = 0.0 # Robot position x[m] 
        self.yr = 0.0 # Robot position y[m] 
        self.theta_r = 0.0 # Robot orientation [rad] 

        # Goal pose
        self.xg = 0.0  # Goal position x[m]
        self.yg = 0.0  # Goal position y[m]

        self.pose_reached = "false"
        self.sign_detected = "none"
        self.sign_temp = "none"

        self.path_right = [Pose2D(x=0.33, y=-0.03, theta=0.0), Pose2D(x=0.29, y=-0.15, theta=0.0)]
        self.path_left = [Pose2D(x=0.33, y=-0.03, theta=0.0), Pose2D(x=0.29, y=0.15, theta=0.0)]
        self.path_straight = [Pose2D(x=0.45, y=0.0, theta=0.0)]

        self.current_path = []
        self.current_index = 0

        self.processing_path = False

        self.restart_odometry = "line"  # Default value for odometry reset

        timer_period = 0.05  
        self.create_timer(timer_period, self.main_timer_cb) 
        self.get_logger().info("Node initialized!!") 

    def main_timer_cb(self): 
        if self.processing_path and self.current_path:
            # Procesamiento de path activo
            pose_destino = self.current_path[self.current_path_index]
            self.xg = pose_destino.x
            self.yg = pose_destino.y

            ed, etheta = self.get_errors()
            if ed < self.tolerance and abs(etheta) < self.angle_tolerance:
                self.current_path_index += 1
                if self.current_path_index >= len(self.current_path):
                    print("Path destino alcanzado")
                    self.processing_path = False
                    self.current_path = []
                    self.current_path_index = 0
                    self.pose_reached = "true"
                    self.pub_pose_reached.publish(String(data=self.pose_reached))
                    self.cmd_vel.linear.x = 0.0
                    self.cmd_vel.angular.z = 0.0
                    self.pub_cmd_vel.publish(self.cmd_vel)
                    return
            else:
                if abs(etheta) > self.angle_tolerance:
                    self.cmd_vel.linear.x = 0.0
                    self.cmd_vel.angular.z = self.kw * etheta * 3
                else:
                    self.cmd_vel.linear.x = 0.1
                    self.cmd_vel.angular.z = self.kw * etheta * 3
                self.pose_reached = "false"
                self.pub_pose_reached.publish(String(data=self.pose_reached))
                self.pub_cmd_vel.publish(self.cmd_vel)
        else:
            print("Esperando a que zebra sea true y se asigne un path...")

    def get_errors(self):
        # Compute the distance and angle errors given the goal and robot pose
        ed = np.sqrt((self.xg - self.xr)**2 + (self.yg - self.yr)**2)
        thetag = np.arctan2(self.yg - self.yr, self.xg - self.xr)
        etheta = thetag - self.theta_r
        # Limit the angle to be between -pi and pi
        etheta = np.arctan2(np.sin(etheta), np.cos(etheta))
        print("Distance error(ed): " + str(ed))
        print("Angle error(etheta): " + str(etheta))
        return ed, etheta

    def pose_cb(self, pose):  
        ## This function receives the /pose from the odometry_node 
        self.xr = pose.x 
        self.yr = pose.y 
        self.theta_r = pose.theta 

    def zebra_cb(self, msg):  
        if msg.data == "true" and not self.processing_path:
            self.pub_restart_odometry.publish(String(data="odometry")) 
            # Selecciona el path según la última señal detectada
            if self.sign_detected == "straight":
                self.current_path = self.path_straight.copy()
            elif self.sign_detected == "right":
                self.current_path = self.path_right.copy()
            elif self.sign_detected == "left":
                self.current_path = self.path_left.copy()
            else:
                self.get_logger().info("No se detectó señal válida, no se asigna path.")
                return
            if self.current_path:
                self.processing_path = True
                self.current_path_index = 0
                self.get_logger().info(f"Procesando path: {self.sign_detected}")
            else:
                self.get_logger().info("Path seleccionado está vacío.")
        elif msg.data == "false" and not self.processing_path:
            self.pub_restart_odometry.publish(String(data="line"))  # Solo si NO se está procesando un path

    def sign_detected_cb(self, msg):
        self.sign_temp = msg.data

        if self.sign_temp == "straight":
            self.sign_detected = "straight"

        elif self.sign_temp == "right":
            self.sign_detected = "right"

        elif self.sign_temp == "left":
            self.sign_detected = "left"

    def shutdown_function(self, signum, frame): 
        self.get_logger().info("Shutting down. Stopping robot...") 
        stop_twist = Twist()  # All zeros to stop the robot 
        self.pub_cmd_vel.publish(stop_twist) 
        rclpy.shutdown() 
        sys.exit(0) 

def main(args=None): 
    rclpy.init(args=args) 
    my_node=OdometryControllerNode() 
    rclpy.spin(my_node) 
    my_node.destroy_node() 
    rclpy.shutdown() 
     
if __name__ == '__main__': 
    main()