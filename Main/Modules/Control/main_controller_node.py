import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Pose2D 
from geometry_msgs.msg import Twist 
from std_msgs.msg import Bool, String, Float32
import numpy as np 
import signal # To handle Ctrl+C 
import sys # To exit the program  

class MainControllerNode(Node):
    def __init__(self):
        super().__init__('main_controller_node')
        
        ###########  INIT PUBLISHERS ################ 
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)

        signal.signal(signal.SIGINT, self.shutdown_function) 

        ############## INIT SUBSCRIBERS ##################  
        self.create_subscription(Float32, "error", self.error_cb, 10)
        self.create_subscription(String, "color_detected", self.color_detected_cb, 10)
        self.create_subscription(String, "sign_detected", self.sign_detected_cb, 10)
        self.create_subscription(String, "pose_reached", self.pose_reached_cb, 10)
        self.create_subscription(String, "zebra", self.zebra_cb, 10)

        ############# DEFINE VARIABLES FOR THE PUZZLEBOT ################
        # Variables for line follower
        self.v = 0.030
        self.kw = 0.017  # Angular velocity gain (increased for faster alignment)
        self.w = 0.0  # Angular velocity (initialized to zero)
        self.speed_multiplier = 3 
        self.cmd_vel = Twist()

        # Variables for stop light detection
        self.stop_light_state = "green"  # Default stop light state
        self.sign_detectec_state = "none"
        self.giveaway_flag = False
        self.danger_flag = False
        self.stop_flag = False
        self.inicial_time = 10

        # Variables for odometry
        self.pose_reached = "true"
        self.zebra = "false"

        self.speed_factor = 1.0

        timer_period = 0.05  
        self.create_timer(timer_period, self.main_timer_cb) 
        self.get_logger().info("Node initialized!!") 

    def main_timer_cb(self):
        # Si zebra es true y aún no se ha llegado al destino, no publicar cmd_vel
        if self.zebra == "true" and self.pose_reached == "false":
            print("Modo odometría activo, esperando a llegar al destino. No se publica cmd_vel desde main_controller.")
            return
        
        
        elif self.zebra == "false" and self.pose_reached == "true":
            # Check if the stop light is red
            if self.stop_light_state == "red":
                print("Stop light is red. Stopping robot.")
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                self.pub_cmd_vel.publish(self.cmd_vel)
                return
            
            elif self.stop_light_state == "yellow":
                print("Stop light is yellow. Stopping robot.")
                # Reduce the linear velocity to half
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                self.pub_cmd_vel.publish(self.cmd_vel)
                return

            elif self.stop_light_state == "green":
                print("Stop light is green. Moving robot.")
                self.speed_factor = 1.0
            
            elif self.stop_light_state == "none":
                print("No stop light detected. Moving robot.")
                self.speed_factor = self.speed_factor


            if self.sign_detectec_state == "stop" and not self.stop_flag:
                # Stop the robot for 10 seconds when a stop sign is detected
                print("Stop sign detected. Stopping robot.")
                self.inicial_time = self.get_clock().now().nanoseconds / 1e9  # Get current time in seconds
                self.stop_flag = True

            elif self.sign_detectec_state == "giveaway" and not self.giveaway_flag:
                # Reduce speed to half when a giveaway sign is detected for 5 seconds
                print("Giveaway sign detected. Slowing down robot.")
                self.giveaway_flag = True
                self.inicial_time = self.get_clock().now().nanoseconds / 1e9  # Get current time in seconds

            elif self.sign_detectec_state == "danger" and not self.danger_flag:
                # Reduce speed to half when a danger sign is detected for 10 seconds
                print("Danger sign detected. Slowing down robot.")
                self.danger_flag = True
                self.inicial_time = self.get_clock().now().nanoseconds / 1e9  # Get current time in seconds
            
            elif self.sign_detectec_state == "none":
                print("No sign detected. Moving robot.")
                speed_sign_detected = 1.0

            if self.giveaway_flag == True and self.get_clock().now().nanoseconds / 1e9 - self.inicial_time < 5:
                speed_sign_detected = 0.5
                self.cmd_vel.linear.x = self.v * self.speed_factor * self.speed_multiplier * speed_sign_detected
                self.cmd_vel.angular.z = self.w
                self.pub_cmd_vel.publish(self.cmd_vel)

                if self.get_clock().now().nanoseconds / 1e9 - self.inicial_time >= 5:
                    self.giveaway_flag = False

                return
            
            elif self.danger_flag == True and self.get_clock().now().nanoseconds / 1e9 - self.inicial_time < 10:
                speed_sign_detected = 0.5
                self.cmd_vel.linear.x = self.v * self.speed_factor * self.speed_multiplier * speed_sign_detected
                self.cmd_vel.angular.z = self.w
                self.pub_cmd_vel.publish(self.cmd_vel)

                if self.get_clock().now().nanoseconds / 1e9 - self.inicial_time >= 10:
                    self.danger_flag = False
                
                return
            
            elif self.stop_flag == True and self.get_clock().now().nanoseconds / 1e9 - self.inicial_time < 10:
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                self.pub_cmd_vel.publish(self.cmd_vel)

                if self.get_clock().now().nanoseconds / 1e9 - self.inicial_time >= 10:
                    self.stop_flag = False
                
                return

            #Pub the speed
            self.cmd_vel.linear.x = self.v * self.speed_factor * self.speed_multiplier
            self.cmd_vel.angular.z = self.w
            self.pub_cmd_vel.publish(self.cmd_vel)

    def error_cb(self, msg):
        # Calculates the error received from the follower_node
        self.error = msg.data
        
        # Calculate omega
        self.w = self.error * self.kw

        # Calculate v
        self.speed_multiplier = 3.7 + abs(self.error) * (0.10 - 3.7) / (80/2)

    def color_detected_cb(self, msg):
        ## This function receives the /stop_light from the stop_light node 
        self.stop_light_state = msg.data
        self.get_logger().info(f"Stop light state: {self.stop_light_state}")

    def sign_detected_cb(self, msg):
        # Receives the /sign_detected from the sign_detected node
        self.sign_detectec_state = msg.data
        self.get_logger().info(f"Sign detected: {self.sign_detectec_state}")

    def zebra_cb(self, msg):
        self.zebra = msg.data

    def pose_reached_cb(self, msg):
        self.pose_reached = msg.data

    def shutdown_function(self, signum, frame): 
        self.get_logger().info("Shutting down. Stopping robot...") 
        stop_twist = Twist()  # All zeros to stop the robot 
        self.pub_cmd_vel.publish(stop_twist) 
        rclpy.shutdown() 
        sys.exit(0) 

def main(args=None): 
    rclpy.init(args=args) 
    my_node=MainControllerNode() 
    rclpy.spin(my_node) 
    my_node.destroy_node() 
    rclpy.shutdown() 


if __name__ == '__main__':
    main()
