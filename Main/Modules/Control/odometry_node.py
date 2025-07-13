import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Pose2D 
from std_msgs.msg import Float32 
from rclpy import qos 
import numpy as np 
from std_msgs.msg import Bool, String

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        self.pub_pose = self.create_publisher(Pose2D, 'pose', 10)

        self.create_subscription(Float32, "VelocityEncR", self.wr_cb, qos.qos_profile_sensor_data)
        self.create_subscription(Float32, "VelocityEncL", self.wl_cb, qos.qos_profile_sensor_data)
        self.create_subscription(String, "start", self.reset_cb, 10)

        self.r = 0.05
        self.L = 0.19
        self.wl = 0.0
        self.wr = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.robot_pose = Pose2D()
        self.prev_time_ns = self.get_clock().now().nanoseconds

        self.reset = ""

        timer_period = 0.05
        self.create_timer(timer_period, self.main_timer_cb)
        self.get_logger().info("odometry node started")

    def main_timer_cb(self):

        if self.reset == "line":
            print("recibiendo: ", str(self.reset))
            self.get_logger().info("Resetting odometry to zero.")
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            self.prev_time_ns = self.get_clock().now().nanoseconds
            self.robot_pose.x = 0.0
            self.robot_pose.y = 0.0
            self.robot_pose.theta = 0.0

        elif self.reset == "odometry":
            print("recibiendo: ", str(self.reset))

            v, w = self.get_robot_vel(self.wr, self.wl)
            self.update_robot_pose(v, w)

            print("xr: ", str(self.x))
            print("yr: ", str(self.y))
            print("theta_r: ", str(self.theta))

            self.pub_pose.publish(self.robot_pose)

    def wl_cb(self, wl):  
        ## This function receives the left wheel speed from the encoders  
        self.wl = wl.data 

         
    def wr_cb(self, wr):  
        ## This function receives the right wheel speed from the encoders 
        self.wr = wr.data 

    def get_robot_vel(self, wr, wl):
        v = self.r * (wr + wl) / 2.0
        w = self.r * (wr - wl) / self.L
        print("v: " + str(v))
        print("w: " + str(w))
        return v, w
    
    def reset_cb(self, msg):
            
        self.reset = msg.data


    def update_robot_pose(self, v, w):

        dt = (self.get_clock().now().nanoseconds - self.prev_time_ns) *10**-9
        self.prev_time_ns = self.get_clock().now().nanoseconds
        self.x = self.x + v * np.cos(self.theta) * dt
        self.y = self.y + v * np.sin(self.theta) * dt
        self.theta = self.theta + w * dt

        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

        self.robot_pose.x = self.x
        self.robot_pose.y = self.y
        self.robot_pose.theta = self.theta

        print("x: " + str(self.x))
        print("y: " + str(self.y))
        print("theta: " + str(self.theta))


def main(args=None):
    rclpy.init(args=args)
    my_node=OdometryNode()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


    