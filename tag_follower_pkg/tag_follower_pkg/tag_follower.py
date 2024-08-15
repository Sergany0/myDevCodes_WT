import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from apriltag_msgs.msg import AprilTagDetectionArray

class TagFollower(Node):
    def __init__(self):
        super().__init__('tag_follower')
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.image_width = 640  # Replace with your image width
        self.image_height = 480  # Replace with your image height

    def listener_callback(self, msg):
        if len(msg.detections) > 0:
            tag = msg.detections[0]  # Assuming one tag is in view
            offset_x = tag.centre.x - (self.image_width / 2)

            twist = Twist()
            twist.linear.x = 0.0  # Adjust as needed
            twist.angular.z = -0.01 * offset_x  # Proportional control for centering
            self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    tag_follower = TagFollower()
    rclpy.spin(tag_follower)
    tag_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
