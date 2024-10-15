import rclpy as rp
from rclpy.node import Node
from geometry_msgs.msg import Twist
import getch 

class Pubcmdvel(Node):
    def __init__(self):
        super().__init__('pubcmdvel')
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.msg = Twist()

        self.is_running = True
        self.create_timer(0.5, self.cmdcallback)  # Check key presses every 100 ms

    def cmdcallback(self):
        k = ord(getch.getch())  # Get the key press and convert to ASCII value

        if k == ord('w'):
            self.msg.linear.x = 10.0  # Move forward
            self.msg.angular.z = 0.0
        elif k == ord('s'):
            self.msg.linear.x = -10.0  # Move backward
            self.msg.angular.z = 0.0
        elif k == ord('a'):
            self.msg.linear.x = 0.0
            self.msg.angular.z = 10.0  # Turn left
        elif k == ord('d'):
            self.msg.linear.x = 0.0
            self.msg.angular.z = -10.0  # Turn right
        elif k == ord('q'):  # Exit condition
            self.is_running = False
            self.destroy_node()  # Clean up the node
            return

        # Publish the command
        self.pub_cmd_vel.publish(self.msg)

def main():
    rp.init()
    pubcmdvel = Pubcmdvel()

    while pubcmdvel.is_running:
        rp.spin_once(pubcmdvel)  # Spin the node

    rp.shutdown()

if __name__ == '__main__':
    main()
