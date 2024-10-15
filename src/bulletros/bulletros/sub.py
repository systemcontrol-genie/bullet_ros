import rclpy as rp
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
import pybullet as p
import pybullet_data
import time

class Subcmdvel(Node):
    
    def __init__(self):
        super().__init__('subcmdvel')
        qos_profile = QoSProfile(depth=10)
        self.sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.cmdcallback, qos_profile)

        p.connect(p.GUI)
        p.setGravity(0, 0, -9.8)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        self.planed = p.loadURDF('plane.urdf')
        self.car = p.loadURDF('/home/lim/bullet_ros/src/bulletros/bulletros/robot2.urdf', basePosition=[0, 0, 0])
        p.changeDynamics(self.car, -1, mass=100)
        self.create_timer(0.016, self.step_simulation)
    def step_simulation(self):
        p.stepSimulation()

    def cmdcallback(self, msg):
        print("Linear Velocity (x):", msg.linear.x)
        print("Angular Velocity (z):", msg.angular.z)

        # 속도 값 가져오기
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        maxForce = 10

        # 모터 제어
        p.setJointMotorControl2(self.car, jointIndex=1, controlMode=p.VELOCITY_CONTROL, targetVelocity=linear_velocity + angular_velocity, force=maxForce)
        
        p.setJointMotorControl2(self.car, jointIndex=2, controlMode=p.VELOCITY_CONTROL, targetVelocity=linear_velocity - angular_velocity, force=maxForce)
def main(args=None):
    rp.init(args=args)
    subcmdvel = Subcmdvel()
    rp.spin(subcmdvel)
    subcmdvel.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
