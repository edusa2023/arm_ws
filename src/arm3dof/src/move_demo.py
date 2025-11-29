#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class ArmMoveDemo(Node):
    def __init__(self):
        super().__init__('arm_move_demo')
        self.declare_parameter('hz', 50.0)
        self.declare_parameter('hold_sec', 2.0)
        self.declare_parameter('slew_rate', 1.0)

        self.hz = self.get_parameter('hz').value
        self.hold_sec = self.get_parameter('hold_sec').value
        self.slew_rate = self.get_parameter('slew_rate').value

        self.pub_j1 = self.create_publisher(Float64, '/arm/j1/cmd_pos', 10)
        self.pub_j2 = self.create_publisher(Float64, '/arm/j2/cmd_pos', 10)
        self.pub_j3 = self.create_publisher(Float64, '/arm/j3/cmd_pos', 10)

        self.poses = [[0.0, -0.5, 0.5], [1.57, 0.0, -0.5], [-1.57, 0.5, -0.5]]
        self.current_joints = [0.0, 0.0, 0.0]
        self.target_joints = self.poses[0]
        self.pose_idx = 0
        self.state = 'move'
        self.hold_until = 0

        self.timer = self.create_timer(1.0/self.hz, self.tick)

    def tick(self):
        if self.state == 'move':
            reached = True
            max_step = self.slew_rate * (1.0/self.hz)
            for i in range(3):
                diff = self.target_joints[i] - self.current_joints[i]
                if abs(diff) > 1e-3:
                    self.current_joints[i] += clamp(diff, -max_step, max_step)
                    reached = False
            self.publish_all()
            if reached:
                self.state = 'hold'
                self.hold_until = self.get_clock().now().nanoseconds + (self.hold_sec * 1e9)
        elif self.state == 'hold':
            self.publish_all()
            if self.get_clock().now().nanoseconds > self.hold_until:
                self.pose_idx = (self.pose_idx + 1) % 3
                self.target_joints = self.poses[self.pose_idx]
                self.state = 'move'

    def publish_all(self):
        self.pub_j1.publish(Float64(data=self.current_joints[0]))
        self.pub_j2.publish(Float64(data=self.current_joints[1]))
        self.pub_j3.publish(Float64(data=self.current_joints[2]))

def main():
    rclpy.init()
    node = ArmMoveDemo()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()