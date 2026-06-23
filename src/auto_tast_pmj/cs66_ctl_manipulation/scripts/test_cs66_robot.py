#!/usr/bin/env python3
"""
CS66 Robot Controller Test Script
This script demonstrates how to use the CS66 robot controller via ROS topics.
"""

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import String

class CS66RobotTester:
    def __init__(self):
        rospy.init_node('cs66_robot_tester', anonymous=True)
        
        # Publishers
        self.joint_command_pub = rospy.Publisher('/cs66/joint_command', Float64MultiArray, queue_size=10)
        self.tcp_command_pub = rospy.Publisher('/cs66/tcp_command', Pose, queue_size=10)
        self.emergency_stop_pub = rospy.Publisher('/cs66/emergency_stop', Bool, queue_size=10)
        
        # Subscribers
        self.joint_state_sub = rospy.Subscriber('/cs66/joint_states', JointState, self.joint_state_callback)
        self.tcp_pose_sub = rospy.Subscriber('/cs66/tcp_pose', Pose, self.tcp_pose_callback)
        self.robot_status_sub = rospy.Subscriber('/cs66/robot_status', String, self.robot_status_callback)
        
        # State variables
        self.current_joint_state = None
        self.current_tcp_pose = None
        self.robot_status = "UNKNOWN"
        
        rospy.loginfo("CS66 Robot Tester initialized")
    
    def joint_state_callback(self, msg):
        """Callback for joint state messages"""
        self.current_joint_state = msg
        rospy.loginfo_throttle(5, f"Joint states: {[f'{pos:.3f}' for pos in msg.position]}")
    
    def tcp_pose_callback(self, msg):
        """Callback for TCP pose messages"""
        self.current_tcp_pose = msg
        rospy.loginfo_throttle(5, f"TCP pose: x={msg.position.x:.3f}, y={msg.position.y:.3f}, z={msg.position.z:.3f}")
    
    def robot_status_callback(self, msg):
        """Callback for robot status messages"""
        self.robot_status = msg.data
        rospy.loginfo_throttle(5, f"Robot status: {self.robot_status}")
    
    def send_joint_command(self, joint_angles):
        """Send joint angle command to robot"""
        if len(joint_angles) != 6:
            rospy.logerr("Joint command must have 6 values")
            return False
        
        msg = Float64MultiArray()
        msg.data = joint_angles
        self.joint_command_pub.publish(msg)
        rospy.loginfo(f"Sent joint command: {joint_angles}")
        return True
    
    def send_tcp_command(self, x, y, z, rx=0, ry=0, rz=0):
        """Send TCP pose command to robot"""
        msg = Pose()
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        msg.orientation.x = rx
        msg.orientation.y = ry
        msg.orientation.z = rz
        msg.orientation.w = 1.0
        
        self.tcp_command_pub.publish(msg)
        rospy.loginfo(f"Sent TCP command: x={x}, y={y}, z={z}")
        return True
    
    def emergency_stop(self, stop=True):
        """Send emergency stop command"""
        msg = Bool()
        msg.data = stop
        self.emergency_stop_pub.publish(msg)
        rospy.loginfo(f"Emergency stop: {'ACTIVATED' if stop else 'RELEASED'}")
    
    def wait_for_robot_ready(self, timeout=30):
        """Wait for robot to be ready"""
        rospy.loginfo("Waiting for robot to be ready...")
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            if self.robot_status == "RUNNING":
                rospy.loginfo("Robot is ready!")
                return True
            
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logerr("Timeout waiting for robot to be ready")
                return False
            
            rospy.sleep(0.1)
        
        return False
    
    def run_test_sequence(self):
        """Run a test sequence to demonstrate robot control"""
        rospy.loginfo("Starting test sequence...")
        
        # Wait for robot to be ready
        if not self.wait_for_robot_ready():
            rospy.logerr("Robot not ready, aborting test")
            return
        
        rospy.sleep(2)  # Wait a bit more
        
        # Test 1: Move to home position
        rospy.loginfo("Test 1: Moving to home position")
        home_position = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]  # Home position
        self.send_joint_command(home_position)
        rospy.sleep(5)  # Wait for movement to complete
        
        # Test 2: Move to a different joint position
        rospy.loginfo("Test 2: Moving to test position")
        test_position = [0.5, -1.0, 0.5, -1.0, 0.5, 0.0]
        self.send_joint_command(test_position)
        rospy.sleep(5)
        
        # Test 3: Move back to home
        rospy.loginfo("Test 3: Moving back to home position")
        self.send_joint_command(home_position)
        rospy.sleep(5)
        
        rospy.loginfo("Test sequence completed!")

def main():
    try:
        tester = CS66RobotTester()
        
        # Wait for topics to be available
        rospy.sleep(2)
        
        # Run test sequence
        tester.run_test_sequence()
        
        # Keep the node running
        rospy.loginfo("Test completed. Press Ctrl+C to exit.")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Test interrupted by user")
    except Exception as e:
        rospy.logerr(f"Test failed: {e}")

if __name__ == '__main__':
    main()
