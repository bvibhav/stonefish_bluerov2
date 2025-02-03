#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import socket
import struct
import json
import time 

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

from tf_transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np

class Patch(Node):
    def __init__(self, node_name, namespace):
        super().__init__(node_name, namespace=namespace)

        self.namespace = self.get_namespace()[1:]

        # Subscribers
        self.create_subscription(Imu, "imu", self._imu_callback, 1),
        self.create_subscription(NavSatFix, "gps", self._gps_callback, 1),
        self.create_subscription(Odometry, "odometry", self._odom_callback, 1),

        # Publishers
        self.pub_pwm = self.create_publisher(Float64MultiArray, "setpoint/pwm", 1)

        # Publish everything
        self.timer = self.create_timer(1/50, self.looper)

        if self.namespace=='bluerov2':
            PORT = 9012
        elif self.namespace=='blueboat':
            PORT = 9002

        self.sock_sitl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_sitl.bind(('', PORT))
        self.sock_sitl.settimeout(0.1)

        self.imu = None
        self.gps = None
        self.odom = None

        # self.gps_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # IPV4, UDP
        # self.gps_addr = ("127.0.0.1", 25100)

    def _imu_callback(self, msg):
        self.imu = msg

    def _gps_callback(self, msg):
        self.gps = msg

    def _odom_callback(self, msg):
        self.odom = msg

    def looper(self):
        if self.imu is None or self.odom is None:
            self.get_logger().info("Wating for callbacks", once=False)
            time.sleep(.5)
            return
        
        self.get_logger().info("Callbacks received", once=True)
        
        try:
            data, address = self.sock_sitl.recvfrom(100)
        except Exception as ex:
            self.get_logger().info("Socket receive failed, is SITL running?", once=False)
            time.sleep(1)
            return 
    
        parse_format = 'HHI16H'
        magic = 18458

        if len(data) != struct.calcsize(parse_format):
            print("got packet of len %u, expected %u" % (len(data), struct.calcsize(parse_format)))
            return 
        
        decoded = struct.unpack(parse_format,data)

        if magic != decoded[0]:
            print("Incorrect protocol magic %u should be %u" % (decoded[0], magic))
            return 

        frame_rate_hz = decoded[1]
        frame_count = decoded[2]
        pwm = decoded[3:]

        if self.namespace=='bluerov2':
            pwm_thrusters = pwm[0:8]
            pwm_setpoint = [(x-1500)/400 for x in pwm_thrusters]

        if self.namespace=='blueboat':
            # print(pwm[2], pwm[0])

            # TAM = np.array([[.5, -.5],[-1, -1]])
            # pwm_setpoint_polar = np.array([(pwm[2]-1500)/400, (pwm[0]-1500)/400])
            # pwm_setpoint = np.matmul(np.linalg.pinv(TAM), pwm_setpoint_polar)    

            # NOTE: OG Blueboat
            pwm_setpoint = np.array([(pwm[0]-1500)/600, (pwm[2]-1500)/600])
            if abs(pwm[0]-1500)<=10:
                pwm_setpoint[0] = 0
            if abs(pwm[2]-1500)<=10:
                pwm_setpoint[1] = 0

            # pwm_setpoint = np.array([(pwm[0]-1500)/400, (pwm[1]-1500)/400])
            # pwm_setpoint[1] *= -1
                
        # print(pwm_setpoint)

        # print([pwm[2], pwm[0]])
        # print("{:.2f} {:.2f}".format(pwm_setpoint[0], pwm_setpoint[1]))

        # print(pwm_setpoint)
        msg_pwm = Float64MultiArray(data=pwm_setpoint)

        # Publish pwm message
        self.pub_pwm.publish(msg_pwm)

        # Set mesasges
        accel = (self.imu.linear_acceleration.x, self.imu.linear_acceleration.y, self.imu.linear_acceleration.z)
        gyro = (self.imu.angular_velocity.x, self.imu.angular_velocity.y, -self.imu.angular_velocity.z)
        
        pose_position = (
            self.odom.pose.pose.position.x,
            self.odom.pose.pose.position.y,
            self.odom.pose.pose.position.z
        )

        pose_attitude = euler_from_quaternion([
            self.odom.pose.pose.orientation.x,
            self.odom.pose.pose.orientation.y,
            self.odom.pose.pose.orientation.z,
            self.odom.pose.pose.orientation.w
        ])
        
        pose_attitude = [pose_attitude[0], pose_attitude[1], pose_attitude[2]]
        # print("Yaw:", np.rad2deg(pose_attitude[2]))
        
        twist_linear = (
            self.odom.twist.twist.linear.x,
            self.odom.twist.twist.linear.y,
            self.odom.twist.twist.linear.z,
            self.odom.twist.twist.angular.x,
            self.odom.twist.twist.angular.y,
            -self.odom.twist.twist.angular.z,
        )
        
        c_time = self.get_clock().now().to_msg()
        c_time = c_time.sec + c_time.nanosec/1e9

        # build JSON format
        IMU_fmt = {
            "gyro" : gyro,
            "accel_body" : accel
        }
        JSON_fmt = {
            "timestamp" : c_time,
            "imu" : IMU_fmt,
            "position" : pose_position,
            "attitude" : pose_attitude,
            "velocity" : twist_linear,                          
        }
        JSON_string = "\n" + json.dumps(JSON_fmt,separators=(',', ':')) + "\n"

        # Send to AP
        self.sock_sitl.sendto(bytes(JSON_string,"ascii"), address)

        # print(self.gps.latitude)

        # gps_data = {
        #         'time_usec' : int(c_time/1e3),                        # (uint64_t) Timestamp (micros since boot or Unix epoch)
        #         'gps_id' : 0,                           # (uint8_t) ID of the GPS for multiple GPS inputs
        #         # 'ignore_flags' : 8,                     # (uint16_t) Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum). All other fields must be provided.
        #         # 'time_week_ms' : 0,                     # (uint32_t) GPS time (milliseconds from start of GPS week)
        #         # 'time_week' : 0,                        # (uint16_t) GPS week number
        #         # 'fix_type' : 3,                         # (uint8_t) 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
        #         'lat' : int(self.gps.latitude*1e7),                              # (int32_t) Latitude (WGS84), in degrees * 1E7
        #         'lon' : int(self.gps.longitude*1e7),                              # (int32_t) Longitude (WGS84), in degrees * 1E7
        #         'alt' : 0,                              # (float) Altitude (AMSL, not WGS84), in m (positive for up)
        #         # 'hdop' : 1,                             # (float) GPS HDOP horizontal dilution of position in m
        #         # 'vdop' : 1,                             # (float) GPS VDOP vertical dilution of position in m
        #         # 'vn' : 0,                               # (float) GPS velocity in m/s in NORTH direction in earth-fixed NED frame
        #         # 've' : 0,                               # (float) GPS velocity in m/s in EAST direction in earth-fixed NED frame
        #         # 'vd' : 0,                               # (float) GPS velocity in m/s in DOWN direction in earth-fixed NED frame
        #         # 'speed_accuracy' : 0,                   # (float) GPS speed accuracy in m/s
        #         # 'horiz_accuracy' : 0,                   # (float) GPS horizontal accuracy in m
        #         # 'vert_accuracy' : 0,                    # (float) GPS vertical accuracy in m
        #         # 'satellites_visible' : 7                # (uint8_t) Number of satellites visible.
        # }

        # gps_data = json.dumps(gps_data)
        # self.gps_sock.sendto(gps_data.encode(), ("127.0.0.1", 25100))

def main(args=None):
    rclpy.init(args=args)

    # patch = Patch(node_name="ardusim_patch")
    patch = Patch(node_name="ardusim_patch", namespace='blueboat')
    
    rclpy.spin(patch)

    # Destroy the node explicitly, otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    patch.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
