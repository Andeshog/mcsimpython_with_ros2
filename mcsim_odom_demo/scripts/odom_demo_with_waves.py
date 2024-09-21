#!/usr/bin/env python3

from MCSimPython.simulator import RVG_DP_6DOF
from MCSimPython.waves import WaveLoad, JONSWAP
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np

class OdomDemo(Node):
    def __init__(self):
        super().__init__('odom_waves_demo')

        self.init_simulator()

        self.publisher_ = self.create_publisher(Odometry, 'odom_waves', 10)
        self.timer_ = self.create_timer(self.dt, self.timer_callback)

        self.time = 0.0

        self.get_logger().info('Odometry demo node has been started.')


    def init_simulator(self):
        self.dt = 0.1
        self.vessel = RVG_DP_6DOF(dt=self.dt, method='RK4')
        self.Uc = 0.0 # Current velocity
        self.beta_c = np.pi / 4 # Current direction
        self.tau = np.array([0.0, 0.0, 0.0 ,0.0, 0.0, 0.0])

        hs = 2.5 # Significant wave height
        tp = 9.0 # Peak period
        gamma = 3.3 # Peak factor
        wp = 2*np.pi/tp # Peak frequency
        wmin = 0.5*wp
        wmax = 3.0*wp

        N = 100 # Number of wave components

        wave_freqs = np.linspace(wmin, wmax, N)

        jonswap = JONSWAP(wave_freqs)

        _, wave_spectrum = jonswap(hs=hs, tp=tp, gamma=gamma)

        dw = (wmax - wmin) / N
        wave_amps = np.sqrt(2 * wave_spectrum * dw)
        rand_phase = np.random.uniform(0, 2*np.pi, size=N)
        wave_angles = np.ones(N) * np.pi / 4

        self.waveload = WaveLoad(
            wave_amps=wave_amps,
            freqs=wave_freqs,
            eps=rand_phase,
            angles=wave_angles,
            config_file=self.vessel._config_file,
            interpolate=True,
            qtf_method="geo-mean",      # Use geometric mean to approximate the QTF matrices.
            deep_water=True,            # Assume deep water conditions.
        )

        self.eta = self.vessel.get_eta()
        self.nu = self.vessel.get_nu()

    def timer_callback(self):
        tau_wave = self.waveload(self.time, self.vessel.get_eta())
        self.tau = tau_wave
        self.vessel.integrate(self.Uc, self.beta_c, self.tau)
        self.eta = self.vessel.get_eta()
        self.nu = self.vessel.get_nu()

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self.eta[0]
        msg.pose.pose.position.y = self.eta[1]
        msg.pose.pose.position.z = self.eta[2]

        quat = self.euler_to_quat(self.eta[3], self.eta[4], self.eta[5])
        msg.pose.pose.orientation.w = quat[0]
        msg.pose.pose.orientation.x = quat[1]
        msg.pose.pose.orientation.y = quat[2]
        msg.pose.pose.orientation.z = quat[3]

        msg.twist.twist.linear.x = self.nu[0]
        msg.twist.twist.linear.y = self.nu[1]
        msg.twist.twist.linear.z = self.nu[2]
        msg.twist.twist.angular.x = self.nu[3]
        msg.twist.twist.angular.y = self.nu[4]
        msg.twist.twist.angular.z = self.nu[5]

        self.publisher_.publish(msg)

        self.time += self.dt

    @staticmethod
    def euler_to_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        w = cy * cp * cr + sy * sp * sr
        x = cy * cp * sr - sy * sp * cr
        y = sy * cp * sr + cy * sp * cr
        z = sy * cp * cr - cy * sp * sr

        return np.array([w, x, y, z])
    
def main(args=None):
    rclpy.init(args=args)
    odom_demo = OdomDemo()
    rclpy.spin(odom_demo)
    odom_demo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()