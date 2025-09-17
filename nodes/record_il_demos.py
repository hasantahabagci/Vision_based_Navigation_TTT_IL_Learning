#!/usr/bin/env python3
import csv, os, signal
from datetime import datetime
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

try:
    from vision_based_navigation_ttt.msg import TauComputation as TauMsg
except Exception:
    TauMsg = None

from std_msgs.msg import Header

def extract_tau_row(msg):
    from math import isnan
    vals = []
    # Reuse extractor from controller (inline here to be standalone)
    attrs = dir(msg)
    for names in (['tau_fl','tau_fr','tau_l','tau_r','tau_c'],
                  ['fl','fr','l','r','c'],
                  ['tau_el','tau_er','tau_l','tau_r','tau_c'],
                  ['el','er','l','r','c']):
        if all(n in attrs for n in names[:4]):
            vals = [float(getattr(msg,n)) for n in names if hasattr(msg,n)]
            break
    if not vals:
        for arr_name in ['tau','taus','ttt','roi_tau','roi_ttt']:
            if hasattr(msg, arr_name):
                arr = getattr(msg, arr_name)
                vals = [float(x) for x in list(arr)]
                break
    if not vals:
        raise RuntimeError("Cannot find τ fields on message.")
    # timestamp if available
    stamp_sec = 0.0
    if hasattr(msg, 'header') and isinstance(msg.header, Header):
        stamp = msg.header.stamp
        stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return stamp_sec, vals

class ILDemoLogger(Node):
    def __init__(self):
        super().__init__('il_demo_logger')
        self.declare_parameter('tau_topic', '/tau_computation')
        self.declare_parameter('cmd_topic', '/jackal_velocity_controller/cmd_vel')
        self.declare_parameter('out_csv',   'assets/il_demos.csv')

        tau_topic = self.get_parameter('tau_topic').get_parameter_value().string_value
        cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        out_csv   = self.get_parameter('out_csv').get_parameter_value().string_value
        os.makedirs(os.path.dirname(out_csv) or '.', exist_ok=True)
        self.f = open(out_csv, 'w', newline='')
        self.w = csv.writer(self.f)
        # header
        self.w.writerow(['t','tau_fl','tau_fr','tau_l','tau_r','tau_c','u','v'])

        self.latest_cmd = Twist()

        if TauMsg is not None:
            self.sub_tau = self.create_subscription(TauMsg, tau_topic, self.cb_tau, 10)
        else:
            self.sub_tau = self.create_subscription(type(None), tau_topic, self.cb_tau, 10)
        self.sub_cmd = self.create_subscription(Twist, cmd_topic, self.cb_cmd, 10)

        self.get_logger().info(f"Logging τ from {tau_topic} + cmd from {cmd_topic} → {out_csv}")

        # Graceful close on Ctrl+C
        signal.signal(signal.SIGINT, self._sigint)

    def cb_cmd(self, msg: Twist):
        self.latest_cmd = msg

    def cb_tau(self, msg):
        t, tau_vals = extract_tau_row(msg)
        # pad to 5 vals
        while len(tau_vals) < 5:
            tau_vals.append(-1.0)
        row = [t, tau_vals[0], tau_vals[1], tau_vals[2], tau_vals[3], tau_vals[4],
               self.latest_cmd.angular.z, self.latest_cmd.linear.x]
        self.w.writerow(row)
        # flush periodically
        if int(datetime.utcnow().timestamp()) % 5 == 0:
            self.f.flush()

    def _sigint(self, *_):
        self.f.flush(); self.f.close()
        self.get_logger().info("Saved and closed CSV. Bye.")
        rclpy.shutdown()

def main():
    rclpy.init()
    node = ILDemoLogger()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
