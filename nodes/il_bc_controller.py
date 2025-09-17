#!/usr/bin/env python3
import json, os, time
from typing import List
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Try to import the custom Tau message; fall back to generic if needed
# Adjust this import if your package name differs.
try:
    from vision_based_navigation_ttt.msg import TauComputation as TauMsg
except Exception:
    TauMsg = None  # We will still subscribe generically and try to introspect fields

# Torch is only used for inference here; install via pip (see steps).
import torch

def _extract_tau_fields(msg) -> List[float]:
    """
    Robust extractor for 4–5 ROI τs from TauComputation.* message.
    Accepts various field namings; -1.0 means 'invalid' per README.
    Priority order: [fl, fr, l, r] and optional center.
    """
    # candidate field name sets (leftmost preferred)
    candidates = [
        ['tau_fl','tau_fr','tau_l','tau_r','tau_c'],
        ['fl','fr','l','r','c'],
        ['tau_el','tau_er','tau_l','tau_r','tau_c'],
        ['el','er','l','r','c'],
        ['roi_fl','roi_fr','roi_l','roi_r','roi_c'],
        ['far_left','far_right','left','right','center'],
        ['ttt_fl','ttt_fr','ttt_l','ttt_r','ttt_c'],
    ]
    attrs = dir(msg)
    for names in candidates:
        if all(n in attrs for n in names[:4]):
            vals = [float(getattr(msg, n)) for n in names if hasattr(msg, n)]
            return vals
    # array-like fallback (e.g., msg.tau or msg.ttt as list[5])
    for arr_name in ['tau','taus','ttt','roi_tau','roi_ttt']:
        if hasattr(msg, arr_name):
            arr = getattr(msg, arr_name)
            try:
                vals = [float(x) for x in list(arr)]
                return vals
            except Exception:
                pass
    raise RuntimeError("Could not find τ fields on TauComputation message.")

class ILBCController(Node):
    def __init__(self):
        super().__init__('il_bc_controller')
        # -------- params ----------
        self.declare_parameter('tau_topic', '/tau_computation')
        self.declare_parameter('cmd_vel_topic', 'jackal_velocity_controller/cmd_vel')
        self.declare_parameter('policy_path', 'assets/il_bc.ts')   # TorchScript file
        self.declare_parameter('stats_path',  'assets/il_bc_stats.json') # input norm
        self.declare_parameter('use_center',  False)  # train w/ center? then True
        self.declare_parameter('v_fixed',     1.0)    # constant forward speed
        self.declare_parameter('max_u',       1.0)    # clamp yaw rate (README param) 
        self.declare_parameter('min_feat_val', -0.5)  # if τ == -1 (invalid), replace with this
        self.declare_parameter('deadband_invalid', 2) # if >=N invalids → skip publish (fallback)

        tau_topic    = self.get_parameter('tau_topic').get_parameter_value().string_value
        cmd_topic    = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        policy_path  = self.get_parameter('policy_path').get_parameter_value().string_value
        stats_path   = self.get_parameter('stats_path').get_parameter_value().string_value
        self.use_c   = self.get_parameter('use_center').get_parameter_value().bool_value
        self.v_fixed = float(self.get_parameter('v_fixed').value)
        self.max_u   = float(self.get_parameter('max_u').value)
        self.min_feat_val = float(self.get_parameter('min_feat_val').value)
        self.deadband_invalid = int(self.get_parameter('deadband_invalid').value)

        # Load model + stats
        if not os.path.exists(policy_path):
            raise FileNotFoundError(f"policy_path not found: {policy_path}")
        if not os.path.exists(stats_path):
            raise FileNotFoundError(f"stats_path not found: {stats_path}")
        self.model = torch.jit.load(policy_path, map_location='cpu').eval()
        with open(stats_path, 'r') as f:
            stats = json.load(f)
        self.mu = np.array(stats['mu'], dtype=np.float32)
        self.std = np.array(stats['std'], dtype=np.float32)
        self.in_dim = len(self.mu)

        self.pub = self.create_publisher(Twist, cmd_topic, 10)
        # Subscribe to τ topic (typed if available, otherwise generic)
        if TauMsg is not None:
            self.sub = self.create_subscription(TauMsg, tau_topic, self.cb_tau, 10)
        else:
            from rclpy.qos import qos_profile_sensor_data
            # generic subscription (serialize/deser handled by ROS2)
            self.sub = self.create_subscription(type(None), tau_topic, self.cb_tau, qos_profile_sensor_data)

        self.last_u = 0.0
        self.get_logger().info(f"IL BC controller up. Sub: {tau_topic} → Pub: {cmd_topic}")

    def _prep_input(self, tau_vals: List[float]) -> np.ndarray:
        # sanitize invalid ROIs: replace -1.0 (per README) with min_feat_val, count invalids
        invalids = 0
        clean = []
        for v in tau_vals:
            if v is None or (isinstance(v, float) and v < 0.0 + 1e-9):
                clean.append(self.min_feat_val)
                invalids += 1
            else:
                clean.append(float(v))
        # reduce to 4-D or 5-D depending on training
        if not self.use_c and len(clean) >= 4:
            x = np.array([clean[0], clean[1], clean[2], clean[3]], dtype=np.float32)
        else:
            x = np.array(clean[:5], dtype=np.float32)
        return x, invalids

    def cb_tau(self, msg):
        try:
            tau_vals = _extract_tau_fields(msg)
        except Exception as e:
            self.get_logger().warn(f"τ extraction failed: {e}")
            return

        x, n_inv = self._prep_input(tau_vals)
        if n_inv >= self.deadband_invalid:
            # Too many invalid ROIs → skip (let other controllers act)
            return

        # normalize
        if x.shape[0] != self.in_dim:
            # handle mismatch by trimming or padding
            if x.shape[0] > self.in_dim:
                x = x[:self.in_dim]
            else:
                x = np.pad(x, (0, self.in_dim - x.shape[0]), constant_values=self.min_feat_val)
        x_n = (x - self.mu) / (self.std + 1e-8)
        with torch.no_grad():
            u = float(self.model(torch.from_numpy(x_n)).item())
        # clamp yaw rate
        u = max(-self.max_u, min(self.max_u, u))

        msg_out = Twist()
        msg_out.linear.x  = self.v_fixed    # constant forward speed (README default ~1 m/s)
        msg_out.angular.z = u
        self.pub.publish(msg_out)
        self.last_u = u

def main():
    rclpy.init()
    node = ILBCController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
