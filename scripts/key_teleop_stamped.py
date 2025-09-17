#!/usr/bin/env python3
import sys
import time
from dataclasses import dataclass

import pygame
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped  # Stamped for Jackal's controller

@dataclass
class Speeds:
    linear_x: float = 0.6   # m/s forward/back
    angular_z: float = 1.0  # rad/s left/right turn

class KeyTeleopStamped(Node):
    def __init__(self,
                 topic='/jackal_velocity_controller/cmd_vel',
                 hz=30.0,
                 speeds: Speeds = Speeds(),
                 frame_id='base_link'):
        super().__init__('key_teleop_stamped')
        self.pub = self.create_publisher(TwistStamped, topic, 10)
        self.dt = 1.0 / hz
        self.speeds = speeds
        self.frame_id = frame_id

        # Minimal pygame window to capture continuous key state
        pygame.init()
        pygame.display.set_caption('Jackal Key Teleop (hold to move)')
        # Small hidden-ish window; we still need it focused to read key state
        self.screen = pygame.display.set_mode((360, 120))
        self.font = pygame.font.SysFont(None, 22)

        self.last_send = time.time()
        self.timer = self.create_timer(self.dt, self._tick)

    def _compose_twist(self, keys):
        """
        Map keys to velocities.
        - Forward/back:   W / S or Up / Down
        - Turn left/right: A / D or Left / Right
        """
        v = 0.0
        w = 0.0

        # Forward/back
        if keys[pygame.K_w] or keys[pygame.K_UP]:
            v += self.speeds.linear_x
        if keys[pygame.K_s] or keys[pygame.K_DOWN]:
            v -= self.speeds.linear_x

        # Turning
        if keys[pygame.K_a] or keys[pygame.K_LEFT]:
            w += self.speeds.angular_z
        if keys[pygame.K_d] or keys[pygame.K_RIGHT]:
            w -= self.speeds.angular_z

        return v, w

    def _publish(self, v, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = v
        msg.twist.angular.z = w
        self.pub.publish(msg)

    def _draw(self, v, w):
        self.screen.fill((245, 245, 245))
        lines = [
            "Click this window, then hold keys to move; release to stop.",
            "W/Up = forward, S/Down = back, A/Left = turn left, D/Right = turn right",
            f"lin_x: {v:+.2f} m/s   ang_z: {w:+.2f} rad/s   topic: {self.pub.topic_name}",
            "Press ESC or close window to quit."
        ]
        y = 10
        for text in lines:
            surf = self.font.render(text, True, (0, 0, 0))
            self.screen.blit(surf, (10, y))
            y += 24
        pygame.display.flip()

    def _tick(self):
        # Handle window events (close, etc.)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                rclpy.shutdown()
                return
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                rclpy.shutdown()
                return

        keys = pygame.key.get_pressed()
        v, w = self._compose_twist(keys)
        self._publish(v, w)
        self._draw(v, w)
        self.last_send = time.time()

def main():
    rclpy.init()
    # Optional CLI overrides:
    #   python3 key_teleop_stamped.py /jackal_velocity_controller/cmd_vel 30
    topic = sys.argv[1] if len(sys.argv) > 1 else '/jackal_velocity_controller/cmd_vel'
    hz = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
    node = KeyTeleopStamped(topic=topic, hz=hz)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        pygame.quit()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
