#!/usr/bin/env python3
"""Decode the DRV8353 fault registers from /motor_states into /diagnostics.

The wheelbase firmware (wheelbase/Core/Src/freertos.c) puts FAULT_STATUS_1 of each
motor driver into JointState.effort[0..2]. Bits as in wheelbase/Core/Inc/drv8353.h.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import JointState

MOTORS = 3
FS1_BITS = {10: 'FAULT', 9: 'VDS_OCP', 8: 'GDF', 7: 'UVLO', 6: 'OTSD',
            5: 'VDS_HA', 4: 'VDS_LA', 3: 'VDS_HB', 2: 'VDS_LB', 1: 'VDS_HC', 0: 'VDS_LC'}
STALE_AFTER_NS = 2_000_000_000


def bit_names(value, table):
    return [name for bit, name in table.items() if value & (1 << bit)]


def decode(faults):
    """Return [(name, level, message, values)] per motor, level 0 OK, 2 ERROR."""
    out = []
    for i, fs1 in enumerate(faults):
        names = bit_names(fs1, FS1_BITS)
        out.append((f'wheelbase: motor {i + 1} driver', 2 if names else 0,
                    ', '.join(names) or 'OK', {'FAULT_STATUS_1': f'0x{fs1:03X}'}))
    return out


class MotorFaultDiagnostics(Node):

    def __init__(self):
        super().__init__('motor_fault_diagnostics')
        self.pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_subscription(JointState, '/motor_states', self.on_states, 10)
        self.create_timer(1.0, self.publish)
        self.last = None     # latest sample
        self.latched = None  # bits OR-ed since last publish, so short faults are not missed
        self.last_rx = None

    def on_states(self, msg):
        if len(msg.effort) < MOTORS:
            self.get_logger().warn(
                f'motor_states has {len(msg.effort)} effort values, expected {MOTORS}',
                throttle_duration_sec=5.0)
            return
        self.last = [int(e) for e in msg.effort[:MOTORS]]
        self.latched = self.last if self.latched is None else [
            a | b for a, b in zip(self.latched, self.last)]
        self.last_rx = self.get_clock().now()

    def publish(self):
        now = self.get_clock().now()
        stale = self.last_rx is None or (now - self.last_rx).nanoseconds > STALE_AFTER_NS
        arr = DiagnosticArray()
        arr.header.stamp = now.to_msg()
        for name, level, message, values in decode(self.latched or [0] * MOTORS):
            status = DiagnosticStatus(name=name, hardware_id='semubot_wheelbase')
            if stale:
                status.level, status.message = DiagnosticStatus.STALE, 'No data on /motor_states'
            else:
                status.level, status.message = bytes([level]), message
                status.values = [KeyValue(key=k, value=v) for k, v in values.items()]
            arr.status.append(status)
        self.pub.publish(arr)
        self.latched = self.last


def main():
    # Plain KeyboardInterrupt on SIGINT; rclpy's handler races spin() on Lyrical
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        rclpy.spin(MotorFaultDiagnostics())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
