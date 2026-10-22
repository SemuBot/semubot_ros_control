#!/usr/bin/env python3
"""Decode /motor_faults from the wheelbase firmware into /diagnostics.

motor_faults layout (SemuBot-Firmware wheelbase/Core/Src/freertos.c):
  [m1 FAULT_STATUS_1, m1 VGS_STATUS_2, m2 FS1, m2 VGS2, m3 FS1, m3 VGS2, fw_flags]
DRV8353 bits as in wheelbase/Core/Inc/drv8353.h.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import UInt16MultiArray

MSG_LEN = 7
FS1_BITS = {10: 'FAULT', 9: 'VDS_OCP', 8: 'GDF', 7: 'UVLO', 6: 'OTSD',
            5: 'VDS_HA', 4: 'VDS_LA', 3: 'VDS_HB', 2: 'VDS_LB', 1: 'VDS_HC', 0: 'VDS_LC'}
VGS2_BITS = {10: 'SA_OC', 9: 'SB_OC', 8: 'SC_OC', 7: 'OTW', 6: 'CPUV',
             5: 'VGS_HA', 4: 'VGS_LA', 3: 'VGS_HB', 2: 'VGS_LB', 1: 'VGS_HC', 0: 'VGS_LC'}
WARN_ONLY = {'OTW'}
FW_CMD_TIMEOUT = 1 << 0
STALE_AFTER_NS = 2_000_000_000


def bit_names(value, table):
    return [name for bit, name in table.items() if value & (1 << bit)]


def decode(data):
    """Return [(name, level, message, values)], level 0 OK, 1 WARN, 2 ERROR."""
    fw = data[6]
    out = []
    for i in range(3):
        fs1, vgs2 = data[2 * i], data[2 * i + 1]
        faults = bit_names(fs1, FS1_BITS) + bit_names(vgs2, VGS2_BITS)
        if fw & (1 << (1 + i)):
            faults.append('SPI_ERR (values stale)')
        level = max((1 if f in WARN_ONLY else 2 for f in faults), default=0)
        out.append((f'wheelbase: motor {i + 1} driver', level, ', '.join(faults) or 'OK',
                    {'FAULT_STATUS_1': f'0x{fs1:03X}', 'VGS_STATUS_2': f'0x{vgs2:03X}'}))
    timeout = bool(fw & FW_CMD_TIMEOUT)
    out.append(('wheelbase: firmware', 1 if timeout else 0,
                'CMD_TIMEOUT: no velocity commands, motors stopped' if timeout else 'OK',
                {'fw_flags': f'0x{fw:04X}'}))
    return out


class MotorFaultDiagnostics(Node):

    def __init__(self):
        super().__init__('motor_fault_diagnostics')
        self.pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_subscription(UInt16MultiArray, '/motor_faults', self.on_faults, 10)
        self.create_timer(1.0, self.publish)
        self.last = None     # latest sample
        self.latched = None  # bits OR-ed since last publish, so short faults are not missed
        self.last_rx = None

    def on_faults(self, msg):
        if len(msg.data) != MSG_LEN:
            self.get_logger().warn(
                f'motor_faults has {len(msg.data)} values, expected {MSG_LEN}',
                throttle_duration_sec=5.0)
            return
        self.last = list(msg.data)
        self.latched = self.last if self.latched is None else [
            a | b for a, b in zip(self.latched, self.last)]
        self.last_rx = self.get_clock().now()

    def publish(self):
        now = self.get_clock().now()
        stale = self.last_rx is None or (now - self.last_rx).nanoseconds > STALE_AFTER_NS
        arr = DiagnosticArray()
        arr.header.stamp = now.to_msg()
        for name, level, message, values in decode(self.latched or [0] * MSG_LEN):
            status = DiagnosticStatus(name=name, hardware_id='semubot_wheelbase')
            if stale:
                status.level, status.message = DiagnosticStatus.STALE, 'No data on /motor_faults'
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
