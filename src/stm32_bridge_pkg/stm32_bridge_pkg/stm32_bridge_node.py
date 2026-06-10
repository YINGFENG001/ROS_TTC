from __future__ import annotations

import json
import re
import threading
import time
from typing import Callable, Optional

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float32, Int32, String

from .protocol import Stm32Message, fields_to_json_dict
from .stm32_serial_client import Stm32CommandError, Stm32SerialClient


class Stm32BridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('stm32_bridge_node')

        self.declare_parameter('stm32.port', '/dev/ttyUSB0')
        self.declare_parameter('stm32.baudrate', 115200)
        self.declare_parameter('stm32.timeout', 1.0)
        self.declare_parameter('command.default_timeout', 2.0)
        self.declare_parameter('motion.default_timeout', 10.0)

        self.declare_parameter('gripper.open_position', 800)
        self.declare_parameter('gripper.close_position', 2048)
        self.declare_parameter('gripper.min_speed', 1)
        self.declare_parameter('gripper.max_speed', 3000)
        self.declare_parameter('gripper.default_speed', 800)

        self.declare_parameter('motor.default_accel', 100)
        self.declare_parameter('motor.default_decel', 100)
        self.declare_parameter('motor.default_rpm', 100)
        self.declare_parameter('legacy.enable', True)
        self.declare_parameter('legacy.motor_device', 'mtor1')
        self.declare_parameter('legacy.motor_max_rpm', 1500)
        self.declare_parameter('legacy.clamp_wait_done', False)

        self.stm32_port = self._string_param('stm32.port')
        self.stm32_baudrate = self._int_param('stm32.baudrate')
        self.stm32_timeout = self._float_param('stm32.timeout')
        self.command_timeout = self._float_param('command.default_timeout')
        self.motion_timeout = self._float_param('motion.default_timeout')

        self.gripper_open_position = self._int_param('gripper.open_position')
        self.gripper_close_position = self._int_param('gripper.close_position')
        self.gripper_min_speed = self._int_param('gripper.min_speed')
        self.gripper_max_speed = self._int_param('gripper.max_speed')
        self.gripper_default_speed = self._int_param('gripper.default_speed')

        self.motor_default_accel = self._int_param('motor.default_accel')
        self.motor_default_decel = self._int_param('motor.default_decel')
        self.motor_default_rpm = self._int_param('motor.default_rpm')
        self.legacy_enabled = self._bool_param('legacy.enable')
        self.legacy_motor_device = self._string_param('legacy.motor_device')
        self.legacy_motor_max_rpm = self._int_param('legacy.motor_max_rpm')
        self.legacy_clamp_wait_done = self._bool_param('legacy.clamp_wait_done')
        if self.legacy_motor_device not in {'mtor1', 'mtor2'}:
            self.legacy_motor_device = 'mtor1'
        if self.legacy_motor_max_rpm < 0:
            self.legacy_motor_max_rpm = 1500

        self.raw_tx_pub = self.create_publisher(String, '/stm32/raw_tx', 10)
        self.raw_rx_pub = self.create_publisher(String, '/stm32/raw_rx', 10)
        self.error_pub = self.create_publisher(String, '/stm32/error', 10)
        self.event_pub = self.create_publisher(String, '/stm32/event', 10)

        self.mtor1_state_pub = self.create_publisher(JointState, '/mtor1/state', 10)
        self.mtor2_state_pub = self.create_publisher(JointState, '/mtor2/state', 10)
        self.clamp_state_pub = self.create_publisher(JointState, '/clamp/state', 10)
        self.clamp_status_text_pub = self.create_publisher(String, '/clamp/status_text', 10)
        self.vacuum_status_text_pub = self.create_publisher(String, '/vacuum/status_text', 10)

        self.client: Optional[Stm32SerialClient] = None
        self._legacy_lock = threading.Lock()
        self._legacy_motor_continuous = False
        self._legacy_motor_direction = 1
        self._connect_client()

        self.create_subscription(String, '/stm32/command', self._cb_stm32_command, 10)
        self.create_subscription(Empty, '/emergency_stop', self._cb_emergency_stop, 10)

        self.create_subscription(Float32, '/mtor1/move', lambda msg: self._cb_motor_move('mtor1', msg), 10)
        self.create_subscription(Float32, '/mtor2/move', lambda msg: self._cb_motor_move('mtor2', msg), 10)
        self.create_subscription(Int32, '/mtor1/rpm', lambda msg: self._cb_motor_rpm('mtor1', msg), 10)
        self.create_subscription(Int32, '/mtor2/rpm', lambda msg: self._cb_motor_rpm('mtor2', msg), 10)
        self.create_subscription(String, '/mtor1/set_params', lambda msg: self._cb_motor_set_params('mtor1', msg), 10)
        self.create_subscription(String, '/mtor2/set_params', lambda msg: self._cb_motor_set_params('mtor2', msg), 10)
        self.create_subscription(Empty, '/mtor1/stop', lambda msg: self._cb_simple_command('mtor1 stop'), 10)
        self.create_subscription(Empty, '/mtor2/stop', lambda msg: self._cb_simple_command('mtor2 stop'), 10)
        self.create_subscription(Empty, '/mtor1/status_query', lambda msg: self._cb_status_query('mtor1'), 10)
        self.create_subscription(Empty, '/mtor2/status_query', lambda msg: self._cb_status_query('mtor2'), 10)

        self.create_subscription(Vector3, '/clamp/move_percent', self._cb_clamp_move_percent, 10)
        self.create_subscription(Empty, '/clamp/status_query', lambda msg: self._cb_status_query('clamp'), 10)

        self.create_subscription(Vector3, '/vacuum/set_params', self._cb_vacuum_set_params, 10)
        self.create_subscription(Empty, '/vacuum/grip', lambda msg: self._cb_simple_command('vacum grip'), 10)
        self.create_subscription(Empty, '/vacuum/release', lambda msg: self._cb_simple_command('vacum release'), 10)
        self.create_subscription(Empty, '/vacuum/stop', lambda msg: self._cb_simple_command('vacum stop'), 10)
        self.create_subscription(Empty, '/vacuum/status_query', lambda msg: self._cb_status_query('vacum'), 10)

        if self.legacy_enabled:
            self.create_subscription(Vector3, '/motor/jog_cmd', self._cb_legacy_motor_jog, 10)
            self.create_subscription(Empty, '/motor/stop', self._cb_legacy_motor_stop, 10)
            self.create_subscription(Vector3, '/gripper/cmd_percent', self._cb_legacy_gripper_percent, 10)

    def destroy_node(self) -> bool:
        if self.client:
            self.client.close()
        return super().destroy_node()

    def _connect_client(self) -> None:
        try:
            self.client = Stm32SerialClient(
                port=self.stm32_port,
                baudrate=self.stm32_baudrate,
                timeout=self.stm32_timeout,
                raw_rx_callback=self._publish_raw_rx,
                raw_tx_callback=self._publish_raw_tx,
                message_callback=self._on_protocol_message,
            )
            self.get_logger().info(
                f'STM32 serial connected: port={self.stm32_port} baudrate={self.stm32_baudrate} 8N1'
            )
        except Exception as exc:
            self.client = None
            self._publish_error(f'STM32 serial connect failed: {exc}')

    def _cb_stm32_command(self, msg: String) -> None:
        command = msg.data.strip()
        if not command:
            self._publish_error('ignore empty /stm32/command')
            return
        wait_for = 'state' if self._command_action(command) == 'status' else 'ack'
        self._run_async(lambda: self._send(command, wait_for=wait_for, timeout=self.command_timeout))

    def _cb_simple_command(self, command: str) -> None:
        self._run_async(lambda: self._send(command, wait_for='ack', timeout=self.command_timeout))

    def _cb_status_query(self, device: str) -> None:
        self._run_async(lambda: self._send(f'{device} status', 'state', self.command_timeout))

    def _cb_motor_move(self, device: str, msg: Float32) -> None:
        rev_0p1 = int(round(float(msg.data) * 10.0))
        command = f'{device} move {rev_0p1}'
        self._run_async(lambda: self._send(command, wait_for='ack', timeout=self.command_timeout))

    def _cb_motor_rpm(self, device: str, msg: Int32) -> None:
        value = int(msg.data)
        self._run_async(lambda: self._send(f'{device} rpm {value}', wait_for='ack', timeout=self.command_timeout))

    def _cb_motor_set_params(self, device: str, msg: String) -> None:
        params = msg.data.strip()
        if not params:
            self._publish_error(f'ignore empty /{device}/set_params')
            return
        self._run_async(lambda: self._send(f'{device} set {params}', wait_for='ack', timeout=self.command_timeout))

    def _cb_clamp_move_percent(self, msg: Vector3) -> None:
        percent = self._clamp(msg.x, 0.0, 100.0)
        wait_for = 'done' if int(msg.z) != 0 else 'ack'
        timeout = self.motion_timeout if wait_for == 'done' else self.command_timeout
        command = f'clamp move {percent:.1f}%'
        self._run_async(lambda: self._send(command, wait_for=wait_for, timeout=timeout))

    def _cb_legacy_motor_stop(self, _: Empty) -> None:
        device = self.legacy_motor_device

        def worker() -> None:
            with self._legacy_lock:
                self._legacy_motor_continuous = False
            self._send(f'{device} stop', wait_for='ack', timeout=self.command_timeout, raise_errors=False)

        self._run_async(worker)

    def _cb_legacy_motor_jog(self, msg: Vector3) -> None:
        device = self.legacy_motor_device
        direction = 1 if msg.x >= 0.0 else -1
        rpm = int(round(self._clamp(abs(msg.y), 0.0, float(self.legacy_motor_max_rpm))))
        duration = max(float(msg.z), 0.0)

        def worker() -> None:
            if rpm <= 0:
                self._send(f'{device} rpm 0', wait_for='ack', timeout=self.command_timeout, raise_errors=False)
                with self._legacy_lock:
                    self._legacy_motor_continuous = False
                return

            start_required = False
            with self._legacy_lock:
                start_required = not self._legacy_motor_continuous

            if start_required:
                start_arg = '+0' if direction > 0 else '-0'
                response = self._send(
                    f'{device} move {start_arg}',
                    wait_for='ack',
                    timeout=self.command_timeout,
                    raise_errors=False,
                )
                if response is None:
                    return
                with self._legacy_lock:
                    self._legacy_motor_continuous = True
                    self._legacy_motor_direction = direction

            signed_rpm = rpm if direction > 0 else -rpm
            self._send(
                f'{device} rpm {signed_rpm}',
                wait_for='ack',
                timeout=self.command_timeout,
                raise_errors=False,
            )
            with self._legacy_lock:
                self._legacy_motor_continuous = True
                self._legacy_motor_direction = direction

            if duration > 0.0:
                time.sleep(duration)
                self._send(f'{device} stop', wait_for='ack', timeout=self.command_timeout, raise_errors=False)
                with self._legacy_lock:
                    self._legacy_motor_continuous = False

        self._run_async(worker)

    def _cb_legacy_gripper_percent(self, msg: Vector3) -> None:
        percent = self._clamp(msg.x, 0.0, 100.0)
        wait_for = 'done' if self.legacy_clamp_wait_done else 'ack'
        timeout = self.motion_timeout if wait_for == 'done' else self.command_timeout
        command = f'clamp move {percent:.1f}%'
        self._run_async(lambda: self._send(command, wait_for=wait_for, timeout=timeout, raise_errors=False))

    def _cb_vacuum_set_params(self, msg: Vector3) -> None:
        min_vac = int(round(self._clamp(msg.x, 0.0, 100.0)))
        max_vac = int(round(self._clamp(msg.y, 0.0, 100.0)))
        timeout_100ms = int(round(self._clamp(msg.z, 1.0, 255.0)))
        command = f'vacum set {min_vac} {max_vac} {timeout_100ms}'
        self._run_async(lambda: self._send(command, wait_for='ack', timeout=self.command_timeout))

    def _cb_emergency_stop(self, _: Empty) -> None:
        commands = ['mtor1 stop', 'mtor2 stop', 'clamp release', 'vacum stop']

        def worker() -> None:
            for command in commands:
                self._send(command, wait_for='ack', timeout=self.command_timeout, raise_errors=False)

        self._run_async(worker)

    def _send(
        self,
        command: str,
        wait_for: str,
        timeout: float,
        raise_errors: bool = True,
    ) -> Optional[Stm32Message]:
        if self.client is None:
            self._publish_error(f'STM32 serial is not connected, cannot send: {command}')
            return None
        try:
            response = self.client.send_command(command, wait_for=wait_for, timeout=timeout)
            return response
        except (TimeoutError, Stm32CommandError, ValueError, OSError) as exc:
            self._publish_error(f'command failed "{command}": {exc}')
            if raise_errors:
                self.get_logger().warning(f'command failed "{command}": {exc}')
            return None

    def _on_protocol_message(self, message: Stm32Message) -> None:
        if message.type == 'event':
            self._publish_event(message)
            return
        if message.type == 'err':
            self._publish_error(message.raw)
            return
        if message.type != 'state':
            return

        if message.device in {'mtor1', 'mtor2'}:
            self._publish_motor_state(message)
        elif message.device == 'clamp':
            self._publish_clamp_state(message)
        elif message.device == 'vacum':
            self._publish_vacuum_status(message)

    def _publish_motor_state(self, message: Stm32Message) -> None:
        rev_0p1 = self._float_field(message, 'rev', 'pos', 'position', default=0.0)
        rpm = self._float_field(message, 'rpm', 'speed', 'velocity', default=0.0)
        fault = self._float_field(message, 'fault', 'err', 'code', default=0.0)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [message.device]
        js.position = [rev_0p1 / 10.0]
        js.velocity = [rpm]
        js.effort = [fault]
        if message.device == 'mtor1':
            self.mtor1_state_pub.publish(js)
        else:
            self.mtor2_state_pub.publish(js)

    def _publish_clamp_state(self, message: Stm32Message) -> None:
        raw_position, percent = self._parse_clamp_position(message)
        speed = self._float_field(message, 'speed', 'velocity', default=0.0)
        current = self._float_field(message, 'current', 'load', 'effort', default=0.0)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['clamp']
        js.position = [percent]
        js.velocity = [speed]
        js.effort = [current]
        self.clamp_state_pub.publish(js)

        status = String()
        status.data = message.raw
        self.clamp_status_text_pub.publish(status)

    def _publish_vacuum_status(self, message: Stm32Message) -> None:
        text = String()
        text.data = json.dumps(fields_to_json_dict(message.fields), ensure_ascii=False, separators=(',', ':'))
        self.vacuum_status_text_pub.publish(text)

    def _publish_event(self, message: Stm32Message) -> None:
        msg = String()
        msg.data = message.raw
        self.event_pub.publish(msg)

        level = message.fields.get('level', '').lower()
        if level == 'fault':
            self._publish_error(message.raw)
        elif level == 'warn':
            self.error_pub.publish(msg)
            self.get_logger().warning(message.raw)
        else:
            self.get_logger().info(message.raw)

    def _publish_raw_tx(self, line: str) -> None:
        msg = String()
        msg.data = line
        self.raw_tx_pub.publish(msg)

    def _publish_raw_rx(self, line: str) -> None:
        msg = String()
        msg.data = line
        self.raw_rx_pub.publish(msg)

    def _publish_error(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.error_pub.publish(msg)
        self.get_logger().error(text)

    def _run_async(self, target: Callable[[], None]) -> None:
        thread = threading.Thread(target=target, daemon=True)
        thread.start()

    def _string_param(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _int_param(self, name: str) -> int:
        value = self.get_parameter(name).get_parameter_value()
        if value.integer_value:
            return int(value.integer_value)
        return int(value.double_value)

    def _bool_param(self, name: str) -> bool:
        return bool(self.get_parameter(name).get_parameter_value().bool_value)

    def _float_param(self, name: str) -> float:
        value = self.get_parameter(name).get_parameter_value()
        if value.double_value:
            return float(value.double_value)
        return float(value.integer_value)

    def _float_field(self, message: Stm32Message, *keys: str, default: float) -> float:
        for key in keys:
            value = message.fields.get(key)
            if value is None:
                continue
            try:
                return float(value)
            except ValueError:
                return default
        return default

    def _optional_float_field(self, message: Stm32Message, *keys: str) -> Optional[float]:
        for key in keys:
            value = message.fields.get(key)
            if value is None:
                continue
            try:
                return float(value)
            except ValueError:
                continue
        return None

    @staticmethod
    def _command_action(command: str) -> str:
        parts = command.split()
        if len(parts) < 2:
            return ''
        return parts[1]

    def _clamp_raw_to_percent(self, raw_position: float) -> float:
        span = self.gripper_close_position - self.gripper_open_position
        if span == 0:
            return 0.0
        percent = (self.gripper_close_position - raw_position) * 100.0 / span
        return self._clamp(percent, 0.0, 100.0)

    def _parse_clamp_position(self, message: Stm32Message) -> tuple[float, float]:
        raw_position = self._optional_float_field(message, 'pos', 'position', 'openPos')
        percent = self._optional_float_field(message, 'pct', 'percent')

        if raw_position is None or percent is None:
            match = re.search(
                r'openPos/Pct\s*=\s*([+-]?\d+(?:\.\d+)?)\s*\(([+-]?\d+(?:\.\d+)?)%\)',
                message.raw,
            )
            if match:
                if raw_position is None:
                    raw_position = float(match.group(1))
                if percent is None:
                    percent = float(match.group(2))

        if raw_position is None:
            raw_position = float(self.gripper_close_position)
        if percent is None:
            percent = self._clamp_raw_to_percent(raw_position)

        return raw_position, self._clamp(percent, 0.0, 100.0)

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return min(max(float(value), lower), upper)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Stm32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
