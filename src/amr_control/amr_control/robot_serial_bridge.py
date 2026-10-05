import os
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import serial


class RobotSerialBridge(Node):
    def __init__(self):
        super().__init__('robot_serial_bridge')
        self.ser = None
        self.connected = False
        self._serial_lock = threading.Lock()
        self._stop_event = threading.Event()
        
        # CẤU HÌNH CỔNG
        self.declare_parameter(
            'port', os.getenv('ROBOT_SERIAL_PORT', '/dev/ttyUSB0')
        )
        self.declare_parameter('baud', 115200)
        self.declare_parameter('reconnect_interval', 2.0)
        self.declare_parameter('linear_scale', 1.0)
        # STM32 motor command convention is opposite to ROS angular.z.
        # Keep /cmd_vel ROS-standard: positive angular.z means CCW/left.
        # Giá trị gốc cơ khí. KHÔNG hack bù bán kính ở đây —
        # Camera Homography đã đo chính xác, không cần bù thêm.
        self.declare_parameter('angular_scale', -1.0)
        self.declare_parameter('max_linear_cmd', 0.25)
        self.declare_parameter('max_angular_cmd', 0.90)
        self.declare_parameter('cmd_timeout', 0.25)
        self.declare_parameter('extended_data_order', 'timestamp_seq')
        
        self.port = str(self.get_parameter('port').value)
        self.baud = int(self.get_parameter('baud').value)
        self.reconnect_interval = max(
            0.2, float(self.get_parameter('reconnect_interval').value)
        )
        self.linear_scale = float(self.get_parameter('linear_scale').value)
        self.angular_scale = float(self.get_parameter('angular_scale').value)
        self.max_linear_cmd = float(self.get_parameter('max_linear_cmd').value)
        self.max_angular_cmd = float(self.get_parameter('max_angular_cmd').value)
        self.cmd_timeout = max(
            0.05, float(self.get_parameter('cmd_timeout').value)
        )
        self.extended_data_order = str(
            self.get_parameter('extended_data_order').value
        )
        self._last_rx_time = None
        self._last_seq = None
        self._last_cmd_time = None
        self._watchdog_stopped = True
        
        # Pub dữ liệu raw cho state_bridge xử lý
        self.state_pub = self.create_publisher(Float32MultiArray, '/robot_state', 10)
        self.link_pub = self.create_publisher(Float32MultiArray, '/espnow_link', 10)
        self.robot_cmd_pub = self.create_publisher(Twist, '/cmd_vel_robot', 10)
        
        # Sub lệnh vận tốc từ controller bám quỹ đạo
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # Thread đọc Serial để không làm treo ROS
        self._connect_serial()
        self.thread = threading.Thread(target=self.read_serial, daemon=True)
        self.thread.start()
        self.watchdog_timer = self.create_timer(0.05, self.watchdog_callback)

    def _connect_serial(self):
        if self._stop_event.is_set():
            return False
        with self._serial_lock:
            if self.ser is not None and self.ser.is_open:
                return True
            try:
                new_serial = serial.Serial(self.port, self.baud, timeout=0.1)
                # A reconnect must never replay an old motion command.
                new_serial.write(b"CMD,0.0000,0.0000\r\n")
                self.ser = new_serial
                self.connected = True
                self._last_cmd_time = None
                self._watchdog_stopped = True
            except (OSError, serial.SerialException) as exc:
                self.ser = None
                self.connected = False
                self.get_logger().error(
                    f"Không thể mở cổng Serial {self.port}: {exc}",
                    throttle_duration_sec=5.0,
                )
                return False

        self.get_logger().info(
            f"Đã kết nối dây qua cổng: {self.port} - Baud: {self.baud}; "
            f"linear_scale={self.linear_scale:.2f}, "
            f"angular_scale={self.angular_scale:.2f}"
        )
        return True

    def _disconnect_serial(self, reason=None):
        with self._serial_lock:
            serial_port = self.ser
            self.ser = None
            self.connected = False
            if serial_port is not None:
                try:
                    serial_port.close()
                except (OSError, serial.SerialException):
                    pass
        if reason is not None:
            self.get_logger().error(
                f"Mất kết nối Serial: {reason}; sẽ thử kết nối lại.",
                throttle_duration_sec=5.0,
            )

    def read_serial(self):
        while rclpy.ok() and not self._stop_event.is_set():
            if not self._connect_serial():
                self._stop_event.wait(self.reconnect_interval)
                continue
            try:
                serial_port = self.ser
                if serial_port is None:
                    continue
                # readline() sleeps up to the configured timeout. The old
                # in_waiting loop polled continuously and consumed a CPU core
                # whenever the controller was idle.
                line = serial_port.readline().decode(
                    'utf-8', errors='ignore'
                ).strip()
                if not line.startswith("DATA,"):
                    continue
                parts = line.split(',')
                parsed = self.parse_data_packet(parts)
                if parsed is None:
                    continue

                msg = Float32MultiArray()
                msg.data = [
                    parsed['rpm_l_x10'],
                    parsed['rpm_r_x10'],
                    parsed['gyro_z_x1000'],
                ]
                self.state_pub.publish(msg)

                link_msg = Float32MultiArray()
                link_msg.data = [
                    parsed['rx_time'],
                    parsed['robot_time_ms'],
                    parsed['seq'],
                    parsed['interarrival_ms'],
                    parsed['seq_gap'],
                ]
                self.link_pub.publish(link_msg)
            except (OSError, serial.SerialException) as exc:
                self._disconnect_serial(exc)

    def parse_data_packet(self, parts):
        rx_time = time.monotonic()
        interarrival_ms = float('nan')
        if self._last_rx_time is not None:
            interarrival_ms = (rx_time - self._last_rx_time) * 1000.0
        self._last_rx_time = rx_time

        robot_time_ms = float('nan')
        seq = float('nan')
        seq_gap = float('nan')

        try:
            if len(parts) == 4:
                rpm_l_x10 = float(parts[1])
                rpm_r_x10 = float(parts[2])
                gyro_z_x1000 = float(parts[3])
            elif len(parts) >= 6:
                if self.extended_data_order == 'seq_timestamp':
                    seq = float(parts[1])
                    robot_time_ms = float(parts[2])
                else:
                    robot_time_ms = float(parts[1])
                    seq = float(parts[2])
                rpm_l_x10 = float(parts[3])
                rpm_r_x10 = float(parts[4])
                gyro_z_x1000 = float(parts[5])

                if self._last_seq is not None:
                    seq_gap = max(0.0, seq - self._last_seq - 1.0)
                self._last_seq = seq
            else:
                return None
        except ValueError:
            return None

        return {
            'rx_time': rx_time,
            'robot_time_ms': robot_time_ms,
            'seq': seq,
            'interarrival_ms': interarrival_ms,
            'seq_gap': seq_gap,
            'rpm_l_x10': rpm_l_x10,
            'rpm_r_x10': rpm_r_x10,
            'gyro_z_x1000': gyro_z_x1000,
        }

    def cmd_callback(self, msg):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn(
                "Serial chưa sẵn sàng, bỏ qua lệnh /cmd_vel.",
                throttle_duration_sec=2.0,
            )
            return

        v_cmd = max(
            -self.max_linear_cmd,
            min(self.max_linear_cmd, msg.linear.x * self.linear_scale),
        )
        w_cmd = max(
            -self.max_angular_cmd,
            min(self.max_angular_cmd, msg.angular.z * self.angular_scale),
        )
        if self.write_command(v_cmd, w_cmd):
            self._last_cmd_time = time.monotonic()
            self._watchdog_stopped = False

    def write_command(self, v_cmd, w_cmd):
        """Send and publish the command after bridge scaling/clamping."""
        if self.ser is None or not self.ser.is_open:
            return False

        # /cmd_vel stays ROS-standard; angular_scale maps it to STM32 convention.
        cmd_str = f"CMD,{v_cmd:.4f},{w_cmd:.4f}\r\n"
        try:
            with self._serial_lock:
                if self.ser is None or not self.ser.is_open:
                    return False
                self.ser.write(cmd_str.encode())
            sent = Twist()
            sent.linear.x = float(v_cmd)
            sent.angular.z = float(w_cmd)
            # The serial stop command is still useful during shutdown, but
            # publishing after the ROS context has already been invalidated
            # raises RCLError and makes a normal Ctrl-C look like a failed run.
            if rclpy.ok():
                self.robot_cmd_pub.publish(sent)
            return True
        except (OSError, serial.SerialException) as exc:
            self._disconnect_serial(exc)
            return False

    def watchdog_callback(self):
        if self._watchdog_stopped or self._last_cmd_time is None:
            return
        if time.monotonic() - self._last_cmd_time <= self.cmd_timeout:
            return
        if self.write_command(0.0, 0.0):
            self._watchdog_stopped = True
            self.get_logger().warn(
                f"/cmd_vel timeout > {self.cmd_timeout:.2f}s; sent stop command."
            )

    def destroy_node(self):
        self._stop_event.set()
        self.write_command(0.0, 0.0)
        self._disconnect_serial()
        if self.thread.is_alive() and threading.current_thread() is not self.thread:
            self.thread.join(timeout=0.5)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
