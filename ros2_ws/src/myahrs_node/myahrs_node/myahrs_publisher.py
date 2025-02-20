import rclpy
from rclpy.node import Node
import serial
import math
import glob
from sensor_msgs.msg import Imu
import tf_transformations

class MyAHRSNode(Node):
    def __init__(self):
        super().__init__('myahrs_node')

        # 🔄 자동으로 사용 가능한 IMU 포트 찾기
        self.serial_port = self.find_imu_port()
        self.baudrate = 115200

        if self.serial_port:
            try:
                self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
                self.get_logger().info(f"Connected to {self.serial_port} at {self.baudrate} baud.")
            except serial.SerialException:
                self.get_logger().error(f"Failed to connect to {self.serial_port}. Retrying...")
                self.ser = None
        else:
            self.get_logger().error("No IMU device found. Retrying...")

        self.imu_publisher = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(0.02, self.read_imu_data)

    def find_imu_port(self):
        """자동으로 사용 가능한 IMU 포트 찾기"""
        ports = glob.glob("/dev/ttyACM*")
        if "/dev/ttyACM0" in ports:
            return "/dev/ttyACM0"
        elif "/dev/ttyACM1" in ports:
            return "/dev/ttyACM1"
        elif ports:
            return ports[0]  # 첫 번째 사용 가능한 포트 반환
        return None  # 사용 가능한 포트가 없으면 None 반환

    def rpy_to_quaternion(self, roll, pitch, yaw):
        quaternion = tf_transformations.quaternion_from_euler(
            math.radians(roll),
            math.radians(pitch),
            math.radians(yaw)
        )
        return quaternion

    def read_imu_data(self):
        """IMU 데이터 읽기 및 ROS2 메시지 변환"""
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn("IMU 포트가 닫혀 있습니다. 다시 시도 중...")
            self.serial_port = self.find_imu_port()
            if self.serial_port:
                try:
                    self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
                    self.get_logger().info(f"Reconnected to {self.serial_port}.")
                except serial.SerialException:
                    self.get_logger().error(f"Failed to reconnect to {self.serial_port}.")
            return

        try:
            line = self.ser.readline().decode('utf-8').strip()
            if not line:
                return
            
            data = self.parse_imu_data(line)
            if data is None:
                self.get_logger().warn("Failed to parse IMU data, skipping this frame.")
                return

            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"

            quaternion = self.rpy_to_quaternion(
                data.get("roll", 0.0),
                data.get("pitch", 0.0),
                data.get("yaw", 0.0)
            )
            imu_msg.orientation.x = quaternion[0]
            imu_msg.orientation.y = quaternion[1]
            imu_msg.orientation.z = quaternion[2]
            imu_msg.orientation.w = quaternion[3]

            imu_msg.linear_acceleration.x = data.get("acc_x", 0.02)
            imu_msg.linear_acceleration.y = data.get("acc_y", 0.02)
            imu_msg.linear_acceleration.z = data.get("acc_z", 9.81)

            imu_msg.angular_velocity.x = data.get("gyro_x", 0.01)
            imu_msg.angular_velocity.y = data.get("gyro_y", 0.01)
            imu_msg.angular_velocity.z = data.get("gyro_z", 0.01)

            self.imu_publisher.publish(imu_msg)

        except Exception as e:
            self.get_logger().error(f"Error reading serial data: {e}")

    def parse_imu_data(self, line):
        """IMU 데이터 파싱"""
        try:
            data_dict = {}

            if line.startswith("$RPY"):
                parts = line.split(",")
                if len(parts) < 5:
                    return None
                
                data_dict["roll"] = float(parts[2])
                data_dict["pitch"] = float(parts[3])
                data_dict["yaw"] = float(parts[4].split("*")[0])

            if "ACC:" in line and "GYRO:" in line:
                acc_part, gyro_part = line.split(";")
                acc_values = [float(x) for x in acc_part.replace("ACC:", "").split(",")]
                gyro_values = [float(x) for x in gyro_part.replace("GYRO:", "").split(",")]

                data_dict["acc_x"], data_dict["acc_y"], data_dict["acc_z"] = acc_values
                data_dict["gyro_x"], data_dict["gyro_y"], data_dict["gyro_z"] = gyro_values

            return data_dict if data_dict else None

        except Exception as e:
            self.get_logger().warn(f"Failed to parse: {line} - {e}")
            return None

def main(args=None):
    rclpy.init(args=args)
    node = MyAHRSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
