from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
from PyQt6.QtCore import QThread, pyqtSignal, QObject
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuSubscriber(Node, QObject):  # 🔹 QObject 추가!
    imu_signal = pyqtSignal(float, float, float)  # RPY 데이터를 전달할 PyQt6 시그널

    def __init__(self):
        Node.__init__(self, 'imu_subscriber')
        QObject.__init__(self)  # 🔹 QObject도 초기화!
        
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)
        self.subscription  # 방지: Pylint가 변수를 사용하지 않는다고 경고하는 것 방지

    def imu_callback(self, msg):
        roll = msg.orientation.x
        pitch = msg.orientation.y
        yaw = msg.orientation.z
        self.imu_signal.emit(roll, pitch, yaw)  # 🔹 PyQt6 시그널 전송

class RosThread(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        rclpy.spin(self.node)  # 🔹 ROS2 노드 실행

class ImuGui(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("IMU Data Viewer")
        layout = QVBoxLayout()
        self.label = QLabel("Waiting for IMU data...")
        layout.addWidget(self.label)
        self.setLayout(layout)

        self.node = ImuSubscriber()
        self.thread = RosThread(self.node)

        self.node.imu_signal.connect(self.update_label)  # 🔹 PyQt6 GUI 업데이트 연결
        self.thread.start()

    def update_label(self, roll, pitch, yaw):
        self.label.setText(f"Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")

def main():
    rclpy.init()
    app = QApplication([])
    gui = ImuGui()
    gui.show()
    app.exec()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
