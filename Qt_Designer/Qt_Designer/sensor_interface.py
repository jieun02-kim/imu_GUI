import sys
import rclpy
from rclpy.node import Node
import numpy as np
from PyQt6 import QtWidgets, QtCore
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from sensor_msgs.msg import Imu
import tf_transformations
from imu_interface import Ui_Dialog  # UI 파일 import
import glob
import subprocess


class ImuSubscriber(Node, QtCore.QObject):
    imu_signal = QtCore.pyqtSignal(float, float, float, float, float, float, float, float, float)

    def __init__(self):
        Node.__init__(self, 'imu_subscriber')
        QtCore.QObject.__init__(self)

        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

    def imu_callback(self, msg):
        """IMU 메시지를 받아 RPY 값으로 변환 후 emit"""
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        rpy = tf_transformations.euler_from_quaternion(quaternion)
        roll, pitch, yaw = np.degrees(rpy)

        acc_x = msg.linear_acceleration.x
        acc_y = msg.linear_acceleration.y
        acc_z = msg.linear_acceleration.z

        gyro_x = msg.angular_velocity.x
        gyro_y = msg.angular_velocity.y
        gyro_z = msg.angular_velocity.z

        self.imu_signal.emit(roll, pitch, yaw, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z)


class RosThread(QtCore.QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)


class IMU3DVisualizer(gl.GLViewWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setCameraPosition(distance=5)

        # 3D 좌표축 추가
        self.axis = gl.GLAxisItem()
        self.addItem(self.axis)

        # ✅ IMU 모형 (상자 형태)
        self.imu_object = gl.GLBoxItem()
        self.imu_object.setSize(1.0, 2.0, 0.1)  # 크기 설정
        self.addItem(self.imu_object)

    def update_orientation(self, roll, pitch, yaw, *_):
        """IMU 데이터를 기반으로 3D 모델 회전 적용"""
        quaternion = tf_transformations.quaternion_from_euler(np.radians(roll), np.radians(pitch), np.radians(yaw))
        self.imu_object.resetTransform()
        self.imu_object.rotate(180 * quaternion[0], 1, 0, 0)
        self.imu_object.rotate(180 * quaternion[1], 0, 1, 0)
        self.imu_object.rotate(180 * quaternion[2], 0, 0, 1)


class ImuGui(QtWidgets.QDialog, Ui_Dialog):
    def __init__(self):
        super().__init__()
        self.setupUi(self)  # UI 불러오기
        self.init_graphs()  # 그래프 초기화
        self.init_3d_view()  # ✅ 3D 뷰어 초기화

        # ✅ 먼저 노드를 정의해야 `self.node.imu_signal.connect()` 호출 가능
        self.node = ImuSubscriber()
        self.thread = RosThread(self.node)

        # ✅ 노드가 먼저 정의된 후, 시그널 연결
        self.node.imu_signal.connect(self.update_ui)
        
        self.thread.start()

        self.comboBox.currentIndexChanged.connect(self.switch_page)
        self.update_labels()  # UI 라벨 초기화

    def update_ui(self, roll, pitch, yaw, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z):
        """IMU 데이터를 수신하여 UI 갱신"""
        self.roll_data = self.roll_data[1:] + [roll]
        self.pitch_data = self.pitch_data[1:] + [pitch]
        self.yaw_data = self.yaw_data[1:] + [yaw]

        self.acc_x_data = self.acc_x_data[1:] + [acc_x]
        self.acc_y_data = self.acc_y_data[1:] + [acc_y]
        self.acc_z_data = self.acc_z_data[1:] + [acc_z]

        self.gyro_x_data = self.gyro_x_data[1:] + [gyro_x]
        self.gyro_y_data = self.gyro_y_data[1:] + [gyro_y]
        self.gyro_z_data = self.gyro_z_data[1:] + [gyro_z]

        # 그래프 업데이트
        self.roll_curve.setData(self.time_data, self.roll_data)
        self.pitch_curve.setData(self.time_data, self.pitch_data)
        self.yaw_curve.setData(self.time_data, self.yaw_data)

        self.acc_x_curve.setData(self.time_data, self.acc_x_data)
        self.acc_y_curve.setData(self.time_data, self.acc_y_data)
        self.acc_z_curve.setData(self.time_data, self.acc_z_data)

        self.gyro_x_curve.setData(self.time_data, self.gyro_x_data)
        self.gyro_y_curve.setData(self.time_data, self.gyro_y_data)
        self.gyro_z_curve.setData(self.time_data, self.gyro_z_data)

        # 3D 뷰어 업데이트
        self.imu_visualizer.update_orientation(roll, pitch, yaw)

        # 데이터를 textBrowser(QTextBrowser)에 표시
        self.textBrowser.setText(f"Roll: \n{roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}\n"
                                f"Acc: \nX={acc_x:.2f}, Y={acc_y:.2f}, Z={acc_z:.2f}\n"
                                f"Gyro: \nX={gyro_x:.2f}, Y={gyro_y:.2f}, Z={gyro_z:.2f}")



    def init_graphs(self):
        """그래프 초기화"""
        self.time_data = list(range(100))
        self.roll_data = [0] * 100
        self.pitch_data = [0] * 100
        self.yaw_data = [0] * 100
        self.acc_x_data = [0] * 100
        self.acc_y_data = [0] * 100
        self.acc_z_data = [0] * 100
        self.gyro_x_data = [0] * 100
        self.gyro_y_data = [0] * 100
        self.gyro_z_data = [0] * 100

        self.graph_rpy.setLayout(QtWidgets.QVBoxLayout())
        self.graph_acc.setLayout(QtWidgets.QVBoxLayout())
        self.graph_gyro.setLayout(QtWidgets.QVBoxLayout())

        self.rpy_plot = pg.PlotWidget()
        self.acc_plot = pg.PlotWidget()
        self.gyro_plot = pg.PlotWidget()

        self.graph_rpy.layout().addWidget(self.rpy_plot)
        self.graph_acc.layout().addWidget(self.acc_plot)
        self.graph_gyro.layout().addWidget(self.gyro_plot)

        self.roll_curve = self.rpy_plot.plot(self.time_data, self.roll_data, pen=pg.mkPen('r', width=2))
        self.pitch_curve = self.rpy_plot.plot(self.time_data, self.pitch_data, pen=pg.mkPen('g', width=2))
        self.yaw_curve = self.rpy_plot.plot(self.time_data, self.yaw_data, pen=pg.mkPen('b', width=2))

        self.acc_x_curve = self.acc_plot.plot(self.time_data, self.acc_x_data, pen=pg.mkPen('r', width=2))
        self.acc_y_curve = self.acc_plot.plot(self.time_data, self.acc_y_data, pen=pg.mkPen('g', width=2))
        self.acc_z_curve = self.acc_plot.plot(self.time_data, self.acc_z_data, pen=pg.mkPen('b', width=2))

        self.gyro_x_curve = self.gyro_plot.plot(self.time_data, self.gyro_x_data, pen=pg.mkPen('r', width=2))
        self.gyro_y_curve = self.gyro_plot.plot(self.time_data, self.gyro_y_data, pen=pg.mkPen('g', width=2))
        self.gyro_z_curve = self.gyro_plot.plot(self.time_data, self.gyro_z_data, pen=pg.mkPen('b', width=2))

    def init_3d_view(self):
        """3D OpenGL 위젯을 초기화하고 UI에 배치"""
        self.imu_visualizer = IMU3DVisualizer(self.widget_3d_gl)
        layout = QtWidgets.QVBoxLayout(self.widget_3d_gl)
        layout.addWidget(self.imu_visualizer)

    def switch_page(self, index):
        """콤보 박스 선택에 따라 페이지 변경"""
        self.stackedWidget.setCurrentIndex(index)
        self.update_model_label()

    def update_labels(self):
        """초기 라벨 값 설정 - 자동으로 포트 및 구독 노드 설정"""
        self.update_model_label()

        # ✅ 포트 자동 감지 복원
        ports = glob.glob("/dev/ttyACM*")
        port = ports[0] if ports else "No device found"
        self.portname.setText(f"Port: {port}")

        # ✅ 구독 중인 ROS2 노드 자동 표시 복원
        try:
            result = subprocess.run(["ros2", "topic", "info", "/imu/data"], capture_output=True, text=True)
            output_lines = result.stdout.splitlines()
            subscribers = [line.strip() for line in output_lines if "Subscribers:" in line]
            sub_node = subscribers[0] if subscribers else "No subscribers found"
        except Exception as e:
            sub_node = f"Error: {e}"
        self.subscribenode.setText(f"Node: {sub_node}")

    def update_model_label(self):
        """콤보박스에서 선택한 센서명을 model 라벨에 반영"""
        self.model.setText(self.comboBox.currentText())

    
if __name__ == "__main__":
    rclpy.init()
    app = QtWidgets.QApplication(sys.argv)
    gui = ImuGui()
    gui.show()
    app.exec()
    rclpy.shutdown()
