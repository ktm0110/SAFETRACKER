import sys
import os
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from PyQt5.QtWidgets import (
    QApplication, QLabel, QWidget, QVBoxLayout, QPushButton, QTextEdit, QGroupBox,
    QGridLayout, QTableWidget, QTableWidgetItem, QSlider
)
from PyQt5.QtGui import QPixmap, QPainter, QColor, QFont, QImage, QPen
from PyQt5.QtCore import Qt, pyqtSignal, QTimer, QDateTime, QThread, QObject

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

# ------------- Map Image Path -------------
MAP_IMAGE_PATH = "course.png"  # fallback
try:
    from ament_index_python.packages import get_package_share_directory
    package_share_directory = get_package_share_directory("control_system_gui")
    MAP_IMAGE_PATH = os.path.join(package_share_directory, "resource", "course.png")
except Exception:
    pass

# ------------- Qt ↔ ROS Bridge -------------
class RosToGuiBridge(QObject):
    image_received_signal = pyqtSignal(object)          # np.ndarray (BGR)
    obstacle_detected_signal = pyqtSignal(float, float) # x‑norm, y‑norm (0‑1)

# ------------- ROS Spin Thread -------------
class RosSpinThread(QThread):
    def __init__(self, node: Node):
        super().__init__()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(node)
        self.node = node

    def run(self):
        self.executor.spin()

    def stop(self):
        self.executor.shutdown()
        if self.node is not None and self.node.is_alive():
            self.node.destroy_node()
        self.wait()

# ------------- ROS Image Subscriber + ArUco -------------
class ImageSubscriberNode(Node):
    def __init__(self, bridge: RosToGuiBridge):
        super().__init__("image_subscriber_node")
        self.bridge = bridge
        self.cv_bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10
        )

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()
        self.marker_length = 0.10
        self.camera_matrix = np.array([[610, 0, 320], [0, 610, 240], [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros((5,), dtype=np.float32)

    def image_callback(self, msg: Image):
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs
            )
            for i, _ in enumerate(ids):
                tvec = tvecs[i]
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvec, 0.05)
                map_x = max(0.0, min(1.0, 0.5 + tvec[0][0] / 0.5))
                map_y = max(0.0, min(1.0, 0.5 - tvec[0][2] / 0.5))
                self.bridge.obstacle_detected_signal.emit(map_x, map_y)

        self.bridge.image_received_signal.emit(frame)

# ------------- GUI -------------
class ZoomableLabel(QLabel):
    zoom_signal = pyqtSignal(float)
    def wheelEvent(self, event):
        self.zoom_signal.emit(1.1 if event.angleDelta().y() > 0 else 0.9)

class SituationMonitorGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Urban Command & Control Dashboard")
        self.resize(1800, 1000)
        self.setStyleSheet("background:#0d1117;color:#c9d1d9;font-family:Segoe UI;font-size:15px;")

        self.setFocusPolicy(Qt.StrongFocus)
        self.setFocus()

        self.obstacles = []
        self.scale_factor = 1.0

        # ----- PRESET POINTS: 세 개를 직접 지정! -----
        self.preset_points = [(0.85, 0.45), (0.94, 0.35), (0.85, 0.2)]
        self.space_press_count = 0

        self.bridge = RosToGuiBridge()
        self.ros_node = ImageSubscriberNode(self.bridge)
        self.bridge.image_received_signal.connect(self.update_latest_image)
        self.bridge.obstacle_detected_signal.connect(self.add_obstacle_from_ros)
        self.spin_thread = RosSpinThread(self.ros_node)
        self.spin_thread.start()

        self._build_ui()

        self.clock = QTimer(self)
        self.clock.timeout.connect(self.update_time)
        self.clock.start(1000)

    # -------- Key Event: 스페이스바로 preset 좌표 추가 --------
    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Space:
            if self.space_press_count < len(self.preset_points):
                x, y = self.preset_points[self.space_press_count]
                self.add_obstacle(x, y)
                self.log_box.append(f"[⚡] SPACE pressed – preset marker {self.space_press_count+1} ({x:.2f}, {y:.2f})")
                self.space_press_count += 1
            else:
                self.log_box.append("[ℹ] 모든 preset 좌표가 이미 표시됨 (최대 3회)")
            return
        super().keyPressEvent(event)

    # -------- UI Build --------
    def _build_ui(self):
        layout = QGridLayout(self)
        layout.setContentsMargins(30, 30, 30, 30)
        layout.setSpacing(30)

        # Map group
        self.map_label = ZoomableLabel()
        self.map_label.setStyleSheet("border:2px solid #58a6ff;background:#161b22;")
        self.map_label.zoom_signal.connect(self._wheel_zoom)
        self.original_pixmap = QPixmap(MAP_IMAGE_PATH)
        self._rescale_map()

        self.zoom_slider = QSlider(Qt.Horizontal)
        self.zoom_slider.setRange(5, 20)
        self.zoom_slider.setValue(10)
        self.zoom_slider.valueChanged.connect(self._slider_zoom)

        map_box = QGroupBox("🛰 Real‑time Location Map")
        map_layout = QVBoxLayout(map_box)
        map_layout.addWidget(self.map_label)
        map_layout.addWidget(self.zoom_slider)

        # Info group
        info_box = QGroupBox("📊 Obstacle Info & Log")
        info_layout = QVBoxLayout(info_box)

        self.count_label = QLabel("Detected Obstacles: 0")
        self.count_label.setFont(QFont("Segoe UI", 14, QFont.Bold))
        info_layout.addWidget(self.count_label)

        clear_btn = QPushButton("🗑 Clear")
        clear_btn.clicked.connect(self.clear_obstacles)
        info_layout.addWidget(clear_btn)

        self.coord_table = QTableWidget(0, 2)
        self.coord_table.setHorizontalHeaderLabels(["X", "Y"])
        self.coord_table.verticalHeader().setVisible(False)
        info_layout.addWidget(self.coord_table)

        self.latest_image_label = QLabel("(No Image)")
        self.latest_image_label.setFixedSize(640, 480)
        self.latest_image_label.setAlignment(Qt.AlignCenter)
        info_layout.addWidget(self.latest_image_label)

        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        info_layout.addWidget(self.log_box)

        self.timestamp = QLabel()
        info_layout.addWidget(self.timestamp)

        layout.addWidget(map_box, 0, 0, 2, 2)
        layout.addWidget(info_box, 0, 2, 2, 1)
        layout.setColumnStretch(0, 2)
        layout.setColumnStretch(1, 2)
        layout.setColumnStretch(2, 1)

        self.setLayout(layout)

    def _rescale_map(self):
        if self.original_pixmap.isNull():
            self.map_label.setText("[Map cannot be loaded]")
            return
        new_width = int(self.original_pixmap.width() * self.scale_factor)
        new_height = int(self.original_pixmap.height() * self.scale_factor)
        scaled = self.original_pixmap.scaled(new_width, new_height, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.pixmap = scaled
        self.update_map()

    def _slider_zoom(self, value):
        self.scale_factor = value / 10.0
        self._rescale_map()

    def _wheel_zoom(self, factor):
        self.scale_factor = max(0.5, min(3.0, self.scale_factor * factor))
        self.zoom_slider.blockSignals(True)
        self.zoom_slider.setValue(int(self.scale_factor * 10))
        self.zoom_slider.blockSignals(False)
        self._rescale_map()

    def update_map(self):
        if self.pixmap.isNull():
            return
        image = self.pixmap.toImage()
        painter = QPainter(image)
        painter.setRenderHint(QPainter.Antialiasing)
        grid_size = 50
        map_width = image.width()
        map_height = image.height()
        painter.setPen(QPen(QColor(100, 100, 100, 80), 1))
        for x in range(0, map_width, grid_size):
            painter.drawLine(x, 0, x, map_height)
        for y in range(0, map_height, grid_size):
            painter.drawLine(0, y, map_width, y)
        for ox, oy in self.obstacles:
            px = int(ox * map_width)
            py = int(oy * map_height)
            painter.setPen(QPen(QColor("#ffa657"), 2))
            painter.setBrush(QColor("#ffa657"))
            marker_radius = 8
            painter.drawEllipse(px - marker_radius, py - marker_radius, marker_radius * 2, marker_radius * 2)
            painter.setFont(QFont("Segoe UI", 10, QFont.Bold))
            painter.setPen(QColor("#c9d1d9"))
            painter.drawText(int(px + marker_radius + 2), int(py + marker_radius / 2), f"({ox:.2f}, {oy:.2f})")
        painter.end()
        self.map_label.setPixmap(QPixmap.fromImage(image))
        self.map_label.setFixedSize(image.size())

    def add_obstacle_from_ros(self, x, y):
        self.add_obstacle(x, y)

    def add_obstacle(self, x, y):
        self.obstacles.append((x, y))
        self.update_map()
        self.count_label.setText(f"Detected Obstacles: {len(self.obstacles)}")
        self.log_box.append(f"[🚧] Obstacle Detected → Location (x={x:.2f}, y={y:.2f})")
        row = self.coord_table.rowCount()
        self.coord_table.insertRow(row)
        self.coord_table.setItem(row, 0, QTableWidgetItem(f"{x:.2f}"))
        self.coord_table.setItem(row, 1, QTableWidgetItem(f"{y:.2f}"))
        self.coord_table.scrollToBottom()

    def clear_obstacles(self):
        self.obstacles.clear()
        self.update_map()
        self.count_label.setText("Detected Obstacles: 0")
        self.latest_image_label.setText("(No Image)")
        self.latest_image_label.setPixmap(QPixmap())
        self.coord_table.setRowCount(0)
        self.log_box.append("[ℹ] Obstacle display cleared.")

    def update_latest_image(self, cv_image):
        if cv_image is None:
            self.log_box.append("[❌] Received image is invalid.")
            return
        try:
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qimg = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qimg).scaled(
                self.latest_image_label.width(), self.latest_image_label.height(),
                Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
            self.latest_image_label.setPixmap(pixmap)
            self.log_box.append(f"[📸] Camera image received and displayed (Resolution: {w}x{h})")
        except Exception as e:
            self.log_box.append(f"[❌] Image display failed: {e}")

    def update_time(self):
        current_time = QDateTime.currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        self.timestamp.setText(f"⏱ Current Time: {current_time}")

    def closeEvent(self, event):
        self.spin_thread.stop()
        super().closeEvent(event)

if __name__ == "__main__":
    if not rclpy.ok():
        rclpy.init(args=None)
    app = QApplication(sys.argv)
    window = SituationMonitorGUI()
    window.show()
    sys.exit(app.exec_())
    if rclpy.ok():
        rclpy.shutdown()

