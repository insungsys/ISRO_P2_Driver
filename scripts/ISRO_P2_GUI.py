#!/usr/bin/env python3
import sys
import io
import math
import serial.tools.list_ports
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QGroupBox, QLabel, QLineEdit, QComboBox, 
                             QPushButton, QTextEdit, QProgressBar, QMessageBox, 
                             QScrollArea, QCheckBox, QFrame)
from PyQt6.QtCore import QThread, pyqtSignal, Qt, QPointF
from PyQt6.QtGui import QPainter, QPen, QColor, QBrush, QFont, QPolygonF

# 작성하신 업데이터 모듈 임포트
import ISRO_P2_Config


class RotationViewer(QWidget):
    """INS Rotation을 Top View와 Side View로 시각화하는 위젯"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(400, 200)
        self.setMaximumHeight(220)
        
        # 회전 각도 (degrees)
        self.rot_x = 0.0  # Roll
        self.rot_y = 0.0  # Pitch  
        self.rot_z = 0.0  # Yaw
        
        # 색상 정의
        self.color_x = QColor(220, 60, 60)    # 빨강 (X축)
        self.color_y = QColor(60, 180, 60)    # 초록 (Y축)
        self.color_z = QColor(60, 100, 220)   # 파랑 (Z축)
        self.color_car = QColor(100, 100, 100)  # 차량
        
    def set_rotation(self, x, y, z):
        """회전 각도 설정 (degrees)"""
        self.rot_x = x
        self.rot_y = y
        self.rot_z = z
        self.update()
    
    def rotate_3d_z(self, vec, angle_deg):
        """Z축 회전 (NovAtel SPAN 규약)"""
        x, y, z = vec
        angle = math.radians(-angle_deg)  # NovAtel 규약에 맞게 부호 반전
        c, s = math.cos(angle), math.sin(angle)
        return (x*c - y*s, x*s + y*c, z)
    
    def rotate_3d_x(self, vec, angle_deg):
        """X축 회전 (NovAtel SPAN 규약)"""
        x, y, z = vec
        angle = math.radians(-angle_deg)  # NovAtel 규약에 맞게 부호 반전
        c, s = math.cos(angle), math.sin(angle)
        return (x, y*c - z*s, y*s + z*c)
    
    def rotate_3d_y(self, vec, angle_deg):
        """Y축 회전 (NovAtel SPAN 규약)"""
        x, y, z = vec
        angle = math.radians(-angle_deg)  # NovAtel 규약에 맞게 부호 반전
        c, s = math.cos(angle), math.sin(angle)
        return (x*c + z*s, y, -x*s + z*c)
    
    def apply_rbv(self, vec):
        """RBV 회전 적용 (Z → X → Y 순서, NovAtel SPAN 규약)"""
        v = self.rotate_3d_z(vec, self.rot_z)  # 1. Z 회전 (Yaw)
        v = self.rotate_3d_x(v, self.rot_x)    # 2. X 회전 (Roll)
        v = self.rotate_3d_y(v, self.rot_y)    # 3. Y 회전 (Pitch)
        return v
    
    def get_rotated_axes(self):
        """회전된 센서 축 벡터들 반환"""
        x_axis = self.apply_rbv((1, 0, 0))
        y_axis = self.apply_rbv((0, 1, 0))
        z_axis = self.apply_rbv((0, 0, 1))
        return x_axis, y_axis, z_axis
    
    def draw_arrow(self, painter, cx, cy, dx, dy, color, label, scale=1.0):
        """화살표 그리기"""
        pen = QPen(color, 2)
        painter.setPen(pen)
        
        # 축 길이
        length = 50 * scale
        ex = cx + dx * length
        ey = cy - dy * length  # Y축 반전 (화면 좌표계)
        
        # 선 그리기
        painter.drawLine(int(cx), int(cy), int(ex), int(ey))
        
        # 화살표 머리
        arrow_size = 8
        angle = math.atan2(-(ey - cy), ex - cx)
        
        p1_x = ex - arrow_size * math.cos(angle - math.pi/6)
        p1_y = ey + arrow_size * math.sin(angle - math.pi/6)
        p2_x = ex - arrow_size * math.cos(angle + math.pi/6)
        p2_y = ey + arrow_size * math.sin(angle + math.pi/6)
        
        painter.drawLine(int(ex), int(ey), int(p1_x), int(p1_y))
        painter.drawLine(int(ex), int(ey), int(p2_x), int(p2_y))
        
        # 레이블
        font = QFont("Arial", 10, QFont.Weight.Bold)
        painter.setFont(font)
        label_x = ex + dx * 12
        label_y = ey - dy * 12
        painter.drawText(int(label_x) - 5, int(label_y) + 5, label)
    
    def draw_car_top(self, painter, cx, cy, angle_z):
        """Top View 차량 그리기 (센서 축이 회전, 차량은 World 기준 고정)"""
        painter.save()
        painter.translate(cx, cy)
        # 차량은 World 기준으로 고정 (회전 안 함)
        # 센서 축만 회전하여 표시
        
        # 차량 몸체 (위에서 본 모습)
        pen = QPen(self.color_car, 2)
        painter.setPen(pen)
        brush = QBrush(QColor(180, 180, 180, 100))
        painter.setBrush(brush)
        
        # 차량 직사각형 (Y가 전진 방향 = 화면 위쪽)
        car_w, car_h = 24, 40
        painter.drawRect(int(-car_w/2), int(-car_h/2), car_w, car_h)
        
        # 전진 방향 표시 (삼각형) - World Front (화면 위쪽)
        front_triangle = QPolygonF([
            QPointF(0, -car_h/2 - 8),
            QPointF(-6, -car_h/2),
            QPointF(6, -car_h/2)
        ])
        painter.setBrush(QBrush(self.color_car))
        painter.drawPolygon(front_triangle)
        
        painter.restore()
    
    def draw_car_side(self, painter, cx, cy, angle_x=0, angle_y=0):
        """Side View 차량 그리기 (오른쪽에서 본 모습, 차량 고정)"""
        painter.save()
        painter.translate(cx, cy)
        # 차량은 World 기준 고정 (회전 안 함)
        
        # 차량 몸체 (옆에서 본 모습)
        pen = QPen(self.color_car, 2)
        painter.setPen(pen)
        brush = QBrush(QColor(180, 180, 180, 100))
        painter.setBrush(brush)
        
        # 차량 직사각형 (좌측이 전방)
        car_w, car_h = 40, 16
        painter.drawRect(int(-car_w/2), int(-car_h/2), car_w, car_h)
        
        # 바퀴
        painter.setBrush(QBrush(self.color_car))
        painter.drawEllipse(int(-car_w/2 + 4), int(car_h/2 - 2), 8, 8)
        painter.drawEllipse(int(car_w/2 - 12), int(car_h/2 - 2), 8, 8)
        
        # 전진 방향 (좌측이 전방)
        front_triangle = QPolygonF([
            QPointF(-car_w/2 - 8, 0),
            QPointF(-car_w/2, -5),
            QPointF(-car_w/2, 5)
        ])
        painter.drawPolygon(front_triangle)
        
        painter.restore()
    
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        w = self.width()
        h = self.height()
        
        # 배경
        painter.fillRect(0, 0, w, h, QColor(245, 245, 245))
        
        # 구분선
        painter.setPen(QPen(QColor(200, 200, 200), 1))
        painter.drawLine(w // 2, 0, w // 2, h)
        
        # 3D 회전된 축 계산 (Z→X→Y 순서)
        x_axis, y_axis, z_axis = self.get_rotated_axes()
        
        # ===== Top View (좌측) =====
        top_cx = w // 4
        top_cy = h // 2 + 10
        
        # 제목
        painter.setPen(QPen(Qt.GlobalColor.black))
        font = QFont("Arial", 11, QFont.Weight.Bold)
        painter.setFont(font)
        painter.drawText(top_cx - 35, 20, "Top View")
        
        font_small = QFont("Arial", 9)
        painter.setFont(font_small)
        painter.setPen(QPen(QColor(100, 100, 100)))
        painter.drawText(top_cx - 60, 35, f"RBV: ({self.rot_x:.0f}, {self.rot_y:.0f}, {self.rot_z:.0f})")
        
        # 차량 그리기 (고정)
        self.draw_car_top(painter, top_cx, top_cy, 0)  # 차량은 회전 안 함
        
        # Top View: X-Y 평면 투영 (x→오른쪽, y→위쪽)
        self.draw_arrow(painter, top_cx, top_cy, x_axis[0], x_axis[1], self.color_x, "X")
        self.draw_arrow(painter, top_cx, top_cy, y_axis[0], y_axis[1], self.color_y, "Y")
        
        # Z축 표시 (z 성분이 양수면 ⊙, 음수면 ⊗)
        painter.setPen(QPen(self.color_z, 2))
        if z_axis[2] > 0.5:
            painter.setBrush(QBrush(self.color_z))
            painter.drawEllipse(top_cx - 4, top_cy - 4, 8, 8)
            painter.drawText(top_cx + 8, top_cy + 4, "Z⊙")
        elif z_axis[2] < -0.5:
            painter.setBrush(QBrush(Qt.GlobalColor.white))
            painter.drawEllipse(top_cx - 4, top_cy - 4, 8, 8)
            painter.drawLine(top_cx - 3, top_cy - 3, top_cx + 3, top_cy + 3)
            painter.drawLine(top_cx - 3, top_cy + 3, top_cx + 3, top_cy - 3)
            painter.drawText(top_cx + 8, top_cy + 4, "Z⊗")
        else:
            # Z가 수평인 경우 화살표로 표시
            self.draw_arrow(painter, top_cx, top_cy, z_axis[0], z_axis[1], self.color_z, "Z")
        
        # 기준 방향 표시
        painter.setPen(QPen(QColor(150, 150, 150)))
        painter.setFont(font_small)
        painter.drawText(top_cx - 10, h - 10, "Front ↑")
        
        # ===== Side View (우측) =====
        side_cx = w * 3 // 4
        side_cy = h // 2 + 10
        
        # 제목
        painter.setPen(QPen(Qt.GlobalColor.black))
        painter.setFont(font)
        painter.drawText(side_cx - 35, 20, "Side View")
        
        painter.setFont(font_small)
        painter.setPen(QPen(QColor(100, 100, 100)))
        painter.drawText(side_cx - 60, 35, f"(Right side)")
        
        # 차량 그리기 (고정)
        self.draw_car_side(painter, side_cx, side_cy, 0, 0)
        
        # Side View: Y-Z 평면 투영 (y→좌측=전방, z→위쪽)
        # 화면에서: -y_axis[1]→오른쪽(전방이 좌측이므로), z_axis[2]→위쪽
        self.draw_arrow(painter, side_cx, side_cy, -y_axis[1], y_axis[2], self.color_y, "Y")
        self.draw_arrow(painter, side_cx, side_cy, -z_axis[1], z_axis[2], self.color_z, "Z")
        
        # X축 표시 (x 성분이 양수면 ⊗ 화면 안으로, 음수면 ⊙)
        painter.setPen(QPen(self.color_x, 2))
        if x_axis[0] > 0.5:
            painter.setBrush(QBrush(Qt.GlobalColor.white))
            painter.drawEllipse(side_cx - 4, side_cy - 4, 8, 8)
            painter.drawLine(side_cx - 3, side_cy - 3, side_cx + 3, side_cy + 3)
            painter.drawLine(side_cx - 3, side_cy + 3, side_cx + 3, side_cy - 3)
            painter.drawText(side_cx + 8, side_cy + 4, "X⊗")
        elif x_axis[0] < -0.5:
            painter.setBrush(QBrush(self.color_x))
            painter.drawEllipse(side_cx - 4, side_cy - 4, 8, 8)
            painter.drawText(side_cx + 8, side_cy + 4, "X⊙")
        else:
            # X가 Y-Z 평면에 있는 경우
            self.draw_arrow(painter, side_cx, side_cy, -x_axis[1], x_axis[2], self.color_x, "X")
        
        # 기준 방향 표시
        painter.setPen(QPen(QColor(150, 150, 150)))
        painter.drawText(side_cx - 15, h - 10, "← Front")


class StdoutRedirector(io.StringIO):
    """stdout을 캡처하여 시그널로 전달하는 클래스"""
    def __init__(self, signal):
        super().__init__()
        self.signal = signal
        self.original_stdout = sys.stdout
        
    def write(self, text):
        if text.strip():
            self.signal.emit(text.strip())
        self.original_stdout.write(text)
        self.original_stdout.flush()
        
    def flush(self):
        self.original_stdout.flush()


class UpdateWorker(QThread):
    log_signal = pyqtSignal(str)
    finish_signal = pyqtSignal(bool)

    def __init__(self, port, config_text):
        super().__init__()
        self.port = port
        self.config_text = config_text

    def run(self):
        self.log_signal.emit(f"Starting update on {self.port}...")
        
        redirector = StdoutRedirector(self.log_signal)
        old_stdout = sys.stdout
        sys.stdout = redirector
        
        try:
            success = ISRO_P2_Config.perform_update(target_port=self.port, config_text=self.config_text)
            self.finish_signal.emit(success)
        except Exception as e:
            self.log_signal.emit(f"Error: {str(e)}")
            import traceback
            self.log_signal.emit(traceback.format_exc())
            self.finish_signal.emit(False)
        finally:
            sys.stdout = old_stdout


class PIM222A_GUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PIM222A Configuration Tool")
        self.setGeometry(100, 100, 1000, 1100)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        self.main_layout = QVBoxLayout(central_widget)
        
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        self.scroll_layout = QVBoxLayout(scroll_content)
        scroll.setWidget(scroll_content)
        
        self.create_connection_group()
        self.create_com_settings_group()
        self.create_ins_settings_group()
        
        self.main_layout.addWidget(self.conn_group)
        self.main_layout.addWidget(scroll)
        
        self.create_log_group()
        self.main_layout.addWidget(self.log_group)
        
        self.update_ins_ui_state()

    def create_connection_group(self):
        self.conn_group = QGroupBox("Connection")
        layout = QHBoxLayout()
        
        layout.addWidget(QLabel("Target Port:"))
        self.port_combo = QComboBox()
        self.refresh_ports()
        layout.addWidget(self.port_combo)
        
        btn_refresh = QPushButton("Refresh")
        btn_refresh.clicked.connect(self.refresh_ports)
        layout.addWidget(btn_refresh)
        
        self.btn_upload = QPushButton("UPLOAD CONFIG")
        self.btn_upload.setStyleSheet("background-color: #d4f1c5; font-weight: bold; padding: 8px;")
        self.btn_upload.clicked.connect(self.start_upload)
        layout.addWidget(self.btn_upload)
        
        self.conn_group.setLayout(layout)

    def refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.port_combo.addItem(f"{p.device}")

    def create_com_settings_group(self):
        group = QGroupBox("Serial Port Settings")
        layout = QHBoxLayout()
        self.com1_ui = self.create_single_com_ui("COM1", "921600", show_protocol=True)
        layout.addWidget(self.com1_ui)
        self.com2_ui = self.create_single_com_ui("COM2", "115200", show_protocol=False)
        layout.addWidget(self.com2_ui)
        group.setLayout(layout)
        self.scroll_layout.addWidget(group)

    def create_single_com_ui(self, port_name, default_baud, show_protocol=False):
        group = QGroupBox(port_name)
        layout = QVBoxLayout()
        
        h_layout = QHBoxLayout()
        h_layout.addWidget(QLabel("Baudrate:"))
        baud_combo = QComboBox()
        baud_combo.addItems(["115200", "460800", "921600"])
        baud_combo.setCurrentText(default_baud)
        h_layout.addWidget(baud_combo)
        layout.addLayout(h_layout)
        
        # [추가] COM1 전용 Interface Mode (TX Protocol) 설정
        if show_protocol:
            proto_layout = QHBoxLayout()
            proto_layout.addWidget(QLabel("TX Protocol:"))
            proto_combo = QComboBox()
            proto_combo.addItems(["PIMTP (Binary/Default)", "NMEA (ASCII Only)"])
            proto_layout.addWidget(proto_combo)
            layout.addLayout(proto_layout)
            setattr(self, f"{port_name.lower()}_protocol", proto_combo)
        
        # 안내 문구
        layout.addWidget(QLabel("Logs (e.g. 'PVA 0.02', 'GGA 1.0'):"))
        info_label = QLabel("* Valid Intervals - Automotive: 0.02, 0.05, 0.1, 0.2, 1.0 / NMEA: 0.1, 0.2, 1.0")
        info_label.setStyleSheet("color: #666; font-size: 10px;")
        layout.addWidget(info_label)
        
        log_edit = QTextEdit()
        
        if port_name == "COM1":
            example_logs = "PVA 0.02\nIMU\nSTATUS\nGGA 1.0"
        else:
            example_logs = "GGA 1.0\nHDT"
            
        log_edit.setPlaceholderText(f"e.g.\n{example_logs}")
        log_edit.setPlainText(example_logs)
        log_edit.setMaximumHeight(100)
        layout.addWidget(log_edit)
        
        group.setLayout(layout)
        setattr(self, f"{port_name.lower()}_baud", baud_combo)
        setattr(self, f"{port_name.lower()}_logs", log_edit)
        return group

    def create_ins_settings_group(self):
        group = QGroupBox("INS & Antenna Installation")
        layout = QVBoxLayout()
        
        opts_layout = QHBoxLayout()
        self.cb_enable_ins = QCheckBox("Enable INS")
        self.cb_enable_ins.setChecked(True)
        self.cb_enable_ins.toggled.connect(self.update_ins_ui_state)
        
        self.cb_dual_ant = QCheckBox("Use Dual Antenna (ANT2)")
        self.cb_dual_ant.setChecked(True)
        self.cb_dual_ant.toggled.connect(self.update_ins_ui_state)
        
        opts_layout.addWidget(self.cb_enable_ins)
        opts_layout.addWidget(self.cb_dual_ant)
        layout.addLayout(opts_layout)
        
        # Rotation 입력과 시각화를 가로로 배치
        rotation_layout = QHBoxLayout()
        
        # 좌측: 입력 필드
        self.rbv_group = QGroupBox("Rotation (RBV) - [deg]")
        rbv_layout = QVBoxLayout()
        
        # X (Roll)
        x_layout = QHBoxLayout()
        x_layout.addWidget(QLabel("X (Roll):"))
        self.rbv_x = QLineEdit("0.0")
        self.rbv_x.textChanged.connect(self.update_rotation_view)
        x_layout.addWidget(self.rbv_x)
        rbv_layout.addLayout(x_layout)
        
        # Y (Pitch)
        y_layout = QHBoxLayout()
        y_layout.addWidget(QLabel("Y (Pitch):"))
        self.rbv_y = QLineEdit("0.0")
        self.rbv_y.textChanged.connect(self.update_rotation_view)
        y_layout.addWidget(self.rbv_y)
        rbv_layout.addLayout(y_layout)
        
        # Z (Yaw)
        z_layout = QHBoxLayout()
        z_layout.addWidget(QLabel("Z (Yaw):"))
        self.rbv_z = QLineEdit("0.0")
        self.rbv_z.textChanged.connect(self.update_rotation_view)
        z_layout.addWidget(self.rbv_z)
        rbv_layout.addLayout(z_layout)
        
        # 좌표계 설명
        coord_label = QLabel("Default: Y=Front, X=Right, Z=Up")
        coord_label.setStyleSheet("color: #666; font-size: 10px;")
        rbv_layout.addWidget(coord_label)
        
        self.rbv_group.setLayout(rbv_layout)
        rotation_layout.addWidget(self.rbv_group)
        
        # 우측: 시각화
        self.rotation_viewer = RotationViewer()
        rotation_layout.addWidget(self.rotation_viewer, stretch=2)
        
        layout.addLayout(rotation_layout)
        
        # Lever Arm 설정
        lever_layout = QHBoxLayout()
        
        self.la1_group = QGroupBox("Lever Arm (ANT1 - Primary) - [m]")
        la1_layout = QHBoxLayout()
        self.la1_x = QLineEdit("0.0"); la1_layout.addWidget(QLabel("X:")); la1_layout.addWidget(self.la1_x)
        self.la1_y = QLineEdit("0.0"); la1_layout.addWidget(QLabel("Y:")); la1_layout.addWidget(self.la1_y)
        self.la1_z = QLineEdit("0.0"); la1_layout.addWidget(QLabel("Z:")); la1_layout.addWidget(self.la1_z)
        self.la1_group.setLayout(la1_layout)
        lever_layout.addWidget(self.la1_group)

        self.la2_group = QGroupBox("Lever Arm (ANT2 - Secondary) - [m]")
        la2_layout = QHBoxLayout()
        self.la2_x = QLineEdit("0.0"); la2_layout.addWidget(QLabel("X:")); la2_layout.addWidget(self.la2_x)
        self.la2_y = QLineEdit("0.0"); la2_layout.addWidget(QLabel("Y:")); la2_layout.addWidget(self.la2_y)
        self.la2_z = QLineEdit("0.0"); la2_layout.addWidget(QLabel("Z:")); la2_layout.addWidget(self.la2_z)
        self.la2_group.setLayout(la2_layout)
        lever_layout.addWidget(self.la2_group)
        
        layout.addLayout(lever_layout)
        
        group.setLayout(layout)
        self.scroll_layout.addWidget(group)
    
    def update_rotation_view(self):
        """회전 값이 변경될 때 시각화 업데이트"""
        try:
            x = float(self.rbv_x.text() or 0)
            y = float(self.rbv_y.text() or 0)
            z = float(self.rbv_z.text() or 0)
            self.rotation_viewer.set_rotation(x, y, z)
        except ValueError:
            pass  # 잘못된 입력 무시

    def update_ins_ui_state(self):
        use_ins = self.cb_enable_ins.isChecked()
        use_dual = self.cb_dual_ant.isChecked()
        
        self.rbv_group.setEnabled(use_ins)
        self.rotation_viewer.setEnabled(use_ins)
        self.la1_group.setEnabled(use_ins)
        self.la2_group.setEnabled(use_ins and use_dual)
        self.cb_dual_ant.setEnabled(use_ins)

    def create_log_group(self):
        self.log_group = QGroupBox("Status & Preview")
        layout = QVBoxLayout()
        
        self.btn_preview = QPushButton("Generate Config Preview")
        self.btn_preview.clicked.connect(self.generate_config)
        layout.addWidget(self.btn_preview)
        
        self.config_preview = QTextEdit()
        self.config_preview.setReadOnly(True)
        self.config_preview.setPlaceholderText("Generated config will appear here...")
        self.config_preview.setMaximumHeight(150)
        layout.addWidget(self.config_preview)
        
        layout.addWidget(QLabel("Update Log:"))
        self.log_output = QTextEdit()
        self.log_output.setReadOnly(True)
        self.log_output.setStyleSheet("background-color: #1e1e1e; color: #00ff00; font-family: monospace;")
        self.log_output.setMinimumHeight(180)
        layout.addWidget(self.log_output)
        
        self.progress_bar = QProgressBar()
        layout.addWidget(self.progress_bar)
        
        self.status_label = QLabel("Ready")
        self.status_label.setStyleSheet("font-weight: bold; color: blue;")
        layout.addWidget(self.status_label)
        
        self.log_group.setLayout(layout)

    def get_closest_valid_interval(self, value, min_val=0.0):
        """입력된 값을 NovAtel 표준 Interval 중 가장 가까운 값으로 보정"""
        VALID_INTERVALS = [0.02, 0.05, 0.1, 0.2, 1.0]
        
        if value < min_val:
            value = min_val
            
        closest = min(VALID_INTERVALS, key=lambda x: abs(x - value))
        return closest

    def generate_config(self):
        """Config 생성 (표준 Interval 보정 적용)"""
        config = []
        
        # [추가] COM1 Interface Mode 설정 (맨 위에 배치)
        if hasattr(self, "com1_protocol"):
            if "NMEA" in self.com1_protocol.currentText():
                config.append("InterfaceMode:COM1,PIMTP,NMEA")
            # PIMTP 선택 시에는 기본값이므로 구문을 추가하지 않음
        
        config.append("IMU:INTERNAL")
        config.append("NMEATALKER:AUTO")
        
        config.append(f"BaudRate:COM1,{self.com1_baud.currentText()}")
        config.append(f"BaudRate:COM2,{self.com2_baud.currentText()}")
        
        # Log 설정 처리
        def process_logs(text_edit, port_name):
            logs = text_edit.toPlainText().strip().split('\n')
            for log in logs:
                if not log.strip(): continue
                parts = log.strip().split()
                msg = parts[0].upper()
                
                # CHANGE 트리거
                if msg in ["HDT", "IMU", "STATUS", "TXT_VERSION", "TXT_RXERROR", "TXT_RXSTATUS", "TXT_ITV", "TXT_SOLUTIONINFO"]:
                    if msg in ["HDT", "TXT_VERSION", "TXT_RXERROR", "TXT_RXSTATUS", "TXT_ITV", "TXT_SOLUTIONINFO"]:
                         config.append(f"LogNMEA:{port_name},{msg},CHANGE,0,0")
                    else:
                         config.append(f"LogAutomotive:{port_name},{msg},CHANGE,0,0")
                
                # TIME 트리거
                else:
                    raw_period = 1.0
                    if len(parts) >= 2:
                        try:
                            raw_period = float(parts[-1]) 
                        except:
                            raw_period = 1.0
                    
                    # NMEA vs Automotive 구분
                    if msg in ["GGA", "VTG", "GSA", "GST", "GSV", "PASHR", "ZDA"]:
                        valid_period = self.get_closest_valid_interval(raw_period, min_val=0.1)
                        config.append(f"LogNMEA:{port_name},{msg},TIME,{valid_period},0")
                    else:
                        valid_period = self.get_closest_valid_interval(raw_period, min_val=0.02)
                        config.append(f"LogAutomotive:{port_name},{msg},TIME,{valid_period},0")

        process_logs(self.com1_logs, "COM1")
        process_logs(self.com2_logs, "COM2")
        
        # INS 설정 추가
        if self.cb_enable_ins.isChecked():
            config.append(f"INSRotation:RBV,{self.rbv_x.text()},{self.rbv_y.text()},{self.rbv_z.text()},3.0,3.0,3.0")
            config.append(f"INSTranslation:Ant1,{self.la1_x.text()},{self.la1_y.text()},{self.la1_z.text()},0.05,0.05,0.05")
            if self.cb_dual_ant.isChecked():
                config.append(f"INSTranslation:Ant2,{self.la2_x.text()},{self.la2_y.text()},{self.la2_z.text()},0.05,0.05,0.05")
        
        final_text = "\n".join(config)
        self.config_preview.setText(final_text)
        return final_text

    def start_upload(self):
        port = self.port_combo.currentText()
        if not port:
            QMessageBox.warning(self, "Error", "No port selected!")
            return
            
        config_text = self.generate_config()
        if not config_text:
            return

        self.btn_upload.setEnabled(False)
        self.status_label.setText("Starting Update Process...")
        self.progress_bar.setRange(0, 0)
        
        self.log_output.clear()
        self.log_output.append("=" * 50)
        self.log_output.append("Starting Config Upload...")
        self.log_output.append("=" * 50)
        
        self.worker = UpdateWorker(port, config_text)
        self.worker.log_signal.connect(self.update_log)
        self.worker.finish_signal.connect(self.update_finished)
        self.worker.start()

    def update_log(self, msg):
        self.status_label.setText(msg[:80] + "..." if len(msg) > 80 else msg)
        self.log_output.append(msg)
        scrollbar = self.log_output.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def update_finished(self, success):
        self.progress_bar.setRange(0, 100)
        self.btn_upload.setEnabled(True)
        
        if success:
            self.progress_bar.setValue(100)
            self.status_label.setText("Update Successful!")
            QMessageBox.information(self, "Success", "Configuration uploaded successfully!\nDevice is rebooting.")
        else:
            self.progress_bar.setValue(0)
            self.status_label.setText("Update Failed.")
            QMessageBox.critical(self, "Failed", "Update failed. Check connections and try again.")


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = PIM222A_GUI()
    window.show()
    sys.exit(app.exec())
