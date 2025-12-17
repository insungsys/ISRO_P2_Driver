#!/usr/bin/env python3
import sys
import serial.tools.list_ports
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QGroupBox, QLabel, QLineEdit, QComboBox, 
                             QPushButton, QTextEdit, QProgressBar, QMessageBox, QScrollArea, QCheckBox)
from PyQt6.QtCore import QThread, pyqtSignal, Qt

# 작성하신 업데이터 모듈 임포트
import ISRO_P2_Config

class UpdateWorker(QThread):
    log_signal = pyqtSignal(str)
    finish_signal = pyqtSignal(bool)

    def __init__(self, port, config_text):
        super().__init__()
        self.port = port
        self.config_text = config_text

    def run(self):
        self.log_signal.emit(f"Starting update on {self.port}...")
        self.log_signal.emit("Waiting for device power cycle (Turn ON device now)...")
        
        try:
            success = ISRO_P2_Config.perform_update(target_port=self.port, config_text=self.config_text)
            self.finish_signal.emit(success)
        except Exception as e:
            self.log_signal.emit(f"Error: {str(e)}")
            self.finish_signal.emit(False)

class PIM222A_GUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PIM222A Configuration Tool (Standard Intervals)")
        self.setGeometry(100, 100, 950, 950)
        
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
        self.com1_ui = self.create_single_com_ui("COM1", "921600")
        layout.addWidget(self.com1_ui)
        self.com2_ui = self.create_single_com_ui("COM2", "115200")
        layout.addWidget(self.com2_ui)
        group.setLayout(layout)
        self.scroll_layout.addWidget(group)

    def create_single_com_ui(self, port_name, default_baud):
        group = QGroupBox(port_name)
        layout = QVBoxLayout()
        
        h_layout = QHBoxLayout()
        h_layout.addWidget(QLabel("Baudrate:"))
        baud_combo = QComboBox()
        baud_combo.addItems(["115200", "230400", "460800", "921600"])
        baud_combo.setCurrentText(default_baud)
        h_layout.addWidget(baud_combo)
        layout.addLayout(h_layout)
        
        # 안내 문구 추가
        layout.addWidget(QLabel("Logs (e.g. 'PVA 0.02', 'GGA 1.0'):"))
        layout.addWidget(QLabel("* Valid Intervals(Automotive): 0.02, 0.05, 0.1, 0.2, 1.0"))
        
        log_edit = QTextEdit()
        
        if port_name == "COM1":
            example_logs = "PVA 0.02\nIMU\nSTATUS"
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
        
        self.rbv_group = QGroupBox("Rotation (RBV) - [deg]")
        rbv_layout = QHBoxLayout()
        self.rbv_x = QLineEdit("0.0"); rbv_layout.addWidget(QLabel("X:")); rbv_layout.addWidget(self.rbv_x)
        self.rbv_y = QLineEdit("0.0"); rbv_layout.addWidget(QLabel("Y:")); rbv_layout.addWidget(self.rbv_y)
        self.rbv_z = QLineEdit("0.0"); rbv_layout.addWidget(QLabel("Z:")); rbv_layout.addWidget(self.rbv_z)
        self.rbv_group.setLayout(rbv_layout)
        layout.addWidget(self.rbv_group)
        
        self.la1_group = QGroupBox("Lever Arm (ANT1 - Primary) - [m]")
        la1_layout = QHBoxLayout()
        self.la1_x = QLineEdit("0.0"); la1_layout.addWidget(QLabel("X:")); la1_layout.addWidget(self.la1_x)
        self.la1_y = QLineEdit("0.0"); la1_layout.addWidget(QLabel("Y:")); la1_layout.addWidget(self.la1_y)
        self.la1_z = QLineEdit("0.0"); la1_layout.addWidget(QLabel("Z:")); la1_layout.addWidget(self.la1_z)
        self.la1_group.setLayout(la1_layout)
        layout.addWidget(self.la1_group)

        self.la2_group = QGroupBox("Lever Arm (ANT2 - Secondary) - [m]")
        la2_layout = QHBoxLayout()
        self.la2_x = QLineEdit("0.0"); la2_layout.addWidget(QLabel("X:")); la2_layout.addWidget(self.la2_x)
        self.la2_y = QLineEdit("0.0"); la2_layout.addWidget(QLabel("Y:")); la2_layout.addWidget(self.la2_y)
        self.la2_z = QLineEdit("0.0"); la2_layout.addWidget(QLabel("Z:")); la2_layout.addWidget(self.la2_z)
        self.la2_group.setLayout(la2_layout)
        layout.addWidget(self.la2_group)
        
        group.setLayout(layout)
        self.scroll_layout.addWidget(group)

    def update_ins_ui_state(self):
        use_ins = self.cb_enable_ins.isChecked()
        use_dual = self.cb_dual_ant.isChecked()
        
        self.rbv_group.setEnabled(use_ins)
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
        layout.addWidget(self.config_preview)
        
        self.progress_bar = QProgressBar()
        layout.addWidget(self.progress_bar)
        
        self.status_label = QLabel("Ready")
        self.status_label.setStyleSheet("font-weight: bold; color: blue;")
        layout.addWidget(self.status_label)
        
        self.log_group.setLayout(layout)

    def get_closest_valid_interval(self, value, min_val=0.0):
        """입력된 값을 NovAtel 표준 Interval 중 가장 가까운 값으로 보정"""
        # NovAtel Supported Intervals (s)
        VALID_INTERVALS = [ 0.02, 0.05, 0.1, 0.2, 1.0]
        
        # 1. 최소값 제한 적용
        if value < min_val:
            value = min_val
            
        # 2. 가장 가까운 표준값 찾기
        closest = min(VALID_INTERVALS, key=lambda x: abs(x - value))
        return closest

    def generate_config(self):
        """Config 생성 (표준 Interval 보정 적용)"""
        config = []
        config.append("IMU:INTERNAL")
        config.append("NMEATALKER:AUTO")
        
        config.append(f"BaudRate:COM1,{self.com1_baud.currentText()}")
        config.append(f"BaudRate:COM2,{self.com2_baud.currentText()}")
        
        if self.cb_enable_ins.isChecked():
            config.append(f"INSRotation:RBV,{self.rbv_x.text()},{self.rbv_y.text()},{self.rbv_z.text()},3.0,3.0,3.0")
            config.append(f"INSTranslation:Ant1,{self.la1_x.text()},{self.la1_y.text()},{self.la1_z.text()},0.05,0.05,0.05")
            if self.cb_dual_ant.isChecked():
                config.append(f"INSTranslation:Ant2,{self.la2_x.text()},{self.la2_y.text()},{self.la2_z.text()},0.05,0.05,0.05")
        
        def process_logs(text_edit, port_name):
            logs = text_edit.toPlainText().strip().split('\n')
            for log in logs:
                if not log.strip(): continue
                parts = log.strip().split()
                msg = parts[0].upper()
                
                # 1. CHANGE 트리거 (HDT, IMU, STATUS)
                if msg in ["HDT", "IMU", "STATUS", "TXT_VERSION", "TXT_RXERROR", "TXT_RXSTATUS", "TXT_ITV", "TXT_SOLUTIONINFO"]:
                    # CHANGE는 주기가 의미 없으므로 무조건 0,0
                    if msg in ["HDT", "TXT_VERSION", "TXT_RXERROR", "TXT_RXSTATUS", "TXT_ITV", "TXT_SOLUTIONINFO"]:
                         config.append(f"LogNMEA:{port_name},{msg},CHANGE,0,0")
                    else:
                         config.append(f"LogAutomotive:{port_name},{msg},CHANGE,0,0")
                
                # 2. TIME 트리거
                else:
                    raw_period = 1.0
                    if len(parts) >= 2:
                        try:
                            # 숫자만 파싱
                            raw_period = float(parts[-1]) 
                        except:
                            raw_period = 1.0
                    
                    # NMEA vs Automotive 구분 및 보정
                    if msg in ["GGA", "VTG", "GSA", "GST", "GSV", "PASHR", "ZDA"]:
                        # NMEA Min = 0.1s
                        valid_period = self.get_closest_valid_interval(raw_period, min_val=0.1)
                        config.append(f"LogNMEA:{port_name},{msg},TIME,{valid_period},0")
                    else:
                        # Automotive Min = 0.02s 
                        valid_period = self.get_closest_valid_interval(raw_period, min_val=0.02)
                        config.append(f"LogAutomotive:{port_name},{msg},TIME,{valid_period},0")

        process_logs(self.com1_logs, "COM1")
        process_logs(self.com2_logs, "COM2")
        
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
        
        self.worker = UpdateWorker(port, config_text)
        self.worker.log_signal.connect(self.update_log)
        self.worker.finish_signal.connect(self.update_finished)
        self.worker.start()

    def update_log(self, msg):
        self.status_label.setText(msg)

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