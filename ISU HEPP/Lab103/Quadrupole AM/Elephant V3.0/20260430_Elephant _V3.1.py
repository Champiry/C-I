# -*- coding: utf-8 -*-
"""
Created on Fri May  1 03:53:15 2026

@author: Afshin-III
"""

"""
Elephant V3.1 - PyQt5 Serial Control GUI

Workflow and data model:
- Setpoint: user-edited GUI values. Clear buttons only reset local GUI setpoints.
- Controller Status: values stored in Arduino local memory after the last accepted update.
- Measurement: true hardware values read by Arduino ADC/GPIO.

Serial workflow:
1. Connect to Arduino.
2. Request all controller status + measurement values once.
3. Continuously request measurement values only.
4. When Update Coil/HC is pressed:
   - send that section's setpoints to Arduino,
   - wait for ACK,
   - request all controller status + measurement values again.

Protocol V3.1 proposal:
PC -> Arduino: [0xAA][CMD][LEN][DATA...][CRC8]
Arduino -> PC: [0x55][RSP][LEN][DATA...][CRC8]
CRC8 is calculated over [CMD/RSP][LEN][DATA...].

PC commands:
0x10 CMD_SET_COIL
0x20 CMD_SET_HC
0x30 CMD_REQUEST_ALL
0x31 CMD_REQUEST_MEASUREMENT
0x40 CMD_TERMINAL_MESSAGE
0x50 CMD_DISABLE_OUTPUT

Arduino responses:
0x90 RSP_ALL_DATA        status + measurement
0x91 RSP_COIL_ACK
0x92 RSP_HC_ACK
0x93 RSP_MEASUREMENT    measurement only
0x94 RSP_TERMINAL
0x95 RSP_DISABLE_ACK
0xE0 RSP_ERROR

Scaling:
Current in A is sent as uint16 = A x 100.
AC current in mA is sent as uint16 = mA.
Temperature in C is sent as uint16 = C x 10.
Frequency in Hz is sent as uint16.
Waveform: 0=Sine, 1=Triangle, 2=Square.
Flags: bit0=power, bit1=polarity CCW.
"""

import sys
import time
from pathlib import Path

import serial
import serial.tools.list_ports

from PyQt5.QtCore import QThread, QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


# -----------------------------------------------------------------------------
# Protocol constants
# -----------------------------------------------------------------------------
HEADER_PC = 0xAA
HEADER_ARDUINO = 0x55

CMD_SET_COIL = 0x10
CMD_SET_HC = 0x20
CMD_REQUEST_ALL = 0x30
CMD_REQUEST_MEASUREMENT = 0x31
CMD_TERMINAL_MESSAGE = 0x40
CMD_DISABLE_OUTPUT = 0x50

RSP_ALL_DATA = 0x90
RSP_COIL_ACK = 0x91
RSP_HC_ACK = 0x92
RSP_MEASUREMENT = 0x93
RSP_TERMINAL = 0x94
RSP_DISABLE_ACK = 0x95
RSP_ERROR = 0xE0

WAVEFORM_TO_CODE = {"Sine": 0, "Triangle": 1, "Square": 2}
CODE_TO_WAVEFORM = {v: k for k, v in WAVEFORM_TO_CODE.items()}


# -----------------------------------------------------------------------------
# Protocol helpers
# -----------------------------------------------------------------------------
def crc8(data: bytes) -> int:
    """CRC-8/ATM, polynomial 0x07, init 0x00."""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def u16_to_bytes(value: int) -> bytes:
    value = max(0, min(65535, int(value)))
    return bytes([(value >> 8) & 0xFF, value & 0xFF])


def bytes_to_u16(msb: int, lsb: int) -> int:
    return ((msb & 0xFF) << 8) | (lsb & 0xFF)


def packet_to_hex(packet: bytes) -> str:
    return " ".join(f"{b:02X}" for b in packet)


def build_pc_packet(cmd: int, payload: bytes = b"") -> bytes:
    length = len(payload)
    crc = crc8(bytes([cmd, length]) + payload)
    return bytes([HEADER_PC, cmd, length]) + payload + bytes([crc])


def encode_flags(power_on: bool, polarity_ccw: bool) -> int:
    flags = 0
    if power_on:
        flags |= 0x01
    if polarity_ccw:
        flags |= 0x02
    return flags


def decode_flags(flags: int):
    power_on = bool(flags & 0x01)
    polarity_ccw = bool(flags & 0x02)
    return power_on, polarity_ccw


def fmt_onoff(value: bool) -> str:
    return "ON" if value else "OFF"


def fmt_pol(value_ccw: bool) -> str:
    return "CCW" if value_ccw else "CW"


def calc_mod_depth(ac_ma: float, dc_a: float) -> float:
    if dc_a <= 0:
        return 0.0
    return (ac_ma / 1000.0) / dc_a * 100.0


# -----------------------------------------------------------------------------
# Serial reader thread
# -----------------------------------------------------------------------------
class SerialReader(QThread):
    packet_received = pyqtSignal(int, bytes)
    raw_received = pyqtSignal(bytes)
    error = pyqtSignal(str)

    def __init__(self, ser: serial.Serial):
        super().__init__()
        self.ser = ser
        self.running = True
        self.rx_buffer = bytearray()

    def stop(self):
        self.running = False
        self.wait(700)

    def run(self):
        while self.running:
            try:
                if self.ser and self.ser.is_open:
                    data = self.ser.read(128)
                    if data:
                        self.raw_received.emit(data)
                        self.rx_buffer.extend(data)
                        self.process_buffer()
                    else:
                        self.msleep(5)
                else:
                    self.msleep(20)
            except Exception as exc:
                self.error.emit(str(exc))
                break

    def process_buffer(self):
        while len(self.rx_buffer) >= 4:
            if self.rx_buffer[0] != HEADER_ARDUINO:
                del self.rx_buffer[0]
                continue

            rsp = self.rx_buffer[1]
            length = self.rx_buffer[2]
            total_len = 4 + length

            if len(self.rx_buffer) < total_len:
                return

            frame = bytes(self.rx_buffer[:total_len])
            del self.rx_buffer[:total_len]

            payload = frame[3 : 3 + length]
            received_crc = frame[-1]
            expected_crc = crc8(bytes([rsp, length]) + payload)

            if received_crc != expected_crc:
                self.error.emit(f"Bad CRC: {packet_to_hex(frame)}")
                continue

            self.packet_received.emit(rsp, payload)


# -----------------------------------------------------------------------------
# Main GUI
# -----------------------------------------------------------------------------
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Elephant V3.1 - Arduino Control Panel")
        self.resize(1220, 730)

        self.ser = None
        self.reader = None
        self.tx_count = 0
        self.rx_count = 0
        self.error_count = 0
        self.last_rx_time = None

        self.measurement_timer = QTimer()
        self.measurement_timer.setInterval(200)
        self.measurement_timer.timeout.connect(self.request_measurement)

        self.ui_timer = QTimer()
        self.ui_timer.setInterval(250)
        self.ui_timer.timeout.connect(self.update_connection_status_age)
        self.ui_timer.start()

        self.build_ui()
        self.apply_style()
        self.refresh_ports()
        self.set_connected_state(False)
        self.update_coil_mod_setpoint()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------
    def build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(8, 6, 8, 6)
        main_layout.setSpacing(4)

        top_bar = QHBoxLayout()
        title = QLabel("Elephant V3.1")
        title.setObjectName("mainTitle")
        self.disable_button = QPushButton("Disable Output")
        self.disable_button.setObjectName("dangerButton")
        self.help_button = QPushButton("Help")
        self.about_button = QPushButton("About")

        top_bar.addWidget(title, 1)
        top_bar.addStretch(3)
        top_bar.addWidget(self.disable_button)
        top_bar.addWidget(self.help_button)
        top_bar.addWidget(self.about_button)
        main_layout.addLayout(top_bar)

        self.disable_button.clicked.connect(self.send_disable_request)
        self.help_button.clicked.connect(self.show_help)
        self.about_button.clicked.connect(self.show_about)

        body_layout = QHBoxLayout()
        body_layout.setSpacing(8)
        left_layout = QVBoxLayout()
        left_layout.setSpacing(6)
        right_layout = QVBoxLayout()
        right_layout.setSpacing(6)

        left_layout.addWidget(self.build_coil_group())
        left_layout.addWidget(self.build_hc_group())
        right_layout.addWidget(self.build_connection_group())
        right_layout.addWidget(self.build_log_group())

        body_layout.addLayout(left_layout, 3)
        body_layout.addLayout(right_layout, 2)

        body_widget = QWidget()
        body_widget.setLayout(body_layout)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setWidget(body_widget)
        main_layout.addWidget(scroll, 1)

        bottom_bar = QHBoxLayout()
        self.tx_label = QLabel("TX: 0")
        self.rx_label = QLabel("RX: 0")
        self.err_label = QLabel("Errors: 0")
        self.conn_age_label = QLabel("Disconnected")
        bottom_bar.addWidget(self.tx_label)
        bottom_bar.addWidget(self.rx_label)
        bottom_bar.addWidget(self.err_label)
        bottom_bar.addStretch()
        bottom_bar.addWidget(self.conn_age_label)
        main_layout.addLayout(bottom_bar)

    def build_coil_group(self):
        group = QGroupBox("Coil #1")
        grid = QGridLayout(group)
        grid.setContentsMargins(8, 10, 8, 8)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(5)

        grid.addWidget(QLabel("Parameter"), 0, 0)
        grid.addWidget(QLabel("Setpoint"), 0, 1)
        grid.addWidget(QLabel("Controller Status"), 0, 2)
        grid.addWidget(QLabel("Measurement"), 0, 3)

        self.coil_pw_set = QPushButton("OFF")
        self.coil_pw_set.setCheckable(True)
        self.coil_pw_set.clicked.connect(lambda: self.update_toggle_button(self.coil_pw_set, "ON", "OFF", "coil"))
        self.coil_pw_stat = QLabel("---")
        self.coil_pw_msr = QLabel("---")

        self.coil_pol_set = QPushButton("CW")
        self.coil_pol_set.setCheckable(True)
        self.coil_pol_set.clicked.connect(lambda: self.update_toggle_button(self.coil_pol_set, "CCW", "CW", "coil"))
        self.coil_pol_stat = QLabel("---")
        self.coil_pol_msr = QLabel("---")

        self.coil_dc_crt_set = QDoubleSpinBox()
        self.coil_dc_crt_set.setRange(0.0, 10.0)
        self.coil_dc_crt_set.setDecimals(2)
        self.coil_dc_crt_set.setSingleStep(0.1)
        self.coil_dc_crt_set.setSuffix(" A")
        self.coil_dc_crt_set.valueChanged.connect(self.update_coil_mod_setpoint)
        self.coil_dc_crt_stat = QLabel("---")
        self.coil_dc_crt_msr = QLabel("---")

        self.coil_ac_crt_set = QSpinBox()
        self.coil_ac_crt_set.setRange(0, 100)
        self.coil_ac_crt_set.setSuffix(" mA")
        self.coil_ac_crt_set.valueChanged.connect(self.update_coil_mod_setpoint)
        self.coil_ac_crt_stat = QLabel("---")
        self.coil_ac_crt_msr = QLabel("---")

        self.coil_mod_set = QLabel("0.00 %")
        self.coil_mod_set.setObjectName("readonlySetpoint")
        self.coil_mod_stat = QLabel("---")
        self.coil_mod_msr = QLabel("---")

        self.coil_mod_freq_set = QSpinBox()
        self.coil_mod_freq_set.setRange(0, 10000)
        self.coil_mod_freq_set.setValue(1000)
        self.coil_mod_freq_set.setSuffix(" Hz")
        self.coil_mod_freq_set.valueChanged.connect(lambda: self.coil_state_label.setText("Modified / not sent"))
        self.coil_mod_freq_stat = QLabel("---")

        self.coil_wf_set = QComboBox()
        self.coil_wf_set.addItems(["Sine", "Triangle", "Square"])
        self.coil_wf_set.currentTextChanged.connect(lambda: self.coil_state_label.setText("Modified / not sent"))
        self.coil_wf_stat = QLabel("---")

        self.coil_temp_msr = QLabel("---")

        rows = [
            ("Power", self.coil_pw_set, self.coil_pw_stat, self.coil_pw_msr),
            ("Polarity", self.coil_pol_set, self.coil_pol_stat, self.coil_pol_msr),
            ("DC Current", self.coil_dc_crt_set, self.coil_dc_crt_stat, self.coil_dc_crt_msr),
            ("AC Current", self.coil_ac_crt_set, self.coil_ac_crt_stat, self.coil_ac_crt_msr),
            ("Modulation Depth", self.coil_mod_set, self.coil_mod_stat, self.coil_mod_msr),
            ("Modulation Frequency", self.coil_mod_freq_set, self.coil_mod_freq_stat, QLabel("--")),
            ("Waveform", self.coil_wf_set, self.coil_wf_stat, QLabel("--")),
            ("Temperature", QLabel("--"), QLabel("--"), self.coil_temp_msr),
        ]

        for row_index, (label, set_widget, stat_widget, msr_widget) in enumerate(rows, start=1):
            grid.addWidget(QLabel(label), row_index, 0)
            grid.addWidget(set_widget, row_index, 1)
            grid.addWidget(stat_widget, row_index, 2)
            grid.addWidget(msr_widget, row_index, 3)

        self.coil_state_label = QLabel("Not sent")
        self.coil_state_label.setObjectName("stateLabel")
        self.clear_coil_button = QPushButton("Clear")
        self.update_coil_button = QPushButton("Update Coil Setpoints")
        self.update_coil_button.setObjectName("primaryButton")
        self.clear_coil_button.clicked.connect(self.clear_coil_setpoints)
        self.update_coil_button.clicked.connect(self.send_coil_setpoints)

        grid.addWidget(self.coil_state_label, 9, 0)
        grid.addWidget(self.clear_coil_button, 9, 1)
        grid.addWidget(self.update_coil_button, 9, 2, 1, 2)

        return group

    def build_hc_group(self):
        group = QGroupBox("Helmholtz Coil")
        grid = QGridLayout(group)
        grid.setContentsMargins(8, 10, 8, 8)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(5)

        grid.addWidget(QLabel("Parameter"), 0, 0)
        grid.addWidget(QLabel("Setpoint"), 0, 1)
        grid.addWidget(QLabel("Controller Status"), 0, 2)
        grid.addWidget(QLabel("Measurement"), 0, 3)

        self.hc_pw_set = QPushButton("OFF")
        self.hc_pw_set.setCheckable(True)
        self.hc_pw_set.clicked.connect(lambda: self.update_toggle_button(self.hc_pw_set, "ON", "OFF", "hc"))
        self.hc_pw_stat = QLabel("---")
        self.hc_pw_msr = QLabel("---")

        self.hc_pol_set = QPushButton("CW")
        self.hc_pol_set.setCheckable(True)
        self.hc_pol_set.clicked.connect(lambda: self.update_toggle_button(self.hc_pol_set, "CCW", "CW", "hc"))
        self.hc_pol_stat = QLabel("---")
        self.hc_pol_msr = QLabel("---")

        self.hc_dc_crt_set = QDoubleSpinBox()
        self.hc_dc_crt_set.setRange(0.0, 10.0)
        self.hc_dc_crt_set.setDecimals(2)
        self.hc_dc_crt_set.setSingleStep(0.1)
        self.hc_dc_crt_set.setSuffix(" A")
        self.hc_dc_crt_set.valueChanged.connect(lambda: self.hc_state_label.setText("Modified / not sent"))
        self.hc_dc_crt_stat = QLabel("---")
        self.hc_dc_crt_msr = QLabel("---")

        self.hc_temp_msr = QLabel("---")

        rows = [
            ("Power", self.hc_pw_set, self.hc_pw_stat, self.hc_pw_msr),
            ("Polarity", self.hc_pol_set, self.hc_pol_stat, self.hc_pol_msr),
            ("Current", self.hc_dc_crt_set, self.hc_dc_crt_stat, self.hc_dc_crt_msr),
            ("Temperature", QLabel("--"), QLabel("--"), self.hc_temp_msr),
        ]

        for row_index, (label, set_widget, stat_widget, msr_widget) in enumerate(rows, start=1):
            grid.addWidget(QLabel(label), row_index, 0)
            grid.addWidget(set_widget, row_index, 1)
            grid.addWidget(stat_widget, row_index, 2)
            grid.addWidget(msr_widget, row_index, 3)

        self.hc_state_label = QLabel("Not sent")
        self.hc_state_label.setObjectName("stateLabel")
        self.clear_hc_button = QPushButton("Clear")
        self.update_hc_button = QPushButton("Update HC Setpoints")
        self.update_hc_button.setObjectName("primaryButton")
        self.clear_hc_button.clicked.connect(self.clear_hc_setpoints)
        self.update_hc_button.clicked.connect(self.send_hc_setpoints)

        grid.addWidget(self.hc_state_label, 5, 0)
        grid.addWidget(self.clear_hc_button, 5, 1)
        grid.addWidget(self.update_hc_button, 5, 2, 1, 2)

        return group

    def build_connection_group(self):
        group = QGroupBox("Connection")
        grid = QGridLayout(group)
        grid.setContentsMargins(8, 10, 8, 8)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(5)

        self.port_combo = QComboBox()
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200", "230400"])
        self.baud_combo.setCurrentText("115200")

        self.refresh_button = QPushButton("Refresh")
        self.connect_button = QPushButton("Connect")
        self.connect_button.setObjectName("primaryButton")
        self.auto_measure_check = QCheckBox("Auto-read measurements 200 ms")
        self.auto_measure_check.setChecked(True)

        self.led = QLabel("●")
        self.led.setObjectName("ledOff")
        self.conn_text = QLabel("Disconnected")

        self.terminal_input = QLineEdit()
        self.terminal_input.setPlaceholderText("Terminal message to Arduino")
        self.terminal_send_button = QPushButton("Send")

        grid.addWidget(QLabel("Port"), 0, 0)
        grid.addWidget(self.port_combo, 0, 1)
        grid.addWidget(QLabel("Baudrate"), 0, 2)
        grid.addWidget(self.baud_combo, 0, 3)
        grid.addWidget(self.refresh_button, 1, 0)
        grid.addWidget(self.connect_button, 1, 1)
        grid.addWidget(self.auto_measure_check, 1, 2, 1, 2)
        grid.addWidget(self.led, 2, 0)
        grid.addWidget(self.conn_text, 2, 1, 1, 3)
        grid.addWidget(QLabel("Terminal"), 3, 0)
        grid.addWidget(self.terminal_input, 3, 1, 1, 2)
        grid.addWidget(self.terminal_send_button, 3, 3)

        self.refresh_button.clicked.connect(self.refresh_ports)
        self.connect_button.clicked.connect(self.toggle_connection)
        self.terminal_send_button.clicked.connect(self.send_terminal_message)
        self.terminal_input.returnPressed.connect(self.send_terminal_message)

        return group

    def build_log_group(self):
        group = QGroupBox("Log")
        layout = QVBoxLayout(group)
        layout.setContentsMargins(8, 10, 8, 8)
        layout.setSpacing(6)

        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMinimumHeight(340)

        log_buttons = QHBoxLayout()
        self.request_all_button = QPushButton("Request All")
        self.clear_log_button = QPushButton("Clear")
        self.save_log_button = QPushButton("Save")
        log_buttons.addWidget(self.request_all_button)
        log_buttons.addStretch()
        log_buttons.addWidget(self.clear_log_button)
        log_buttons.addWidget(self.save_log_button)

        self.request_all_button.clicked.connect(self.request_all)
        self.clear_log_button.clicked.connect(self.log_box.clear)
        self.save_log_button.clicked.connect(self.save_logs)

        layout.addWidget(self.log_box)
        layout.addLayout(log_buttons)
        return group

    # ------------------------------------------------------------------
    # Styling
    # ------------------------------------------------------------------
    def apply_style(self):
        self.setFont(QFont("Segoe UI", 9))
        self.setStyleSheet("""
            QMainWindow { background-color: #f4f6f8; }
            QLabel#mainTitle {
                font-size: 22px;
                font-weight: 800;
                color: #111827;
                padding: 4px 8px;
            }
            QGroupBox {
                font-weight: 700;
                border: 1px solid #cfd6df;
                border-radius: 8px;
                margin-top: 8px;
                padding: 8px;
                background-color: #ffffff;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 5px;
                color: #374151;
            }
            QPushButton {
                padding: 4px 8px;
                min-height: 20px;
                border-radius: 6px;
                border: 1px solid #b8c2cc;
                background-color: #f9fafb;
            }
            QPushButton:hover { background-color: #eef2f7; }
            QPushButton:checked {
                background-color: #86efac;
                border: 1px solid #16a34a;
                font-weight: 700;
            }
            QPushButton#primaryButton {
                background-color: #2563eb;
                color: white;
                font-weight: 800;
                border: 1px solid #1d4ed8;
            }
            QPushButton#primaryButton:hover { background-color: #1d4ed8; }
            QPushButton#dangerButton {
                background-color: #dc2626;
                color: white;
                font-weight: 800;
                border: 1px solid #991b1b;
            }
            QPushButton#dangerButton:hover { background-color: #b91c1c; }
            QComboBox, QSpinBox, QDoubleSpinBox, QLineEdit {
                padding: 2px 4px;
                min-height: 20px;
                border: 1px solid #b8c2cc;
                border-radius: 5px;
                background-color: white;
            }
            QTextEdit {
                border: 1px solid #cfd6df;
                border-radius: 7px;
                background-color: #111827;
                color: #d1d5db;
                font-family: Consolas, monospace;
                font-size: 9pt;
            }
            QLabel#ledOn {
                color: #16a34a;
                font-size: 22px;
                font-weight: bold;
            }
            QLabel#ledOff {
                color: #dc2626;
                font-size: 22px;
                font-weight: bold;
            }
            QLabel#readonlySetpoint {
                padding: 4px;
                background-color: #eef2ff;
                border: 1px solid #c7d2fe;
                border-radius: 5px;
                font-weight: 700;
            }
            QLabel#stateLabel {
                padding: 4px;
                color: #4b5563;
                font-weight: 700;
            }
        """)

    # ------------------------------------------------------------------
    # Local setpoint behavior
    # ------------------------------------------------------------------
    def update_toggle_button(self, button: QPushButton, on_text: str, off_text: str, section: str):
        button.setText(on_text if button.isChecked() else off_text)
        if section == "coil":
            self.coil_state_label.setText("Modified / not sent")
        else:
            self.hc_state_label.setText("Modified / not sent")

    def update_coil_mod_setpoint(self):
        dc_a = self.coil_dc_crt_set.value()
        ac_ma = self.coil_ac_crt_set.value()
        self.coil_mod_set.setText(f"{calc_mod_depth(ac_ma, dc_a):.2f} %")
        if hasattr(self, "coil_state_label"):
            self.coil_state_label.setText("Modified / not sent")

    def clear_coil_setpoints(self):
        self.coil_pw_set.setChecked(False)
        self.coil_pw_set.setText("OFF")
        self.coil_pol_set.setChecked(False)
        self.coil_pol_set.setText("CW")
        self.coil_dc_crt_set.setValue(0.0)
        self.coil_ac_crt_set.setValue(0)
        self.coil_mod_freq_set.setValue(0)
        self.coil_wf_set.setCurrentText("Sine")
        self.update_coil_mod_setpoint()
        self.coil_state_label.setText("Cleared locally / not sent")
        self.log("Coil setpoints cleared locally. Hardware unchanged until Update Coil Setpoints is pressed.")

    def clear_hc_setpoints(self):
        self.hc_pw_set.setChecked(False)
        self.hc_pw_set.setText("OFF")
        self.hc_pol_set.setChecked(False)
        self.hc_pol_set.setText("CW")
        self.hc_dc_crt_set.setValue(0.0)
        self.hc_state_label.setText("Cleared locally / not sent")
        self.log("HC setpoints cleared locally. Hardware unchanged until Update HC Setpoints is pressed.")

    # ------------------------------------------------------------------
    # Serial connection
    # ------------------------------------------------------------------
    def refresh_ports(self):
        current = self.port_combo.currentText() if hasattr(self, "port_combo") else ""
        self.port_combo.clear()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo.addItems(ports)
        if current in ports:
            self.port_combo.setCurrentText(current)
        elif ports:
            self.port_combo.setCurrentIndex(0)
        self.log(f"Ports refreshed: {', '.join(ports) if ports else 'none found'}")

    def toggle_connection(self):
        if self.ser and self.ser.is_open:
            self.disconnect_serial()
        else:
            self.connect_serial()

    def connect_serial(self):
        port = self.port_combo.currentText().strip()
        if not port:
            QMessageBox.warning(self, "No COM Port", "Please select a COM port first.")
            return

        baud = int(self.baud_combo.currentText())

        try:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.05)
            time.sleep(1.5)  # Arduino reset delay

            self.reader = SerialReader(self.ser)
            self.reader.packet_received.connect(self.handle_packet)
            self.reader.raw_received.connect(self.handle_raw_rx)
            self.reader.error.connect(self.handle_serial_error)
            self.reader.start()

            self.set_connected_state(True)
            self.log(f"Connected to {port} at {baud} baud.")

            self.request_all()
            if self.auto_measure_check.isChecked():
                self.measurement_timer.start()
                self.log("Auto measurement polling started: 200 ms.")

        except Exception as exc:
            self.ser = None
            QMessageBox.critical(self, "Connection Error", str(exc))
            self.log(f"Connection failed: {exc}")
            self.set_connected_state(False)

    def disconnect_serial(self):
        self.measurement_timer.stop()

        if self.reader:
            self.reader.stop()
            self.reader = None

        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

        self.ser = None
        self.set_connected_state(False)
        self.log("Disconnected.")

    def set_connected_state(self, connected: bool):
        self.connect_button.setText("Disconnect" if connected else "Connect")
        self.led.setObjectName("ledOn" if connected else "ledOff")
        self.led.setStyleSheet("")
        self.conn_text.setText("Connected" if connected else "Disconnected")
        self.conn_age_label.setText("Connected" if connected else "Disconnected")

        self.port_combo.setEnabled(not connected)
        self.baud_combo.setEnabled(not connected)
        self.refresh_button.setEnabled(not connected)
        self.update_coil_button.setEnabled(connected)
        self.update_hc_button.setEnabled(connected)
        self.request_all_button.setEnabled(connected)
        self.terminal_send_button.setEnabled(connected)
        self.disable_button.setEnabled(connected)

    # ------------------------------------------------------------------
    # Packet senders
    # ------------------------------------------------------------------
    def send_packet(self, cmd: int, payload: bytes = b""):
        if not self.ser or not self.ser.is_open:
            self.log("Cannot send: serial port is not connected.")
            return

        packet = build_pc_packet(cmd, payload)
        try:
            self.ser.write(packet)
            self.tx_count += 1
            self.tx_label.setText(f"TX: {self.tx_count}")
            self.log(f"TX: {packet_to_hex(packet)}")
        except Exception as exc:
            self.handle_serial_error(f"TX error: {exc}")

    def request_all(self):
        self.send_packet(CMD_REQUEST_ALL, b"")

    def request_measurement(self):
        self.send_packet(CMD_REQUEST_MEASUREMENT, b"")

    def send_coil_setpoints(self):
        flags = encode_flags(self.coil_pw_set.isChecked(), self.coil_pol_set.isChecked())
        dc_scaled = round(self.coil_dc_crt_set.value() * 100.0)
        ac_scaled = self.coil_ac_crt_set.value()
        freq = self.coil_mod_freq_set.value()
        waveform = WAVEFORM_TO_CODE[self.coil_wf_set.currentText()]

        payload = (
            bytes([flags])
            + u16_to_bytes(dc_scaled)
            + u16_to_bytes(ac_scaled)
            + u16_to_bytes(freq)
            + bytes([waveform])
        )

        self.coil_state_label.setText("Sent / waiting ACK")
        self.send_packet(CMD_SET_COIL, payload)

    def send_hc_setpoints(self):
        flags = encode_flags(self.hc_pw_set.isChecked(), self.hc_pol_set.isChecked())
        current_scaled = round(self.hc_dc_crt_set.value() * 100.0)
        payload = bytes([flags]) + u16_to_bytes(current_scaled)

        self.hc_state_label.setText("Sent / waiting ACK")
        self.send_packet(CMD_SET_HC, payload)

    def send_terminal_message(self):
        text = self.terminal_input.text().strip()
        if not text:
            return
        payload = text.encode("ascii", errors="replace")[:60]
        self.send_packet(CMD_TERMINAL_MESSAGE, payload)
        self.terminal_input.clear()

    def send_disable_request(self):
        reply = QMessageBox.question(
            self,
            "Disable Output Request",
            "Send Disable Output request to Arduino?\n\n"
            "For now this only sends a safety command/digital-output request. "
            "The shutdown procedure can be defined later in Arduino firmware.",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if reply == QMessageBox.Yes:
            self.send_packet(CMD_DISABLE_OUTPUT, bytes([1]))

    # ------------------------------------------------------------------
    # Packet handlers
    # ------------------------------------------------------------------
    def handle_raw_rx(self, data: bytes):
        pass

    def handle_packet(self, rsp: int, payload: bytes):
        self.rx_count += 1
        self.rx_label.setText(f"RX: {self.rx_count}")
        self.last_rx_time = time.time()

        if rsp == RSP_ALL_DATA:
            self.handle_all_data(payload)
        elif rsp == RSP_MEASUREMENT:
            self.handle_measurement(payload)
        elif rsp == RSP_COIL_ACK:
            self.coil_state_label.setText("ACK received / requesting status")
            self.log(f"Coil setpoints acknowledged. Echo: {packet_to_hex(payload)}")
            self.request_all()
        elif rsp == RSP_HC_ACK:
            self.hc_state_label.setText("ACK received / requesting status")
            self.log(f"HC setpoints acknowledged. Echo: {packet_to_hex(payload)}")
            self.request_all()
        elif rsp == RSP_TERMINAL:
            message = payload.decode("ascii", errors="replace")
            self.log(f"Arduino terminal: {message}")
        elif rsp == RSP_DISABLE_ACK:
            self.log("Disable Output request acknowledged by Arduino.")
            self.request_all()
        elif rsp == RSP_ERROR:
            code = payload[0] if payload else 0
            self.error_count += 1
            self.err_label.setText(f"Errors: {self.error_count}")
            self.log(f"Arduino error response: 0x{code:02X}")
        else:
            self.error_count += 1
            self.err_label.setText(f"Errors: {self.error_count}")
            self.log(f"Unknown response 0x{rsp:02X}, payload={packet_to_hex(payload)}")

    def handle_all_data(self, payload: bytes):
        # Payload length = 24 bytes:
        # Status block, 11 bytes:
        # [coil_flags_stat]
        # [coil_dc_stat_h][coil_dc_stat_l]
        # [coil_ac_stat_h][coil_ac_stat_l]
        # [coil_freq_stat_h][coil_freq_stat_l]
        # [coil_wf_stat]
        # [hc_flags_stat]
        # [hc_dc_stat_h][hc_dc_stat_l]
        # Measurement block, 13 bytes:
        # [coil_flags_msr]
        # [coil_dc_msr_h][coil_dc_msr_l]
        # [coil_ac_msr_h][coil_ac_msr_l]
        # [coil_temp_msr_h][coil_temp_msr_l]
        # [hc_flags_msr]
        # [hc_dc_msr_h][hc_dc_msr_l]
        # [hc_temp_msr_h][hc_temp_msr_l]
        # [safety_flags]
        if len(payload) != 24:
            self.log(f"Bad all-data length: {len(payload)}")
            return

        i = 0
        coil_flags_stat = payload[i]; i += 1
        coil_dc_stat_x100 = bytes_to_u16(payload[i], payload[i + 1]); i += 2
        coil_ac_stat_ma = bytes_to_u16(payload[i], payload[i + 1]); i += 2
        coil_freq_stat_hz = bytes_to_u16(payload[i], payload[i + 1]); i += 2
        coil_wf_stat_code = payload[i]; i += 1
        hc_flags_stat = payload[i]; i += 1
        hc_dc_stat_x100 = bytes_to_u16(payload[i], payload[i + 1]); i += 2

        status_payload_consumed = i
        measurement_payload = payload[status_payload_consumed:]

        self.update_status_fields(
            coil_flags_stat,
            coil_dc_stat_x100,
            coil_ac_stat_ma,
            coil_freq_stat_hz,
            coil_wf_stat_code,
            hc_flags_stat,
            hc_dc_stat_x100,
        )
        self.handle_measurement(measurement_payload, log_it=False)

        self.coil_state_label.setText("Status updated")
        self.hc_state_label.setText("Status updated")
        self.log("ALL DATA received: controller status + measurements updated.")

    def handle_measurement(self, payload: bytes, log_it: bool = True):
        # Payload length = 13 bytes:
        # [coil_flags_msr]
        # [coil_dc_msr_h][coil_dc_msr_l]
        # [coil_ac_msr_h][coil_ac_msr_l]
        # [coil_temp_msr_h][coil_temp_msr_l]
        # [hc_flags_msr]
        # [hc_dc_msr_h][hc_dc_msr_l]
        # [hc_temp_msr_h][hc_temp_msr_l]
        # [safety_flags]
        if len(payload) != 13:
            self.log(f"Bad measurement length: {len(payload)}")
            return

        i = 0
        coil_flags_msr = payload[i]; i += 1
        coil_dc_msr_x100 = bytes_to_u16(payload[i], payload[i + 1]); i += 2
        coil_ac_msr_ma = bytes_to_u16(payload[i], payload[i + 1]); i += 2
        coil_temp_msr_x10 = bytes_to_u16(payload[i], payload[i + 1]); i += 2
        hc_flags_msr = payload[i]; i += 1
        hc_dc_msr_x100 = bytes_to_u16(payload[i], payload[i + 1]); i += 2
        hc_temp_msr_x10 = bytes_to_u16(payload[i], payload[i + 1]); i += 2
        safety_flags = payload[i]

        coil_power_msr, coil_ccw_msr = decode_flags(coil_flags_msr)
        hc_power_msr, hc_ccw_msr = decode_flags(hc_flags_msr)

        coil_dc_msr_a = coil_dc_msr_x100 / 100.0
        coil_temp_msr_c = coil_temp_msr_x10 / 10.0
        hc_dc_msr_a = hc_dc_msr_x100 / 100.0
        hc_temp_msr_c = hc_temp_msr_x10 / 10.0
        coil_mod_msr = calc_mod_depth(coil_ac_msr_ma, coil_dc_msr_a)

        self.coil_pw_msr.setText(fmt_onoff(coil_power_msr))
        self.coil_pol_msr.setText(fmt_pol(coil_ccw_msr))
        self.coil_dc_crt_msr.setText(f"{coil_dc_msr_a:.2f} A")
        self.coil_ac_crt_msr.setText(f"{coil_ac_msr_ma:d} mA")
        self.coil_mod_msr.setText(f"{coil_mod_msr:.2f} %")
        self.coil_temp_msr.setText(f"{coil_temp_msr_c:.1f} °C")

        self.hc_pw_msr.setText(fmt_onoff(hc_power_msr))
        self.hc_pol_msr.setText(fmt_pol(hc_ccw_msr))
        self.hc_dc_crt_msr.setText(f"{hc_dc_msr_a:.2f} A")
        self.hc_temp_msr.setText(f"{hc_temp_msr_c:.1f} °C")

        if log_it:
            self.log(
                f"MEAS: coil={coil_dc_msr_a:.2f} A, AC={coil_ac_msr_ma} mA, "
                f"coilT={coil_temp_msr_c:.1f} C, HC={hc_dc_msr_a:.2f} A, "
                f"HCT={hc_temp_msr_c:.1f} C, safety=0x{safety_flags:02X}"
            )

    def update_status_fields(
        self,
        coil_flags_stat: int,
        coil_dc_stat_x100: int,
        coil_ac_stat_ma: int,
        coil_freq_stat_hz: int,
        coil_wf_stat_code: int,
        hc_flags_stat: int,
        hc_dc_stat_x100: int,
    ):
        coil_power_stat, coil_ccw_stat = decode_flags(coil_flags_stat)
        hc_power_stat, hc_ccw_stat = decode_flags(hc_flags_stat)

        coil_dc_stat_a = coil_dc_stat_x100 / 100.0
        hc_dc_stat_a = hc_dc_stat_x100 / 100.0
        coil_mod_stat = calc_mod_depth(coil_ac_stat_ma, coil_dc_stat_a)

        self.coil_pw_stat.setText(fmt_onoff(coil_power_stat))
        self.coil_pol_stat.setText(fmt_pol(coil_ccw_stat))
        self.coil_dc_crt_stat.setText(f"{coil_dc_stat_a:.2f} A")
        self.coil_ac_crt_stat.setText(f"{coil_ac_stat_ma:d} mA")
        self.coil_mod_stat.setText(f"{coil_mod_stat:.2f} %")
        self.coil_mod_freq_stat.setText(f"{coil_freq_stat_hz:d} Hz")
        self.coil_wf_stat.setText(CODE_TO_WAVEFORM.get(coil_wf_stat_code, f"Unknown({coil_wf_stat_code})"))

        self.hc_pw_stat.setText(fmt_onoff(hc_power_stat))
        self.hc_pol_stat.setText(fmt_pol(hc_ccw_stat))
        self.hc_dc_crt_stat.setText(f"{hc_dc_stat_a:.2f} A")

    def handle_serial_error(self, msg: str):
        self.error_count += 1
        self.err_label.setText(f"Errors: {self.error_count}")
        self.log(f"ERROR: {msg}")

    # ------------------------------------------------------------------
    # Other actions
    # ------------------------------------------------------------------
    def save_logs(self):
        default_name = f"elephant_v31_log_{time.strftime('%Y%m%d_%H%M%S')}.txt"
        path, _ = QFileDialog.getSaveFileName(
            self,
            "Save Log File",
            str(Path.home() / default_name),
            "Text Files (*.txt);;All Files (*)",
        )
        if not path:
            return
        try:
            Path(path).write_text(self.log_box.toPlainText(), encoding="utf-8")
            self.log(f"Logs saved to: {path}")
        except Exception as exc:
            QMessageBox.critical(self, "Save Error", str(exc))

    def show_help(self):
        QMessageBox.information(
            self,
            "Help",
            "Workflow:\n"
            "1. Connect to Arduino.\n"
            "2. GUI requests controller status and measurements once.\n"
            "3. GUI continuously requests measurements.\n"
            "4. Edit Setpoint values.\n"
            "5. Press Update Coil Setpoints or Update HC Setpoints.\n"
            "6. After ACK, GUI requests all status again.\n\n"
            "Clear only changes GUI setpoints; it does not change hardware.",
        )

    def show_about(self):
        QMessageBox.information(
            self,
            "About",
            "Elephant V3.1\n"
            "Arduino Mega 2560 serial control panel\n"
            "Data model: Setpoint / Controller Status / Measurement.",
        )

    def update_connection_status_age(self):
        if self.ser and self.ser.is_open:
            if self.last_rx_time:
                age = time.time() - self.last_rx_time
                msg = f"Connected | last RX {age:.1f} s ago"
            else:
                msg = "Connected | waiting for RX"
            self.conn_text.setText(msg)
            self.conn_age_label.setText(msg)

    def log(self, text: str):
        if not hasattr(self, "log_box"):
            return
        timestamp = time.strftime("%H:%M:%S")
        self.log_box.append(f"[{timestamp}] {text}")

    def closeEvent(self, event):
        self.disconnect_serial()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
