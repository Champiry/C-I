# -*- coding: utf-8 -*-
"""
Created on Mon Jun  1 19:56:01 2026

@author: Afshin-III
"""
"""
Elephant V4.1 - Ethernet Control GUI
Author: Afshin + ChatGPT

Purpose:
    Ethernet GUI for Node0 Arduino Mega 2560 + W5100 and IT-M3233 power supply.

Main V4.1 rules:
    - GUI polling sends only MEAS? to Node0.
    - MEAS? must never update AD9833/MCP41010.
    - SET_COIL / SET_HC are sent only when user presses Update buttons.
    - Polling is paused during Update commands to avoid TCP command collision.
    - Default polling interval is 2000 ms to reduce network/SPI-related noise.
    - Power supply communication is separate via SCPI.

Required:
    pip install PyQt5

Node0 V4.1 protocol:
    PING
    MEAS?
    SET_COIL coil_pw_set=1 coil_pol_set=0 coil_dc_crt_set=2.500 coil_ac_crt_set=100 mod_freq_set=1000 wf_set=Sin
    SET_HC hc_pw_set=1 hc_pol_set=0 hc_dc_crt_set=1.500

Expected Node0 responses:
    PONG Node0 Elephant V4.1
    MEAS coil_pw_msr=... coil_pol_msr=... coil_dc_crt_msr=... coil_ac_crt_msr=... coil_temp_msr=... hc_pw_msr=... hc_pol_msr=... hc_dc_crt_msr=... hc_temp_msr=...
    OK SET_COIL
    OK SET_HC
    ERR ...
"""

import sys
import platform
import subprocess
import socket
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, Optional, Tuple

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QSpinBox,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


# ============================================================
# Constants
# ============================================================

APP_TITLE = "Elephant V4.1 - Ethernet Control GUI"

RES_COIL_1_OHM = 0.75
MOD_DEPTH_DC_THRESHOLD_A = 0.01

DEFAULT_NODE0_IP = "192.168.1.50"
DEFAULT_NODE0_GATEWAY = "192.168.1.1"
DEFAULT_NODE0_PORT = 5000

DEFAULT_PS_IP = "192.168.1.101"
DEFAULT_PS_GATEWAY = "192.168.1.1"
DEFAULT_PS_PORT = 30000

DEFAULT_MOD_FREQ_HZ = 1000
DEFAULT_POLL_INTERVAL_MS = 2000

NODE0_SOCKET_TIMEOUT_S = 1.2
PS_SOCKET_TIMEOUT_S = 2.0


# ============================================================
# Data Model
# ============================================================

@dataclass
class SystemState:
    # Connection
    ip_node0: str = DEFAULT_NODE0_IP
    gateway_node0: str = DEFAULT_NODE0_GATEWAY
    port_node0: int = DEFAULT_NODE0_PORT

    ip_ps: str = DEFAULT_PS_IP
    gateway_ps: str = DEFAULT_PS_GATEWAY
    port_ps: int = DEFAULT_PS_PORT

    node0_connected: bool = False
    ps_connected: bool = False

    # Coil #1 setpoints
    coil_pw_set: int = 0
    coil_pol_set: int = 0
    coil_dc_crt_set: float = 0.0
    coil_ac_crt_set: int = 0
    mod_freq_set: int = DEFAULT_MOD_FREQ_HZ
    wf_set: str = "Sin"

    # Coil #1 measured values
    coil_pw_msr: int = 0
    coil_pol_msr: int = 0
    coil_dc_crt_msr: float = 0.0
    coil_ac_crt_msr: int = 0
    coil_temp_msr: float = 0.0

    # Helmholtz setpoints
    hc_pw_set: int = 0
    hc_pol_set: int = 0
    hc_dc_crt_set: float = 0.0

    # Helmholtz measured values
    hc_pw_msr: int = 0
    hc_pol_msr: int = 0
    hc_dc_crt_msr: float = 0.0
    hc_temp_msr: float = 0.0

    # Power supply setpoints
    ps_pw_set: int = 0
    ps_vlt_set: float = 0.0
    ps_crt_set: float = 0.0

    # Power supply measured values
    ps_pw_msr: int = 0
    ps_vlt_msr: float = 0.0
    ps_crt_msr: float = 0.0


# ============================================================
# UI Helpers
# ============================================================

class StatusLamp(QLabel):
    def __init__(self, text: str = "OFF"):
        super().__init__(text)
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumWidth(115)
        self.set_state("off")

    def set_state(self, state: str):
        colors = {
            "off": "#777777",
            "on": "#1f9d55",
            "fault": "#cc3333",
            "warn": "#cc9900",
        }
        color = colors.get(state, "#777777")
        self.setStyleSheet(
            f"""
            QLabel {{
                color: white;
                background-color: {color};
                border-radius: 6px;
                padding: 4px 8px;
                font-weight: bold;
            }}
            """
        )


class AckLamp(QLabel):
    def __init__(self):
        super().__init__("●")
        self.setAlignment(Qt.AlignCenter)
        self.setFixedWidth(26)
        self.set_state("off")

    def set_state(self, state: str):
        colors = {
            "off": "#aaaaaa",
            "ok": "#1f9d55",
            "err": "#cc3333",
            "wait": "#cc9900",
        }
        self.setStyleSheet(
            f"color: {colors.get(state, '#aaaaaa')}; font-size: 20px; font-weight: bold;"
        )


class ReadOnlyLine(QLineEdit):
    def __init__(self, text: str = ""):
        super().__init__(text)
        self.setReadOnly(True)
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet("background-color: #dddddd;")


class ToggleBox(QComboBox):
    def __init__(self, labels=("0 / OFF", "1 / ON")):
        super().__init__()
        self.addItems(labels)
        self.setStyleSheet("background-color: #94d05f;")

    def value(self) -> int:
        return self.currentIndex()

    def set_value(self, value: int):
        self.setCurrentIndex(1 if int(value) else 0)


class YellowDoubleSpin(QDoubleSpinBox):
    def __init__(self, min_val: float, max_val: float, decimals: int = 3, step: float = 0.1):
        super().__init__()
        self.setRange(min_val, max_val)
        self.setDecimals(decimals)
        self.setSingleStep(step)
        self.setStyleSheet("background-color: #ffd966;")


class YellowSpin(QSpinBox):
    def __init__(self, min_val: int, max_val: int, step: int = 1):
        super().__init__()
        self.setRange(min_val, max_val)
        self.setSingleStep(step)
        self.setStyleSheet("background-color: #ffd966;")


class PurpleCombo(QComboBox):
    def __init__(self, items):
        super().__init__()
        self.addItems(items)
        self.setStyleSheet("background-color: #7f3fbf; color: white;")


# ============================================================
# Main Window
# ============================================================

class ElephantV41(QMainWindow):
    def __init__(self):
        super().__init__()
        self.state = SystemState()

        self.node0_sock: Optional[socket.socket] = None
        self.ps_sock: Optional[socket.socket] = None

        self.setWindowTitle(APP_TITLE)
        self.resize(1320, 780)

        self._build_ui()
        self._connect_signals()
        self._start_timer()
        self.refresh_all_fields()

    # ========================================================
    # UI construction
    # ========================================================
    def _build_ui(self):
        central = QWidget()
        main = QHBoxLayout(central)

        left = self._build_left_panel()
        right = self._build_right_panel()

        main.addWidget(left, 1)
        main.addWidget(right, 1)

        self.setCentralWidget(central)

    def _build_left_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)

        title = QLabel("Elephant V4.1")
        title.setStyleSheet("font-size: 18px; font-weight: bold;")
        layout.addWidget(title)

        layout.addWidget(self._build_coil_box())
        layout.addWidget(self._build_hc_box())
        layout.addWidget(self._build_ps_box())
        layout.addStretch()

        return panel

    def _build_right_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)

        top_buttons = QHBoxLayout()
        self.btn_help = QPushButton("Help")
        self.btn_about = QPushButton("About")
        top_buttons.addWidget(self.btn_help)
        top_buttons.addWidget(self.btn_about)
        layout.addLayout(top_buttons)

        layout.addWidget(self._build_connection_box())
        layout.addWidget(self._build_terminal_box())
        layout.addWidget(self._build_log_box(), 1)

        return panel

    @staticmethod
    def _add_table_header(grid: QGridLayout):
        grid.addWidget(QLabel("Parameter"), 0, 0)
        grid.addWidget(QLabel("Set Point"), 0, 1)
        grid.addWidget(QLabel("Measured Value"), 0, 2)

    def _build_coil_box(self) -> QGroupBox:
        box = QGroupBox("Coil #1")
        grid = QGridLayout(box)
        self._add_table_header(grid)

        self.coil_pw_set = ToggleBox()
        self.coil_pol_set = ToggleBox(("0 / CW", "1 / CCW"))
        self.coil_dc_crt_set = YellowDoubleSpin(0.0, 10.0, 3, 0.1)
        self.coil_ac_crt_set = YellowSpin(0, 1000, 10)
        self.coil_mod_set_cal = ReadOnlyLine("0.000")
        self.mod_freq_set = YellowSpin(0, 10000, 10)
        self.mod_freq_set.setValue(DEFAULT_MOD_FREQ_HZ)
        self.wf_set = PurpleCombo(["Sin", "Tri", "Sqr"])
        self.coil_temp_blank = ReadOnlyLine("-")
        self.coil_loss_set_cal = ReadOnlyLine("0.000")

        self.coil_pw_msr = ReadOnlyLine("0")
        self.coil_pol_msr = ReadOnlyLine("0")
        self.coil_dc_crt_msr = ReadOnlyLine("0.000")
        self.coil_ac_crt_msr = ReadOnlyLine("0")
        self.coil_mod_msr_cal = ReadOnlyLine("0.000")
        self.mod_freq_blank = ReadOnlyLine("-")
        self.wf_blank = ReadOnlyLine("-")
        self.coil_temp_msr = ReadOnlyLine("0.0")
        self.coil_loss_msr_cal = ReadOnlyLine("0.000")

        rows = [
            ("Power", self.coil_pw_set, self.coil_pw_msr),
            ("Polarity", self.coil_pol_set, self.coil_pol_msr),
            ("DC Current (A)", self.coil_dc_crt_set, self.coil_dc_crt_msr),
            ("AC Current (mA)", self.coil_ac_crt_set, self.coil_ac_crt_msr),
            ("Modulation Depth (%)", self.coil_mod_set_cal, self.coil_mod_msr_cal),
            ("Modulation Frequency (Hz)", self.mod_freq_set, self.mod_freq_blank),
            ("Waveform", self.wf_set, self.wf_blank),
            ("Temperature (°C)", self.coil_temp_blank, self.coil_temp_msr),
            ("Power Loss (W)", self.coil_loss_set_cal, self.coil_loss_msr_cal),
        ]

        for r, (name, set_widget, msr_widget) in enumerate(rows, start=1):
            grid.addWidget(QLabel(name), r, 0)
            grid.addWidget(set_widget, r, 1)
            grid.addWidget(msr_widget, r, 2)

        self.coil_ack = AckLamp()
        self.btn_clear_coil = QPushButton("Clear")
        self.btn_update_coil = QPushButton("Update Coil")
        self.btn_clear_coil.setStyleSheet("background-color: #94d05f;")
        self.btn_update_coil.setStyleSheet("background-color: #94d05f;")

        button_row = len(rows) + 1
        grid.addWidget(self.btn_clear_coil, button_row, 1)
        grid.addWidget(self.btn_update_coil, button_row, 2)
        grid.addWidget(self.coil_ack, button_row, 3)

        return box

    def _build_hc_box(self) -> QGroupBox:
        box = QGroupBox("Helmholtz Coil")
        grid = QGridLayout(box)
        self._add_table_header(grid)

        self.hc_pw_set = ToggleBox()
        self.hc_pol_set = ToggleBox(("0 / CW", "1 / CCW"))
        self.hc_dc_crt_set = YellowDoubleSpin(0.0, 10.0, 3, 0.1)
        self.hc_temp_blank = ReadOnlyLine("-")

        self.hc_pw_msr = ReadOnlyLine("0")
        self.hc_pol_msr = ReadOnlyLine("0")
        self.hc_dc_crt_msr = ReadOnlyLine("0.000")
        self.hc_temp_msr = ReadOnlyLine("0.0")

        rows = [
            ("Power", self.hc_pw_set, self.hc_pw_msr),
            ("Polarity", self.hc_pol_set, self.hc_pol_msr),
            ("Current (A)", self.hc_dc_crt_set, self.hc_dc_crt_msr),
            ("Temperature (°C)", self.hc_temp_blank, self.hc_temp_msr),
        ]

        for r, (name, set_widget, msr_widget) in enumerate(rows, start=1):
            grid.addWidget(QLabel(name), r, 0)
            grid.addWidget(set_widget, r, 1)
            grid.addWidget(msr_widget, r, 2)

        self.hc_ack = AckLamp()
        self.btn_clear_hc = QPushButton("Clear")
        self.btn_update_hc = QPushButton("Update HC")
        self.btn_clear_hc.setStyleSheet("background-color: #94d05f;")
        self.btn_update_hc.setStyleSheet("background-color: #94d05f;")

        button_row = len(rows) + 1
        grid.addWidget(self.btn_clear_hc, button_row, 1)
        grid.addWidget(self.btn_update_hc, button_row, 2)
        grid.addWidget(self.hc_ack, button_row, 3)

        return box

    def _build_ps_box(self) -> QGroupBox:
        box = QGroupBox("Power Supply #1")
        grid = QGridLayout(box)
        self._add_table_header(grid)

        self.ps_pw_set = ToggleBox()
        self.ps_vlt_set = YellowDoubleSpin(0.0, 60.0, 3, 0.1)
        self.ps_crt_set = YellowDoubleSpin(0.0, 10.0, 3, 0.1)

        self.ps_pw_msr = ReadOnlyLine("0")
        self.ps_vlt_msr = ReadOnlyLine("0.000")
        self.ps_crt_msr = ReadOnlyLine("0.000")

        rows = [
            ("Power", self.ps_pw_set, self.ps_pw_msr),
            ("V max (V)", self.ps_vlt_set, self.ps_vlt_msr),
            ("I max (A)", self.ps_crt_set, self.ps_crt_msr),
        ]

        for r, (name, set_widget, msr_widget) in enumerate(rows, start=1):
            grid.addWidget(QLabel(name), r, 0)
            grid.addWidget(set_widget, r, 1)
            grid.addWidget(msr_widget, r, 2)

        self.ps_ack = AckLamp()
        self.btn_clear_ps = QPushButton("Clear")
        self.btn_update_ps = QPushButton("Update PS")
        self.btn_read_ps = QPushButton("Read PS")
        self.btn_clear_ps.setStyleSheet("background-color: #94d05f;")
        self.btn_update_ps.setStyleSheet("background-color: #94d05f;")
        self.btn_read_ps.setStyleSheet("background-color: #94d05f;")

        button_row = len(rows) + 1
        grid.addWidget(self.btn_clear_ps, button_row, 1)
        grid.addWidget(self.btn_update_ps, button_row, 2)
        grid.addWidget(self.ps_ack, button_row, 3)
        grid.addWidget(self.btn_read_ps, button_row + 1, 2)

        return box

    def _build_connection_box(self) -> QGroupBox:
        box = QGroupBox("Connection")
        grid = QGridLayout(box)

        node_title = QLabel("Node0 / Coil #1 IP")
        node_title.setStyleSheet("background-color: #b4c6e7; font-weight: bold;")
        grid.addWidget(node_title, 0, 0, 1, 5)

        self.ip_node0 = QLineEdit(self.state.ip_node0)
        self.gateway_node0 = QLineEdit(self.state.gateway_node0)
        self.port_node0 = QSpinBox()
        self.port_node0.setRange(1, 65535)
        self.port_node0.setValue(self.state.port_node0)

        self.node0_lamp = StatusLamp("Ping Status")
        self.btn_ping_node0 = QPushButton("Ping")
        self.btn_connect_node0 = QPushButton("Connect")

        grid.addWidget(QLabel("IP"), 1, 0)
        grid.addWidget(self.ip_node0, 1, 1)
        grid.addWidget(self.node0_lamp, 1, 2)
        grid.addWidget(self.btn_ping_node0, 1, 3)
        grid.addWidget(self.btn_connect_node0, 1, 4)

        grid.addWidget(QLabel("Gateway"), 2, 0)
        grid.addWidget(self.gateway_node0, 2, 1)
        grid.addWidget(QLabel("Port"), 3, 0)
        grid.addWidget(self.port_node0, 3, 1)

        ps_title = QLabel("Power Supply #1")
        ps_title.setStyleSheet("background-color: #b4c6e7; font-weight: bold;")
        grid.addWidget(ps_title, 4, 0, 1, 5)

        self.ip_ps = QLineEdit(self.state.ip_ps)
        self.gateway_ps = QLineEdit(self.state.gateway_ps)
        self.port_ps = QSpinBox()
        self.port_ps.setRange(1, 65535)
        self.port_ps.setValue(self.state.port_ps)

        self.ps_lamp = StatusLamp("Ping Status")
        self.btn_ping_ps = QPushButton("Ping")
        self.btn_connect_ps = QPushButton("Connect")

        grid.addWidget(QLabel("IP"), 5, 0)
        grid.addWidget(self.ip_ps, 5, 1)
        grid.addWidget(self.ps_lamp, 5, 2)
        grid.addWidget(self.btn_ping_ps, 5, 3)
        grid.addWidget(self.btn_connect_ps, 5, 4)

        grid.addWidget(QLabel("Gateway"), 6, 0)
        grid.addWidget(self.gateway_ps, 6, 1)
        grid.addWidget(QLabel("Port"), 7, 0)
        grid.addWidget(self.port_ps, 7, 1)

        self.poll_interval = QSpinBox()
        self.poll_interval.setRange(200, 10000)
        self.poll_interval.setValue(DEFAULT_POLL_INTERVAL_MS)
        self.poll_interval.setSuffix(" ms")

        self.btn_disconnect_all = QPushButton("Disconnect All")

        grid.addWidget(QLabel("MEAS? Poll"), 8, 0)
        grid.addWidget(self.poll_interval, 8, 1)
        grid.addWidget(self.btn_disconnect_all, 8, 3, 1, 2)

        return box

    def _build_terminal_box(self) -> QGroupBox:
        box = QGroupBox("Terminal / Node0")
        layout = QHBoxLayout(box)

        self.terminal_input = QLineEdit()
        self.terminal_input.setPlaceholderText("Node0 command, e.g. PING / MEAS? / SET_COIL ...")
        self.btn_terminal_send = QPushButton("Send")
        self.btn_terminal_send.setStyleSheet("background-color: #94d05f;")

        layout.addWidget(self.terminal_input)
        layout.addWidget(self.btn_terminal_send)

        return box

    def _build_log_box(self) -> QGroupBox:
        box = QGroupBox("Log")
        layout = QVBoxLayout(box)

        self.log = QTextEdit()
        self.log.setReadOnly(True)

        buttons = QHBoxLayout()
        self.btn_clear_log = QPushButton("Clear")
        self.btn_save_log = QPushButton("Save")
        self.btn_clear_log.setStyleSheet("background-color: #94d05f;")
        self.btn_save_log.setStyleSheet("background-color: #94d05f;")

        buttons.addWidget(self.btn_clear_log)
        buttons.addWidget(self.btn_save_log)

        layout.addWidget(self.log)
        layout.addLayout(buttons)

        return box

    def _connect_signals(self):
        self.btn_ping_node0.clicked.connect(self.ping_node0)
        self.btn_connect_node0.clicked.connect(self.connect_node0)
        self.btn_ping_ps.clicked.connect(self.ping_ps)
        self.btn_connect_ps.clicked.connect(self.connect_ps)
        self.btn_disconnect_all.clicked.connect(self.disconnect_all)

        self.btn_terminal_send.clicked.connect(self.send_terminal_to_node0)
        self.btn_clear_log.clicked.connect(self.log.clear)
        self.btn_save_log.clicked.connect(self.save_log_placeholder)

        self.btn_update_coil.clicked.connect(self.update_coil)
        self.btn_clear_coil.clicked.connect(self.clear_coil_setpoints)
        self.btn_update_hc.clicked.connect(self.update_hc)
        self.btn_clear_hc.clicked.connect(self.clear_hc_setpoints)
        self.btn_update_ps.clicked.connect(self.update_ps)
        self.btn_read_ps.clicked.connect(self.read_ps)
        self.btn_clear_ps.clicked.connect(self.clear_ps_setpoints)

        self.coil_dc_crt_set.valueChanged.connect(self.recalculate_local_values)
        self.coil_ac_crt_set.valueChanged.connect(self.recalculate_local_values)
        self.ps_vlt_set.valueChanged.connect(self.recalculate_local_values)
        self.ps_crt_set.valueChanged.connect(self.recalculate_local_values)
        self.poll_interval.valueChanged.connect(self.update_poll_interval)

    def _start_timer(self):
        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self.poll_node0_meas)
        self.poll_timer.start(DEFAULT_POLL_INTERVAL_MS)

    # ========================================================
    # Logging / state
    # ========================================================
    def add_log(self, text: str):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log.append(f"[{timestamp}] {text}")

    def read_gui_to_state(self):
        self.state.ip_node0 = self.ip_node0.text().strip()
        self.state.gateway_node0 = self.gateway_node0.text().strip()
        self.state.port_node0 = int(self.port_node0.value())

        self.state.ip_ps = self.ip_ps.text().strip()
        self.state.gateway_ps = self.gateway_ps.text().strip()
        self.state.port_ps = int(self.port_ps.value())

        self.state.coil_pw_set = self.coil_pw_set.value()
        self.state.coil_pol_set = self.coil_pol_set.value()
        self.state.coil_dc_crt_set = float(self.coil_dc_crt_set.value())
        self.state.coil_ac_crt_set = int(self.coil_ac_crt_set.value())
        self.state.mod_freq_set = int(self.mod_freq_set.value())
        self.state.wf_set = self.wf_set.currentText()

        self.state.hc_pw_set = self.hc_pw_set.value()
        self.state.hc_pol_set = self.hc_pol_set.value()
        self.state.hc_dc_crt_set = float(self.hc_dc_crt_set.value())

        self.state.ps_pw_set = self.ps_pw_set.value()
        self.state.ps_vlt_set = float(self.ps_vlt_set.value())
        self.state.ps_crt_set = float(self.ps_crt_set.value())

    def refresh_all_fields(self):
        self.coil_pw_msr.setText(str(self.state.coil_pw_msr))
        self.coil_pol_msr.setText(str(self.state.coil_pol_msr))
        self.coil_dc_crt_msr.setText(f"{self.state.coil_dc_crt_msr:.3f}")
        self.coil_ac_crt_msr.setText(str(int(self.state.coil_ac_crt_msr)))
        self.coil_temp_msr.setText(f"{self.state.coil_temp_msr:.1f}")

        self.hc_pw_msr.setText(str(self.state.hc_pw_msr))
        self.hc_pol_msr.setText(str(self.state.hc_pol_msr))
        self.hc_dc_crt_msr.setText(f"{self.state.hc_dc_crt_msr:.3f}")
        self.hc_temp_msr.setText(f"{self.state.hc_temp_msr:.1f}")

        self.ps_pw_msr.setText(str(self.state.ps_pw_msr))
        self.ps_vlt_msr.setText(f"{self.state.ps_vlt_msr:.3f}")
        self.ps_crt_msr.setText(f"{self.state.ps_crt_msr:.3f}")

        self.recalculate_local_values()

    def update_poll_interval(self):
        self.poll_timer.setInterval(int(self.poll_interval.value()))
        self.add_log(f"Node0 MEAS? polling interval = {self.poll_interval.value()} ms")

    # ========================================================
    # Ping
    # ========================================================
    def run_ping(self, ip: str, label: str) -> bool:
        ip = ip.strip()
        if not ip:
            self.add_log(f"{label} ping failed: empty IP")
            return False

        if "windows" in platform.system().lower():
            cmd = ["ping", "-n", "1", "-w", "1000", ip]
        else:
            cmd = ["ping", "-c", "1", "-W", "1", ip]

        self.add_log("--------------------------------------------------")
        self.add_log(f"PING {label}: {ip}")

        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=3)
            output = ((result.stdout or "") + (result.stderr or "")).strip()
            self.add_log(f"Return code: {result.returncode}")
            if output:
                self.add_log(output)
            return result.returncode == 0
        except Exception as exc:
            self.add_log(f"PING {label}: ERROR: {exc}")
            return False

    def ping_node0(self):
        self.read_gui_to_state()
        ok = self.run_ping(self.state.ip_node0, "Node0")
        if ok:
            self.node0_lamp.set_state("on")
            self.node0_lamp.setText("Node0 Ping OK")
        else:
            self.node0_lamp.set_state("off")
            self.node0_lamp.setText("Ping Status")

    def ping_ps(self):
        self.read_gui_to_state()
        ok = self.run_ping(self.state.ip_ps, "IT-M3233")
        if ok:
            self.ps_lamp.set_state("on")
            self.ps_lamp.setText("PS Ping OK")
        else:
            self.ps_lamp.set_state("off")
            self.ps_lamp.setText("Ping Status")

    # ========================================================
    # Node0 TCP
    # ========================================================
    def node0_close_socket(self):
        if self.node0_sock is not None:
            try:
                self.node0_sock.close()
            except Exception:
                pass
        self.node0_sock = None
        self.state.node0_connected = False

    def node0_open_socket(self) -> bool:
        self.read_gui_to_state()
        self.node0_close_socket()

        try:
            self.add_log(f"Opening Node0 socket: {self.state.ip_node0}:{self.state.port_node0}")
            self.node0_sock = socket.create_connection(
                (self.state.ip_node0, self.state.port_node0),
                timeout=NODE0_SOCKET_TIMEOUT_S,
            )
            self.node0_sock.settimeout(NODE0_SOCKET_TIMEOUT_S)
            self.state.node0_connected = True
            self.node0_lamp.set_state("on")
            self.node0_lamp.setText("Node0 ON")
            self.add_log("Node0 socket opened.")
            return True
        except Exception as exc:
            self.node0_sock = None
            self.state.node0_connected = False
            self.node0_lamp.set_state("fault")
            self.node0_lamp.setText("Node0 Fault")
            self.add_log(f"Node0 socket failed: {exc}")
            return False

    def node0_transaction(self, command: str, expect_response: bool = True) -> Tuple[bool, str]:
        if self.node0_sock is None:
            if not self.node0_open_socket():
                return False, "SOCKET_OPEN_FAILED"

        tx = command.strip() + "\n"
        self.add_log(f"Node0 TX: {command}")

        try:
            self.node0_sock.sendall(tx.encode("utf-8"))

            if not expect_response:
                return True, ""

            data = self.node0_sock.recv(4096)
            response = data.decode("utf-8", errors="replace").strip()
            self.add_log(f"Node0 RX: {response if response else '[empty response]'}")
            return True, response

        except Exception as exc:
            self.add_log(f"Node0 TCP failed: {exc}")
            self.node0_close_socket()
            self.node0_lamp.set_state("fault")
            self.node0_lamp.setText("Node0 Fault")
            return False, str(exc)

    def connect_node0(self):
        if not self.node0_open_socket():
            return

        ok, response = self.node0_transaction("PING", expect_response=True)

        if ok and response.upper().startswith("PONG"):
            self.node0_lamp.set_state("on")
            self.node0_lamp.setText("Node0 ON")
            self.add_log("Node0 connection test SUCCESS.")
        else:
            self.node0_lamp.set_state("warn")
            self.node0_lamp.setText("No PONG")
            self.add_log("Node0 connected, but PING response was unexpected.")

    def poll_node0_meas(self):
        if not self.state.node0_connected or self.node0_sock is None:
            return

        ok, response = self.node0_transaction("MEAS?", expect_response=True)

        if ok and response.startswith("MEAS"):
            self.parse_node0_meas(response)
        elif ok and response:
            self.add_log(f"Unexpected Node0 polling response: {response}")

    def send_terminal_to_node0(self):
        command = self.terminal_input.text().strip()
        if not command:
            return

        was_running = self.poll_timer.isActive()
        self.poll_timer.stop()

        try:
            ok, response = self.node0_transaction(command, expect_response=True)
            if ok and response.startswith("MEAS"):
                self.parse_node0_meas(response)
        finally:
            if was_running:
                self.poll_timer.start(int(self.poll_interval.value()))

    # ========================================================
    # Power supply TCP / SCPI
    # ========================================================
    def ps_close_socket(self):
        if self.ps_sock is not None:
            try:
                self.ps_sock.close()
            except Exception:
                pass
        self.ps_sock = None
        self.state.ps_connected = False

    def ps_open_socket(self) -> bool:
        self.read_gui_to_state()
        self.ps_close_socket()

        try:
            self.add_log(f"Opening PS socket: {self.state.ip_ps}:{self.state.port_ps}")
            self.ps_sock = socket.create_connection(
                (self.state.ip_ps, self.state.port_ps),
                timeout=PS_SOCKET_TIMEOUT_S,
            )
            self.ps_sock.settimeout(PS_SOCKET_TIMEOUT_S)
            self.state.ps_connected = True
            self.ps_lamp.set_state("on")
            self.ps_lamp.setText("PS ON")
            self.add_log("PS socket opened.")
            return True
        except Exception as exc:
            self.ps_sock = None
            self.state.ps_connected = False
            self.ps_lamp.set_state("fault")
            self.ps_lamp.setText("PS Fault")
            self.add_log(f"PS socket failed: {exc}")
            return False

    def ps_transaction(self, command: str, expect_response: bool = True) -> Tuple[bool, str]:
        if self.ps_sock is None:
            if not self.ps_open_socket():
                return False, "SOCKET_OPEN_FAILED"

        tx = command.strip() + "\n"
        self.add_log(f"PS TX: {command}")

        try:
            self.ps_sock.sendall(tx.encode("ascii"))

            if not expect_response:
                self.add_log("PS RX: [no response expected]")
                return True, ""

            data = self.ps_sock.recv(4096)
            response = data.decode("ascii", errors="replace").strip()
            self.add_log(f"PS RX: {response if response else '[empty response]'}")
            return True, response

        except Exception as exc:
            self.add_log(f"PS TCP failed: {exc}")
            self.ps_close_socket()
            self.ps_lamp.set_state("fault")
            self.ps_lamp.setText("PS Fault")
            return False, str(exc)

    def connect_ps(self):
        if not self.ps_open_socket():
            return

        ok, response = self.ps_transaction("*IDN?", expect_response=True)

        if ok and response:
            self.ps_lamp.set_state("on")
            self.ps_lamp.setText("PS ON")
            self.add_log("IT-M3233 connection test SUCCESS.")
        else:
            self.ps_lamp.set_state("fault")
            self.ps_lamp.setText("PS Fault")
            self.add_log("IT-M3233 connection test FAILED.")

    # ========================================================
    # Update commands
    # ========================================================
    def update_coil(self):
        self.read_gui_to_state()
        self.coil_ack.set_state("wait")

        command = (
            "SET_COIL "
            f"coil_pw_set={self.state.coil_pw_set} "
            f"coil_pol_set={self.state.coil_pol_set} "
            f"coil_dc_crt_set={self.state.coil_dc_crt_set:.3f} "
            f"coil_ac_crt_set={self.state.coil_ac_crt_set} "
            f"mod_freq_set={self.state.mod_freq_set} "
            f"wf_set={self.state.wf_set}"
        )

        was_running = self.poll_timer.isActive()
        self.poll_timer.stop()

        try:
            ok, response = self.node0_transaction(command, expect_response=True)
            self.coil_ack.set_state("ok" if ok and response.startswith("OK") else "err")
        finally:
            if was_running:
                self.poll_timer.start(int(self.poll_interval.value()))

    def update_hc(self):
        self.read_gui_to_state()
        self.hc_ack.set_state("wait")

        command = (
            "SET_HC "
            f"hc_pw_set={self.state.hc_pw_set} "
            f"hc_pol_set={self.state.hc_pol_set} "
            f"hc_dc_crt_set={self.state.hc_dc_crt_set:.3f}"
        )

        was_running = self.poll_timer.isActive()
        self.poll_timer.stop()

        try:
            ok, response = self.node0_transaction(command, expect_response=True)
            self.hc_ack.set_state("ok" if ok and response.startswith("OK") else "err")
        finally:
            if was_running:
                self.poll_timer.start(int(self.poll_interval.value()))

    def update_ps(self):
        self.read_gui_to_state()
        self.ps_ack.set_state("wait")

        commands = [
            "SYST:REM",
            f"VOLT {self.state.ps_vlt_set:.3f}",
            f"CURR {self.state.ps_crt_set:.3f}",
            "OUTP ON" if self.state.ps_pw_set else "OUTP OFF",
        ]

        all_ok = True

        for cmd in commands:
            ok, _ = self.ps_transaction(cmd, expect_response=False)
            all_ok = all_ok and ok

        self.ps_ack.set_state("ok" if all_ok else "err")
        self.add_log("Update PS: OK." if all_ok else "Update PS: one or more commands failed.")

    def read_ps(self):
        ok_v, v_resp = self.ps_transaction("MEAS:VOLT?", expect_response=True)
        ok_i, i_resp = self.ps_transaction("MEAS:CURR?", expect_response=True)
        ok_o, o_resp = self.ps_transaction("OUTP?", expect_response=True)

        try:
            if ok_v and v_resp:
                self.state.ps_vlt_msr = float(v_resp)
        except ValueError:
            self.add_log(f"Could not parse PS voltage: {v_resp}")

        try:
            if ok_i and i_resp:
                self.state.ps_crt_msr = float(i_resp)
        except ValueError:
            self.add_log(f"Could not parse PS current: {i_resp}")

        if ok_o and o_resp:
            self.state.ps_pw_msr = 1 if o_resp.strip().upper() in ("1", "ON") else 0

        self.refresh_all_fields()

    # ========================================================
    # Parsing
    # ========================================================
    def parse_node0_meas(self, line: str):
        values = self._parse_key_value_line(line)

        # Accept both GUI names and hardware-map names
        aliases = {
            "coil_1_pw_msr": "coil_pw_msr",
            "coil_1_pol_msr": "coil_pol_msr",
            "coil_1_dc_crt_msr": "coil_dc_crt_msr",
            "coil_1_ac_crt_msr": "coil_ac_crt_msr",
            "coil_1_temp_msr": "coil_temp_msr",
            "hc_crt_msr": "hc_dc_crt_msr",
        }

        normalized = {}
        for key, value in values.items():
            normalized[aliases.get(key, key)] = value

        def get_int(name: str, old: int) -> int:
            try:
                return int(float(normalized.get(name, old)))
            except (TypeError, ValueError):
                return old

        def get_float(name: str, old: float) -> float:
            try:
                return float(normalized.get(name, old))
            except (TypeError, ValueError):
                return old

        self.state.coil_pw_msr = get_int("coil_pw_msr", self.state.coil_pw_msr)
        self.state.coil_pol_msr = get_int("coil_pol_msr", self.state.coil_pol_msr)
        self.state.coil_dc_crt_msr = get_float("coil_dc_crt_msr", self.state.coil_dc_crt_msr)
        self.state.coil_ac_crt_msr = get_int("coil_ac_crt_msr", self.state.coil_ac_crt_msr)
        self.state.coil_temp_msr = get_float("coil_temp_msr", self.state.coil_temp_msr)

        self.state.hc_pw_msr = get_int("hc_pw_msr", self.state.hc_pw_msr)
        self.state.hc_pol_msr = get_int("hc_pol_msr", self.state.hc_pol_msr)
        self.state.hc_dc_crt_msr = get_float("hc_dc_crt_msr", self.state.hc_dc_crt_msr)
        self.state.hc_temp_msr = get_float("hc_temp_msr", self.state.hc_temp_msr)

        self.refresh_all_fields()

    @staticmethod
    def _parse_key_value_line(line: str) -> Dict[str, str]:
        parts = line.strip().split()
        values = {}

        for part in parts[1:]:
            if "=" in part:
                key, value = part.split("=", 1)
                values[key.strip()] = value.strip()

        return values

    # ========================================================
    # Calculations / clear / shutdown
    # ========================================================
    def recalculate_local_values(self):
        # Modulation depth setpoint
        dc_set = float(self.coil_dc_crt_set.value())
        ac_set_a = float(self.coil_ac_crt_set.value()) / 1000.0

        if dc_set < MOD_DEPTH_DC_THRESHOLD_A:
            mod_set = 0.0
        else:
            mod_set = (ac_set_a / dc_set) * 100.0

        self.coil_mod_set_cal.setText(f"{mod_set:.3f}")

        # Modulation depth measured
        dc_msr = self.state.coil_dc_crt_msr
        ac_msr_a = self.state.coil_ac_crt_msr / 1000.0

        if dc_msr < MOD_DEPTH_DC_THRESHOLD_A:
            mod_msr = 0.0
        else:
            mod_msr = (ac_msr_a / dc_msr) * 100.0

        self.coil_mod_msr_cal.setText(f"{mod_msr:.3f}")

        # Power loss
        loss_set = self.calc_channel_loss(self.ps_vlt_set.value(), self.ps_crt_set.value())
        loss_msr = self.calc_channel_loss(self.state.ps_vlt_msr, self.state.ps_crt_msr)

        self.coil_loss_set_cal.setText(f"{loss_set:.3f}")
        self.coil_loss_msr_cal.setText(f"{loss_msr:.3f}")

    @staticmethod
    def calc_channel_loss(ps_voltage: float, ps_current: float) -> float:
        return max(ps_current * (ps_voltage - ps_current * RES_COIL_1_OHM), 0.0)

    def clear_coil_setpoints(self):
        self.coil_pw_set.set_value(0)
        self.coil_pol_set.set_value(0)
        self.coil_dc_crt_set.setValue(0.0)
        self.coil_ac_crt_set.setValue(0)
        self.mod_freq_set.setValue(DEFAULT_MOD_FREQ_HZ)
        self.wf_set.setCurrentText("Sin")
        self.coil_ack.set_state("off")
        self.recalculate_local_values()
        self.add_log("Coil setpoints cleared in GUI only. Hardware not changed.")

    def clear_hc_setpoints(self):
        self.hc_pw_set.set_value(0)
        self.hc_pol_set.set_value(0)
        self.hc_dc_crt_set.setValue(0.0)
        self.hc_ack.set_state("off")
        self.add_log("HC setpoints cleared in GUI only. Hardware not changed.")

    def clear_ps_setpoints(self):
        self.ps_pw_set.set_value(0)
        self.ps_vlt_set.setValue(0.0)
        self.ps_crt_set.setValue(0.0)
        self.ps_ack.set_state("off")
        self.recalculate_local_values()
        self.add_log("PS setpoints cleared in GUI only. Hardware not changed.")

    def save_log_placeholder(self):
        self.add_log("Save log placeholder. File saving can be added later.")

    def disconnect_all(self):
        self.node0_close_socket()
        self.ps_close_socket()

        self.node0_lamp.set_state("off")
        self.node0_lamp.setText("Ping Status")
        self.ps_lamp.set_state("off")
        self.ps_lamp.setText("Ping Status")

        self.add_log("Disconnected Node0 and PS.")

    def closeEvent(self, event):
        self.disconnect_all()
        event.accept()


# ============================================================
# Main
# ============================================================

def main():
    app = QApplication(sys.argv)
    win = ElephantV41()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
