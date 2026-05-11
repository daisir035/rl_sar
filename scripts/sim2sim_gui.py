#!/usr/bin/env python3
"""rl_sar Sim2Sim Launcher GUI"""

import os
import sys
import signal
import subprocess

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QComboBox, QPushButton, QTextEdit, QLabel, QStatusBar,
    QCheckBox
)
from PyQt5.QtCore import Qt, QProcess
from PyQt5.QtGui import QFont, QTextCursor


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Robots that have MuJoCo mjcf/ descriptions
MUJOCO_ROBOTS = {"0315", "b2", "b2w", "d1", "g1", "go2", "go2w"}


class LogPanel(QTextEdit):
    """Read-only log panel with auto-scroll."""

    def __init__(self, max_height=None, parent=None):
        super().__init__(parent)
        self.setReadOnly(True)
        self.setFont(QFont("Monospace", 9))
        if max_height:
            self.setMaximumHeight(max_height)

    def append_log(self, text):
        self.moveCursor(QTextCursor.End)
        self.insertPlainText(text)
        self.moveCursor(QTextCursor.End)

    def clear_log(self):
        self.clear()


class Sim2SimGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("rl_sar Sim2Sim Launcher")
        self.setMinimumSize(800, 700)

        self.build_process = None
        self.sim_process = None
        self.bridge_process = None

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        # --- Build section ---
        build_group = QGroupBox("Build")
        build_layout = QHBoxLayout()

        build_layout.addWidget(QLabel("Mode:"))
        self.build_mode_combo = QComboBox()
        self.build_mode_combo.addItems(["-mj (MuJoCo)", "-m (CMake)", "Default (ROS)"])
        build_layout.addWidget(self.build_mode_combo)

        self.btn_build = QPushButton("Build")
        self.btn_build.setShortcut("Ctrl+B")
        self.btn_build.clicked.connect(self.start_build)
        build_layout.addWidget(self.btn_build)

        self.btn_clean = QPushButton("Clean")
        self.btn_clean.clicked.connect(self.start_clean)
        build_layout.addWidget(self.btn_clean)

        self.btn_open_dir = QPushButton("Open Project Dir")
        self.btn_open_dir.clicked.connect(lambda: self._open_in_file_manager(PROJECT_ROOT))
        build_layout.addWidget(self.btn_open_dir)

        build_group.setLayout(build_layout)
        main_layout.addWidget(build_group)

        # --- Build log ---
        self.build_log = LogPanel(max_height=120)
        main_layout.addWidget(self.build_log)

        # --- Simulation config section ---
        sim_group = QGroupBox("Simulation Config")
        sim_layout = QVBoxLayout()

        row1 = QHBoxLayout()
        row1.addWidget(QLabel("Robot:"))
        self.robot_combo = QComboBox()
        self.robot_combo.currentTextChanged.connect(self.on_robot_changed)
        row1.addWidget(self.robot_combo, 1)

        row1.addWidget(QLabel("Scene:"))
        self.scene_combo = QComboBox()
        row1.addWidget(self.scene_combo, 1)

        self.btn_refresh = QPushButton("Refresh")
        self.btn_refresh.setShortcut("Ctrl+R")
        self.btn_refresh.clicked.connect(self.populate_robots)
        row1.addWidget(self.btn_refresh)

        sim_layout.addLayout(row1)

        # Info row: show policy info and binary status
        info_row = QHBoxLayout()
        self.lbl_policy = QLabel("Policy: —")
        self.lbl_policy.setStyleSheet("color: #888; font-size: 11px;")
        info_row.addWidget(self.lbl_policy)

        self.lbl_binary = QLabel("Binary: checking...")
        self.lbl_binary.setStyleSheet("font-size: 11px;")
        info_row.addWidget(self.lbl_binary)
        sim_layout.addLayout(info_row)

        row2 = QHBoxLayout()
        self.btn_start_sim = QPushButton("Start Simulation")
        self.btn_start_sim.setShortcut("Ctrl+Return")
        self.btn_start_sim.setStyleSheet(
            "QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 8px; }"
            "QPushButton:disabled { background-color: #666; }"
        )
        self.btn_start_sim.clicked.connect(self.start_simulation)
        row2.addWidget(self.btn_start_sim)

        self.btn_stop_sim = QPushButton("Stop Simulation")
        self.btn_stop_sim.setShortcut("Ctrl+Escape")
        self.btn_stop_sim.setStyleSheet(
            "QPushButton { background-color: #f44336; color: white; font-weight: bold; padding: 8px; }"
            "QPushButton:disabled { background-color: #666; }"
        )
        self.btn_stop_sim.clicked.connect(self.stop_simulation)
        self.btn_stop_sim.setEnabled(False)
        row2.addWidget(self.btn_stop_sim)

        sim_layout.addLayout(row2)

        # Remote control row
        row3 = QHBoxLayout()
        self.chk_bridge = QCheckBox("Enable Remote Control (UDP Bridge)")
        row3.addWidget(self.chk_bridge)

        row3.addWidget(QLabel("Target:"))
        self.bridge_host_combo = QComboBox()
        self.bridge_host_combo.setEditable(True)
        self.bridge_host_combo.addItems(["127.0.0.1", "0.0.0.0"])
        self.bridge_host_combo.setMaximumWidth(160)
        row3.addWidget(self.bridge_host_combo)

        self.btn_open_remote = QPushButton("Open in Browser")
        self.btn_open_remote.setEnabled(False)
        self.btn_open_remote.clicked.connect(self.open_remote_browser)
        row3.addWidget(self.btn_open_remote)
        sim_layout.addLayout(row3)

        sim_group.setLayout(sim_layout)
        main_layout.addWidget(sim_group)

        # --- Simulation log ---
        sim_log_group = QGroupBox("Simulation Log")
        sim_log_layout = QVBoxLayout()
        self.sim_log = LogPanel()
        sim_log_layout.addWidget(self.sim_log)
        sim_log_group.setLayout(sim_log_layout)
        main_layout.addWidget(sim_log_group)

        # --- Status bar ---
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Ready")

        # --- Populate ---
        self.populate_robots()
        self._check_binary()

    # ---- Scanning ----

    def populate_robots(self):
        policy_dir = os.path.join(PROJECT_ROOT, "policy")
        if not os.path.isdir(policy_dir):
            return
        # Only show robots that have MuJoCo support
        robots = sorted([
            d for d in os.listdir(policy_dir)
            if os.path.isdir(os.path.join(policy_dir, d)) and d in MUJOCO_ROBOTS
        ])
        self.robot_combo.blockSignals(True)
        self.robot_combo.clear()
        self.robot_combo.addItems(robots)
        self.robot_combo.blockSignals(False)
        if robots:
            self.on_robot_changed(robots[0])

    def on_robot_changed(self, robot_name):
        if not robot_name:
            return
        self.populate_scenes(robot_name)
        self._update_policy_info(robot_name)

    def populate_scenes(self, robot_name):
        self.scene_combo.clear()
        mjcf_dir = os.path.join(
            PROJECT_ROOT, "src", "rl_sar_zoo", f"{robot_name}_description", "mjcf"
        )
        if not os.path.isdir(mjcf_dir):
            return
        scenes = []
        for xml_file in sorted(os.listdir(mjcf_dir)):
            if not xml_file.endswith(".xml"):
                continue
            name = xml_file[:-4]
            if name == robot_name:
                continue  # skip robot model XML, keep scene XMLs
            scenes.append(name)
        # Also check subdirectories (external_scenes, unitree_terrain, etc.)
        for subdir in sorted(os.listdir(mjcf_dir)):
            sub_path = os.path.join(mjcf_dir, subdir)
            if not os.path.isdir(sub_path):
                continue
            for xml_file in sorted(os.listdir(sub_path)):
                if xml_file.endswith(".xml"):
                    scenes.append(f"{subdir}/{xml_file[:-4]}")
        self.scene_combo.addItems(scenes)

    def _update_policy_info(self, robot_name):
        """Show available policies for the selected robot."""
        policy_dir = os.path.join(PROJECT_ROOT, "policy", robot_name)
        if not os.path.isdir(policy_dir):
            self.lbl_policy.setText("Policy: (none)")
            return
        policies = []
        for entry in sorted(os.listdir(policy_dir)):
            full = os.path.join(policy_dir, entry)
            if os.path.isdir(full) and os.path.isfile(os.path.join(full, "config.yaml")):
                policies.append(entry)
        if policies:
            self.lbl_policy.setText(f"Policy: {', '.join(policies)}")
        else:
            self.lbl_policy.setText("Policy: (none found)")

    def _check_binary(self):
        binary = self._binary_path()
        if os.path.isfile(binary):
            self.lbl_binary.setText(f"Binary: {os.path.basename(binary)} (found)")
            self.lbl_binary.setStyleSheet("color: #4CAF50; font-size: 11px;")
        else:
            self.lbl_binary.setText(f"Binary: {os.path.basename(binary)} (NOT found, build first)")
            self.lbl_binary.setStyleSheet("color: #f44336; font-size: 11px;")

    def _binary_path(self):
        return os.path.join(PROJECT_ROOT, "cmake_build", "bin", "rl_sim_mujoco")

    # ---- Build ----

    def _get_build_args(self):
        mode_text = self.build_mode_combo.currentText()
        if "-mj" in mode_text:
            return ["-mj"]
        elif "-m" in mode_text:
            return ["-m"]
        return []

    def start_build(self):
        if self.build_process and self.build_process.state() != QProcess.NotRunning:
            return
        self.build_log.clear_log()
        self.build_log.append_log(">>> Building...\n")
        self.status_bar.showMessage("Building...")
        self.btn_build.setEnabled(False)

        self.build_process = QProcess(self)
        self.build_process.setWorkingDirectory(PROJECT_ROOT)
        self.build_process.setProcessChannelMode(QProcess.MergedChannels)
        self.build_process.readyReadStandardOutput.connect(self._read_build_output)
        self.build_process.finished.connect(self._build_finished)

        args = self._get_build_args()
        self.build_process.start("bash", ["build.sh"] + args)

    def _read_build_output(self):
        if self.build_process:
            data = self.build_process.readAllStandardOutput().data().decode(errors="replace")
            self.build_log.append_log(data)

    def _build_finished(self, exit_code, exit_status):
        self.btn_build.setEnabled(True)
        self._check_binary()
        if exit_code == 0:
            self.build_log.append_log("\n>>> Build succeeded!\n")
            self.status_bar.showMessage("Build succeeded")
        else:
            self.build_log.append_log(f"\n>>> Build failed (exit code: {exit_code})\n")
            self.status_bar.showMessage("Build failed")

    def start_clean(self):
        if self.build_process and self.build_process.state() != QProcess.NotRunning:
            return
        self.build_log.clear_log()
        self.build_log.append_log(">>> Cleaning...\n")
        self.status_bar.showMessage("Cleaning...")
        self.btn_clean.setEnabled(False)

        self.build_process = QProcess(self)
        self.build_process.setWorkingDirectory(PROJECT_ROOT)
        self.build_process.setProcessChannelMode(QProcess.MergedChannels)
        self.build_process.readyReadStandardOutput.connect(self._read_build_output)
        self.build_process.finished.connect(self._clean_finished)
        self.build_process.start("bash", ["build.sh", "-c"])

    def _clean_finished(self, exit_code, exit_status):
        self.btn_clean.setEnabled(True)
        self._check_binary()
        self.build_log.append_log("\n>>> Clean done\n")
        self.status_bar.showMessage("Clean done")

    # ---- Simulation ----

    def start_simulation(self):
        if self.sim_process and self.sim_process.state() != QProcess.NotRunning:
            return

        robot = self.robot_combo.currentText()
        scene = self.scene_combo.currentText()
        if not robot or not scene:
            self.sim_log.append_log("Error: select a robot and scene first\n")
            return

        binary = self._binary_path()
        if not os.path.isfile(binary):
            self.sim_log.append_log(f"Error: binary not found: {binary}\n")
            self.sim_log.append_log("Please build first (Ctrl+B)\n")
            return

        self.sim_log.clear_log()
        self.sim_log.append_log(f">>> Starting simulation: {robot} / {scene}\n")
        self.status_bar.showMessage(f"Running: {robot} / {scene}")
        self.btn_start_sim.setEnabled(False)
        self.btn_stop_sim.setEnabled(True)

        self.sim_process = QProcess(self)
        self.sim_process.setWorkingDirectory(PROJECT_ROOT)
        self.sim_process.setProcessChannelMode(QProcess.MergedChannels)
        self.sim_process.readyReadStandardOutput.connect(self._read_sim_output)
        self.sim_process.finished.connect(self._sim_finished)
        self.sim_process.start(binary, [robot, scene])

        # Start bridge if checkbox is checked
        if self.chk_bridge.isChecked():
            self._start_bridge()

    def _read_sim_output(self):
        if self.sim_process:
            data = self.sim_process.readAllStandardOutput().data().decode(errors="replace")
            self.sim_log.append_log(data)

    def _sim_finished(self, exit_code, exit_status):
        self.btn_start_sim.setEnabled(True)
        self.btn_stop_sim.setEnabled(False)
        self.sim_log.append_log(f"\n>>> Simulation exited (code: {exit_code})\n")
        self.status_bar.showMessage("Simulation ended")
        self._stop_bridge()

    def stop_simulation(self):
        if self.sim_process and self.sim_process.state() != QProcess.NotRunning:
            self.sim_process.kill()
            self.sim_process.waitForFinished(3000)
            self.sim_log.append_log("\n>>> Simulation stopped\n")
            self.status_bar.showMessage("Simulation stopped")
        self._stop_bridge()

    # ---- UDP Bridge ----

    def _start_bridge(self):
        if self.bridge_process and self.bridge_process.state() != QProcess.NotRunning:
            return
        bridge_script = os.path.join(PROJECT_ROOT, "scripts", "udp_remote_bridge.py")
        if not os.path.isfile(bridge_script):
            self.sim_log.append_log("Warning: udp_remote_bridge.py not found\n")
            return

        host = self.bridge_host_combo.currentText().strip()
        self.bridge_process = QProcess(self)
        self.bridge_process.setWorkingDirectory(PROJECT_ROOT)
        self.bridge_process.setProcessChannelMode(QProcess.MergedChannels)
        self.bridge_process.readyReadStandardOutput.connect(self._read_bridge_output)
        self.bridge_process.finished.connect(self._bridge_finished)
        self.bridge_process.start(sys.executable, [bridge_script, host])

        self.btn_open_remote.setEnabled(True)
        self.sim_log.append_log(f">>> UDP bridge started (target: {host}:9876)\n")

    def _read_bridge_output(self):
        if self.bridge_process:
            data = self.bridge_process.readAllStandardOutput().data().decode(errors="replace")
            self.sim_log.append_log(f"[bridge] {data}")

    def _bridge_finished(self, exit_code, exit_status):
        self.btn_open_remote.setEnabled(False)
        self.sim_log.append_log(f">>> UDP bridge exited (code: {exit_code})\n")

    def _stop_bridge(self):
        if self.bridge_process and self.bridge_process.state() != QProcess.NotRunning:
            self.bridge_process.kill()
            self.bridge_process.waitForFinished(2000)
        self.btn_open_remote.setEnabled(False)

    def open_remote_browser(self):
        import webbrowser
        host = self.bridge_host_combo.currentText().strip()
        if host in ("0.0.0.0", "127.0.0.1"):
            host = "127.0.0.1"
        webbrowser.open(f"http://{host}:5000/")

    # ---- Utils ----

    def _open_in_file_manager(self, path):
        """Open a directory in the system file manager."""
        if os.path.isdir(path):
            subprocess.Popen(["xdg-open", path])

    def closeEvent(self, event):
        self.stop_simulation()
        if self.build_process and self.build_process.state() != QProcess.NotRunning:
            self.build_process.kill()
            self.build_process.waitForFinished(3000)
        event.accept()


def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = Sim2SimGUI()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
