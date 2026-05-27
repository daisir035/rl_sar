#!/usr/bin/env python3
"""
rl_sar Sim2Sim Launcher GUI Pro

Tab-based integrated workspace:
  - Build & Config: compile, robot/scene/policy selection, launch simulation
  - Remote Control: embedded web view for UDP bridge joystick
  - Log: build & simulation output

The MuJoCo viewer opens as a separate GLFW window and is auto-tiled
next to this GUI via X11 window management (python-xlib).
"""

import os
import sys
import signal
import subprocess
import time
import threading
import re

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QComboBox, QPushButton, QTextEdit, QLabel, QStatusBar,
    QCheckBox, QTabWidget, QSplitter, QSizePolicy, QFileDialog, QMessageBox,
    QSpinBox
)
from PyQt5.QtCore import Qt, QProcess, QTimer, QUrl
from PyQt5.QtGui import QFont, QTextCursor, QIcon, QColor, QTextCharFormat
from PyQt5.QtWebEngineWidgets import QWebEngineView, QWebEnginePage


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Robots that have MuJoCo mjcf/ descriptions
MUJOCO_ROBOTS = {"0315", "b2", "b2w", "d1", "g1", "go2", "go2w"}

FLASK_PORT = 5000


# ──────────────────────────────────────────────────────────────
#  Log panel
# ──────────────────────────────────────────────────────────────

class LogPanel(QTextEdit):
    """Read-only log with auto-scroll and ANSI color support."""

    # Standard ANSI 16-color palette
    _ANSI_COLORS = {
        30: QColor(0, 0, 0),       # black
        31: QColor(205, 49, 49),    # red
        32: QColor(13, 188, 121),   # green
        33: QColor(229, 229, 16),   # yellow
        34: QColor(36, 114, 200),   # blue
        35: QColor(188, 63, 188),   # magenta
        36: QColor(17, 168, 205),   # cyan
        37: QColor(229, 229, 229),  # white
        90: QColor(102, 102, 102),  # bright black (gray)
        91: QColor(241, 76, 76),    # bright red
        92: QColor(35, 209, 139),   # bright green
        93: QColor(245, 245, 67),   # bright yellow
        94: QColor(59, 142, 234),   # bright blue
        95: QColor(214, 112, 214),  # bright magenta
        96: QColor(41, 184, 219),   # bright cyan
        97: QColor(255, 255, 255),  # bright white
    }

    _ANSI_RE = re.compile(r'\x1b\[([0-9;]*)m')

    def __init__(self, max_height=None, parent=None):
        super().__init__(parent)
        self.setReadOnly(True)
        self.setFont(QFont("Monospace", 9))
        if max_height:
            self.setMaximumHeight(max_height)
        self._current_fmt = QTextCharFormat()

    def append_log(self, text):
        """Append text, interpreting ANSI color escape codes."""
        self.moveCursor(QTextCursor.End)
        pos = 0
        for m in self._ANSI_RE.finditer(text):
            # Insert plain text before this escape
            if m.start() > pos:
                self._insert_colored(text[pos:m.start()])
            # Parse SGR parameters
            codes = m.group(1)
            if codes == '' or codes == '0':
                self._current_fmt = QTextCharFormat()
            else:
                for code_str in codes.split(';'):
                    try:
                        code = int(code_str)
                    except ValueError:
                        continue
                    if code == 0:
                        self._current_fmt = QTextCharFormat()
                    elif code == 1:
                        self._current_fmt.setFontWeight(QFont.Bold)
                    elif code in self._ANSI_COLORS:
                        self._current_fmt.setForeground(self._ANSI_COLORS[code])
            pos = m.end()
        # Remaining text after last escape
        if pos < len(text):
            self._insert_colored(text[pos:])
        self.moveCursor(QTextCursor.End)

    def _insert_colored(self, text):
        if self._current_fmt.isValid() and self._current_fmt != QTextCharFormat():
            cursor = self.textCursor()
            cursor.setCharFormat(self._current_fmt)
            cursor.insertText(text)
        else:
            self.insertPlainText(text)

    def append_log_colored(self, text, color):
        """Append text with an explicit QColor."""
        fmt = QTextCharFormat()
        fmt.setForeground(color)
        self.moveCursor(QTextCursor.End)
        cursor = self.textCursor()
        cursor.setCharFormat(fmt)
        cursor.insertText(text)
        self.moveCursor(QTextCursor.End)

    def clear_log(self):
        self.clear()
        self._current_fmt = QTextCharFormat()


# ──────────────────────────────────────────────────────────────
#  X11 window tiling helper
# ──────────────────────────────────────────────────────────────

class WindowTiler:
    """Use python-xlib to find and move the MuJoCo GLFW window."""

    @staticmethod
    def find_window_by_title(keyword, timeout=10):
        """Poll for a window whose title contains *keyword*."""
        try:
            from Xlib import display as xdisplay
        except ImportError:
            return None

        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                d = xdisplay.Display()
                root = d.screen().root
                tree = root.query_tree()
                for w in tree.children:
                    try:
                        name = w.get_wm_name() or ""
                        if keyword.lower() in name.lower():
                            return w
                    except Exception:
                        continue
                d.close()
            except Exception:
                pass
            time.sleep(0.5)
        return None

    @staticmethod
    def tile_gui_and_mujoco(gui_window_id, mujoco_w, width_ratio=0.45):
        """Place MuJoCo window to the right of the GUI."""
        try:
            from Xlib import display as xdisplay
            from Xlib import X as Xlib_X

            d = xdisplay.Display()
            root = d.screen().root
            geom = root.get_geometry()

            screen_w = geom.width
            screen_h = geom.height

            gui_w = int(screen_w * width_ratio)
            mujoco_x = gui_w
            mujoco_w_val = screen_w - gui_w

            # Move MuJoCo window
            mujoco_w.configure(
                x=mujoco_x, y=0,
                width=mujoco_w_val, height=screen_h - 40
            )
            d.sync()
            d.close()
            return True
        except Exception as e:
            print(f"[WindowTiler] Failed: {e}")
            return False


# ──────────────────────────────────────────────────────────────
#  Main window
# ──────────────────────────────────────────────────────────────

class Sim2SimGUIPro(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("rl_sar Sim2Sim Pro")
        self.setMinimumSize(900, 750)

        self.build_process = None
        self.sim_process = None
        self.bridge_process = None
        self._mujoco_window = None

        # ── Central widget with tabs ──
        central = QWidget()
        self.setCentralWidget(central)
        root_layout = QVBoxLayout(central)
        root_layout.setContentsMargins(4, 4, 4, 4)

        self.tabs = QTabWidget()
        root_layout.addWidget(self.tabs)

        self._build_tab_config()
        self._build_tab_remote()
        self._build_tab_log()

        # ── Status bar ──
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Ready")

        # ── Initial populate ──
        self.populate_robots()
        self._check_binary()

    # ──────────────────────────────────────────────────────────
    #  Tab 1: Build & Config
    # ──────────────────────────────────────────────────────────

    def _build_tab_config(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        # --- Build section ---
        build_group = QGroupBox("Build")
        bl = QHBoxLayout()
        bl.addWidget(QLabel("Mode:"))
        self.build_mode_combo = QComboBox()
        self.build_mode_combo.addItems(["-mj (MuJoCo)", "-m (CMake)", "Default (ROS)"])
        bl.addWidget(self.build_mode_combo)

        self.btn_build = QPushButton("Build")
        self.btn_build.setShortcut("Ctrl+B")
        self.btn_build.clicked.connect(self.start_build)
        bl.addWidget(self.btn_build)

        self.btn_clean = QPushButton("Clean")
        self.btn_clean.clicked.connect(self.start_clean)
        bl.addWidget(self.btn_clean)

        self.btn_open_dir = QPushButton("Open Project Dir")
        self.btn_open_dir.clicked.connect(lambda: self._xdg_open(PROJECT_ROOT))
        bl.addWidget(self.btn_open_dir)
        build_group.setLayout(bl)
        layout.addWidget(build_group)

        # --- Build log (compact) ---
        self.build_log = LogPanel(max_height=100)
        layout.addWidget(self.build_log)

        # --- Terrain generation ---
        terrain_group = QGroupBox("Terrain Generation")
        tl = QHBoxLayout()
        tl.addWidget(QLabel("Preset:"))
        self.terrain_preset_combo = QComboBox()
        self.terrain_preset_combo.addItems([
            "flat", "stairs", "suspend_stairs", "slope", "rough_ground",
            "obstacles", "perlin_hfield", "image_hfield", "mixed", "extreme"
        ])
        self.terrain_preset_combo.setToolTip("Select terrain type to generate")
        tl.addWidget(self.terrain_preset_combo, 1)

        tl.addWidget(QLabel("Seed:"))
        self.terrain_seed_spin = QSpinBox()
        self.terrain_seed_spin.setRange(0, 999999)
        self.terrain_seed_spin.setValue(42)
        self.terrain_seed_spin.setSpecialValueText("Random")
        tl.addWidget(self.terrain_seed_spin)

        self.btn_generate_terrain = QPushButton("Generate")
        self.btn_generate_terrain.setToolTip("Generate terrain scene XML (Ctrl+G)")
        self.btn_generate_terrain.setShortcut("Ctrl+G")
        self.btn_generate_terrain.clicked.connect(self._generate_terrain)
        tl.addWidget(self.btn_generate_terrain)

        self.lbl_terrain_status = QLabel("")
        self.lbl_terrain_status.setStyleSheet("color: #4CAF50; font-size: 11px;")
        tl.addWidget(self.lbl_terrain_status)
        terrain_group.setLayout(tl)
        layout.addWidget(terrain_group)

        # --- Simulation config ---
        sim_group = QGroupBox("Simulation")
        sl = QVBoxLayout()

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
        sl.addLayout(row1)

        # Policy & Model row
        row_pol = QHBoxLayout()
        row_pol.addWidget(QLabel("Policy:"))
        self.policy_combo = QComboBox()
        self.policy_combo.currentTextChanged.connect(self.on_policy_changed)
        row_pol.addWidget(self.policy_combo, 1)

        row_pol.addWidget(QLabel("Model:"))
        self.model_combo = QComboBox()
        self.model_combo.currentTextChanged.connect(self._on_model_changed)
        row_pol.addWidget(self.model_combo, 1)

        self.btn_browse = QPushButton("Browse...")
        self.btn_browse.setToolTip("Select model file from any location")
        self.btn_browse.clicked.connect(self._browse_model)
        row_pol.addWidget(self.btn_browse)

        self.btn_convert = QPushButton("Convert")
        self.btn_convert.setToolTip("Convert training checkpoint to TorchScript")
        self.btn_convert.clicked.connect(self._convert_current_checkpoint)
        self.btn_convert.setEnabled(False)
        row_pol.addWidget(self.btn_convert)
        sl.addLayout(row_pol)

        # Info
        info_row = QHBoxLayout()
        self.lbl_model_status = QLabel("Model: —")
        self.lbl_model_status.setStyleSheet("color: #888; font-size: 11px;")
        info_row.addWidget(self.lbl_model_status)

        self.lbl_binary = QLabel("Binary: checking...")
        self.lbl_binary.setStyleSheet("font-size: 11px;")
        info_row.addWidget(self.lbl_binary)
        sl.addLayout(info_row)

        # Remote control toggle
        ctrl_row = QHBoxLayout()
        self.chk_bridge = QCheckBox("Enable Remote Control (auto-start bridge)")
        self.chk_bridge.setChecked(True)
        ctrl_row.addWidget(self.chk_bridge)

        ctrl_row.addWidget(QLabel("Target:"))
        self.bridge_host_combo = QComboBox()
        self.bridge_host_combo.setEditable(True)
        self.bridge_host_combo.addItems(["127.0.0.1", "0.0.0.0"])
        self.bridge_host_combo.setMaximumWidth(160)
        ctrl_row.addWidget(self.bridge_host_combo)
        sl.addLayout(ctrl_row)

        # Start / Stop
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
        sl.addLayout(row2)

        sim_group.setLayout(sl)
        layout.addWidget(sim_group)

        self.tabs.addTab(tab, "Build && Config")

    # ──────────────────────────────────────────────────────────
    #  Tab 2: Remote Control (embedded web view)
    # ──────────────────────────────────────────────────────────

    def _build_tab_remote(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(0, 0, 0, 0)

        # Toolbar
        toolbar = QHBoxLayout()
        self.btn_reload_remote = QPushButton("Reload")
        self.btn_reload_remote.clicked.connect(self._reload_remote_view)
        toolbar.addWidget(self.btn_reload_remote)

        self.lbl_remote_url = QLabel(f"http://127.0.0.1:{FLASK_PORT}/")
        self.lbl_remote_url.setStyleSheet("color: #888; font-size: 11px;")
        toolbar.addWidget(self.lbl_remote_url)
        toolbar.addStretch()

        self.lbl_remote_status = QLabel("Bridge: not running")
        self.lbl_remote_status.setStyleSheet("color: #e94560; font-size: 11px;")
        toolbar.addWidget(self.lbl_remote_status)

        layout.addLayout(toolbar)

        # WebEngine view
        self.web_view = QWebEngineView()
        # Suppress noisy JS console messages
        page = QWebEnginePage(self.web_view)
        self.web_view.setPage(page)
        self.web_view.setUrl(QUrl("about:blank"))
        layout.addWidget(self.web_view, 1)

        self.tabs.addTab(tab, "Remote Control")

    # ──────────────────────────────────────────────────────────
    #  Tab 3: Log
    # ──────────────────────────────────────────────────────────

    def _build_tab_log(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(0, 0, 0, 0)

        toolbar = QHBoxLayout()
        btn_clear = QPushButton("Clear Log")
        btn_clear.clicked.connect(lambda: self.sim_log.clear_log())
        toolbar.addWidget(btn_clear)
        toolbar.addStretch()
        layout.addLayout(toolbar)

        self.sim_log = LogPanel()
        layout.addWidget(self.sim_log)

        self.tabs.addTab(tab, "Log")

    # ──────────────────────────────────────────────────────────
    #  Robot / scene scanning
    # ──────────────────────────────────────────────────────────

    def populate_robots(self):
        policy_dir = os.path.join(PROJECT_ROOT, "policy")
        if not os.path.isdir(policy_dir):
            return
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
        self.populate_policies(robot_name)

    def populate_policies(self, robot_name):
        self.policy_combo.blockSignals(True)
        self.policy_combo.clear()
        policy_dir = os.path.join(PROJECT_ROOT, "policy", robot_name)
        if os.path.isdir(policy_dir):
            policies = sorted([
                d for d in os.listdir(policy_dir)
                if os.path.isdir(os.path.join(policy_dir, d))
                and os.path.isfile(os.path.join(policy_dir, d, "config.yaml"))
            ])
            self.policy_combo.addItems(policies)
        self.policy_combo.blockSignals(False)
        if self.policy_combo.count() > 0:
            self.on_policy_changed(self.policy_combo.currentText())

    def on_policy_changed(self, policy_name):
        robot_name = self.robot_combo.currentText()
        if not robot_name or not policy_name:
            self.model_combo.clear()
            return
        self.populate_models(robot_name, policy_name)

    def populate_models(self, robot_name, policy_name):
        self.model_combo.blockSignals(True)
        self.model_combo.clear()
        policy_dir = os.path.join(PROJECT_ROOT, "policy", robot_name, policy_name)
        if os.path.isdir(policy_dir):
            # Read current model_name from config.yaml
            current_model = self._read_config_model_name(policy_dir)
            # Scan for .pt and .onnx files
            all_files = sorted([
                f for f in os.listdir(policy_dir)
                if (f.endswith(".pt") or f.endswith(".onnx"))
                and os.path.isfile(os.path.join(policy_dir, f))
            ])
            self.model_combo.addItems(all_files)
            # Select the one currently in config.yaml
            if current_model and current_model in all_files:
                idx = all_files.index(current_model)
                self.model_combo.setCurrentIndex(idx)
            elif all_files:
                self.model_combo.setCurrentIndex(0)
        self.model_combo.blockSignals(False)
        self._update_model_status()

    def _read_config_model_name(self, policy_dir):
        config_path = os.path.join(policy_dir, "config.yaml")
        if not os.path.isfile(config_path):
            return None
        try:
            with open(config_path, "r") as f:
                content = f.read()
            m = re.search(r'model_name\s*:\s*["\']?([^"\'\n]+)', content)
            if m:
                return m.group(1).strip()
        except Exception:
            pass
        return None

    def _on_model_changed(self, model_name):
        self._update_model_status()

    def _is_torchscript(self, filepath):
        """Check if a .pt file is a valid TorchScript model."""
        try:
            import zipfile
            with zipfile.ZipFile(filepath) as z:
                names = z.namelist()
                return any("code/" in n for n in names) or any("constants.pkl" in n for n in names)
        except Exception:
            return False

    def _update_model_status(self):
        model = self.model_combo.currentText()
        if not model:
            self.lbl_model_status.setText("Model: (none)")
            self.lbl_model_status.setStyleSheet("color: #888; font-size: 11px;")
            self.btn_convert.setEnabled(False)
            return

        robot = self.robot_combo.currentText()
        policy = self.policy_combo.currentText()
        if not robot or not policy:
            self.lbl_model_status.setText(f"Model: {model}")
            self.lbl_model_status.setStyleSheet("color: #888; font-size: 11px;")
            self.btn_convert.setEnabled(False)
            return

        filepath = os.path.join(PROJECT_ROOT, "policy", robot, policy, model)
        if model.endswith(".onnx"):
            self.lbl_model_status.setText(f"Model: {model} (ONNX, convert to TorchScript)")
            self.lbl_model_status.setStyleSheet("color: #2196F3; font-size: 11px;")
            self.btn_convert.setEnabled(True)
            self.btn_convert.setText("ONNX→PT")
        elif self._is_torchscript(filepath):
            self.lbl_model_status.setText(f"Model: {model} (TorchScript OK)")
            self.lbl_model_status.setStyleSheet("color: #4CAF50; font-size: 11px;")
            self.btn_convert.setEnabled(False)
            self.btn_convert.setText("Convert")
        else:
            self.lbl_model_status.setText(f"Model: {model} (checkpoint, needs convert)")
            self.lbl_model_status.setStyleSheet("color: #ffa500; font-size: 11px;")
            self.btn_convert.setEnabled(True)
            self.btn_convert.setText("Convert")

    def _browse_model(self):
        """Open file dialog to select a .pt or .onnx from any location."""
        robot = self.robot_combo.currentText()
        policy = self.policy_combo.currentText()
        if not robot or not policy:
            self.sim_log.append_log_colored(
                "Error: select a robot and policy first\n", QColor(241, 76, 76)
            )
            return

        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select Model File",
            "",
            "Model Files (*.pt *.onnx);;PyTorch (*.pt);;ONNX (*.onnx);;All Files (*)",
        )
        if not file_path:
            return

        policy_dir = os.path.join(PROJECT_ROOT, "policy", robot, policy)
        os.makedirs(policy_dir, exist_ok=True)

        filename = os.path.basename(file_path)
        dst = os.path.join(policy_dir, filename)

        if os.path.abspath(file_path) == os.path.abspath(dst):
            self.populate_models(robot, policy)
            idx = self.model_combo.findText(filename)
            if idx >= 0:
                self.model_combo.setCurrentIndex(idx)
            return

        if os.path.exists(dst):
            reply = QMessageBox.question(
                self,
                "File Exists",
                f'"{filename}" already exists in the policy directory. Overwrite?',
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if reply != QMessageBox.Yes:
                return

        try:
            import shutil
            shutil.copy2(file_path, dst)
            self.sim_log.append_log_colored(
                f">>> Copied model to policy dir: {filename}\n", QColor(35, 209, 139)
            )
        except Exception as e:
            self.sim_log.append_log_colored(
                f">>> Failed to copy model: {e}\n", QColor(241, 76, 76)
            )
            return

        self.populate_models(robot, policy)
        idx = self.model_combo.findText(filename)
        if idx >= 0:
            self.model_combo.setCurrentIndex(idx)

    def _convert_current_checkpoint(self):
        """Convert selected model to TorchScript (checkpoint or ONNX)."""
        robot = self.robot_combo.currentText()
        policy = self.policy_combo.currentText()
        model = self.model_combo.currentText()
        if not robot or not policy or not model:
            return

        src = os.path.join(PROJECT_ROOT, "policy", robot, policy, model)
        if not os.path.isfile(src):
            return

        if model.endswith(".onnx"):
            self._convert_onnx(src, robot, policy, model)
        else:
            self._convert_pt_checkpoint(src, robot, policy, model)

    def _convert_pt_checkpoint(self, src, robot, policy, model):
        """Convert training checkpoint .pt to TorchScript."""
        base, ext = os.path.splitext(model)
        out_name = f"{base}_torchscript{ext}"
        dst = os.path.join(PROJECT_ROOT, "policy", robot, policy, out_name)

        self.sim_log.append_log_colored(f">>> Converting checkpoint {model} -> {out_name}...\n", QColor(17, 168, 205))
        try:
            import importlib.util
            spec = importlib.util.spec_from_file_location(
                "convert_checkpoint",
                os.path.join(PROJECT_ROOT, "scripts", "convert_checkpoint.py")
            )
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            success = mod.convert(src, dst)
            if success:
                self.sim_log.append_log_colored(f">>> Done: {out_name}\n", QColor(35, 209, 139))
                self.populate_models(robot, policy)
                idx = self.model_combo.findText(out_name)
                if idx >= 0:
                    self.model_combo.setCurrentIndex(idx)
            else:
                self.sim_log.append_log_colored(">>> Conversion failed\n", QColor(241, 76, 76))
        except Exception as e:
            self.sim_log.append_log_colored(f">>> Conversion error: {e}\n", QColor(241, 76, 76))

    def _convert_onnx(self, src, robot, policy, model):
        """Convert ONNX model to TorchScript using project's convert_policy.py."""
        base = os.path.splitext(model)[0]
        out_name = f"{base}.pt"
        dst = os.path.join(PROJECT_ROOT, "policy", robot, policy, out_name)

        self.sim_log.append_log_colored(f">>> Converting ONNX {model} -> {out_name}...\n", QColor(17, 168, 205))
        try:
            convert_script = os.path.join(PROJECT_ROOT, "src", "rl_sar", "scripts", "convert_policy.py")
            result = subprocess.run(
                [sys.executable, convert_script, src],
                capture_output=True, text=True, timeout=60
            )
            # convert_policy.py outputs to same dir as input with .pt extension
            default_out = os.path.splitext(src)[0] + ".pt"
            if result.returncode == 0 and os.path.isfile(default_out):
                # Move to policy dir if different
                if default_out != dst:
                    import shutil
                    shutil.move(default_out, dst)
                self.sim_log.append_log_colored(f">>> Done: {out_name}\n", QColor(35, 209, 139))
                self.populate_models(robot, policy)
                idx = self.model_combo.findText(out_name)
                if idx >= 0:
                    self.model_combo.setCurrentIndex(idx)
            else:
                self.sim_log.append_log_colored(f">>> Conversion failed:\n{result.stderr}\n", QColor(241, 76, 76))
        except subprocess.TimeoutExpired:
            self.sim_log.append_log_colored(">>> Conversion timed out\n", QColor(241, 76, 76))
        except Exception as e:
            self.sim_log.append_log_colored(f">>> Conversion error: {e}\n", QColor(241, 76, 76))

    def _apply_model_selection(self):
        """Update config.yaml model_name to the selected .pt file."""
        robot = self.robot_combo.currentText()
        policy = self.policy_combo.currentText()
        model = self.model_combo.currentText()
        if not robot or not policy or not model:
            return False

        if model.endswith(".onnx"):
            self.sim_log.append_log_colored("Error: ONNX file selected. Convert to TorchScript first.\n", QColor(241, 76, 76))
            return False

        config_path = os.path.join(PROJECT_ROOT, "policy", robot, policy, "config.yaml")
        if not os.path.isfile(config_path):
            return False

        try:
            with open(config_path, "r") as f:
                content = f.read()
            new_content = re.sub(
                r'(model_name\s*:\s*)["\']?[^"\'\n]+["\']?',
                rf'\g<1>"{model}"',
                content
            )
            with open(config_path, "w") as f:
                f.write(new_content)
            self.sim_log.append_log_colored(f">>> config.yaml model_name -> {model}\n", QColor(17, 168, 205))
            return True
        except Exception as e:
            self.sim_log.append_log_colored(f">>> Failed to update config.yaml: {e}\n", QColor(241, 76, 76))
            return False

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
                continue
            scenes.append(name)
        for subdir in sorted(os.listdir(mjcf_dir)):
            sub_path = os.path.join(mjcf_dir, subdir)
            if not os.path.isdir(sub_path):
                continue
            for xml_file in sorted(os.listdir(sub_path)):
                if xml_file.endswith(".xml"):
                    scenes.append(f"{subdir}/{xml_file[:-4]}")
        self.scene_combo.addItems(scenes)

    def _generate_terrain(self):
        """Generate terrain scene XML using terrain_generator.py."""
        robot = self.robot_combo.currentText()
        if not robot:
            self.sim_log.append_log_colored("Error: select a robot first\n", QColor(241, 76, 76))
            return

        preset = self.terrain_preset_combo.currentText()
        seed = self.terrain_seed_spin.value()
        if seed == 0 and self.terrain_seed_spin.specialValueText() == "Random":
            import random
            seed = random.randint(1, 999999)

        self.sim_log.append_log_colored(f">>> Generating terrain: {preset} (seed={seed}) for {robot}...\n", QColor(17, 168, 205))
        self.status_bar.showMessage(f"Generating terrain: {preset}")

        try:
            import importlib.util
            spec = importlib.util.spec_from_file_location(
                "terrain_generator",
                os.path.join(PROJECT_ROOT, "scripts", "terrain_generator.py")
            )
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)

            tg = mod.TerrainGenerator(robot)
            tg.generate(preset, seed=seed)
            output_name = f"scene_{preset}_s{seed}"
            path = tg.save(output_name)

            self.sim_log.append_log_colored(f">>> Terrain saved: {path}\n", QColor(35, 209, 139))
            self.lbl_terrain_status.setText(f"Generated: {output_name}")
            self.status_bar.showMessage("Terrain generated")

            # Refresh scenes and select the newly generated one
            self.populate_scenes(robot)
            idx = self.scene_combo.findText(output_name)
            if idx >= 0:
                self.scene_combo.setCurrentIndex(idx)
        except Exception as e:
            self.sim_log.append_log_colored(f">>> Terrain generation failed: {e}\n", QColor(241, 76, 76))
            self.status_bar.showMessage("Terrain generation failed")
            import traceback
            self.sim_log.append_log(traceback.format_exc())

    def _update_policy_info(self, robot_name):
        policy_dir = os.path.join(PROJECT_ROOT, "policy", robot_name)
        if not os.path.isdir(policy_dir):
            self.lbl_policy.setText("Policy: (none)")
            return
        policies = []
        for entry in sorted(os.listdir(policy_dir)):
            full = os.path.join(policy_dir, entry)
            if os.path.isdir(full) and os.path.isfile(os.path.join(full, "config.yaml")):
                policies.append(entry)
        self.lbl_policy.setText(f"Policy: {', '.join(policies)}" if policies else "Policy: (none)")

    def _check_binary(self):
        binary = self._binary_path()
        if os.path.isfile(binary):
            self.lbl_binary.setText(f"Binary: rl_sim_mujoco (found)")
            self.lbl_binary.setStyleSheet("color: #4CAF50; font-size: 11px;")
        else:
            self.lbl_binary.setText(f"Binary: rl_sim_mujoco (NOT found, build first)")
            self.lbl_binary.setStyleSheet("color: #f44336; font-size: 11px;")

    def _binary_path(self):
        return os.path.join(PROJECT_ROOT, "cmake_build", "bin", "rl_sim_mujoco")

    # ──────────────────────────────────────────────────────────
    #  Build
    # ──────────────────────────────────────────────────────────

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
        self.build_log.append_log_colored(">>> Building...\n", QColor(17, 168, 205))
        self.status_bar.showMessage("Building...")
        self.btn_build.setEnabled(False)

        self.build_process = QProcess(self)
        self.build_process.setWorkingDirectory(PROJECT_ROOT)
        self.build_process.setProcessChannelMode(QProcess.MergedChannels)
        self.build_process.readyReadStandardOutput.connect(self._on_build_output)
        self.build_process.finished.connect(self._on_build_finished)
        self.build_process.start("bash", ["build.sh"] + self._get_build_args())

    def _on_build_output(self):
        if self.build_process:
            data = self.build_process.readAllStandardOutput().data().decode(errors="replace")
            self.build_log.append_log(data)

    def _on_build_finished(self, exit_code, exit_status):
        self.btn_build.setEnabled(True)
        self._check_binary()
        if exit_code == 0:
            self.build_log.append_log_colored("\n>>> Build succeeded!\n", QColor(35, 209, 139))
            self.status_bar.showMessage("Build succeeded")
        else:
            self.build_log.append_log_colored(f"\n>>> Build failed (exit code: {exit_code})\n", QColor(241, 76, 76))
            self.status_bar.showMessage("Build failed")

    def start_clean(self):
        if self.build_process and self.build_process.state() != QProcess.NotRunning:
            return
        self.build_log.clear_log()
        self.build_log.append_log_colored(">>> Cleaning...\n", QColor(17, 168, 205))
        self.status_bar.showMessage("Cleaning...")
        self.btn_clean.setEnabled(False)

        self.build_process = QProcess(self)
        self.build_process.setWorkingDirectory(PROJECT_ROOT)
        self.build_process.setProcessChannelMode(QProcess.MergedChannels)
        self.build_process.readyReadStandardOutput.connect(self._on_build_output)
        self.build_process.finished.connect(self._on_clean_finished)
        self.build_process.start("bash", ["build.sh", "-c"])

    def _on_clean_finished(self, exit_code, exit_status):
        self.btn_clean.setEnabled(True)
        self._check_binary()
        self.build_log.append_log_colored("\n>>> Clean done\n", QColor(35, 209, 139))
        self.status_bar.showMessage("Clean done")

    # ──────────────────────────────────────────────────────────
    #  Simulation
    # ──────────────────────────────────────────────────────────

    def start_simulation(self):
        if self.sim_process and self.sim_process.state() != QProcess.NotRunning:
            return

        robot = self.robot_combo.currentText()
        scene = self.scene_combo.currentText()
        if not robot or not scene:
            self.sim_log.append_log_colored("Error: select a robot and scene first\n", QColor(241, 76, 76))
            self.tabs.setCurrentIndex(2)  # switch to log tab
            return

        binary = self._binary_path()
        if not os.path.isfile(binary):
            self.sim_log.append_log_colored(f"Error: binary not found: {binary}\n", QColor(241, 76, 76))
            self.sim_log.append_log_colored("Please build first (Ctrl+B)\n", QColor(245, 245, 67))
            self.tabs.setCurrentIndex(2)
            return

        # Apply selected model to config.yaml before launching
        if not self._apply_model_selection():
            self.sim_log.append_log_colored("Warning: could not update config.yaml model_name\n", QColor(245, 245, 67))

        self.sim_log.clear_log()
        self.sim_log.append_log_colored(f">>> Starting: {robot} / {scene}\n", QColor(35, 209, 139))
        self.status_bar.showMessage(f"Running: {robot} / {scene}")
        self.btn_start_sim.setEnabled(False)
        self.btn_stop_sim.setEnabled(True)

        # Start bridge first if needed
        if self.chk_bridge.isChecked():
            self._start_bridge()

        # Start MuJoCo binary
        self.sim_process = QProcess(self)
        self.sim_process.setWorkingDirectory(PROJECT_ROOT)
        self.sim_process.setProcessChannelMode(QProcess.MergedChannels)
        self.sim_process.readyReadStandardOutput.connect(self._on_sim_output)
        self.sim_process.finished.connect(self._on_sim_finished)
        self.sim_process.start(binary, [robot, scene])

        # Switch to Remote Control tab
        self.tabs.setCurrentIndex(1)

        # Tile MuJoCo window after a short delay
        QTimer.singleShot(2000, self._tile_mujoco_window)

    def _on_sim_output(self):
        if self.sim_process:
            data = self.sim_process.readAllStandardOutput().data().decode(errors="replace")
            self.sim_log.append_log(data)

    def _on_sim_finished(self, exit_code, exit_status):
        self.btn_start_sim.setEnabled(True)
        self.btn_stop_sim.setEnabled(False)
        self.sim_log.append_log_colored(f"\n>>> Simulation exited (code: {exit_code})\n", QColor(17, 168, 205))
        self.status_bar.showMessage("Simulation ended")
        self._stop_bridge()

    def stop_simulation(self):
        if self.sim_process and self.sim_process.state() != QProcess.NotRunning:
            self.sim_process.kill()
            self.sim_process.waitForFinished(3000)
            self.sim_log.append_log_colored("\n>>> Simulation stopped\n", QColor(245, 245, 67))
            self.status_bar.showMessage("Simulation stopped")
        self._stop_bridge()

    # ──────────────────────────────────────────────────────────
    #  MuJoCo window tiling
    # ──────────────────────────────────────────────────────────

    def _tile_mujoco_window(self):
        """Find the MuJoCo GLFW window and tile it to the right of this GUI."""
        def _do_tile():
            win = WindowTiler.find_window_by_title("MuJoCo", timeout=12)
            if win:
                WindowTiler.tile_gui_and_mujoco(None, win)
                self.sim_log.append_log_colored("[WindowTiler] MuJoCo window tiled\n", QColor(59, 142, 234))

        t = threading.Thread(target=_do_tile, daemon=True)
        t.start()

    # ──────────────────────────────────────────────────────────
    #  UDP Bridge
    # ──────────────────────────────────────────────────────────

    def _start_bridge(self):
        if self.bridge_process and self.bridge_process.state() != QProcess.NotRunning:
            return
        bridge_script = os.path.join(PROJECT_ROOT, "scripts", "udp_remote_bridge.py")
        if not os.path.isfile(bridge_script):
            self.sim_log.append_log_colored("Warning: udp_remote_bridge.py not found\n", QColor(245, 245, 67))
            return

        host = self.bridge_host_combo.currentText().strip()
        self.bridge_process = QProcess(self)
        self.bridge_process.setWorkingDirectory(PROJECT_ROOT)
        self.bridge_process.setProcessChannelMode(QProcess.MergedChannels)
        self.bridge_process.readyReadStandardOutput.connect(self._on_bridge_output)
        self.bridge_process.finished.connect(self._on_bridge_finished)
        self.bridge_process.start(sys.executable, [bridge_script, host])

        self.sim_log.append_log_colored(f">>> UDP bridge started (target: {host}:9876)\n", QColor(188, 63, 188))
        self.lbl_remote_status.setText("Bridge: starting...")
        self.lbl_remote_status.setStyleSheet("color: #ffa500; font-size: 11px;")

        # Load the web view after Flask has had time to start
        QTimer.singleShot(1500, self._load_remote_view)

    def _on_bridge_output(self):
        if self.bridge_process:
            data = self.bridge_process.readAllStandardOutput().data().decode(errors="replace")
            self.sim_log.append_log(f"[bridge] {data}")

    def _on_bridge_finished(self, exit_code, exit_status):
        self.sim_log.append_log_colored(f">>> UDP bridge exited (code: {exit_code})\n", QColor(188, 63, 188))
        self.lbl_remote_status.setText("Bridge: stopped")
        self.lbl_remote_status.setStyleSheet("color: #e94560; font-size: 11px;")

    def _stop_bridge(self):
        if self.bridge_process and self.bridge_process.state() != QProcess.NotRunning:
            self.bridge_process.kill()
            self.bridge_process.waitForFinished(2000)
        self.lbl_remote_status.setText("Bridge: not running")
        self.lbl_remote_status.setStyleSheet("color: #e94560; font-size: 11px;")

    def _load_remote_view(self):
        url = f"http://127.0.0.1:{FLASK_PORT}/"
        self.web_view.setUrl(QUrl(url))
        self.lbl_remote_url.setText(url)
        self.lbl_remote_status.setText("Bridge: running")
        self.lbl_remote_status.setStyleSheet("color: #4CAF50; font-size: 11px;")

    def _reload_remote_view(self):
        self.web_view.reload()

    # ──────────────────────────────────────────────────────────
    #  Utils
    # ──────────────────────────────────────────────────────────

    def _xdg_open(self, path):
        if os.path.isdir(path):
            subprocess.Popen(["xdg-open", path])

    def closeEvent(self, event):
        self.stop_simulation()
        if self.build_process and self.build_process.state() != QProcess.NotRunning:
            self.build_process.kill()
            self.build_process.waitForFinished(3000)
        event.accept()


# ──────────────────────────────────────────────────────────────
#  Entry point
# ──────────────────────────────────────────────────────────────

def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QApplication(sys.argv)
    icon_path = os.path.join(PROJECT_ROOT, "scripts", "app_icon.png")
    if os.path.isfile(icon_path):
        app.setWindowIcon(QIcon(icon_path))
    app.setStyle("Fusion")
    window = Sim2SimGUIPro()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
