#!/usr/bin/env python3
"""
Flask HTTP -> UDP bridge for rl_sim_mujoco remote control.

Run this script on the same machine as rl_sim_mujoco:
    python3 scripts/udp_remote_bridge.py

Then open http://<machine-ip>:5000/ on your mobile browser.
"""

import socket
import sys
from pathlib import Path

from flask import Flask, request, jsonify

app = Flask(__name__, static_folder=str(Path(__file__).parent))

UDP_HOST = "127.0.0.1"
UDP_PORT = 9876


def send_udp(data: str) -> str:
    """Send a UDP packet to rl_sim_mujoco and return the response."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.5)
    try:
        sock.sendto(data.encode(), (UDP_HOST, UDP_PORT))
        resp, _ = sock.recvfrom(256)
        return resp.decode()
    except socket.timeout:
        return "timeout"
    finally:
        sock.close()


HTML_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
<title>0315 Remote Control</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: #1a1a2e; color: #eee;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    height: 100vh; display: flex; flex-direction: column;
    overflow: hidden; touch-action: none;
  }
  header {
    padding: 10px; text-align: center; background: #16213e;
    font-size: 1.1rem; font-weight: bold; border-bottom: 2px solid #0f3460;
  }
  .status {
    padding: 6px; text-align: center; font-size: 0.85rem;
    background: #0f3460;
  }
  .main {
    flex: 1; display: flex; flex-direction: column;
    padding: 10px; gap: 10px;
  }
  .joystick-area {
    flex: 1; display: flex; gap: 10px;
  }
  .pad {
    flex: 1; background: #16213e; border-radius: 12px;
    position: relative; touch-action: none;
    border: 2px solid #0f3460;
  }
  .pad-label {
    position: absolute; top: 8px; left: 0; right: 0;
    text-align: center; font-size: 0.75rem; color: #888;
    pointer-events: none;
  }
  .knob {
    position: absolute; width: 60px; height: 60px;
    background: #e94560; border-radius: 50%;
    left: 50%; top: 50%; transform: translate(-50%, -50%);
    box-shadow: 0 2px 8px rgba(0,0,0,0.4);
    pointer-events: none;
  }
  .states {
    display: grid; grid-template-columns: repeat(2, 1fr); gap: 8px;
  }
  .btn {
    padding: 14px; border: none; border-radius: 10px;
    font-size: 1rem; font-weight: bold; color: #fff;
    background: #0f3460; cursor: pointer;
    transition: background 0.15s;
  }
  .btn:active { background: #e94560; }
  .btn.primary { background: #533483; }
  .info {
    display: flex; justify-content: space-between;
    font-size: 0.8rem; color: #aaa; padding: 4px 2px;
  }
</style>
</head>
<body>
<header>0315 Remote Control</header>
<div class="status" id="status">Connecting...</div>
<div class="main">
  <div class="info">
    <span id="vel-display">VEL: x=0.00 y=0.00 yaw=0.00</span>
    <span id="state-display">State: —</span>
  </div>
  <div class="joystick-area">
    <div class="pad" id="pad-move">
      <div class="pad-label">Move (X / Y)</div>
      <div class="knob"></div>
    </div>
    <div class="pad" id="pad-rotate">
      <div class="pad-label">Rotate (Yaw)</div>
      <div class="knob"></div>
    </div>
  </div>
  <div class="states">
    <button class="btn primary" data-state="getup">GetUp (0)</button>
    <button class="btn primary" data-state="locomotion">RL Run (1)</button>
    <button class="btn" data-state="getdown">GetDown (9)</button>
    <button class="btn" data-state="passive">Passive (P)</button>
  </div>
</div>
<script>
const statusEl = document.getElementById('status');
const velEl = document.getElementById('vel-display');
const stateEl = document.getElementById('state-display');

let cmd = { x: 0.0, y: 0.0, yaw: 0.0 };
let lastSent = '';

function sendCmd() {
  const payload = `x=${cmd.x.toFixed(2)}&y=${cmd.y.toFixed(2)}&yaw=${cmd.yaw.toFixed(2)}`;
  if (payload === lastSent) return;
  lastSent = payload;
  fetch('/cmd', {
    method: 'POST',
    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
    body: payload
  }).then(r => r.text()).then(t => {
    statusEl.textContent = 'Connected';
    statusEl.style.color = '#4ecca3';
  }).catch(e => {
    statusEl.textContent = 'Disconnected';
    statusEl.style.color = '#e94560';
  });
}

function updateVelDisplay() {
  velEl.textContent = `VEL: x=${cmd.x.toFixed(2)} y=${cmd.y.toFixed(2)} yaw=${cmd.yaw.toFixed(2)}`;
}

setInterval(sendCmd, 50); // 20 Hz

function setupPad(element, axisX, axisY, maxVal) {
  const knob = element.querySelector('.knob');
  const rect = () => element.getBoundingClientRect();
  let active = false;
  let cx, cy, radius;

  function start(e) {
    e.preventDefault();
    active = true;
    const r = rect();
    cx = r.left + r.width / 2;
    cy = r.top + r.height / 2;
    radius = Math.min(r.width, r.height) / 2 - 35;
    move(e);
  }
  function move(e) {
    if (!active) return;
    e.preventDefault();
    const touch = e.touches ? e.touches[0] : e;
    let dx = touch.clientX - cx;
    let dy = touch.clientY - cy;
    const dist = Math.sqrt(dx * dx + dy * dy);
    if (dist > radius) {
      dx = (dx / dist) * radius;
      dy = (dy / dist) * radius;
    }
    knob.style.transform = `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`;

    const nx = dx / radius;
    const ny = dy / radius;
    if (axisX !== null) cmd[axisX] = parseFloat((-nx * maxVal).toFixed(2));
    if (axisY !== null) cmd[axisY] = parseFloat((-ny * maxVal).toFixed(2)); // Y up is negative in screen coords
    updateVelDisplay();
  }
  function end(e) {
    if (!active) return;
    active = false;
    knob.style.transform = `translate(-50%, -50%)`;
    if (axisX !== null) cmd[axisX] = 0.0;
    if (axisY !== null) cmd[axisY] = 0.0;
    updateVelDisplay();
  }

  element.addEventListener('touchstart', start, { passive: false });
  element.addEventListener('touchmove', move, { passive: false });
  element.addEventListener('touchend', end);
  element.addEventListener('touchcancel', end);
  element.addEventListener('mousedown', start);
  window.addEventListener('mousemove', move);
  window.addEventListener('mouseup', end);
}

setupPad(document.getElementById('pad-move'), 'y', 'x', 1.5);
setupPad(document.getElementById('pad-rotate'), 'yaw', null, 2.0);

document.querySelectorAll('.btn[data-state]').forEach(btn => {
  btn.addEventListener('click', () => {
    const state = btn.dataset.state;
    stateEl.textContent = 'State: ' + state;
    fetch('/cmd', {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: `state=${state}`
    }).catch(() => {});
  });
});
</script>
</body>
</html>
"""


@app.route("/")
def index():
    return HTML_PAGE


@app.route("/cmd", methods=["POST"])
def cmd():
    x = request.form.get("x", "0.0")
    y = request.form.get("y", "0.0")
    yaw = request.form.get("yaw", "0.0")
    state = request.form.get("state", "")

    parts = []
    if state:
        parts.append(f"state={state}")
    parts.append(f"x={x}")
    parts.append(f"y={y}")
    parts.append(f"yaw={yaw}")
    payload = "&".join(parts)

    resp = send_udp(payload)
    return jsonify({"udp_response": resp})


if __name__ == "__main__":
    # Allow overriding target via env
    UDP_HOST = sys.argv[1] if len(sys.argv) > 1 else UDP_HOST
    print(f"Bridge: HTTP 0.0.0.0:5000  ->  UDP {UDP_HOST}:{UDP_PORT}")
    print(f"Open http://<this-machine-ip>:5000/ on your phone")
    app.run(host="0.0.0.0", port=5000, debug=False)
