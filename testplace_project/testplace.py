"""
Web Python (Flask) hợp nhất 2 board:
  - ESP32 thường  : quạt + fuzzy + DHT11   (http://10.23.194.239, /api/fan)
  - ESP32-CAM     : cửa servo + nhận diện  (http://10.23.194.111, /open)
Chạy:  pip install flask requests   ->   python app.py   ->   mở http://127.0.0.1:5000
"""

import threading
import time
from collections import deque


import requests
from pathlib import Path
from flask import Flask, jsonify, render_template, request

# ======================= CẤU HÌNH =======================
FAN_URL = "http://10.23.194.239"      # IP ESP32 quạt (xem Serial Monitor hoặc OLED lúc khởi động)
CAM_URL = "http://10.23.194.111"    # IP ESP32-CAM (xem Serial Monitor)
REMOTE_KEY = "12"      # phải TRÙNG REMOTE_KEY trong face_door_servo.ino
POLL_SECONDS = 2
TIMEOUT = 1.5
HOST, PORT = "127.0.0.1", 5000       # 127.0.0.1: chỉ máy này truy cập. Đổi "0.0.0.0" sẽ cho cả mạng LAN bấm mở cửa!
# =========================================================

BASE_DIR = Path(__file__).resolve().parent

app = Flask(
    __name__,
    template_folder=str(BASE_DIR / "templates")
)
lock = threading.Lock()
state = {
    "fan": {"ok": False, "error": "chưa có dữ liệu", "data": None, "ts": 0},
    "cam": {"ok": False, "error": "chưa có dữ liệu", "data": None, "ts": 0},
}
history = deque(maxlen=300)   # {t, temp, hum, fan}
events = deque(maxlen=200)    # {t, level, text}


def log(text, level="info"):
    with lock:
        events.appendleft({"t": time.strftime("%H:%M:%S"), "level": level, "text": text})


def fetch(url):
    r = requests.get(url, timeout=TIMEOUT)
    r.raise_for_status()
    return r.json()


def poll_loop():
    prev = {"fan_ok": None, "cam_ok": None, "door": None, "mode": None, "seen_ts": None,
            "logged_ts": 0.0, "logged_name": ""}
    while True:
        now = time.time()
        for key, url in (("fan", FAN_URL + "/api/status"), ("cam", CAM_URL + "/status")):
            try:
                data = fetch(url)
                with lock:
                    state[key].update(ok=True, error="", data=data, ts=now)
            except Exception as e:
                with lock:
                    state[key].update(ok=False, error=f"{type(e).__name__}: {str(e)[:140]}")

        with lock:
            fan, cam = dict(state["fan"]), dict(state["cam"])

        # --- lịch sử cho biểu đồ ---
        if fan["ok"] and fan["data"] and fan["data"].get("temperature") is not None:
            d = fan["data"]
            with lock:
                history.append({"t": time.strftime("%H:%M:%S"), "temp": d["temperature"],
                                "hum": d["humidity"], "fan": d["fan_percent"]})

        # --- sự kiện ---
        for name, st in (("Quạt (ESP32)", fan), ("Cửa (ESP32-CAM)", cam)):
            k = "fan_ok" if name.startswith("Quạt") else "cam_ok"
            if prev[k] is not None and st["ok"] != prev[k]:
                log(f"{name} đã kết nối lại" if st["ok"] else f"Mất kết nối {name}", "ok" if st["ok"] else "err")
            elif prev[k] is None and not st["ok"]:
                log(f"Chưa kết nối được {name}: {st['error']}", "err")
            prev[k] = st["ok"]

        if fan["ok"] and fan["data"]:
            mode = fan["data"].get("mode")
            if prev["mode"] is not None and mode != prev["mode"]:
                log("Quạt chuyển sang chế độ " + ("TAY" if mode == "manual" else "TỰ ĐỘNG (fuzzy)"), "info")
            prev["mode"] = mode

        if cam["ok"] and cam["data"]:
            d = cam["data"]
            if prev["door"] is not None and d["door"] != prev["door"]:
                log("Cửa đã MỞ" if d["door"] == "open" else "Cửa đã KHÓA", "ok")
            prev["door"] = d["door"]
            ago = d.get("last_seen_s_ago", -1)
            if ago >= 0 and d.get("last_name"):
                seen_ts = now - ago                      # thời điểm board nhận ra mặt lần cuối
                is_new = prev["seen_ts"] is not None and seen_ts > prev["seen_ts"] + 1.5
                # người đứng trước camera được nhận ra liên tục: chỉ ghi 1 dòng mỗi 10 giây cho cùng một người
                if is_new and (d["last_name"] != prev["logged_name"] or now - prev["logged_ts"] > 10):
                    log(f"Nhận diện khuôn mặt: {d['last_name']}", "ok")
                    prev["logged_ts"], prev["logged_name"] = now, d["last_name"]
                prev["seen_ts"] = seen_ts

        time.sleep(POLL_SECONDS)


@app.route("/")
def index():
    return render_template("index.html", poll_ms=POLL_SECONDS * 1000, cam_url=CAM_URL)


@app.route("/api/state")
def api_state():
    with lock:
        return jsonify(fan=dict(state["fan"]), cam=dict(state["cam"]), now=time.time(),
                       history=list(history)[-150:], events=list(events)[:60])


@app.route("/api/fan", methods=["POST"])
def api_fan():
    body = request.get_json(silent=True) or {}
    mode = body.get("mode")
    if mode not in ("auto", "manual"):
        return jsonify(ok=False, error="mode phải là auto hoặc manual"), 400
    params = {"mode": mode}
    if mode == "manual":
        try:
            params["percent"] = max(0, min(100, float(body.get("percent", 0))))
        except (TypeError, ValueError):
            return jsonify(ok=False, error="percent không hợp lệ"), 400
    try:
        r = requests.get(FAN_URL + "/api/fan", params=params, timeout=TIMEOUT)
        r.raise_for_status()
        log("Đặt quạt " + (f"tay {params['percent']:.0f}%" if mode == "manual" else "về tự động"), "info")
        return jsonify(ok=True)
    except Exception as e:
        return jsonify(ok=False, error=f"{type(e).__name__}: {str(e)[:140]}"), 502


@app.route("/api/door/open", methods=["POST"])
def api_door_open():
    try:
        r = requests.get(CAM_URL + "/open", params={"key": REMOTE_KEY}, timeout=TIMEOUT)
        if r.status_code == 403:
            return jsonify(ok=False, error="Sai REMOTE_KEY hoặc ESP32-CAM tắt mở cửa từ xa"), 403
        r.raise_for_status()
        log("Mở cửa từ web", "ok")
        return jsonify(ok=True)
    except Exception as e:
        return jsonify(ok=False, error=f"{type(e).__name__}: {str(e)[:140]}"), 502


if __name__ == "__main__":
    threading.Thread(target=poll_loop, daemon=True).start()
    print(f"Quạt: {FAN_URL} | Cửa: {CAM_URL}")
    print(f"Mở trình duyệt: http://{HOST}:{PORT}")
    app.run(host=HOST, port=PORT, debug=False)
