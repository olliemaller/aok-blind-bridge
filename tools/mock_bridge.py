# Local mock of the bridge API for testing ui/index.html in a browser.
# python3 tools/mock_bridge.py [port]   then open http://localhost:8099
import json, os, sys, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
T0 = time.time()
names = ["Lounge left", "Lounge right", "Kitchen", "Study", "Bedroom", "Dining"]
rooms = ["Lounge", "Lounge", "Kitchen", "Study", "Bedroom", ""]
REV = [1]
NET = dict(brand="A-OK", tz="", tz_name="")
blinds = [dict(id=n.lower().replace(" ", "_"), name=n, channels=str(i + 1), open_ms=24000 + i * 900, close_ms=22000 + i * 700,
               invert=False, favourite=(40 if i == 0 else -1), device_class="shade", room=rooms[i]) for i, n in enumerate(names)]
blinds.append(dict(id="lounge", name="Lounge (both)", channels="1,2", open_ms=24000, close_ms=22000, invert=False, favourite=-1, device_class="shade", room="Lounge"))
state = {b["id"]: dict(pos=[100, 60, 0, 35, 100, -1, 80][i], motion="stopped", start=0, startpos=0, goal=0) for i, b in enumerate(blinds)}
remote = dict(prefix="A1B2C3", remote_id="3F", open="F42A", close="BCF2", stop="DC12", listen=True, seen="1,2,3,4,5,6,7")
cal = dict(active=False)
cal_hist = [dict(name="Kitchen", direction="close", ms=22140, previous_ms=30000, when=int(T0) - 3600)]
logs = []
def log(level, text):
    logs.append(dict(seq=len(logs) + 1, ms=int((time.time() - T0) * 1000), level=level, text=text, when=int(time.time())))
log("I", "Motolux Blind Bridge 1.1.0 started (7 blinds, reset reason 1)")
log("I", "CC1101 radio ready at 433.92 MHz")
log("I", "Wi-Fi connected to HomeWiFi, IP 192.168.0.150, signal -58 dBm")
log("W", "MQTT 192.168.0.202:1883: broker not reachable (code -2)")
log("I", "MQTT connected to 192.168.0.202:1883")
frames = [
    dict(ago_s=4, payload="A1B2C33F0001F42A", remote_id="3F", prefix="A1B2C3", channels="1", action="open", matched=True, repeats=3, kind="up", code="F4"),
    dict(ago_s=40, payload="A1B2C33F0002DB11", remote_id="3F", prefix="A1B2C3", channels="2", action="unknown", matched=True, repeats=1, kind="release", code="DB"),
    dict(ago_s=95, payload="0F0E0D7A0004BC10", remote_id="7A", prefix="0F0E0D", channels="3", action="close", matched=False, repeats=2, kind="down", code="BC"),
    dict(ago_s=300, payload="A1B2C33F0004AC00", remote_id="3F", prefix="A1B2C3", channels="4", action="unknown", matched=True, repeats=1, kind="program", code="AC"),
    dict(ago_s=420, payload="A1B2C33F00049900", remote_id="3F", prefix="A1B2C3", channels="4", action="unknown", matched=True, repeats=1, kind="other", code="99"),
]

def travel(b, m): return b["open_ms"] if m == "opening" else b["close_ms"]
def cur(b):
    s = state[b["id"]]
    if s["motion"] == "stopped": return s["pos"]
    d = (time.time() - s["start"]) * 1000 * 100 / travel(b, s["motion"])
    p = s["startpos"] + d if s["motion"] == "opening" else s["startpos"] - d
    if (s["motion"] == "opening" and p >= s["goal"]) or (s["motion"] == "closing" and p <= s["goal"]):
        s["pos"] = s["goal"]; s["motion"] = "stopped"; return s["pos"]
    return int(p)

def move(b, target):
    s = state[b["id"]]; p = cur(b)
    if p < 0: p = 0 if target == 100 else 100
    if target == p: return
    s.update(startpos=p, start=time.time(), goal=target, motion="opening" if target > p else "closing")
    log("I", f"Sent {'Up' if target > p else 'Down'} on channel {b['channels']}")

def full_state():
    out = dict(fw="1.2.0", rev=REV[0], brand=NET["brand"], uptime=int(time.time() - T0) + 7300, heap=181234, reset_reason=1, time=int(time.time()), log_seq=len(logs),
               wifi=dict(connected=True, ssid="HomeWiFi", ip="192.168.0.150", rssi=-58, ap=False, hostname="aok-blinds"),
               mqtt=dict(configured=True, connected=True, state=0),
               radio=dict(ready=True, overflow=0, mhz=433.92, last_tx_ago_s=12, last_rx_ago_s=4),
               remote=dict(remote, timings={}, complete=True, learning=False, learn_status="complete"),
               blinds=[], frames=frames, cal_history=cal_hist,
               calibration=dict(cal, elapsed_ms=int((time.time() - cal.get("t", 0)) * 1000)) if cal["active"] else dict(active=False))
    for b in blinds:
        s = state[b["id"]]; p = cur(b)
        o = dict(b, position=p, motion=s["motion"], target=s["goal"], source="web", group="," in b["channels"])
        if s["motion"] != "stopped":
            o["rate"] = 100000 / travel(b, s["motion"]); o["goal"] = s["goal"]
        out["blinds"].append(o)
    return out

class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def send(self, code, body, ctype="application/json"):
        data = body if isinstance(body, bytes) else json.dumps(body).encode()
        self.send_response(code); self.send_header("Content-Type", ctype); self.send_header("Content-Length", str(len(data))); self.end_headers(); self.wfile.write(data)
    def do_GET(self):
        u = urlparse(self.path); q = parse_qs(u.query)
        if u.path == "/": return self.send(200, open(os.path.join(ROOT, "ui", "index.html"), "rb").read(), "text/html")
        if u.path == "/api/state":
            st = full_state()
            if "lite" in q: st.pop("frames"); st.pop("cal_history")
            return self.send(200, st)
        if u.path == "/api/backup": return self.send(200, dict(app="aok-blind-bridge", version="1.2.0", remote=remote, blinds=blinds))
        if u.path == "/api/config":
            return self.send(200, dict(network=dict(ssid="HomeWiFi", pass_set=True, hostname="aok-blinds", admin_set=False, mqtt_host="192.168.0.202",
                mqtt_port=1883, mqtt_user="mqtt", mqtt_pass_set=True, base_topic="aok_blinds", discovery_prefix="homeassistant", **NET),
                pins=dict(sck=18, miso=19, mosi=23, cs=5, gdo0=12, gdo2=27, led=2, mhz=433.92, pa_dbm=10), remote=remote, blinds=blinds))
        if u.path == "/api/log":
            after = int(q.get("after", ["0"])[0])
            return self.send(200, dict(seq=len(logs), lines=[l for l in logs if l["seq"] > after]))
        if u.path == "/api/scan": return self.send(200, dict(scanning=False, networks=[dict(ssid="HomeWiFi", rssi=-55, secure=True), dict(ssid="Telstra1234", rssi=-77, secure=True)]))
        self.send(404, {"ok": False})
    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0)); body = json.loads(self.rfile.read(n) or b"{}")
        p = urlparse(self.path).path
        if p == "/api/network":
            for k in ("brand", "tz", "tz_name"):
                if k in body: NET[k] = body[k]
            log("I", "Network settings saved"); return self.send(200, {"ok": True, "rebooting": False})
        if p == "/api/command" and body["id"] == "all":
            for b in blinds:
                if "," in b["channels"]: continue
                if body["cmd"] == "OPEN": move(b, 100)
                elif body["cmd"] == "CLOSE": move(b, 0)
                else: state[b["id"]].update(pos=cur(b), motion="stopped")
            log("I", f"All blinds: {body['cmd']} (web)"); return self.send(200, {"ok": True})
        if p == "/api/command":
            b = next(x for x in blinds if x["id"] == body["id"]); c = body["cmd"]
            log("I", f"Web page: {b['name']} → {c}")
            if c == "OPEN": move(b, 100)
            elif c == "CLOSE": move(b, 0)
            elif c == "STOP": state[b["id"]].update(pos=cur(b), motion="stopped")
            elif c == "FAVOURITE": move(b, b["favourite"])
            elif c.isdigit(): move(b, int(c))
            elif c in ("SYNC_OPEN", "SYNC_CLOSE"): state[b["id"]].update(pos=100 if c == "SYNC_OPEN" else 0, motion="stopped")
            return self.send(200, {"ok": True})
        if p == "/api/calibrate":
            a = body.get("action")
            if a in ("start_close", "start_open"):
                b = next(x for x in blinds if x["id"] == body["id"])
                cal.update(active=True, id=b["id"], name=b["name"], direction="opening" if a == "start_open" else "closing", t=time.time())
                return self.send(200, {"ok": True})
            if a == "done":
                ms = int((time.time() - cal["t"]) * 1000); b = next(x for x in blinds if x["id"] == cal["id"])
                k = "open_ms" if cal["direction"] == "opening" else "close_ms"; prev = b[k]; b[k] = ms
                cal_hist.append(dict(name=b["name"], direction="open" if k == "open_ms" else "close", ms=ms, previous_ms=prev, when=int(time.time())))
                cal["active"] = False; REV[0] += 1; return self.send(200, {"ok": True, "measured_ms": ms})
            if a == "cancel": cal["active"] = False
            if a == "clear_history": cal_hist.clear()
            return self.send(200, {"ok": True})
        if p == "/api/blinds":
            blinds[:] = body["blinds"]
            for b in blinds:
                b.setdefault("id", b["name"].lower().replace(" ", "_")); state.setdefault(b["id"], dict(pos=-1, motion="stopped", start=0, startpos=0, goal=0))
            return self.send(200, {"ok": True})
        self.send(200, {"ok": True})

port = int(sys.argv[1]) if len(sys.argv) > 1 else 8099
ThreadingHTTPServer(("127.0.0.1", port), H).serve_forever()
