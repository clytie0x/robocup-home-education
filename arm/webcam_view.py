"""Live view of the arm's webcam in a browser: http://<jetson>:8082/  (also /snap.jpg = one frame)."""
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

latest = {'jpg': None}


def grab():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    while True:
        ok, f = cap.read()
        if not ok:
            latest['jpg'] = None          # camera gone: do not keep showing the last frame
            time.sleep(0.5)
            cap.release()
            cap = cv2.VideoCapture(0)
            continue
        latest['jpg'] = cv2.imencode('.jpg', f, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()


PAGE = b'''<!doctype html><html><head><meta charset="utf-8"><title>Arm webcam</title>
<style>body{background:#111;color:#ddd;font-family:sans-serif;text-align:center;margin:12px}
img{max-width:100%;border:1px solid #444}</style></head>
<body><div>Arm webcam (640x480)</div><img src="/stream"></body></html>'''


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=f')
            self.end_headers()
            try:
                while True:
                    j = latest['jpg']
                    if j:
                        self.wfile.write(b'--f\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                         + str(len(j)).encode() + b'\r\n\r\n' + j + b'\r\n')
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError):
                return
        elif self.path == '/snap.jpg' and latest['jpg']:
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.end_headers()
            self.wfile.write(latest['jpg'])
        elif self.path == '/snap.jpg':
            self.send_response(503)
            self.end_headers()
            self.wfile.write(b'no camera frame (is the webcam plugged in?)')
        else:
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(PAGE)


threading.Thread(target=grab, daemon=True).start()
ThreadingHTTPServer(('0.0.0.0', 8082), H).serve_forever()
