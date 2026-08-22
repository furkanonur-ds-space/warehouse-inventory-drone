import socket
import threading
import logging
from flask import Flask, request, jsonify

# --- FLASK SERVER (CAMERA DETECTIONS, PORT 5000) ---
app = Flask(__name__)

# Silence Flask's own startup and access logs to keep the console readable
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

@app.route('/detect', methods=['POST'])
def detect():
    data = request.json
    # Print prominently when a red box is detected
    print(f"\n [VISUAL DETECTION] {data.get('object')} found. Coordinates: X={data['coordinates']['x']}, Y={data['coordinates']['y']}\n")
    return jsonify({"status": "success"}), 200

# --- UDP SERVER (VEHICLE STATUS, PORT 5005) ---
def udp_listener():
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.bind(("127.0.0.1", 5005))
    print(" UDP listener started on port 5005...")
    
    while True:
        data, addr = udp_sock.recvfrom(1024)
        print(data.decode("utf-8"))

if __name__ == '__main__':
    print("==================================================")
    print(" Starting central detection server...")
    print("==================================================")

    # Start the UDP listener in a background thread
    udp_thread = threading.Thread(target=udp_listener, daemon=True)
    udp_thread.start()

    # Run the Flask HTTP server on the main thread
    print(" HTTP listener started on port 5000...\n")
    app.run(host='127.0.0.1', port=5000)
