import socket
import threading
import logging
from flask import Flask, request, jsonify

# --- FLASK SUNUCUSU (KAMERA TESPITLERI ICIN - PORT 5000) ---
app = Flask(__name__)

# Flask'in kendi baslangic ve erisim loglarini gizleyelim ki ekran kirlenmesin
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

@app.route('/detect', methods=['POST'])
def detect():
    data = request.json
    # Kirmizi kutu tespit edildiginde terminalde dikkat cekici sekilde yazdir
    print(f"\n [GÖRSEL İSTİHBARAT] {data.get('object')} bulundu! | Koordinatlar: X={data['coordinates']['x']}, Y={data['coordinates']['y']}\n")
    return jsonify({"status": "success"}), 200

# --- UDP SUNUCUSU (DRONE DURUMLARI ICIN - PORT 5005) ---
def udp_listener():
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.bind(("127.0.0.1", 5005))
    print(" UDP Dinleyicisi (Port: 5005) baslatildi...")
    
    while True:
        data, addr = udp_sock.recvfrom(1024)
        print(data.decode("utf-8"))

if __name__ == '__main__':
    print("==================================================")
    print(" Merkezi Istihbarat Sunucusu Baslatiliyor...")
    print("==================================================")

    # UDP dinleyicisini arka planda (ayri bir thread olarak) baslat
    udp_thread = threading.Thread(target=udp_listener, daemon=True)
    udp_thread.start()

    # Ana thread uzerinde Flask HTTP sunucusunu baslat
    print(" HTTP Dinleyicisi (Port: 5000) baslatildi...\n")
    app.run(host='127.0.0.1', port=5000)
