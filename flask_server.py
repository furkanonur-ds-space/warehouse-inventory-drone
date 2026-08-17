from flask import Flask, request, jsonify
import logging

app = Flask(__name__)

# Flask'in kendi baslangic loglarini gizleyelim ki sadece bizim kirmizi uyari yazimiz dikkat ceksin
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

@app.route('/detect', methods=['POST'])
def detect():
    data = request.json
    print(f"\n [GÖRSEL İSTİHBARAT] {data.get('object')} bulundu! | Koordinatlar: X={data['coordinates']['x']}, Y={data['coordinates']['y']}\n")
    return jsonify({"status": "success", "message": "Istihbarat alindi"}), 200

if __name__ == '__main__':
    print("==================================================")
    print(" Otonom Görüntü İşleme Sunucusu (Port: 5000) Dinliyor...")
    print("==================================================")
    app.run(host='127.0.0.1', port=5000)
