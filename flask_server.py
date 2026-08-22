from flask import Flask, request, jsonify
import logging

app = Flask(__name__)

# Silence Flask's own startup logs so only the detection notice stands out
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

@app.route('/detect', methods=['POST'])
def detect():
    data = request.json
    print(f"\n [VISUAL DETECTION] {data.get('object')} found. Coordinates: X={data['coordinates']['x']}, Y={data['coordinates']['y']}\n")
    return jsonify({"status": "success", "message": "Detection received"}), 200

if __name__ == '__main__':
    print("==================================================")
    print(" Autonomous image processing server listening on port 5000...")
    print("==================================================")
    app.run(host='127.0.0.1', port=5000)
