import os
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import sys
import cv2
import numpy as np

try:
    import gz.transport13 as trans
    from gz.msgs10.image_pb2 import Image
    print("[INFO] Gazebo Harmonic libraries loaded!")
except ImportError as e:
    print(f"[ERROR] Library not found: {e}")
    sys.exit(1)

latest_frame = None

def on_image(msg):
    global latest_frame
    try:
        # 1. Grab the frame from Gazebo
        img = np.frombuffer(msg.data, dtype=np.uint8)
        img = img.reshape((msg.height, msg.width, 3))
        frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        # 2. Convert to greyscale, which simplifies processing
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 3. Square (box) detection
        # Find edges
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        
        # Extract lines and contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            # Only if the shape is large enough, so small scratches are ignored
            if area > 10000:
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
                
                # Eger seklin 4 kosesi varsa (Kare/Dikdortgense)
                if len(approx) == 4:
                    # Draw a thick green box around it
                    cv2.drawContours(frame, [approx], -1, (0, 255, 0), 4)
                    
                    # Label it
                    x, y = approx[0][0]
                    cv2.putText(frame, "HEDEF BULUNDU!", (x, y - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    break

        latest_frame = frame
    except Exception as e:
        pass

node = trans.Node()
topic = "/world/default/model/x500_0/link/base_link/sensor/camera/image"

if node.subscribe(Image, topic, on_image):
    print("[SUCCESS] Camera connected! Looking for the landing pad...")
else:
    sys.exit(1)

while True:
    if latest_frame is not None:
        cv2.imshow("Drone View - Target Detection", latest_frame)
    
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
