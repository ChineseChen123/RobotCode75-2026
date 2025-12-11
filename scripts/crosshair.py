import cv2
from flask import Flask, Response
import threading
import ntcore

app = Flask(__name__)

# Global variable to store the processed frame and a lock for thread-safe access
output_frame = None
lock = threading.Lock()

def capture_frames():
    global output_frame

    cap = cv2.VideoCapture("http://localhost:1182/?action=stream")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Warning: Failed to grab frame from MJPG server")
            break

        img = cv2.imread('./IMG_3916.jpeg')

        # Resize img to match the frame size
        img_resized = cv2.resize(img, (frame.shape[1], frame.shape[0]))

        # Mixing percentage (0.0 to 1.0)
        mix_percent = 0.8  # 60% frame, 40% img

        # Blend the images
        frame = cv2.addWeighted(frame, mix_percent, img_resized, 1 - mix_percent, 0)
        
        # Draw a crosshair on the frame
        height, width = frame.shape[:2]
        center_x, center_y = width // 2, height // 2
        color = (0, 255, 0)  # Green color for the crosshair
        thickness = 4
        
        # Draw horizontal line
        cv2.line(frame, (0, center_y), (width, center_y), color, thickness)
        # Draw vertical line
        cv2.line(frame, (center_x, 0), (center_x, height), color, thickness)
        
        # Save the processed frame in a thread-safe manner
        with lock:
            output_frame = frame.copy()
        
        # A brief delay; cv2.waitKey can also be used for timing and processing GUI events
        cv2.waitKey(1)
    
    cap.release()

def generate():
    """Generator function that yields processed frames in MJPG format."""
    global output_frame
    while True:
        with lock:
            if output_frame is None:
                continue
            # Encode the frame in JPEG format
            ret, jpeg = cv2.imencode('.jpg', output_frame)
            if not ret:
                continue
            frame_bytes = jpeg.tobytes()
        # Yield a byte frame in MJPG format
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/')
def video_feed():
    # Route that streams the video feed
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')



if __name__ == '__main__':
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4("Crosshair Client")
    inst.setServerTeam(75) # where TEAM=190, 294, etc, or use inst.setServer("hostname") or similar
    inst.startDSClient()

    table = inst.getTable("CameraPublisher")
    table.setDefaultStringArray("Crosshair/streams", ["mjpg:http://127.0.0.1:8080"])
    table.putBoolean("Crosshair/connected", True)
    table.putString("Crosshair/mode", "640x480 MJPEG 30 fps")
    table.setDefaultStringArray("Crosshair/modes", ["640x480 MJPEG 30 fps"])
    table.putString("Crosshair/source", "cv:")
    table.putString("Crosshair/description", "")
    # Start the frame capture in a background
    # thread
    t = threading.Thread(target=capture_frames)
    t.daemon = True
    t.start()
    
    # Run the Flask app to serve the MJPG stream
    app.run(host='0.0.0.0', port=8080, threaded=True)