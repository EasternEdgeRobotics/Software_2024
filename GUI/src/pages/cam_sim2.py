import cv2
from flask import Flask, Response

app = Flask(__name__)
    
@app.route('/')
def video_feed():
    return Response(generate(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def generate():
    cap = cv2.VideoCapture(0)  # 0 for the first webcam
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        ret, jpeg = cv2.imencode('.jpg', frame)
        if not ret:
            break
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n\r\n')

if __name__ == '__main__':
    app.run(host='localhost', port=8880, threaded=True)