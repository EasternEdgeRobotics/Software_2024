from flask import Flask, Response, send_file
import cv2
import configparser
import numpy as np
import time
app = Flask(__name__)

# Read configuration from spyglass.conf
config = configparser.ConfigParser()
config.read('spyglass.conf')

# Get configuration values
http_port = int(config.get('DEFAULT', 'HTTP_PORT', fallback=8080))
resolution = tuple(
    map(int, config.get('DEFAULT', 'RESOLUTION', fallback='640x480').split('x')))
fps = int(config.get('DEFAULT', 'FPS', fallback=15))
stream_url = config.get('DEFAULT', 'STREAM_URL', fallback='/stream')
snapshot_url = config.get('DEFAULT', 'SNAPSHOT_URL', fallback='/snapshot')

# Open the camera
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
camera.set(cv2.CAP_PROP_FPS, fps)

# Add a delay to give the camera time to start up
time.sleep(2)


def generate_frames():
    while True:
        success, frame = camera.read()
        if not success:
            print("Could not read frame from camera. Make sure the camera is connected and not being used by another process.")
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route(stream_url)
def stream_video():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route(snapshot_url)
def snapshot():
    success, frame = camera.read()
    if not success:
        return "Could not get frame", 500
    else:
        _, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        return send_file(
            np.fromstring(frame, np.uint8),
            attachment_filename='snapshot.jpg',
            mimetype='image/jpeg'
        )


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=http_port)
