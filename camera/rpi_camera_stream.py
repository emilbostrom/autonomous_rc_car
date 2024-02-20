#111!/usr/bin/env python
# -*- coding: utf-8 -*
#sudo apt-get install python3-flask
#pip3 install opencv-python
from flask import Flask, render_template, Response
import cv2
app = Flask(__name__)
#app.config["CACHE_TYPE"] = "null"
@app.route('/')
def index():
    """Video streaming home page."""
    return render_template('index.html')
def gen():
    """Video streaming generator function."""
    vs = cv2.VideoCapture(0)
    while True:
        ret,frame=vs.read()
        ret, jpeg = cv2.imencode('.jpg', frame)
        frame=jpeg.tobytes()
        yield (b'--frame\r\n'
        b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        
    vs.release()
    cv2.destroyAllWindows() 