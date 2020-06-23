# This file requires completion

import os
from datetime import datetime
import cv2
import numpy as np
import socketio
import eventlet
import eventlet.wsgi
from PIL import Image
from flask import Flask
from io import BytesIO, StringIO
import matplotlib.image as mpimg
import time

from perception import perception_step
from rover import RoverState
from supporting_fns import update_rover

sio = socketio.Server()
app = Flask(__name__)

@sio.on('telemetry')
def telemetry(sid,data):
    print(data.items())

@sio.on('connect')
def connect(sid, environ):
    print("connect ",sid)
    sample_data = {}
    sio.emit(
        "get_samples",
        sample_data,
        skip_sid=True)

app = socketio.Middleware(sio, app)

eventlet.wsgi.server(eventlet.listen(('',4567)),app)
