from flask import Flask, render_template
from flask_socketio import SocketIO
import serial
from serial.serialutil import SerialException
import time

app = Flask(__name__)
socketio = SocketIO(app)

ser = serial.Serial('COM3', 921600, timeout=1)

def send_command(hex_data):
    try:
        ser.write(bytes.fromhex(hex_data))
    except SerialException as e:
        print(f"An error occurred: {e}")
        ser.close()
        ser.open()

@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('forward')
def handle_forward():
    send_command('41 54 90 07 eb fc 08 05 70 00 00 07 01 95 54 0d 0a')

@socketio.on('backward')
def handle_backward():
    send_command('41 54 90 07 eb fc 08 05 70 00 00 07 01 6a aa 0d 0a')

@socketio.on('stop')
def handle_stop():
    send_command('41 54 90 07 eb fc 08 05 70 00 00 07 00 7f ff 0d 0a')

if __name__ == '__main__':
    socketio.run(app, debug=False, host='0.0.0.0', port=5000)
