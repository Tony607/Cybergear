import pyaudio
import numpy as np
import matplotlib.pyplot as plt
import csv
from datetime import datetime
from scipy.signal import find_peaks_cwt
from pcan_cybergear import CANMotorController
import can

bus = can.interface.Bus(bustype="pcan", channel="PCAN_USBBUS1", bitrate=1000000)
motor = CANMotorController(bus, motor_id=127, main_can_id=254)

# 速度模式
motor.write_single_param("run_mode", value=2)
motor.enable()
motor.write_single_param("spd_ref", value=0)


# Initialize PyAudio
p = pyaudio.PyAudio()

# Set up stream parameters
RATE = 8000  # samples per second
CHUNK = 1024  # number of samples per chunk

# Open audio stream
stream = p.open(format=pyaudio.paInt16, channels=1, rate=RATE, input=True, frames_per_buffer=CHUNK)

# Initialize the plot
plt.ion()  # Turn on interactive mode
fig, ax = plt.subplots()

def map_whistle_to_speed(whistle_freq):
    freq_min, freq_max = 600, 2000
    speed_min, speed_max = 0, 20
    return speed_min + ((speed_max - speed_min) / (freq_max - freq_min)) * (whistle_freq - freq_min)

# Function to save abs_fourier to CSV
def save_to_csv(data, frequencies):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M_%S")
    filename = f"./data/{timestamp}.csv"
    
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Frequency', 'Magnitude'])
        for freq, mag in zip(frequencies, data):
            writer.writerow([freq, mag])
            
    print(f"Data saved to {filename}")

# Function to handle key events
def on_key(event):
    if event.key == ' ':
        save_to_csv(abs_fourier, frequencies)
    elif event.key == 'escape':  # 'esc' key
        motor.disable()
        print("Motor disabled.")
    elif event.key == 'enter':  # 'enter' key
        motor.enable()
        print("Motor enabled.")

# Connect the event to the function
fig.canvas.mpl_connect('key_press_event', on_key)

# Function to detect whistle frequency
def detect_whistle(data):
    global abs_fourier, frequencies  # Declare as global to access it in save_to_csv
    
    # Fourier Transform
    fourier = np.fft.fft(data)
    abs_fourier = np.abs(fourier)[:len(fourier)//2]  # Take only the first half
    frequencies = np.fft.fftfreq(len(fourier))[:len(fourier)//2] * RATE
    
    # Dynamic threshold based on 5 times the mean
    threshold = 50 * np.median(abs_fourier)
    
    # Find peaks using find_peaks_cwt
    peaks = find_peaks_cwt(abs_fourier, np.arange(1, 10))
    
    # Filter peaks based on the threshold
    peaks = [p for p in peaks if abs_fourier[p] > threshold]
    
    # Update the plot
    ax.clear()
    ax.plot(frequencies, abs_fourier)
    ax.scatter(frequencies[peaks], abs_fourier[peaks], color='red')
    ax.set_yscale('log')  # 设置 Y 轴为对数尺度
    ax.set_xlim([0, RATE // 2])
    plt.draw()
    plt.pause(0.01)
    
    # Check if it's a whistle by ensuring there's only one dominant peak
    if len(peaks) == 1 and abs(frequencies[peaks[0]]) > 500:
        return abs(frequencies[peaks[0]])
    else:
        return None

# Main loop to read audio data and detect whistle
try:
    while True:
        # Check if Matplotlib window is closed
        if not plt.fignum_exists(fig.number):
            print("Matplotlib window closed.")
            break
        
        audio_data = np.frombuffer(stream.read(CHUNK), dtype=np.int16)
        whistle_freq = detect_whistle(audio_data)
        
        if whistle_freq is not None:
            print(f"Whistle detected at frequency: {whistle_freq} Hz")
            # Map whistle frequency to motor speed and update
            motor_speed = map_whistle_to_speed(whistle_freq)
            motor.write_single_param("spd_ref", value=motor_speed)
            print(f"Motor speed set to {motor_speed} rad/s")
        else:
            print("No whistle detected.")
except KeyboardInterrupt:
    print("Interrupted by user.")
finally:
    motor.disable()
    print("Motor disabled.")
    stream.stop_stream()
    stream.close()
    p.terminate()
    plt.close(fig)
    bus.shutdown()  # Shutdown the CAN bus