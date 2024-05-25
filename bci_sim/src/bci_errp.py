#!/usr/bin/python3
import rospy
import collections
import numpy as np
from scipy.signal import detrend, butter, filtfilt, decimate, lfilter, resample
from joblib import load
from pyOpenBCI import OpenBCICyton
from sklearn.preprocessing import StandardScaler

WINDOW_SIZE_MS = 800
WAIT_TIME_MS = 120
SAMPLE_RATE = 125
# WINDOW_SIZE_SAMPLES = int((WINDOW_SIZE_MS / 1000) * SAMPLE_RATE)
# WAIT_TIME_SAMPLES = int((WAIT_TIME_MS / 1000) * SAMPLE_RATE)
WINDOW_SIZE_SAMPLES = 100
WAIT_TIME_SAMPLES = 15

model = load('path_to_model')
# model = load('path_to_model')

buffer = collections.deque(maxlen=WINDOW_SIZE_SAMPLES)
last_predictions = collections.deque(maxlen=9)
k = 0
rospy.set_param('risky', 'normal')
rospy.set_param('stop', 1)

def lowpass(sig, fc, fs, butter_filt_order=4):
    B, A = butter(butter_filt_order, np.array(fc)/(fs/2), btype='low')
    return lfilter(B, A, sig, axis=0)

def butter_bandpass_filter(data, lowcut, highcut, fs, order=5):
    nyquist = 0.5 * fs
    low = lowcut / nyquist
    high = highcut / nyquist
    b, a = butter(order, [low, high], btype='band')
    return filtfilt(b, a, data)

def process_eeg_window(window_data):
    global k
    #bandpass filter
    filtered_segments = np.apply_along_axis(butter_bandpass_filter, 0, window_data, 1, 10, SAMPLE_RATE)

    #normalization
    normalized_segments = filtered_segments - np.mean(filtered_segments, axis=0)

    #preparation of data for prediction
    scaler = StandardScaler()
    data = scaler.fit_transform(normalized_segments.reshape(-1, 16)).reshape(1, 100, 16)
    prediction = model.predict(data)
    last_predictions.append(prediction)
    print((prediction > 0.5).astype(int).flatten())
    rospy.set_param('stop', int((prediction > 0.5).astype(int).flatten()[0]))

def eeg_callback(sample):
    channel_data = sample.channels_data

    buffer.append(channel_data)

    if len(buffer) == WINDOW_SIZE_SAMPLES:
        process_eeg_window(list(buffer))
        for _ in range(WAIT_TIME_SAMPLES):
            buffer.popleft()

if __name__ == '__main__':
    rospy.init_node('openbci_errp_node', anonymous=True)

    port = '/dev/ttyUSB0'
    board = OpenBCICyton(port=port, daisy=True)

    print("Starting stream...")
    board.start_stream(eeg_callback)

    rospy.spin()