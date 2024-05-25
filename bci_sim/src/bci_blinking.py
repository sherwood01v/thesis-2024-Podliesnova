#!/usr/bin/python3
import rospy
import collections
import numpy as np
from scipy.signal import detrend
from joblib import load
from pyOpenBCI import OpenBCICyton

WINDOW_SIZE_SAMPLES = 102
WAIT_TIME_SAMPLES = 10

model = load('path_to_model')

buffer = collections.deque(maxlen=WINDOW_SIZE_SAMPLES)

def process_eeg_window(window_data):
    data = np.array(detrend(window_data)).reshape(1, -1)
    prediction = model.predict(data)
    print(prediction)

def eeg_callback(sample):
    channel_data = sample.channels_data[0] #from first channel

    buffer.append(channel_data)

    if len(buffer) == WINDOW_SIZE_SAMPLES:
        process_eeg_window(list(buffer))

        for _ in range(WAIT_TIME_SAMPLES):
            buffer.popleft()

if __name__ == '__main__':
    rospy.init_node('openbci_blikning_node', anonymous=True)

    port = '/dev/ttyUSB0'
    board = OpenBCICyton(port=port, daisy=True)

    print("Starting stream...")
    board.start_stream(eeg_callback)

    rospy.spin()