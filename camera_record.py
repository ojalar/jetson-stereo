# MIT License
# Copyright (c) 2019-2022 JetsonHacks, 2022 Risto Ojala

# Script for recording calibration data of stereo CSI cameras on Jetson Nano
# Adapted from the Jetson Nano CSI python example by JetsonHacks
# https://github.com/JetsonHacksNano/CSI-Camera/blob/master/simple_camera.py 

import cv2
import time
import numpy as np

""" 
gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
Flip the image by setting the flip_method (most common values: 0 and 2)
display_width and display_height determine the size of each camera pane in the window on the screen
"""

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1640,
    capture_height=1232,
    display_width=1640,
    display_height=1232,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d !"
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


def main():
    # open videostreams from both cameras
    video_capture_0 = cv2.VideoCapture(gstreamer_pipeline(sensor_id=0), cv2.CAP_GSTREAMER)
    video_capture_1 = cv2.VideoCapture(gstreamer_pipeline(sensor_id=1), cv2.CAP_GSTREAMER)
    # check if both streams opened fine
    if video_capture_0.isOpened() and video_capture_1.isOpened():
        try:
            # initialise timestamp
            timestamp = time.time()
            # loop until user exits by pressing 'q'
            while True:
                # capture frames
                ret_0, frame_0 = video_capture_0.read()
                ret_1, frame_1 = video_capture_1.read()
                # exit if problem in frame acquisition
                if ret_0 == False or ret_1 == False:
                    break
                
                # prevent buffering by only displaying images once a second
                if time.time() < timestamp + 1:
                    continue
                # record timestamp for displaying
                timestamp = time.time()
                
                # resize and format frames for displaying
                frame_0_rs = cv2.resize(frame_0, (640, 480))
                frame_1_rs = cv2.resize(frame_1, (640, 480))
                frame_show = np.concatenate((frame_0_rs, frame_1_rs), axis=1)
                # show frames for half a second
                cv2.imshow("stereo view", frame_show)
                key = cv2.waitKey(500) & 0xFF 
                # record frames if user presses 'c'
                if key == ord('c'):
                    cv2.imwrite(str(int(timestamp)) + "_camera0" + ".png", frame_0)
                    cv2.imwrite(str(int(timestamp)) + "_camera1" + ".png", frame_1)
                # exit program if user presses 'q'
                elif key == ord('q'):
                    break
        # prepare for abrupt exiting, close streams cleanly in case of errors/user terminates the program
        finally:
            video_capture_0.release()
            video_capture_1.release()
            cv2.destroyAllWindows()
    else:
        print("Error: Unable to open camera")

if __name__ == "__main__":
    main()
