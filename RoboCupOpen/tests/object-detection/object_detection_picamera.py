import os
import cv2
import numpy as np
import time
from picamera2 import Picamera2

WIDTH = 820
HEIGHT = 616
FPS = 30

lower_c = np.array([0, 120, 120])
upper_c = np.array([60, 255, 255])

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi', fourcc, FPS, (WIDTH, HEIGHT))

start = 0

tuning = Picamera2.load_tuning_file(os.path.join(os.getcwd(), 'imx219_without_ct.json'))
picam2 = Picamera2(tuning=tuning)
config = picam2.create_video_configuration(
    main={"format": 'RGB888', "size": (WIDTH, HEIGHT)},
    controls={"FrameDurationLimits": (1000000//FPS, 1000000//FPS)}
)
picam2.configure(config)
picam2.start()

while (True):
    start = time.time()
    # Take a frame
    frame = picam2.capture_array()
    res = frame
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Threshold the HSV image to get only wanted colors
    mask = cv2.inRange(hsv, lower_c, upper_c)

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        # thresh = frame.copy()
        # res = frame.copy()
        # thresh[mask == 255] = (255, 255, 255)
        # res = cv2.addWeighted(frame, 0.7, thresh, 0.3, 0, res)

        max_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(max_contour)
        cv2.rectangle(res, (x, y), (x+w, y+h), (0, 255, 0), 2)

    # cv2.imshow('window', res)
    out.write(res)

    if cv2.waitKey(33) == 27:
        break

    print(1/(time.time()-start))

out.release()
cv2.destroyAllWindows()
