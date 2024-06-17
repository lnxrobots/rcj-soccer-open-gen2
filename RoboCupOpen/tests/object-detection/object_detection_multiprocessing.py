from pickletools import uint8
import cv2
import numpy as np
import multiprocessing as mp
from multiprocessing import shared_memory
import time

WIDTH = 640
HEIGHT = 480
FPS = 60
OUT_FPS = 30

def read_camera(frame):
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)
    start = time.time()
    while True:
        ret, fr = cap.read()
        if not ret:
            continue
        # print(1/(time.time()-start))
        # start = time.time()
        np.copyto(frame, fr)

def compute(frame):
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('output.avi', fourcc, OUT_FPS, (WIDTH, HEIGHT))
    start = time.time()
    while(True):
        if frame is None:
            continue
        print(1/OUT_FPS - (time.time()-start))
        start = time.time()
        res = frame
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Threshold the HSV image to get only wanted colors
        mask = cv2.inRange(hsv, lower_c, upper_c)

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            thresh = frame.copy()
            res = frame.copy()
            thresh[mask == 255] = (255, 255, 255)
            res = cv2.addWeighted(frame, 0.7, thresh, 0.3, 0, res)

            max_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(max_contour)
            cv2.rectangle(res, (x, y), (x+w, y+h), (0, 255, 0), 2)
            # print(x, y)
        out.write(res)
        time.sleep(max(1/OUT_FPS - (time.time()-start), 0))


lower_c = np.array([0, 170, 80])
upper_c = np.array([60, 255, 180])

shm = shared_memory.SharedMemory(create=True, size=WIDTH*HEIGHT*3)
frame = np.ndarray((HEIGHT, WIDTH, 3), dtype=np.uint8, buffer=shm.buf)

p_read = mp.Process(target=read_camera, args=(frame,))
p_compute = mp.Process(target=compute, args=(frame,))

p_read.start()
p_compute.start()

p_read.join()
p_compute.join()
