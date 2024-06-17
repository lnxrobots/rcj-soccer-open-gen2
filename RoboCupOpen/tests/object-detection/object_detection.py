import cv2
import numpy as np
import time

WIDTH = 640
HEIGHT = 480
FPS = 60

signatures = {
    'sig1': ((0, 147, 144), (60, 253, 255)),
    'sig2': ((90, 170, 120), (180, 255, 180))
}

start = 0

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output_opencv.avi', fourcc, FPS/2, (WIDTH, HEIGHT))

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
cap.set(cv2.CAP_PROP_FPS, FPS)

while(True):
    # Take a frame
    ret, frame = cap.read()
    if not ret:
        break
    start = time.time()

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Threshold the HSV image to get only wanted colors
    res = frame.copy()

    masks = []
    for name, limits in signatures.items():
        mask = cv2.inRange(hsv, *limits)
        masks.append(mask)

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        if contours:
            max_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(max_contour)
            cv2.rectangle(res, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(res, name, (x, y-5), cv2.FONT_HERSHEY_SIMPLEX , 0.5, (0, 255, 0), 1)

    # thresh = res.copy()
    # thresh[sum(masks) >= 255] = (255, 255, 255)
    # res = cv2.addWeighted(res, 0.7, thresh, 0.3, 0, res)

    out.write(res)

    print(1/(time.time()-start))

cap.release()
# out.release()
cv2.destroyAllWindows()
