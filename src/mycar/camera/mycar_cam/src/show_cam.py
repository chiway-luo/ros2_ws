#!/usr/bin/env python3
import cv2
#################################################################
#####################使用python3启动,不是ros2节点#################
#################################################################

dev = "/dev/video0"   # 也可以换成 /dev/v4l/by-id/xxx
cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    raise RuntimeError(f"打不开摄像头: {dev}")

while True:
    ret, frame = cap.read()
    if not ret:
        print("读帧失败，换 /dev/video1 或检查格式")
        break
    cv2.imshow("USB Camera", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
