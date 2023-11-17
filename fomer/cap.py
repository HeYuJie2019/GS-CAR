import threading
import cv2
import global_value
import time
global_value._init()  #全局变量初始化，初始化字典
def capture_video(camera_index):
    cap = cv2.VideoCapture(camera_index)
    while cap.isOpened():
        ret, frame = cap.read()
        if camera_index == 0:
            global_value.set_value('frame_down', frame)
        elif camera_index == 2:
            global_value.set_value('frame_up', frame)
camera_thread0 = threading.Thread(target=capture_video, args=(0,))
camera_thread2 = threading.Thread(target=capture_video, args=(2,))
camera_thread2.start()
camera_thread0.start()
frame_down = global_value.get_value('frame_down')
frame_up = global_value.get_value('frame_up')
if frame_down is not None:
    cv2.imwrite('frame_down.jpg', frame_down)
    cv2.imwrite('frame_up.jpg', frame_up)