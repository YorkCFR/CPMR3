Note: must install ultralytics
pip3 install -U ultralytics

opencv_camera will publish from /dev/video0 to /mycamera/image_raw
yolo_pose runs the yolov8 pose detector (and displays what it is capturing in a window). For each frame with only one person in it you will receive a collection of body parts and their locations in rows and columns.
