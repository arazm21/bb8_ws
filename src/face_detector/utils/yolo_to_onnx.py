from ultralytics import YOLO
model = YOLO('/home/bb8/bb8_ws/src/face_detector/models/yolov8s.pt')
model.export(format='onnx', imgsz=320)
