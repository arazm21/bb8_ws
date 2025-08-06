from ultralytics import YOLO

# from path
#path = '../models/yolov8n.pt'
model = YOLO('yolov8n.pt')

# Export the model to NCNN format
model.export(format="ncnn", imgsz=320)  # creates 'yolov8s_ncnn_model'