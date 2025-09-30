from ultralytics import YOLO

# from path
#path = '../models/yolov8n.pt'
model = YOLO('yolov10n.pt')

# Export the model to NCNN format
model.export(format="ncnn", imgsz=320)  # creates 'yolovXZ_ncnn_model'