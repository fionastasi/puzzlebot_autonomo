from ultralytics import YOLO
import torch

print(torch.cuda.is_available())   # Debe salir True
print(torch.cuda.device_count())   # Cuántas GPUs reconoce
print(torch.cuda.get_device_name(0))  # Nombre de la GPU 0


# Load a model
model = YOLO("yolov8n.yaml")  # build a new model from scratch
model = YOLO("yolov8n.pt")  # load a pretrained model (recommended for training)

# Use the model
results = model.train(data="config2.yaml", epochs=80, cfg = 'default_copy.yaml')  # train the model