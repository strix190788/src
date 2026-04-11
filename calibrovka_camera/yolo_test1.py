from ultralytics import YOLO
import cv2
from picamera2 import Picamera2, Preview
import time

picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"format": "BGR888", "size": (640, 480)})
picam2.configure(config)
picam2.start()
time.sleep(1)

while True:
    frame = picam2.capture_array()
    print("FRAME", frame.shape)
    break

picam2.stop()

# Загружаем предобученную модель
# При первом запуске скачает веса (~6 MB для nano)
model = YOLO('yolov8n.pt') # nano — самая лёгкая
# Детекция на изображении
results = model(frame)
# Показываем результат
for result in results:
  # Получаем изображение с нарисованными боксами
  annotated = result.plot()

  # Сохраняем
  cv2.imwrite('detection_result.jpg', annotated)

  # Выводим информацию о найденных объектах
  boxes = result.boxes
  for box in boxes:
    # Координаты
    x1, y1, x2, y2 = box.xyxy[0].tolist()# Уверенность
    confidence = box.conf[0].item()

    # Класс
    class_id = int(box.cls[0].item())
    class_name = model.names[class_id]

    print(f"Найден: {class_name}")
    print(f" Координаты: ({x1:.0f}, {y1:.0f}) - ({x2:.0f}, {y2:.0f})")
    print(f" Уверенность: {confidence:.2%}")
    print()
print("Результат сохранён: detection_result.jpg")
