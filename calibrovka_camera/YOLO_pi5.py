#!/usr/bin/env python3
"""Оптимизированный YOLO для Raspberry Pi 5"""

from ultralytics import YOLO
import cv2
import time

class OptimizedYoloRPi:
    """Оптимизированный детектор для Raspberry Pi"""
    
    def __init__(self):
        # Используем самую лёгкую модель
        self.model = YOLO('yolov8n.pt')
        
        # Уменьшаем разрешение для скорости
        self.input_size = 320  # вместо 640
        
        # Снижаем порог уверенности
        self.confidence = 0.4
        
        # Ограничиваем классы (опционально)
        # self.classes = [0, 1, 2]  # только person, bicycle, car
        
    def detect(self, frame):
        """Детекция с оптимизациями"""
        # Ресайз для скорости
        h, w = frame.shape[:2]
        scale = self.input_size / max(h, w)
        
        if scale < 1:
            new_w = int(w * scale)
            new_h = int(h * scale)
            frame_small = cv2.resize(frame, (new_w, new_h))
        else:
            frame_small = frame
            scale = 1
        
        # Детекция
        results = self.model(
            frame_small,
            conf=self.confidence,
            verbose=False,
            imgsz=self.input_size
        )
        
        # Масштабируем координаты обратно
        detections = []
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                
                # Масштабируем координаты
                x1 /= scale
                y1 /= scale
                x2 /= scale
                y2 /= scale
                
                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]
                confidence = float(box.conf[0])
                
                detections.append({
                    'class': class_name,
                    'confidence': confidence,
                    'bbox': [int(x1), int(y1), int(x2), int(y2)]
                })
        
        return detections
    
    def draw_detections(self, frame, detections):
        """Рисуем детекции на изображении"""
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            label = f"{det['class']} {det['confidence']:.2f}"
            
            # Рисуем бокс
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Рисуем подпись
            cv2.putText(frame, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        return frame


def benchmark_rpi():
    """Бенчмарк для Raspberry Pi"""
    detector = OptimizedYoloRPi()
    
    # Тестовое изображение
    test_frame = cv2.imread('test_image.jpg')
    if test_frame is None:
        test_frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    
    # Прогрев
    for _ in range(5):
        detector.detect(test_frame)
    
    # Бенчмарк
    times = []
    for _ in range(20):
        start = time.time()
        detections = detector.detect(test_frame)
        elapsed = time.time() - start
        times.append(elapsed)
    
    avg_time = sum(times) / len(times)
    fps = 1.0 / avg_time
    
    print(f"""
╔══════════════════════════════════════════════════════════╗
║             YOLO Benchmark на Raspberry Pi               ║
╠══════════════════════════════════════════════════════════╣
║  Модель: YOLOv8n                                         ║
║  Входное разрешение: {detector.input_size}×{detector.input_size}                          ║
║  Среднее время: {avg_time*1000:.1f} мс                                   ║
║  FPS: {fps:.1f}                                                ║
╚══════════════════════════════════════════════════════════╝
    """)


if __name__ == '__main__':
    import numpy as np
    benchmark_rpi()