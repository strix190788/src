#!/usr/bin/env python3
"""Детекция объектов в реальном времени с веб-камеры"""

from ultralytics import YOLO
import cv2
import time

class RealtimeDetector:
    def __init__(self, model_name='yolov8n.pt', confidence_threshold=0.5):
        self.model = YOLO(model_name)
        self.confidence_threshold = confidence_threshold
        
    def detect_from_camera(self, camera_index=0):
        """Запуск детекции с камеры"""
        cap = cv2.VideoCapture(camera_index)
        
        if not cap.isOpened():
            print("Не удалось открыть камеру!")
            return
        
        print("Нажмите 'q' для выхода")
        
        fps_counter = 0
        fps_start_time = time.time()
        fps = 0
        
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            # Детекция
            results = self.model(frame, conf=self.confidence_threshold, verbose=False)
            
            # Рисуем результаты
            annotated = results[0].plot()
            
            # Считаем FPS
            fps_counter += 1
            if time.time() - fps_start_time >= 1.0:
                fps = fps_counter
                fps_counter = 0
                fps_start_time = time.time()
            
            # Добавляем FPS на изображение
            cv2.putText(annotated, f"FPS: {fps}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Добавляем количество объектов
            num_objects = len(results[0].boxes)
            cv2.putText(annotated, f"Objects: {num_objects}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            cv2.imshow('YOLO Detection', annotated)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    detector = RealtimeDetector(
        model_name='yolov8n.pt',
        confidence_threshold=0.5
    )
    detector.detect_from_camera()