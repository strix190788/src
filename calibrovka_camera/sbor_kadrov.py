#!/usr/bin/env python3
"""Сбор изображений для калибровки камеры"""

import cv2
import os
from datetime import datetime

class CalibrationImageCollector:
    def __init__(self, output_dir='calibration_images'):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        self.image_count = 0
        
        # Параметры шахматной доски
        self.board_size = (8, 5)  # Внутренние углы (squares - 1)
        
    def collect_from_camera(self, camera_index=0):
        """Собираем изображения с веб-камеры"""
        cap = cv2.VideoCapture(camera_index)
        
        if not cap.isOpened():
            print("Не удалось открыть камеру!")
            return
        
        print(f"""
╔══════════════════════════════════════════════════════════╗
║           СБОР КАЛИБРОВОЧНЫХ ИЗОБРАЖЕНИЙ                 ║
╠══════════════════════════════════════════════════════════╣
║                                                          ║
║   Инструкции:                                            ║
║   1. Держите шахматную доску перед камерой              ║
║   2. Нажмите ПРОБЕЛ для сохранения кадра                ║
║   3. Меняйте положение и угол доски                     ║
║   4. Соберите 15-20 разных ракурсов                     ║
║   5. Нажмите ESC для завершения                         ║
║                                                          ║
║   Советы:                                                ║
║   - Покройте всю площадь кадра (углы важны!)            ║
║   - Меняйте расстояние до камеры                        ║
║   - Наклоняйте доску под разными углами                 ║
║                                                          ║
╚══════════════════════════════════════════════════════════╝
        """)
        
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            # Пытаемся найти шахматную доску
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(
                gray, self.board_size,
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
            )
            
            # Рисуем результат
            display = frame.copy()
            if found:
                cv2.drawChessboardCorners(display, self.board_size, corners, found)
                status = "✓ Доска найдена! Нажмите ПРОБЕЛ"
                color = (0, 255, 0)
            else:
                status = "✗ Доска не найдена"
                color = (0, 0, 255)
            
            cv2.putText(display, status, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            cv2.putText(display, f"Собрано: {self.image_count}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
            cv2.imshow('Calibration', display)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord(' ') and found:
                # Сохраняем изображение
                filename = f"calib_{self.image_count:02d}.png"
                filepath = os.path.join(self.output_dir, filename)
                cv2.imwrite(filepath, frame)
                self.image_count += 1
                print(f"Сохранено: {filepath}")
                
            elif key == 27:  # ESC
                break
        
        cap.release()
        cv2.destroyAllWindows()
        print(f"\nСобрано {self.image_count} изображений в {self.output_dir}/")


if __name__ == '__main__':
    collector = CalibrationImageCollector()
    collector.collect_from_camera()