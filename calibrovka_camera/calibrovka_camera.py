#!/usr/bin/env python3
"""Коррекция дисторсии изображений"""

import cv2
import numpy as np
import json

def load_calibration(filename='camera_calibration.json'):
    """Загружаем калибровку из файла"""
    with open(filename, 'r') as f:
        data = json.load(f)
    
    K = np.array(data['camera_matrix'])
    D = np.array(data['distortion_coefficients'])
    image_size = (data['image_width'], data['image_height'])
    
    return K, D, image_size


def undistort_image(image, K, D):
    """Убираем дисторсию с изображения"""
    h, w = image.shape[:2]
    
    # Получаем оптимальную новую матрицу камеры
    # alpha=0 — все пиксели валидны (могут быть чёрные края)
    # alpha=1 — сохраняем все исходные пиксели
    new_K, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
    
    # Убираем дисторсию
    undistorted = cv2.undistort(image, K, D, None, new_K)
    
    # Обрезаем по ROI (опционально)
    x, y, w, h = roi
    if w > 0 and h > 0:
        undistorted = undistorted[y:y+h, x:x+w]
    
    return undistorted


def compare_distortion(image, K, D):
    """Показываем сравнение до и после"""
    undistorted = undistort_image(image, K, D)
    
    # Ресайзим для сравнения
    h, w = image.shape[:2]
    undistorted_resized = cv2.resize(undistorted, (w, h))
    
    # Склеиваем горизонтально
    comparison = np.hstack([image, undistorted_resized])
    
    # Добавляем подписи
    cv2.putText(comparison, "Original", (10, 30), 
               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(comparison, "Undistorted", (w + 10, 30), 
               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    return comparison


# Пример использования
if __name__ == '__main__':
    # Загружаем калибровку
    K, D, image_size = load_calibration()
    
    # Загружаем изображение
    img = cv2.imread('test_image.jpg')
    
    if img is not None:
        comparison = compare_distortion(img, K, D)
        cv2.imwrite('distortion_comparison.png', comparison)
        print("Сравнение сохранено: distortion_comparison.png")