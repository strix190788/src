#!/usr/bin/env python3
"""Генерация шахматной доски для калибровки"""

import cv2
import numpy as np

def create_chessboard(squares_x=9, squares_y=6, square_size_mm=25, dpi=300):
    """
    Создаёт изображение шахматной доски для печати.
    
    squares_x, squares_y: количество КВАДРАТОВ (не углов!)
    square_size_mm: размер квадрата в мм
    dpi: разрешение печати
    """
    # Пиксели на квадрат
    px_per_mm = dpi / 25.4
    square_px = int(square_size_mm * px_per_mm)
    
    # Размер изображения
    width = squares_x * square_px
    height = squares_y * square_px
    
    # Создаём белое изображение
    board = np.ones((height, width), dtype=np.uint8) * 255
    
    # Рисуем чёрные квадраты
    for i in range(squares_y):
        for j in range(squares_x):
            if (i + j) % 2 == 0:
                x1 = j * square_px
                y1 = i * square_px
                x2 = x1 + square_px
                y2 = y1 + square_px
                board[y1:y2, x1:x2] = 0
    
    # Добавляем белую рамку
    border = 50
    bordered = np.ones((height + 2*border, width + 2*border), dtype=np.uint8) * 255
    bordered[border:-border, border:-border] = board
    
    return bordered

# Генерируем доску 9×6 квадратов (8×5 внутренних углов)
board = create_chessboard(squares_x=9, squares_y=6, square_size_mm=25)
cv2.imwrite('chessboard_9x6.png', board)
print("Шахматная доска сохранена: chessboard_9x6.png")
print("Напечатайте её на листе A4!")
print("Размер квадрата: 25мм")
print("Внутренних углов: 8×5")
