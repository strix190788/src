import cv2
import numpy as np
import glob
import json

class CameraCalibrator:
    def __init__(self, board_size=(8, 5), square_size=0.025):
        """
        board_size: количество внутренних углов (width, height)
        square_size: размер квадрата в метрах
        """
        self.board_size = board_size
        self.square_size = square_size
        
        # Подготавливаем "идеальные" 3D координаты углов
        # (0,0,0), (1,0,0), (2,0,0)...
        self.objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)
        self.objp *= square_size
        
        # Списки для хранения точек
        self.objpoints = []  # 3D точки в мировых координатах
        self.imgpoints = []  # 2D точки в координатах изображения
        
        self.image_size = None
        
    def process_images(self, image_pattern):
        """Обрабатываем все калибровочные изображения"""
        images = glob.glob(image_pattern)
        
        if len(images) == 0:
            print(f"Не найдено изображений по шаблону: {image_pattern}")
            return False
        
        print(f"Найдено {len(images)} изображений")
        
        successful = 0
        for fname in images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            if self.image_size is None:
                self.image_size = gray.shape[::-1]
            
            # Ищем углы шахматной доски
            flags = (cv2.CALIB_CB_ADAPTIVE_THRESH + 
                    cv2.CALIB_CB_NORMALIZE_IMAGE +
                    cv2.CALIB_CB_FAST_CHECK)
            
            ret, corners = cv2.findChessboardCorners(gray, self.board_size, flags)
            
            if ret:
                # Уточняем положение углов с субпиксельной точностью
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 
                           30, 0.001)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                
                self.objpoints.append(self.objp)
                self.imgpoints.append(corners2)
                
                successful += 1
                print(f"  ✓ {fname}")
            else:
                print(f"  ✗ {fname} — доска не найдена")
        
        print(f"\nУспешно обработано: {successful}/{len(images)}")
        return successful >= 10  # Минимум 10 изображений для хорошей калибровки
    
    def calibrate(self):
        """Выполняем калибровку"""
        if len(self.objpoints) < 10:
            print("Недостаточно изображений! Нужно минимум 10.")
            return None
        
        print("\nКалибрую камеру...")
        
        # Главная функция калибровки
        ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
            self.objpoints,
            self.imgpoints,
            self.image_size,
            None, None
        )
        
        if ret:
            print("\n" + "="*60)
            print("КАЛИБРОВКА УСПЕШНА!")
            print("="*60)
            
            print(f"\nОшибка репроекции: {ret:.4f} пикселей")
            print("(Хорошо если < 0.5, отлично если < 0.2)")
            
            print(f"\nIntrinsic Matrix K:")
            print(f"  fx = {K[0,0]:.2f}")
            print(f"  fy = {K[1,1]:.2f}")
            print(f"  cx = {K[0,2]:.2f}")
            print(f"  cy = {K[1,2]:.2f}")
            
            print(f"\nDistortion coefficients:")
            print(f"  k1 = {dist[0,0]:.6f}")
            print(f"  k2 = {dist[0,1]:.6f}")
            print(f"  p1 = {dist[0,2]:.6f}")
            print(f"  p2 = {dist[0,3]:.6f}")
            print(f"  k3 = {dist[0,4]:.6f}")
            
            return {
                'reprojection_error': ret,
                'camera_matrix': K,
                'distortion_coeffs': dist,
                'image_size': self.image_size
            }
        else:
            print("Калибровка не удалась!")
            return None
    
    def save_calibration(self, calibration, filename='camera_calibration.json'):
        """Сохраняем результаты в файл"""
        data = {
            'image_width': self.image_size[0],
            'image_height': self.image_size[1],
            'camera_matrix': calibration['camera_matrix'].tolist(),
            'distortion_coefficients': calibration['distortion_coeffs'].tolist(),
            'reprojection_error': calibration['reprojection_error']
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        
        print(f"\nКалибровка сохранена в {filename}")
        
    def save_calibration_yaml(self, calibration, filename='camera.yaml'):
        """Сохраняем в YAML формате для ROS"""
        K = calibration['camera_matrix']
        D = calibration['distortion_coeffs']
        
        yaml_content = f"""image_width: {self.image_size[0]}
image_height: {self.image_size[1]}
camera_name: robot_camera
camera_matrix:
  rows: 3
  cols: 3
  data: [{K[0,0]}, {K[0,1]}, {K[0,2]}, {K[1,0]}, {K[1,1]}, {K[1,2]}, {K[2,0]}, {K[2,1]}, {K[2,2]}]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [{D[0,0]}, {D[0,1]}, {D[0,2]}, {D[0,3]}, {D[0,4]}]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: [{K[0,0]}, 0, {K[0,2]}, 0, 0, {K[1,1]}, {K[1,2]}, 0, 0, 0, 1, 0]
"""
        with open(filename, 'w') as f:
            f.write(yaml_content)
        
        print(f"ROS калибровка сохранена в {filename}")


def main():
    # Создаём калибратор
    # Внимание: размер квадрата в МЕТРАХ!
    calibrator = CameraCalibrator(
        board_size=(8, 5),      # 8×5 внутренних углов
        square_size=0.025       # 25 мм = 0.025 м
    )
    
    # Обрабатываем изображения
    if calibrator.process_images('calibration_images/*.png'):
        # Калибруем
        result = calibrator.calibrate()
        
        if result:
            # Сохраняем
            calibrator.save_calibration(result)
            calibrator.save_calibration_yaml(result)


if __name__ == '__main__':
    main()