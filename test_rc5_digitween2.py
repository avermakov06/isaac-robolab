from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
from task_rc5 import RC5Task
from isaacsim.robot_motion.motion_generation import ArticulationKinematicsSolver, LulaKinematicsSolver
from isaacsim.core.utils.rotations import euler_angles_to_quat

from API.rc_api import RobotApi

import threading
import time
from typing import List, Tuple

class DualRobotController:
    def __init__(self, simulation_app):
        # Инициализация симуляции
        
        self.simulation_app = simulation_app
        self.world = World(stage_units_in_meters=1.0)
        self.task = RC5Task()
        self.world.add_task(self.task)
        self.world.reset()
        
        # Получение RC5 из симуляции
        task_params = self.task.get_params()
        self.sim_robot = self.world.scene.get_object(task_params["robot_name"]["value"])
        self.articulation_controller = self.sim_robot.get_articulation_controller()
        
        # Инициализация IK решателя
        self.ik, self.aik = self.create_IK_solver(self.task._robot)
        
        # Инициализация реального RC5
        self.real_robot = RobotApi("10.10.10.10")
        self.real_robot_connected = False
        
        # Общие настройки
        self.trajectory_points = []
        self.return_home_point = None
        self.current_point_index = 0
        self.is_running = True
        self.real_robot_finished = False
        self.sim_target_position = None
        self.sim_target_rotation = None
        
    def to_meters(self, p):
        return tuple(x * 0.001 for x in p)
    
    def to_mm(self, p):
        return tuple(x * 1000 for x in p)
    
    def create_IK_solver(self, robot):
        kinematics_solver = LulaKinematicsSolver(
            robot_description_path="controller_configs/rc5_official_urdf.yaml",
            urdf_path="assets/urdf_rc5/Robot.urdf",
        )
        end_effector_name = "body6"
        articulation_kinematics_solver = ArticulationKinematicsSolver(robot, kinematics_solver, end_effector_name)
        return kinematics_solver, articulation_kinematics_solver
    
    def setup_trajectory(self):
        # точки в мм
        p1 = (-374, 485, 68)
        p2 = (-46, 565, 68)
        p_home = (122., 316., 385.)
        rot = (-180., 0., -45.)
        
        # Конвертация в метры и настройка
        p1_m = self.to_meters(p1)
        p2_m = self.to_meters(p2)
        p_home_m = self.to_meters(p_home)
        
        # Сохранение точек траектории
        self.trajectory_points = [
            p_home_m + rot,
            p1_m + rot,
            p2_m + rot
        ]
        
        # Начальная точка для возврата
        self.return_home_point = p_home_m + rot
        
        print("Траектория настроена:")
        for i, point in enumerate(self.trajectory_points):
            print(f"Точка {i}: позиция {point[:3]} м, вращение {point[3:]} градусов")
    
    def connect_real_robot(self):
        try:
            self.real_robot.controller_state.set('run', await_sec=120)
            self.real_robot_connected = True
            print("Реальный робот успешно подключен")
            
            
        except Exception as e:
            print(f"Ошибка подключения к реальному роботу: {e}")
            self.real_robot_connected = False
    
    def format_point_for_real_robot(self, point):
        """Форматирование точки для реального робота"""
        # point содержит: (x, y, z, rx, ry, rz) в метрах и градусах
        # Преобразуем позицию в мм, вращение оставляем в градусах
        x_mm, y_mm, z_mm = self.to_mm(point[:3])
        rx, ry, rz = point[3:]
        
        # Формируем точку в правильном формате для API
        # Возможно нужно: [x, y, z, rx, ry, rz] или другой формат
        formatted_point = [x_mm, y_mm, z_mm, rx, ry, rz]
        print(f"Форматированная точка: {formatted_point}")
        return formatted_point
    
    def check_robot_reachability(self, point):
        """Проверка достижимости точки"""
        try:
            
            # Пробуем вычислить IK для точки
            result = self.real_robot.motion.check_reachability(point)
            print(f"Точка достижима: {result}")
            return result
            
        except Exception as e:
            print(f"Ошибка проверки достижимости: {e}")
            return False
    
    def control_real_robot(self):
        """Управление реальным роботом в отдельном потоке"""
        if not self.real_robot_connected:
            return
            
        try:
            # Добавляем точки траектории
            for point in self.trajectory_points:
                self.real_robot.motion.linear.add_new_waypoint(point)
            
            # Запускаем движение
            self.real_robot.motion.mode.set('move')
            
            # Ожидаем завершения каждой точки
            for i in range(len(self.trajectory_points)):
                self.real_robot.motion.wait_waypoint_completion(i)
                self.current_point_index = i
                time.sleep(3)  # Небольшая пауза между точками
                
        except Exception as e:
            print(f"Ошибка управления реальным роботом: {e}")
    
    def return_real_robot_to_home(self):
        """Возврат реального робота в начальную позицию"""
        if not self.real_robot_connected:
            return
            
        try:
            print("Возврат реального робота в начальную позицию...")
                       
            # Добавляем точку возврата домой
            home_point_formatted = self.format_point_for_real_robot(self.return_home_point)
            print(f"Точка возврата: {home_point_formatted}")
            
            self.real_robot.motion.linear.add_new_waypoint(home_point_formatted)
            
            # Запускаем движение
            self.real_robot.motion.mode.set('move')
            
            # Ожидаем завершения
            self.real_robot.motion.wait_waypoint_completion(0)
            print("Реальный робот вернулся в начальную позицию")
            
        except Exception as e:
            print(f"Ошибка при возврате реального робота: {e}")
    
    def control_sim_robot(self, target_position: Tuple[float, float, float], rotation: Tuple[float, float, float]):
        """Управление роботом в симуляции через IK"""
        try:
            # Получаем текущую позу робота
            robot_base_translation, robot_base_orientation = self.sim_robot.get_world_pose()
            self.ik.set_robot_base_pose(robot_base_translation, robot_base_orientation)
            
            # Конвертируем вращение в кватернион
            rot_quat = euler_angles_to_quat(np.array(rotation), degrees=True)
            
            # Вычисляем IK
            action, success = self.aik.compute_inverse_kinematics(target_position, rot_quat)
            
            if success:
                self.articulation_controller.apply_action(action)
                return True
            else:
                print("IK решение не найдено")
                return False
                
        except Exception as e:
            print(f"Ошибка управления симуляционным роботом: {e}")
            return False
    
    def return_sim_robot_to_home(self):
        """Возврат симуляционного робота в начальную позицию"""
        try:
            print("Возврат симуляционного робота в начальную позицию...")
            
            if self.return_home_point is not None:
                target_position = np.array(self.return_home_point[:3])
                rotation = self.return_home_point[3:]
                
                # Плавно перемещаем робота домой
                for step in range(100):
                    success = self.control_sim_robot(target_position, rotation)
                    if success:
                        self.world.step(render=True)
                        time.sleep(0.01)
                    if step % 20 == 0:
                        print(f"Шаг возврата {step}/100")
                
                print("Симуляционный робот вернулся в начальную позицию")
                
        except Exception as e:
            print(f"Ошибка при возврате симуляционного робота: {e}")
    
    def get_sim_target_from_real(self, real_point_index: int) -> Tuple[np.ndarray, np.ndarray]:
        """Преобразование точки реального робота в целевую позицию для симуляции"""
        if real_point_index < len(self.trajectory_points):
            point = self.trajectory_points[real_point_index]
            # Разделяем позицию и вращение
            target_position = np.array(point[:3])
            rotation = point[3:]
            return target_position, rotation
        return None, None
    
    def run_synchronized(self):
        """Основной цикл синхронизированного управления"""
        # Настройка траектории
        self.setup_trajectory()
        
        # Подключение к реальному роботу
        self.connect_real_robot()
        
        # Запуск реального робота в отдельном потоке
        real_robot_thread = None
        if self.real_robot_connected:
            real_robot_thread = threading.Thread(target=self.control_real_robot)
            real_robot_thread.daemon = True
            real_robot_thread.start()
            print("Поток управления реальным роботом запущен")
        
        # Основной цикл симуляции
        i = 0
        reset_needed = False
        
        print("Запуск синхронизированного управления...")
        
        while self.simulation_app.is_running() and self.is_running:
            self.world.step(render=True)
            
            # Обработка состояний симуляции
            if self.world.is_stopped() and not reset_needed:
                reset_needed = True
            if self.world.is_playing():
                if reset_needed:
                    self.world.reset()
                    reset_needed = False
                
                # Получаем текущую целевую позицию из траектории реального робота
                target_position, rotation = self.get_sim_target_from_real(self.current_point_index)
                
                if target_position is not None:
                    # Сохраняем текущую цель для плавного движения
                    self.sim_target_position = target_position
                    self.sim_target_rotation = rotation
                    
                    # Управляем симуляционным роботом
                    success = self.control_sim_robot(target_position, rotation)
                    
                    if not success:
                        print(f"Не удалось достичь точки {self.current_point_index}")
                
                # Плавное движение к цели даже если реальный робот уже ушел дальше
                elif self.sim_target_position is not None and not self.real_robot_finished:
                    success = self.control_sim_robot(self.sim_target_position, self.sim_target_rotation)
                
                i += 1
                
                # Проверка завершения траектории
                if (self.real_robot_connected and 
                    self.real_robot_finished and 
                    self.current_point_index >= len(self.trajectory_points) - 1):
                    print("Траектория завершена, начинаем возврат в начальную позицию...")
                    self.return_both_robots_to_home()
                    self.is_running = False
            
            # Небольшая пауза для снижения нагрузки и лучшей синхронизации
            time.sleep(0.01)  # Уменьшил паузу для более плавной симуляции
        
        # Завершение
        self.cleanup()
    
    def return_both_robots_to_home(self):
        """Возврат обоих роботов в начальную позицию"""
        print("Возврат обоих роботов в начальную позицию...")
        
        # Возврат реального робота
        if self.real_robot_connected:
            return_thread = threading.Thread(target=self.return_real_robot_to_home)
            return_thread.daemon = True
            return_thread.start()
        
        # Возврат симуляционного робота
        self.return_sim_robot_to_home()
    
    def cleanup(self):
        """Очистка ресурсов"""
        print("Очистка ресурсов...")
        self.is_running = False
        
        # Возврат в начальную позицию если еще не сделано
        if not self.real_robot_finished or self.current_point_index < len(self.trajectory_points) - 1:
            self.return_both_robots_to_home()
        
        if self.real_robot_connected:
            # Остановка реального робота
            try:
                self.real_robot.controller_state.set('hold')
                print("Реальный робот переведен в режим HOLD")
            except Exception as e:
                print(f"Ошибка при остановке реального робота: {e}")
        
        time.sleep(15.0)
        # Закрытие симуляции
        if hasattr(self, 'simulation_app'):
            self.simulation_app.close()
            print("Симуляция закрыта")


def main():
    # Создание и запуск двойного контроллера
    dual_controller = DualRobotController(simulation_app)
    
    try:
        dual_controller.run_synchronized()
    except KeyboardInterrupt:
        print("Прервано пользователем")
        dual_controller.cleanup()
    except Exception as e:
        print(f"Ошибка: {e}")
        dual_controller.cleanup()

if __name__ == "__main__":
    main()