from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
from task_rc5 import RC5Task
from isaacsim.robot_motion.motion_generation import ArticulationKinematicsSolver, LulaKinematicsSolver
from isaacsim.core.utils.rotations import euler_angles_to_quat

from API.rc_api import RobotApi

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
        
    def create_IK_solver(self, robot):
        kinematics_solver = LulaKinematicsSolver(
            robot_description_path="controller_configs/rc5_official_urdf.yaml",
            urdf_path="assets/urdf_rc5/Robot.urdf",
        )
        end_effector_name = "body6"
        articulation_kinematics_solver = ArticulationKinematicsSolver(robot, kinematics_solver, end_effector_name)
        return kinematics_solver, articulation_kinematics_solver
    
    def setup_trajectory(self):
        # точки в метрах (оригинальные значения)
        p1 = (0.374, 0.485, 0.068)   
        p2 = (0.046, 0.665, 0.168) 
        p_home = (0.122, 0.316, 0.385)
        rot = (-180., 0., -45.)
        
        # Сохранение точек траектории в формате (position, rotation)
        self.trajectory_points = [
            (np.array(p_home), np.array(rot)),  # Точка 0 - дом
            (np.array(p1), np.array(rot)),      # Точка 1
            (np.array(p2), np.array(rot)),      # Точка 2
            (np.array(p_home), np.array(rot))   # Точка 3 - возврат домой
        ]
        
        # Начальная точка для возврата
        self.return_home_point = (np.array(p_home), np.array(rot))
        
        print("Траектория настроена:")
        for i, (position, rotation) in enumerate(self.trajectory_points):
            print(f"Точка {i}: позиция {position} м, вращение {rotation} градусов")
    
    def connect_real_robot(self):
        try:
            self.real_robot.controller_state.set('run', await_sec=120)
            self.real_robot_connected = True
            print("Реальный робот успешно подключен")
            
        except Exception as e:
            print(f"Ошибка подключения к реальному роботу: {e}")
            self.real_robot_connected = False
    
    def format_point_for_real_robot(self, point_index):
        """Форматирование точки для реального робота"""
        if point_index >= len(self.trajectory_points):
            return None
            
        position, rotation = self.trajectory_points[point_index]
        
        # Оставляем точки в метрах
        x, y, z = position
        rx, ry, rz = rotation
        
        formatted_point = [x, y, z, rx, ry, rz]
        print(f"Форматированная точка {point_index}: {formatted_point}")
        return formatted_point
    
    def check_sim_robot_reached_target(self, target_position: np.ndarray, tolerance=0.01):
        """Проверка, достиг ли симуляционный робот целевой позиции"""
        try:
            # Получаем текущую позицию концевика
            current_pose = self.aik.compute_end_effector_pose()
            if current_pose is None:
                return False
                
            current_position = current_pose[0]
            
            # Вычисляем расстояние до цели
            distance = np.linalg.norm(current_position - target_position)
            
            # Проверяем, находится ли в пределах допуска
            reached = distance <= tolerance
            
            # print(f"Расстояние до цели: {distance:.4f} м (допуск: {tolerance} м) - {'ДОСТИГНУТО' if reached else 'В ПРОЦЕССЕ'}")
                
            return reached
            
        except Exception as e:
            print(f"Ошибка проверки позиции: {e}")
            return False
    
    def move_real_robot_to_point(self, point_index):
        """Движение реального робота к одной точке"""
        if not self.real_robot_connected:
            print(f"Реальный робот не подключен - симулируем движение к точке {point_index}")
            time.sleep(3)
            return True
            
        try:
            formatted_point = self.format_point_for_real_robot(point_index)
            if not formatted_point:
                return False
            
            print(f"Реальный робот движется к точке {point_index}")
            
            # Очищаем предыдущие точки
            try:
                if hasattr(self.real_robot.motion.linear, 'clear_waypoints'):
                    self.real_robot.motion.linear.clear_waypoints()
            except:
                pass
            
            # Добавляем точку
            self.real_robot.motion.linear.add_new_waypoint(formatted_point)
            self.real_robot.motion.mode.set('move')
            
            print(f"Ожидание завершения движения к точке {point_index}...")
            
            # Простое ожидание завершения
            time.sleep(5)  # Ждем 5 секунд для завершения движения
            
            print(f"Реальный робот достиг точки {point_index}")
            return True
            
        except Exception as e:
            print(f"Ошибка при движении реального робота: {e}")
            return False
    
    def move_sim_robot_to_point(self, point_index, max_steps=150):
        """Движение симуляционного робота к одной точке"""
        try:
            target_position, rotation = self.trajectory_points[point_index]
            print(f"Симуляционный робот движется к точке {point_index}: {target_position}")
            
            steps_without_progress = 0
            last_distance = float('inf')
            min_distance = float('inf')
            
            for step in range(max_steps):
                # Применяем IK для целевой позиции
                success = self.control_sim_robot(target_position, rotation)
                
                if not success:
                    # Пробуем альтернативный подход
                    current_pose = self.aik.compute_end_effector_pose()
                    if current_pose is not None:
                        current_pos = current_pose[0]
                        # Пробуем промежуточную позицию
                        intermediate_pos = current_pos + (target_position - current_pos) * 0.5
                        success = self.control_sim_robot(intermediate_pos, rotation)
                
                # Шаг симуляции
                self.world.step(render=True)
                
                # Проверяем достижение цели
                if self.check_sim_robot_reached_target(target_position, 0.01):
                    print(f"Симуляционный робот достиг точки {point_index} за {step} шагов")
                    return True
                
                # Проверка прогресса
                current_pose = self.aik.compute_end_effector_pose()
                if current_pose is not None:
                    current_position = current_pose[0]
                    current_distance = np.linalg.norm(current_position - target_position)
                    
                    # Обновляем минимальное расстояние
                    min_distance = min(min_distance, current_distance)
                    
                    # Проверяем прогресс
                    if abs(current_distance - last_distance) < 0.001:
                        steps_without_progress += 1
                    else:
                        steps_without_progress = 0
                    
                    last_distance = current_distance
                    
                    # Вывод прогресса
                    if step % 30 == 0:
                        print(f"Шаг {step}: расстояние {current_distance:.4f} м")
                
                # Проверка застревания
                if steps_without_progress > 40:
                    print(f"Робот застрял на расстоянии {min_distance:.4f} м")
                    return min_distance <= 0.03  # Ослабленный критерий
                
                time.sleep(0.02)
            
            print(f"Достигнут лимит шагов. Минимальное расстояние: {min_distance:.4f} м")
            return min_distance <= 0.02
            
        except Exception as e:
            print(f"Ошибка движения симуляционного робота: {e}")
            return False
    
    def control_sim_robot(self, target_position: np.ndarray, rotation: np.ndarray):
        """Управление роботом в симуляции через IK"""
        try:
            # Получаем текущую позу робота
            robot_base_translation, robot_base_orientation = self.sim_robot.get_world_pose()
            self.ik.set_robot_base_pose(robot_base_translation, robot_base_orientation)
            
            # Конвертируем вращение в кватернион
            rot_quat = euler_angles_to_quat(rotation, degrees=True)
            
            # Вычисляем IK
            action, success = self.aik.compute_inverse_kinematics(target_position, rot_quat)
            
            if success:
                self.articulation_controller.apply_action(action)
                return True
            else:
                return False
                
        except Exception as e:
            print(f"Ошибка IK: {e}")
            return False
    
    def run_sequential_movement(self):
        """Основной цикл поочередного движения - ВСЕ В ОДНОМ ПОТОКЕ"""
        # Настройка траектории
        self.setup_trajectory()
        
        # Подключение к реальному роботу
        self.connect_real_robot()
        
        print("Запуск поочередного управления...")
        
        # Последовательное движение по точкам
        for point_index in range(len(self.trajectory_points)):
            if not self.is_running or not self.simulation_app.is_running():
                break
                
            print(f"\n" + "="*50)
            print(f"ОБРАБОТКА ТОЧКИ {point_index}")
            print("="*50)
            
            # ШАГ 1: Реальный робот движется к точке
            print("ШАГ 1: Движение реального робота...")
            real_success = self.move_real_robot_to_point(point_index)
            
            if real_success:
                print("Реальный робот завершил движение")
            else:
                print("Проблемы с реальным роботом")
            
            # Пауза между роботами
            time.sleep(1.0)
            
            # ШАГ 2: Симуляционный робот движется к точке
            print("ШАГ 2: Движение симуляционного робота...")
            sim_success = self.move_sim_robot_to_point(point_index)
            
            if sim_success:
                print("Симуляционный робот завершил движение")
            else:
                print("Проблемы с симуляционным роботом")
            
            # Обновляем текущий индекс
            self.current_point_index = point_index
            
            # Пауза перед следующей точкой
            if point_index < len(self.trajectory_points) - 1:
                print(f"Пауза перед следующей точкой...")
                time.sleep(1.0)
        
        print("\nВСЕ ТОЧКИ ВЫПОЛНЕНЫ!")
        
        # Возврат в домашнюю позицию
        self.return_to_home()
        
        # Завершение
        self.cleanup()
    
    def return_to_home(self):
        """Возврат обоих роботов в домашнюю позицию"""
        print("\nВозврат в домашнюю позицию...")
        
        # Реальный робот
        if self.real_robot_connected:
            try:
                print("Возврат реального робота...")
                self.move_real_robot_to_point(0)
                print("Реальный робот вернулся домой")
            except Exception as e:
                print(f"Ошибка возврата реального робота: {e}")
        
        # Симуляционный робот
        try:
            print("Возврат симуляционного робота...")
            self.move_sim_robot_to_point(0)
            print("Симуляционный робот вернулся домой")
        except Exception as e:
            print(f"Ошибка возврата симуляционного робота: {e}")
    
    def cleanup(self):
        """Очистка ресурсов"""
        print("Очистка ресурсов...")
        self.is_running = False
        
        if self.real_robot_connected:
            try:
                self.real_robot.controller_state.set('hold')
                print("Реальный робот переведен в режим HOLD")
            except Exception as e:
                print(f"Ошибка при остановке реального робота: {e}")
        
        time.sleep(3.0)
        if hasattr(self, 'simulation_app'):
            self.simulation_app.close()
            print("Симуляция закрыта")


def main():
    dual_controller = DualRobotController(simulation_app)
    
    try:
        dual_controller.run_sequential_movement()
    except KeyboardInterrupt:
        print("Прервано пользователем")
        dual_controller.cleanup()
    except Exception as e:
        print(f"Ошибка: {e}")
        dual_controller.cleanup()

if __name__ == "__main__":
    main()