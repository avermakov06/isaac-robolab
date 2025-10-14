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
        # Initialization simulation
        
        self.simulation_app = simulation_app
        self.world = World(stage_units_in_meters=1.0)
        self.task = RC5Task()
        self.world.add_task(self.task)
        self.world.reset()
        
        # Getting RC5 from simulation
        task_params = self.task.get_params()
        self.sim_robot = self.world.scene.get_object(task_params["robot_name"]["value"])
        self.articulation_controller = self.sim_robot.get_articulation_controller()
        
        # Initialization IK solver
        self.ik, self.aik = self.create_IK_solver(self.task._robot)
        
        # Initialization real RC5
        self.real_robot = RobotApi("10.10.10.10")
        self.real_robot_connected = False
        
        # General settings
        self.trajectory_points = []
        self.current_point_index = 0
        self.is_running = True
        
    def to_meters(self, p):
        return tuple(x * 0.001 for x in p)
    
    def create_IK_solver(self, robot):
        kinematics_solver = LulaKinematicsSolver(
            robot_description_path="controller_configs/rc5_official_urdf.yaml",
            urdf_path="assets/urdf_rc5/Robot.urdf",
        )
        end_effector_name = "body6"
        articulation_kinematics_solver = ArticulationKinematicsSolver(robot, kinematics_solver, end_effector_name)
        return kinematics_solver, articulation_kinematics_solver
    
    def setup_trajectory(self):
        # points in mm
        p1 = (-374, 485, 68)
        p2 = (-46, 565, 68)
        p_home = (122., 316., 385.)
        rot = (-180., 0., -45.)
        
        # Conversion in meters and setting up
        p1_m = self.to_meters(p1)
        p2_m = self.to_meters(p2)
        p_home_m = self.to_meters(p_home)
        
        # Saving trajectory points
        self.trajectory_points = [
            p_home_m + rot,
            p1_m + rot,
            p2_m + rot
        ]
    
    def connect_real_robot(self):
        try:
            self.real_robot.controller_state.set('run', await_sec=120)
            self.real_robot_connected = True
            print("Реальный робот успешно подключен")
        except Exception as e:
            print(f"Ошибка подключения к реальному роботу: {e}")
            self.real_robot_connected = False
    
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
                time.sleep(0.1)  # Небольшая пауза между точками
                
        except Exception as e:
            print(f"Ошибка управления реальным роботом: {e}")
    
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
                    # Управляем симуляционным роботом
                    success = self.control_sim_robot(target_position, rotation)
                    
                    if not success:
                        print(f"Не удалось достичь точки {self.current_point_index}")
                
                i += 1
                
                        
                time.sleep(20.0)
                # Проверка завершения траектории
                if (self.real_robot_connected and 
                    not real_robot_thread.is_alive() and 
                    self.current_point_index >= len(self.trajectory_points) - 1):
                    print("Траектория завершена")
                    self.is_running = False
        
        # Завершение
        self.cleanup()
    
    def cleanup(self):
        """Очистка ресурсов"""
        if self.real_robot_connected:
            # Остановка реального робота
            try:
                self.real_robot.controller_state.set('hold')
            except:
                pass
        
        self.simulation_app.close()


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