from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
from task_rc5 import RC5Task
from isaacsim.core.utils.rotations import euler_angles_to_quat

from API.rc_api import RobotApi

import time
from typing import List, Tuple

from omni.isaac.core.simulation_context import SimulationContext

world = World(stage_units_in_meters=1.0)

sim_context = SimulationContext.instance()

# Отключаем физику
sim_context.set_simulation_dt(0.0)

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
        
        # Инициализация реального RC5
        self.real_robot = RobotApi("10.10.10.10")
        self.real_robot_connected = False
        
        # Общие настройки
        self.trajectory_points = []
        self.return_home_point = None
        self.current_point_index = 0
        self.is_running = True
        self.real_robot_finished = False
        
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
    
    def get_real_robot_joint_positions(self):
        """Получение текущих позиций суставов реального робота"""
        try:
            if not self.real_robot_connected:
                print("Реальный робот не подключен, возвращаем None")
                return None
            
            # Получаем фактические позиции суставов с реального робота
            joint_positions = self.real_robot.motion.joint.get_actual_position()
            
            if joint_positions is not None:
                print(f"Позиции суставов реального робота: {joint_positions}")
                return np.array(joint_positions)
            else:
                print("Не удалось получить позиции суставов реального робота")
                return None
                
        except Exception as e:
            print(f"Ошибка получения позиций суставов реального робота: {e}")
            return None
    
    def set_sim_robot_joint_positions(self, joint_positions):
        """Установка позиций суставов симуляционного робота"""
        try:
            if joint_positions is None:
                print("Переданы пустые позиции суставов")
                return False
            
            # Создаем действие для установки позиций суставов
            action = ArticulationAction()
            action.joint_positions = joint_positions
            action.joint_efforts = None
            action.joint_velocities = None
            
            # Применяем действие
            self.articulation_controller.apply_action(action)
            
            # Делаем несколько шагов симуляции для стабилизации
            for _ in range(50):
                self.world.step(render=True)
                time.sleep(0.01)
            
            print(f"Установлены позиции суставов симуляционного робота: {joint_positions}")
            return True
            
        except Exception as e:
            print(f"Ошибка установки позиций суставов: {e}")
            return False
    
    def sync_sim_robot_with_real(self):
        """Синхронизация симуляционного робота с реальным по позициям суставов"""
        print("Синхронизация симуляционного робота с реальным...")
        
        # Получаем позиции суставов реального робота
        real_joint_positions = self.get_real_robot_joint_positions()
        
        if real_joint_positions is not None:
            # Устанавливаем такие же позиции в симуляции
            success = self.set_sim_robot_joint_positions(real_joint_positions)
            if success:
                print("✓ Синхронизация завершена успешно")
                return True
            else:
                print("✗ Ошибка синхронизации позиций суставов")
                return False
        else:
            print("✗ Не удалось получить позиции суставов для синхронизации")
            return False
    
    def move_real_robot_to_point(self, point_index):
        """Движение реального робота к одной точке с синхронизацией"""
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
            
            # Ждем завершения движения реального робота
            time.sleep(10)
            
            # СИНХРОНИЗАЦИЯ: после достижения точки реальным роботом
            # считываем его позу и устанавливаем в симуляцию
            print("Достигнута точка реальным роботом, начинаем синхронизацию...")
            sync_success = self.sync_sim_robot_with_real()
            
            if sync_success:
                print(f"✓ Реальный робот достиг точки {point_index} и синхронизирован с симуляцией")
            else:
                print(f"✓ Реальный робот достиг точки {point_index}, но синхронизация не удалась")
            
            return True
            
        except Exception as e:
            print(f"Ошибка при движении реального робота: {e}")
            return False
    
    def run_sequential_movement(self):
        """Основной цикл поочередного движения с синхронизацией"""
        while True:
            # Настройка траектории
            self.setup_trajectory()

            # Подключение к реальному роботу
            self.connect_real_robot()

            # Начальная синхронизация
            if self.real_robot_connected:
                print("Начальная синхронизация роботов...")
                self.sync_sim_robot_with_real()

            print("Запуск поочередного управления с синхронизацией...")

            # Последовательное движение по точкам
            for point_index in range(len(self.trajectory_points)):
                if not self.is_running or not self.simulation_app.is_running():
                    break

                print(f"\n" + "="*50)
                print(f"ОБРАБОТКА ТОЧКИ {point_index}")
                print("="*50)

                # ШАГ 1: Реальный робот движется к точке и синхронизируется
                print("ШАГ 1: Движение реального робота с синхронизацией...")
                real_success = self.move_real_robot_to_point(point_index)

                if real_success:
                    print("✓ Реальный робот завершил движение и синхронизирован")
                else:
                    print("✗ Проблемы с реальным роботом")
                    # В этом упрощенном варианте просто пропускаем точку
                    print("Пропускаем точку и переходим к следующей")

                # Пауза между точками
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
                print("✓ Реальный робот вернулся домой")
            except Exception as e:
                print(f"✗ Ошибка возврата реального робота: {e}")
        
        # Синхронизация финальной позиции
        if self.real_robot_connected:
            print("Финальная синхронизация...")
            self.sync_sim_robot_with_real()
    
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
        
        time.sleep(2.0)
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
