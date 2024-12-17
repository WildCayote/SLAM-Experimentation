from typing import Tuple
import pygame

from sensor import LIDAR

class RobotAgent:
    def __init__(self, spawn_point: Tuple[float, float], radius:float, color:Tuple[float, float, float], lidar_sensor:LIDAR, movement_speed:float):
        # create instance variables for the parameters
        self.spawn_point = spawn_point
        self.radius = radius
        self.color = color
        self.sensor = lidar_sensor
        self.movement_speed = movement_speed