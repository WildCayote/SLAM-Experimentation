from typing import Tuple, List
import pygame, math

from sensor import LIDAR

class RobotAgent:
    def __init__(self, spawn_point: Tuple[float, float], radius:float, color:Tuple[float, float, float], lidar_sensor:LIDAR, movement_speed:float):
        # create instance variables for the parameters
        self.agent_position = spawn_point
        self.radius = radius
        self.color = color
        self.sensor = lidar_sensor
        self.movement_speed = movement_speed
        self.point_cloud = []
        self.ray_cloud = []
        self.ray_color = self.sensor.ray_color

        # define colors to be used
        self.RED = (255, 0, 0)
        self.GREY = (70, 70, 70)
   
    @staticmethod
    def LIDAR_to_points(distance:float, angle:float, LIDAR_pos:Tuple[float, float]):
        x = int(LIDAR_pos[0] + math.cos(angle) * distance)
        y = int(LIDAR_pos[1] + math.sin(angle) * distance)

        return (x, y)
    
    def __save_readings(self, readings:List, wasted_rays:List):
        for reading in readings:
            # convert the LIDAR reading into cartesian point data
            point = RobotAgent.LIDAR_to_points(reading[0], reading[1], reading[2])

            # check if the point is already stored, if not add it to the store
            if point not in self.point_cloud: self.point_cloud.append([point, reading[2]])
        
        for ray in wasted_rays:
            # convert the LIDAR ray into cartesina point data
            point = RobotAgent.LIDAR_to_points(ray[0], ray[1], ray[2])

            # check if the point is already stored, if not add it to the store
            if point not in self.ray_cloud: self.ray_cloud.append([point, ray[2]])

    def detect_obstacle(self, save:bool=True):
        # reset the lists for detection
        self.point_cloud = []
        self.ray_cloud = []

        # perform a detection using the sensor
        data, wasted_rays, _ = self.sensor.detect_obstacles()

        # add them to list containing data
        if save:
            self.__save_readings(
                readings=data,
                wasted_rays=wasted_rays
            )      

        else:
            return data, wasted_rays

    def draw_agent(self, surface: pygame.Surface):
        # loop through the point cloud
        # show the point and also trace the ray
        for line in self.point_cloud:
            end_point = line[0]
            start_point = line[1]
            pygame.draw.line(
                surface=surface,
                color=self.ray_color,
                end_pos=end_point,
                start_pos=start_point
            )
            pygame.draw.circle(
                surface=surface,
                color=self.RED,
                center=(int(end_point[0]), int(end_point[1])),
                radius=1
            )

        # loop through the wasted rays and plot them
        for line in self.ray_cloud:
            end_point = line[0]
            start_point = line[1]
            pygame.draw.line(
                surface=surface,
                color=self.ray_color,
                end_pos=end_point,
                start_pos=start_point
            )
        
        # draw the robot
        pygame.draw.circle(
            surface=surface,
            color=self.GREY,
            center=self.agent_position,
            radius=self.radius
        )
    
    def move(self, direction:str):
        # keep the new position in memory
        if direction == 'UP':
            new_position = (self.agent_position[0], self.agent_position[1] - self.movement_speed)
        
        if direction == 'DOWN':
            new_position = (self.agent_position[0], self.agent_position[1] + self.movement_speed)
        
        if direction == 'LEFT':
            new_position = (self.agent_position[0] - self.movement_speed, self.agent_position[1])

        if direction == 'RIGHT':
            new_position = (self.agent_position[0] + self.movement_speed, self.agent_position[1])

        # update the lidar
        self.sensor.position = new_position
        detections, _ = self.detect_obstacle(save=False) 

        # check for collision using the new lidar position
        collision = False
        for detection in detections:
            distance = detection[0]

            if 10 <= distance <= 23:
                collision = True
                break
        
        if collision:
            # return the detector to its position
            self.sensor.position = self.agent_position
        
        else:
            # move the agent to the new position
            self.agent_position = new_position