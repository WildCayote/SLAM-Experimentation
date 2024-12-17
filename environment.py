from typing import Tuple, List
import pygame, math

class RobotEnvironment:
    def __init__(self, dimensions:Tuple[int, int], world_path:str, world_name:str = 'SLAM Simulation'):
        # create instance variables for the parameters
        self.world_name = world_name
        self.map_height, self.map_width = dimensions

        # declare colors
        self.WHITE = (255, 255, 255)
        self.BLACK = (0, 0, 0)
        self.GREY = (70, 70, 70)
        self.RED = (255, 0, 0)
        self.GREEN = (0, 255, 0)
        self.BLUE = (0, 0, 255)

        # declare a list for hodling the point cloud detected by the LIDAR
        self.point_cloud = []
        self.ray_cloud = []

        pygame.init()
        
        # load the provided map
        self.world_map = pygame.image.load(world_path)
        self.world_map = pygame.transform.scale(surface=self.world_map, size=(self.map_width, self.map_height))
        
        # display the window name
        pygame.display.set_caption(self.world_name)

        # now obtain the window/world created by pygame
        self.map = pygame.display.set_mode((self.map_width, self.map_height))

        # overlay the map of the world on to the window
        self.map.blit(self.world_map, (0,0))
        
    @staticmethod
    def LIDAR_to_points(distance:float, angle:float, LIDAR_pos:Tuple[float, float]):
        x = int(LIDAR_pos[0] + math.cos(angle) * distance)
        y = int(LIDAR_pos[1] + math.sin(angle) * distance)

        return (x, y)

    def save_reading(self, readings:List, wasted_rays:List):
        for reading in readings:
            # convert the LIDAR reading into cartesian point data
            point = RobotEnvironment.LIDAR_to_points(reading[0], reading[1], reading[2])

            # check if the point is already stored, if not add it to the store
            if point not in self.point_cloud: self.point_cloud.append([point, reading[2]])
        
        for ray in wasted_rays:
            # convert the LIDAR ray into cartesina point data
            point = RobotEnvironment.LIDAR_to_points(ray[0], ray[1], ray[2])

            # check if the point is already stored, if not add it to the store
            if point not in self.ray_cloud: self.ray_cloud.append([point, ray[2]])

    def show_reading(self, ray_color:Tuple[float, float, float]):
        # copy the real map
        self.information_map = self.map.copy()

        # loop through the point cloud
        # show the point and also trace the ray
        for line in self.point_cloud:
            end_point = line[0]
            start_point = line[1]
            pygame.draw.line(
                surface=self.information_map,
                color=ray_color,
                end_pos=end_point,
                start_pos=start_point
            )
            pygame.draw.circle(
                surface=self.information_map,
                color=self.RED,
                center=(int(end_point[0]), int(end_point[1])),
                radius=1
            )
        
        # loop through the wasted rays and plot them
        for line in self.ray_cloud:
            end_point = line[0]
            start_point = line[1]
            pygame.draw.line(
                surface=self.information_map,
                color=ray_color,
                end_pos=end_point,
                start_pos=start_point
            )

if __name__ == '__main__':
    # create a world
    world = RobotEnvironment(
        dimensions=(500, 500),
        world_path='./maps/map-1.png'
    )

