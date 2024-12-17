from typing import Tuple, List
import pygame, math, random, copy

from agent import RobotAgent
from sensor import LIDAR

class RobotEnvironment:
    def __init__(self, dimensions:Tuple[int, int], world_path:str, world_name:str = 'SLAM Simulation', agent_radius:float = 30):
        # create instance variables for the parameters
        self.world_name = world_name
        self.map_height, self.map_width = dimensions
        self.world_path = world_path
        self.agent_radius = agent_radius

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

        # declare a list for holding the current robot positions
        # declare a list for holding agents
        self.robot_pos = []
        self.agents = []

        pygame.init()
        
        # load the provided map
        self.world_map = self.load_map()
        
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

    def load_map(self):
        result = pygame.image.load(self.world_path)
        result = pygame.transform.scale(surface=result, size=(self.map_width, self.map_height))
        return result
    
    def select_random_point(self):
        rand_x = random.randint(a=0, b=self.map_width)
        rand_y = random.randint(a=0, b=self.map_height)
        return rand_x, rand_y
    
    def validate_spawn_point(self, x:float, y:float, agent_size:float):
        # initialize a lidar detector on the provided point with a good enough space
        # the error of the lidar is going to be 0
        clearance = 10
        safe_zone = agent_size + clearance
        detector = LIDAR(
            rotation_speed=100,
            detection_range=safe_zone,
            map=self.map.copy(),
            error=(0.0,0.0),
            ray_color=self.WHITE
        )

        # now detect the points
        detector.position = (x, y)
        data, wasted_rays, _ = detector.detect_obstacles()

        # check if there are any detections of obstacles in the safe_zone
        if len(data) == 0: return True
        else: return False

    def select_spawn_point(self, agent_radius:float, max_attempts:int = 100):
        explored = []
        for _ in range(max_attempts):
            # select an initial random point
            candidate_x, candidate_y = self.select_random_point()

            # check if the point is valid
            valid = self.validate_spawn_point(x=candidate_x, y=candidate_y, agent_size=agent_radius)

            if valid and (candidate_x, candidate_y) not in self.robot_pos and (candidate_x, candidate_y) not in explored: return candidate_x, candidate_y

            # save the points that are explored
            explored.append((candidate_x, candidate_y))

    def create_agents(self, num_agents:int = 1):
        for _ in range(num_agents):
            # select a spawn point
            x, y = self.select_spawn_point(agent_radius=self.agent_radius)
            self.robot_pos.append((x, y))

            # instantiate the agent and its respective lidar
            agent_sensor = LIDAR(
                rotation_speed=300,
                detection_range=90,
                map=self.map,
                error=(0.01, 0.01),
                ray_color=self.GREEN
            )
            agent_sensor.position = (x, y)
            agent = RobotAgent(
                spawn_point=(x, y),
                radius=self.agent_radius,
                color=self.GREY,
                lidar_sensor=agent_sensor,
                movement_speed=20
            )

            # add the agent to the agent tracking list
            self.agents.append(
                agent
            )
    
    def show_agents(self):
        for agent_pos in self.robot_pos:
            x, y = agent_pos
            # draw the robot
            pygame.draw.circle(
                surface=self.information_map,
                color=self.GREY,
                center=(x, y),
                radius=self.agent_radius
            )

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

    def show_world(self, ray_color: Tuple[float, float, float]):
        self.show_reading(ray_color)
        self.show_agents()

    def update(self):
        # copy the real map
        self.information_map = self.map.copy()
        for agent in self.agents:
            agent.detect_obstacle()
            agent.draw_agent(surface=self.information_map)

if __name__ == '__main__':
    # create a world
    world = RobotEnvironment(
        dimensions=(500, 500),
        world_path='./maps/map-1.png'
    )
