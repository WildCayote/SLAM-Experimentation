from typing import Tuple
import numpy as np
import pygame, math

class LIDAR:
    def __init__(self, rotation_speed:float, detection_range:float, map:pygame.Surface, error: float):
        # create instance variables for the parameters
        self.rotation_speed = rotation_speed
        self.range = detection_range
        self.map = map 
        self.error = error

        # define the position of the sensor in the world
        # this will later be attached to a robot's position using the sensor
        self.position = (0, 0) 

        # obtain the width and height of the world being displayed in pygame
        self.map_width, self.map_height = pygame.display.get_surface().get_size()
        
        # a list for holding sensed obstacles
        self.obstacles = []
    
    @staticmethod
    def distance(point_a:Tuple[float, float], point_b:Tuple[float, float]):
        difference = (point_a[0] - point_b[0], point_a[1] - point_b[1])
        result = np.linalg.norm(difference)
        return result
    
    @staticmethod
    def add_uncertainity(distance:float, angle:float, sigma):
        # find the mean for the distance and angle measurements
        mean = np.array([distance, angle])

        # define the covariance of the measurement
        covaraince = np.diag(sigma ** 2)

        # select new distance and angle measurements from the given mean and covariance
        new_distance, new_angle = np.random.multivariate_normal(mean=mean, cov=covaraince)

        # select only positive values
        new_distance = max(new_distance, 0)
        new_angle = max(new_angle, 0)

        return [new_distance, new_angle]

    def detect_obstacles(self):
        data = []
        x1, y1 = self.position

        # rotate once and obtain 60 samples, from 0Deg (0rad) to 360Deg (2πrad)
        for angle in np.linspace(0, 2 * math.pi, 60, False):
            # define the endpoint of the line along the selected angle, the length of the line is going to be equal to the range of the LIDAR
            x2 = math.cos(angle) * self.range + x1
            y2 = math.sin(angle) * self.range - y1 # the reason it is set to -y1 is because pygame has its origin at the top left corner, hence the positive y axis goes downwards

            # create another loop that will sample points along the line and check for object
            samples = 100
            for i in range(0, 100):
                # determine the step size
                step_size = i / 100
                
                # select the point
                sample_x = int(x1 + step_size * (x2 - x1))
                sample_y = int(y1 + step_size * (y2 - y1))

                # check if the sampled point in with in the screen
                if 0 < sample_x < self.map_width and 0 < sample_y < self.map_height:
                    # obtain the color of that point
                    sample_color = self.map.get_at((sample_x, sample_y))

                    # check if the color is black, black is used to signify a wall in the map
                    if sample_color == (0,0,0):
                        # calculate the distance between that point and the sensor
                        sample_distance = LIDAR.distance(point_a=self.position, point_b=(sample_x, sample_y))

                        # add uncertainity/error to the readings to simulate somekind of real world error of the sensor
                        readings = LIDAR.add_uncertainity(distance=sample_distance, angle=angle, sigma=self.error)

                        # add the position of the LIDAR to the readings
                        readings.append(self.position)

                        # store current angle reading to the data
                        data.append(readings)
                        
                        # break the sampling along the line since LIDAR reads the closest object and nothing further along a given line
                        break
        
        # now return the data read, if no obhect is detected the return False
        if len(data) > 0: return data
        else: return False
                        
