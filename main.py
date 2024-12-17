import pygame

from environment import RobotEnvironment
from sensor import LIDAR


# create a world
world = RobotEnvironment(
    dimensions=(800, 900),
    world_path='./maps/map-1.png'
)

# create an agent
world.create_agents(num_agents=1)

# copy the real world/map
original_map = world.map.copy()

# copy the newly created map, the one filled with black, as the information map
world.information_map = world.map.copy()

# initialize the LIDAR
sensor = LIDAR(
    rotation_speed=300,
    detection_range=90,
    map=original_map,
    error=(0.01, 0.01),
    ray_color=world.GREEN
)

# setup pygame
running = True

while running:
    # fill the original map with black
    image = world.load_map()
    world.map.blit(image, (0,0))

    # poll for event
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        # check if the cursor is on the map
        if pygame.mouse.get_focused():
            sensor_on = True
        elif not pygame.mouse.get_focused():
            sensor_on = False
    
    # if the cursor is on focus, on the map
    if sensor_on:
        # get the position of the cursor and set it as the position for the LIDAR
        cursor_position = pygame.mouse.get_pos()
        sensor.position = cursor_position

        # sense the obstacles
        # data, wasted_rays, sensor_color = sensor.detect_obstacles()

        # # remove points detected earlier
        # world.point_cloud = []
        # world.ray_cloud = []

        # # save the new readings
        # world.save_reading(readings=data, wasted_rays=wasted_rays)

        # # plot the readings
        # world.show_world(ray_color=sensor_color)

        world.update()

        # Draw the information map on top of the original map
        world.map.blit(world.information_map, (0, 0))
        pygame.display.update()
