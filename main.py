import pygame

from environment import RobotEnvironment
from sensor import LIDAR


# create a world
world = RobotEnvironment(
    dimensions=(800, 900),
    world_path='./maps/map-1.png'
)

# copy the real world/map
original_map = world.map.copy()

# initialize the LIDAR
sensor = LIDAR(
    rotation_speed=100,
    detection_range=90,
    map=original_map,
    error=(0.5, 0.01)
)

# copy the newly created map, the one filled with black, as the information map
world.information_map = world.map.copy()

# setup pygame
running = True

while running:
    # fill the original map with black
    world.map.fill(color=world.BLACK)

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
        sensor_data = sensor.detect_obstacles()

        # draw the data on the world
        if sensor_data:
            # remove points detected earlier
            world.point_cloud = []

            # save the new readings
            world.save_reading(readings=sensor_data)

            # plot the readings
            world.show_reading()
            
            # Draw the information map on top of the original map
            world.map.blit(world.information_map, (0, 0))
        pygame.display.update()
