import pygame

from environment import RobotEnvironment


# create a world
world = RobotEnvironment(
    dimensions=(800, 900),
    world_path='./maps/map-1.png',
    agent_radius=20
)

# create an agent
world.create_agents(num_agents=1)

# create a variable that indicates which agent to control
agent_index = 0

# copy the newly created map, the one filled with black, as the information map
world.information_map = world.map.copy()

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
        
    keys = pygame.key.get_pressed()
    if keys[pygame.K_UP]:
        world.move_agent(
            agent_idx=agent_index,
            direction='UP'
        )

    if keys[pygame.K_DOWN]:
        world.move_agent(
            agent_idx=agent_index,
            direction='DOWN'
        )
            
    if keys[pygame.K_RIGHT]:
        world.move_agent(
            agent_idx=agent_index,
            direction='RIGHT'
        )

    if keys[pygame.K_LEFT]:
        world.move_agent(
            agent_idx=agent_index,
            direction='LEFT'
        )

    world.update()

    # Draw the information map on top of the original map
    world.map.blit(world.information_map, (0, 0))
    pygame.display.update()
