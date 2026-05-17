import pygame
import numpy as np
from environment import RobotEnvironment
from agent import RobotAgent
from slam import EKFSlAM


# create a world
world = RobotEnvironment(
    dimensions=(800, 900),
    world_path='./maps/map-1.png',
    agent_radius=20
)

# create an agent
world.create_agents(num_agents=1)

# initiale state of the robot (x, y, theta)
initial_state = np.array([0, 0, 0], dtype=float)

# initiale covariance of the state estimation
initial_covariance = np.eye(3)

# create a SLAM instance
slam = EKFSlAM(
    initial_state=initial_state,
    initial_covariance=initial_covariance
)

# run the heavier line extraction every few frames instead of every loop
frame_index = 0
line_detection_interval = 5


# create a variable that indicates which agent to control
agent_index = 0

# copy the newly created map, the one filled with black, as the information map
world.information_map = world.map.copy()

# setup pygame
running = True

while running:
    frame_index += 1

    # fill the original map with black
    image = world.world_map
    world.map.blit(image, (0,0))

    # poll for event
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    # previous pose of the bot
    previous_pose = world.agents[agent_index].agent_position, world.agents[agent_index].theta

    keys = pygame.key.get_pressed()
    # Movement model: UP/DOWN = forward/backward, LEFT/RIGHT = rotate
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

    # --- SLAM steps ---
    # 1. Compute control input (how much the agent moved/rotated)
    current_pose = world.agents[agent_index].agent_position, world.agents[agent_index].theta
    control_input = np.array([
        current_pose[0][0] - previous_pose[0][0],  # delta x
        current_pose[0][1] - previous_pose[0][1],  # delta y
        current_pose[1] - previous_pose[1]          # delta theta
    ])

    if not np.all(control_input == 0):
        print(f"Control input: {control_input}")

    # 2. Call slam.state_predict(control_input, motion_model)
    state_predict = slam.state_predict(control_input=control_input)


    # 3. Get LIDAR data
    lidar_data = world.agents[agent_index].detections
    points = []
    for reading in lidar_data:
        point = RobotAgent.LIDAR_to_points(reading[0], reading[1], reading[2])
        points.append(point)

    # 4. extract lines less frequently to keep the frame loop responsive
    lines = []
    if frame_index % line_detection_interval == 0:
        lines = EKFSlAM.landmark_detection(points=points)
        if len(lines) > 0:
            print(f"Detected {len(lines)} lines")
            print(f"Lines: {lines}")
    
    # 5. associate
    new_landmarks = []
    for line in lines:
        associated_landmark_idx = EKFSlAM.associate_line(detected_line=line, landmarks=slam.landmarks)
        if associated_landmark_idx is not None:
            print(f"Line {line} associated with landmark {associated_landmark_idx}")
            slam.update(associated_landmark_idx, line)
        else:
            print(f"Line {line} is a new landmark")
            new_landmarks.append(line)

    # add the new landmarks to the slam instance
    slam.add_landmarks(new_landmarks)


    world.update()

    # Draw the information map on top of the original map
    world.map.blit(world.information_map, (0, 0))
    pygame.display.update()
