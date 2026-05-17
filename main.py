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


def normalize_segment(segment):
    if segment is None:
        return None
    start_point, end_point = segment
    return (
        np.array(start_point, dtype=float),
        np.array(end_point, dtype=float),
    )


def append_segment(existing_segment, new_segment):
    if new_segment is None:
        return existing_segment
    if existing_segment is None:
        return normalize_segment(new_segment)

    existing_start, existing_end = existing_segment
    new_start, new_end = normalize_segment(new_segment)

    direction = existing_end - existing_start
    if np.linalg.norm(direction) < 1e-8:
        return normalize_segment(new_segment)

    direction = direction / np.linalg.norm(direction)
    origin = existing_start

    points = np.vstack([existing_start, existing_end, new_start, new_end])
    projections = (points - origin) @ direction
    min_index = np.argmin(projections)
    max_index = np.argmax(projections)

    merged_start = points[min_index]
    merged_end = points[max_index]
    return merged_start, merged_end


landmark_segments = []


# create a variable that indicates which agent to control
agent_index = 0

# copy the newly created map, the one filled with black, as the information map
world.information_map = world.map.copy()

# helper surface for the side visualization pane
pose_view = pygame.Surface((world.map_width, world.map_height))
pose_view.fill((245, 245, 245))

# simple font for state text
font = pygame.font.SysFont("consolas", 18)

# setup pygame
running = True

while running:
    frame_index += 1

    # reset the world canvas and the side pane background
    world.map.blit(world.world_map, (0, 0))
    pose_view.fill((245, 245, 245))

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
        lines, line_segments = EKFSlAM.landmark_detection(points=points)
        if len(lines) > 0:
            print(f"Detected {len(lines)} lines")
            print(f"Lines: {lines}")
    
    # 5. associate
    new_landmarks = []
    new_landmark_segments = []
    for line in lines:
        associated_landmark_idx = EKFSlAM.associate_line(detected_line=line, landmarks=slam.landmarks)
        segment = line_segments.pop(0) if len(line_segments) > 0 else None
        if associated_landmark_idx is not None:
            print(f"Line {line} associated with landmark {associated_landmark_idx}")
            slam.update(associated_landmark_idx, line)

            if segment is not None and associated_landmark_idx < len(landmark_segments):
                landmark_segments[associated_landmark_idx] = append_segment(
                    landmark_segments[associated_landmark_idx],
                    segment,
                )
        else:
            print(f"Line {line} is a new landmark")
            new_landmarks.append(line)
            new_landmark_segments.append(segment)

    # add the new landmarks to the slam instance
    if len(new_landmarks) > 0:
        slam.add_landmarks(new_landmarks)
        for segment in new_landmark_segments:
            landmark_segments.append(normalize_segment(segment))


    world.update()

    # build the side visualization pane without touching SLAM logic
    if len(landmark_segments) > 0:
        for segment in landmark_segments:
            if segment is None:
                continue
            start_point, end_point = segment
            pygame.draw.line(
                surface=pose_view,
                color=(220, 50, 50),
                start_pos=(int(start_point[0]), int(start_point[1])),
                end_pos=(int(end_point[0]), int(end_point[1])),
                width=3,
            )

    if world.agents:
        world.agents[agent_index].draw_pose(surface=pose_view)

        robot = world.agents[agent_index]
        state_text = [
            f"x: {robot.agent_position[0]:.1f}",
            f"y: {robot.agent_position[1]:.1f}",
            f"theta: {np.degrees(robot.theta):.1f} deg",
            f"landmarks: {len(slam.landmarks)}",
        ]
        for i, text in enumerate(state_text):
            label = font.render(text, True, (20, 20, 20))
            pose_view.blit(label, (15, 15 + i * 24))

    # draw the main world pane and the side pose pane
    world.display.blit(world.information_map, world.world_view_rect)
    world.display.blit(pose_view, world.viz_view_rect)

    pygame.display.update()
