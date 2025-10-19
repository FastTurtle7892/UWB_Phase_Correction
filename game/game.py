import pygame
import random
from settings import * 
import time
import rclpy
from objects import *

# Generate balloons
def generate_random_balloon():
    image_path, color = random.choice(BALLOON_IMAGES)
    return Balloon(image_path, color, size=(50, 70)) 

# Game Loop
def game_loop(screen, node, camera, balloons, camera_pointer, uwb_pointer):
    clock = pygame.time.Clock()
    running = True

    # Background : camera covered
    bg_covered = pygame.image.load(CAMERA_COVERED)
    bg_covered = pygame.transform.scale(bg_covered, (WIDTH, HEIGHT))
    
    # Background : camera covered X
    bg_uncovered = pygame.image.load(CAMERA_UNCOVERED)
    bg_uncovered = pygame.transform.scale(bg_uncovered, (WIDTH, HEIGHT))
  
    score = 0
    s_time = time.time()
    font = pygame.font.SysFont("arial", 30, bold=True)
    button_home = pygame.Rect(40, 10, 150, 50)
    initial_phase = True
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            # 'Home' Button after starting game
            if event.type == pygame.MOUSEBUTTONDOWN:
                mouse_pos = pygame.mouse.get_pos()
                if button_home.collidepoint(mouse_pos):
                    return "menu"  
                    
        # When 4 Big Balloons disappeared
        if initial_phase and not balloons:
            initial_phase = False  
        
        #initial_phase = False
        
        # Start generating random balloons
        if not initial_phase and random.random() < 0.05:  
            balloons.append(generate_random_balloon())

        screen.fill((0, 0, 0))
        
        # When balloon explodes, the popping motion is visible for 1 seconds
        for balloon in balloons[:]:
            if balloon.is_exploding:
                if pygame.time.get_ticks() - balloon.explode_start_time > 10:
                    balloons.remove(balloon)
            else:
                balloon.update() 

	# ROS 2
        rclpy.spin_once(node, timeout_sec=0.01)
        
        # Count Game Limit Time
        elapsed_time = time.time() - s_time
        remaining_time = max(0, TIME_LIMIT - int(elapsed_time))
	
	# After Limit Time -> Game quits
        if remaining_time <= 0:
            running = False
            break	
	
	# Get camera frame as background
        frame, result = camera.get_frame()
        if frame is not None:
            if camera.is_camera_covered(frame):
                screen.blit(bg_covered, (0, 0))  
            else:
                frame_surface = pygame.surfarray.make_surface(frame.swapaxes(0, 1))
                screen.blit(pygame.transform.scale(frame_surface, (WIDTH, HEIGHT)), (0, 0)) 
                screen.blit(bg_uncovered, (0, 0))

	# Get vision coordinates
        camera_x, camera_y = None, None
        if result and result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                camera_x = int(hand_landmarks.landmark[9].x * WIDTH)
                camera_y = int(hand_landmarks.landmark[9].y * HEIGHT)
                camera_pointer.update((camera_x, camera_y))

	# Get uwb coordinates
        uwb_x, uwb_y = None, None  
        uwb_position = node.filtered_position
        if uwb_position is not None:
            uwb_x, uwb_y, uwb_z = uwb_position
            uwb_x = uwb_x * WIDTH - 150
            uwb_y = HEIGHT - uwb_y * HEIGHT + 100
            uwb_pointer.update((uwb_x, uwb_y))

	# Draw balloons and vision/uwb pointers
        for balloon in balloons:
            balloon.update()
            balloon.draw(screen)

        camera_pointer.draw(screen)
        uwb_pointer.draw(screen)

	# If pointer within the balloon's range -> Balloon pops
        for balloon in balloons[:]:
            balloon_rect = balloon.get_rect()

            if camera_x is not None and camera_y is not None:
                if balloon_rect.collidepoint(camera_x, camera_y):
                    score += BALLOON_SCORES.get(balloon.color, 0)
                    balloon.explode()

            if uwb_x is not None and uwb_y is not None:  
                if balloon_rect.collidepoint(uwb_x, uwb_y):
                    score += BALLOON_SCORES.get(balloon.color, 0)
                    balloon.explode()

	# Update Score and Time
        score_text = font.render(f"{score:03}", True, (255, 255, 255))
        time_text = font.render(f"{remaining_time:03}", True, (255, 255, 255))

        screen.blit(score_text, (710, 85)) 
        screen.blit(time_text, (710, 25)) 

        pygame.display.flip()
        clock.tick(30)
    
    return score
