import pygame
import rclpy
from objects import Balloon, CameraPointer, UWBPointer
from settings import BALLOON_IMAGES, WIDTH, HEIGHT
from uwb import Distance2Coord
from camera import Camera

# Initialize Game : Init ROS, pygame, camera, balloons, uwb ......
def initialize_game():
    pygame.init()
    rclpy.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.FULLSCREEN)
    #screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("풍선 터뜨리기")

    node = Distance2Coord()
    camera = Camera()
    balloons = initialize_balloons() 
    camera_pointer = CameraPointer()
    uwb_pointer = UWBPointer()
    
    return screen, node, camera, balloons, camera_pointer, uwb_pointer

# Initialize Balloons : Generate 4 big balloons
def initialize_balloons():
    initial_balloons = [
        Balloon(image_path, color, size=(150, 210))  
        for image_path, color in BALLOON_IMAGES
    ]
    return initial_balloons
