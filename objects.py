import pygame
import random
from settings import WIDTH, HEIGHT, EXPLOSION

# Balloons
class Balloon:
    def __init__(self, image_path, color, size=(50, 70)):
        self.x = random.randint(50, WIDTH - 50)
        self.y = random.randint(50, HEIGHT - 50)
        self.color = color 
        self.image = pygame.image.load(image_path)
        self.image = pygame.transform.scale(self.image, size) 
        self.width = self.image.get_width()
        self.height = self.image.get_height()
        self.speed = random.uniform(1, 20)
        self.is_exploding = False
        self.explode_start_time = None
        self.explosion_image = pygame.image.load(EXPLOSION)
        self.explosion_image = pygame.transform.scale(self.explosion_image, size)

    def update(self):
        if not self.is_exploding:
            self.y -= self.speed
            if self.y < -self.height:
                self.y = HEIGHT + self.height
                self.x = random.randint(50, WIDTH - 50) 

    def draw(self, screen):
        if self.is_exploding:
            screen.blit(self.explosion_image, (int(self.x), int(self.y)))
        else:
            screen.blit(self.image, (int(self.x), int(self.y)))

    def explode(self):
        self.is_exploding = True
        self.explode_start_time = pygame.time.get_ticks()

    def get_rect(self):
        return pygame.Rect(self.x, self.y, self.width, self.height)

# Camera Pointer
class CameraPointer:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.color = (0, 255, 0)
        self.size = 10

    def update(self, position):
        self.x, self.y = position

    def draw(self, screen):
        pygame.draw.circle(screen, self.color, (int(self.x), int(self.y)), self.size)

# UWB Pointer
class UWBPointer:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.color = 'YELLOW'
        self.size = 10

    def update(self, position):
        self.x, self.y = position

    def draw(self, screen):
        pygame.draw.circle(screen, self.color, (int(self.x), int(self.y)), self.size)

# For Checking Button Area from Background        
# class Button:
#     def __init__(self, x, y, width, height, text, color, text_color, font):
#         self.rect = pygame.Rect(x, y, width, height)
#         self.text = text
#         self.color = color
#         self.text_color = text_color
#         self.font = font
# 
#     def draw(self, surface):
#         pygame.draw.rect(surface, self.color, self.rect, border_radius=10)
#         text_obj = self.font.render(self.text, True, self.text_color)
#         text_rect = text_obj.get_rect(center=self.rect.center)
#         surface.blit(text_obj, text_rect)
# 
#     def is_clicked(self, mouse_pos):
#         return self.rect.collidepoint(mouse_pos)
