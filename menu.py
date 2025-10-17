import pygame
from settings import MENU_IMAGE, INPUT_ID, FINAL_SCORE, WIDTH, HEIGHT

# Main menu : Splash Screen
def main_menu(screen):
    # Load Background
    background = pygame.image.load(MENU_IMAGE)
    background = pygame.transform.scale(background, (WIDTH, HEIGHT))

    # Start Button
    button_start_rect = pygame.Rect(290, 279, 220, 60)
    # Ranking Button
    button_rank_rect = pygame.Rect(290, 370, 220, 60)

    clock = pygame.time.Clock()

    while True:
        screen.blit(background, (0, 0))

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return "quit"
            if event.type == pygame.MOUSEBUTTONDOWN:
                mouse_pos = event.pos
                if button_start_rect.collidepoint(mouse_pos):
                    return "start"
                elif button_rank_rect.collidepoint(mouse_pos):
                    return "ranking"

        pygame.display.flip()
        clock.tick(30)

# Get User ID
def get_user_id(screen):
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("arial", 85, bold=True)
    input_box = pygame.Rect(WIDTH // 2 - 140, HEIGHT // 2 - 50, 300, 100)  
    color_active = (0, 0, 0)
    color_inactive = (0, 0, 0)
    color = color_inactive
    active = False
    user_id = ""

    background = pygame.image.load(INPUT_ID).convert_alpha()
    background = pygame.transform.scale(background, (WIDTH, HEIGHT))


    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return None
            if event.type == pygame.MOUSEBUTTONDOWN:
                if input_box.collidepoint(event.pos):
                    active = not active
                else:
                    active = False
                color = color_active if active else color_inactive
            if event.type == pygame.KEYDOWN and active:
                if event.key == pygame.K_RETURN:
                    return user_id  
                elif event.key == pygame.K_BACKSPACE:
                    user_id = user_id[:-1]  
                else:
                    user_id += event.unicode  

        screen.fill((0, 0, 0))  
        screen.blit(background, (0, 0))
        pygame.draw.rect(screen, color, input_box, 10)  
        text_surface = font.render(user_id, True, (0, 0, 0))  
        screen.blit(text_surface, (input_box.x + 20, input_box.y + 10))
        pygame.display.flip()

# Final Screen : Show Final Score   
def show_final_screen(screen, score):
    clock = pygame.time.Clock()
    running = True

    background = pygame.image.load(FINAL_SCORE).convert_alpha()
    background = pygame.transform.scale(background, (WIDTH, HEIGHT))

    font_score = pygame.font.SysFont("arial", 70, bold=True)  

    buttons = {
        "home": pygame.Rect(40, 10, 150, 50), 
        "retry": pygame.Rect(120, 430, 200, 50),  
        "ranking": pygame.Rect(485, 430, 200, 50), 
    }

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.MOUSEBUTTONDOWN:
                mouse_pos = pygame.mouse.get_pos()

                if buttons["home"].collidepoint(mouse_pos):
                    return "home"  
                elif buttons["retry"].collidepoint(mouse_pos):
                    return "retry"  
                elif buttons["ranking"].collidepoint(mouse_pos):
                    return "ranking"  

        screen.blit(background, (0, 0))
        score_text = font_score.render(f"{score:03}", True, (255, 255, 255))  

        screen.blit(score_text, (350, 262)) 

        pygame.display.flip()
        clock.tick(30)
        clock.tick(30)
