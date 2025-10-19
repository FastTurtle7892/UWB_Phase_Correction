import pygame

# For Button Clicked
def handle_events(buttons, actions):
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            return "quit"
        if event.type == pygame.MOUSEBUTTONDOWN:
            mouse_pos = event.pos
            for button, action in zip(buttons, actions):
                if button.is_clicked(mouse_pos):
                    return action() 
    return None

