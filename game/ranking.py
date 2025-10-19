import pygame
from settings import RANKING, WIDTH, HEIGHT

# Save id & score to .txt file
def save_to_ranking(user_id, score):
    try:
        with open("ranking.txt", "a") as file:
            file.write(f"{user_id} {score}\n")  
        print("Saved!!")
    except Exception as e:
        print(f"Err: {e}")

# Ranking Screen
def show_ranking_screen(screen):
    clock = pygame.time.Clock()
    running = True

    ranking_background = pygame.image.load(RANKING).convert_alpha()
    ranking_background = pygame.transform.scale(ranking_background, (WIDTH, HEIGHT))
    button_home = pygame.Rect(40, 10, 150, 50)

    # Show rankings up to 8th 
    try:
        with open("ranking.txt", "r") as file:
            rankings = [line.strip().split(" ") for line in file.readlines()]  
        rankings = sorted(rankings, key=lambda x: int(x[1]), reverse=True)[:8]  
    except FileNotFoundError:
        rankings = []

    font_ranking = pygame.font.SysFont("arial", 36, bold=True)

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.MOUSEBUTTONDOWN:
                mouse_pos = event.pos
                if button_home.collidepoint(mouse_pos):
                    return "menu"

        screen.blit(ranking_background, (0, 0))  

        y_offset = 110
        for idx, (user_id, score) in enumerate(rankings):
            ranking_text = font_ranking.render(f"{idx + 1}. {user_id} - {score}", True, (0, 0, 0))
            screen.blit(ranking_text, (130, y_offset))
            y_offset += 60

        pygame.display.flip()
        clock.tick(30)
