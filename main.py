import pygame
import rclpy
from settings import *
from init import *
from menu import *
from game import *
from ranking import *
from controller import handle_events
from objects import Balloon, CameraPointer, UWBPointer
from camera import Camera
from uwb import Distance2Coord
import random
import time

# Run whole process based on state
# State : menu, play, score, ranking
def main():
    try:
        screen, node, camera, balloons, camera_pointer, uwb_pointer = initialize_game()
        game_state = "menu"  

        while game_state != "quit":
            if game_state == "menu":
                menu_result = main_menu(screen)
                if menu_result == "start":
                    user_id = get_user_id(screen)
                    if not user_id:
                        break 
                    game_state = "play"
                elif menu_result == "ranking":
                    game_state = "ranking"
            elif game_state == "play":
                score = game_loop(screen, node, camera, balloons, camera_pointer, uwb_pointer)
                balloons = initialize_balloons()
                if(score != "menu"):
                    save_to_ranking(user_id, score)  
                    game_state = "score"  
                else:
                    game_state = score
            elif game_state == "score":
                final_result = show_final_screen(screen, score)
                if final_result == "menu":
                    game_state = "menu"
                elif final_result == "retry":
                    balloons = initialize_balloons()
                    game_state = "play"
                elif final_result == "ranking":
                    game_state = "ranking"
            elif game_state == "ranking":
                ranking_result = show_ranking_screen(screen)
                game_state = ranking_result  

    except KeyboardInterrupt:
        print("게임이 종료되었습니다.")
    finally:
        camera.release()
        rclpy.shutdown()
        pygame.quit()


if __name__ == "__main__":
    main()

