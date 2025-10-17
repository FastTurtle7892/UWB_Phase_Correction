# Screen Settings
WIDTH = 800
HEIGHT = 600

# Anchors Settings
ANCHORS = [(0, 0), (1.45, 0), (0, 0.79), (1.45, 0.79)] 

# Background Image
MENU_IMAGE = "assets/images/001.png"
INPUT_ID = "assets/images/006.png"
CAMERA_COVERED = "assets/images/002.png"
CAMERA_UNCOVERED = "assets/images/003.png"
FINAL_SCORE = "assets/images/004.png"
RANKING = "assets/images/005.png"

# Balloons Setting
RED_BALLOON = "assets/images/red_balloon.png"
BLUE_BALLOON = "assets/images/blue_balloon.png"
GREEN_BALLOON = "assets/images/green_balloon.png"
BLACK_BALLOON = "assets/images/black_balloon.png"
EXPLOSION = "assets/images/explosion.png"

BALLOON_IMAGES = [
    (RED_BALLOON, "red"),
    (BLUE_BALLOON, "blue"),
    (GREEN_BALLOON, "green"),
    (BLACK_BALLOON, "black"),
]

BALLOON_SCORES = {
    "black": 9,
    "red": 7,
    "blue": 5,
    "green": 3,
}

# Game Time Setting
TIME_LIMIT = 50
