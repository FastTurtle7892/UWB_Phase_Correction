import cv2
import mediapipe as mp

class Camera:
# Initialize Camera a& Load Mediapipe
    def __init__(self):
        self.cap = cv2.VideoCapture(2)
        #self.cap = cv2.VideoCapture("./assets/yebin.mp4")
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7)

# Get Camera Image
    def get_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return None, None
        frame = cv2.flip(frame, 1)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.hands.process(frame)
        return frame, result

# Check Camera is covered
    def is_camera_covered(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
        brightness = gray.mean()  
        return brightness < 50  

    def release(self):
        self.cap.release()
