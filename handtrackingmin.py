#write by Abdulrahman mahmoud mohamed saleh
# Import necessary libraries
import pyfirmata  # For Arduino communication
import cv2  # For image processing and computer vision
import mediapipe as mp  # For hand tracking
import time  # For handling time-related tasks

# Initialize connection to Arduino
board = pyfirmata.Arduino('com10')
it = board.util.Iterator(board)
#it.start()  # Uncomment to start the iterator
pin11 = board.get_pin('d:11:p')  # control pin 11 for PWM

# Initialize the webcam for capturing video
cap = cv2.VideoCapture(0)

# Set up MediaPipe for hand tracking
mpHands = mp.solutions.hands
hands = mpHands.Hands()
mpDrow = mp.solutions.drawing_utils

# Variables for calculating FPS
pTime = 0
cTime = 0

# Initialize variables for hand landmark positions
cx1, cy1, cx, cy = 0, 0, 0, 0
distance = 0

# Main loop for processing video frames
while True:
    success, img = cap.read()  # Read a frame from the webcam
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # Convert frame to RGB
    results = hands.process(imgRGB)  # Process the frame for hand landmarks

    # Check if any hand landmarks are detected
    if results.multi_hand_landmarks:
        for handLms in results.multi_hand_landmarks:  # Iterate through each hand
            for id, lm in enumerate(handLms.landmark):  # Iterate through each landmark
                h, w, c = img.shape  # Get image dimensions
                if id == 4 :  # Check for thumb tip
                    cx, cy = int(lm.x * w), int(lm.y * h)  # Calculate coordinates
                    cv2.circle(img, (cx, cy), 10, (255, 0, 0), cv2.FILLED)  # Draw a circle at thumb tip
                    print(id, cx, cy)  # Print coordinates
                if id == 8:  # Check for index finger tip
                    cx1, cy1 = int(lm.x * w), int(lm.y * h)  # Calculate coordinates
                    cv2.circle(img, (cx1, cy1), 10, (255, 0, 0), cv2.FILLED)  # Draw a circle at index finger tip
                    print(id, cx1, cy1)  # Print coordinates

                # Calculate the distance between thumb and index finger
                distance = pow(pow(cx-cx1,2)+pow(cy-cy1,2), .5)
                cv2.line(img,(cx,cy), (cx1,cy1),(255,0,0),2)  # Draw line between thumb and index finger

            # Control the motor speed based on the distance
            if distance >130:
                pin11.write(1)
                cv2.circle(img, (x, y), 10, (0, 255, 0), cv2.FILLED)
            elif distance >25:
                pin11.write(distance / 200)
                cv2.circle(img, (x, y), 10, (0, 255, 0), cv2.FILLED)
            else:
                pin11.write(0)
                cv2.circle(img, (x, y), 10, (0, 0, 255), cv2.FILLED)

            # Draw hand landmarks and connections
            mpDrow.draw_landmarks(img, handLms ,mpHands.HAND_CONNECTIONS)

    # Calculate and display FPS
    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime
    cv2.putText(img, f'FPS:{int(fps)}', (40, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
    cv2.putText(img, f'distance:{int(distance)}', (450, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

    # Show the image
    cv2.imshow("Image", img)
    cv2.waitKey(1)  # Wait for a key press (1 ms)
