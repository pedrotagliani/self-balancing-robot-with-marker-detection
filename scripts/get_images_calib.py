import cv2

# URL for the camera stream
url = 'INSERT URL'

# Create VideoCapture object to connect to the video stream
cap = cv2.VideoCapture(url)

# Check if the connection to the stream was successful
if not cap.isOpened():
    print("Error: Could not open the video stream. Check the URL or connection.")
    exit()

num = 1

# Main loop
while True:
    # Open the url before capturing the frame
    cap.open(url)

    # Read a frame from the video stream
    ret, frame = cap.read()

    frame = cv2.rotate(frame, cv2.ROTATE_180)

    k = cv2.waitKey(1) & 0b11111111

    cv2.imshow('Calibration...', frame)

    if k == ord('q'):
        break
    elif k == ord('s'):
        print('Imagen capturada.')
        cv2.imwrite(f'scripts/calibration_images/charuco_calibration_image{str(num)}.png', frame)
        num += 1

# Release the video capture object and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()