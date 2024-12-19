import cv2
import pickle
import serial

# https://stackoverflow.com/questions/36503170/python-communication-to-arduino-via-bluetooth
def start_communication_with_bluetooth(port):
    # Create the serial communication object
    ser = serial.Serial(port, 115200)
    # time.sleep(4)  # Wait for the connection to establish

    return ser

# Start serial communication via bluetooth module
bluetoothCommunication = start_communication_with_bluetooth("COM7")

# URL for the camera stream
url = 'INSERT URL'

# Create VideoCapture object to connect to the video stream
cap = cv2.VideoCapture(url)

# ESP32-CAM takes a few seconds to start up
while cap.isOpened() == False:
    cap = cv2.VideoCapture(url)

print("Conexión conexión establecida con la cámara.")

bluetoothCommunication.write("p\n".encode("utf-8"))

while True:

    # Read a line from the serial connection
    if bluetoothCommunication.in_waiting > 0:  # Check if there's incoming data
        # serial_port.in_waiting -----------> Indicate the number of bytes currently available in the input buffer
        response = bluetoothCommunication.readline().decode().strip()  # Read the line, decode it to string, and strip whitespace

        if response == "ok":
            print("Conexión bluetooth establecidaa.")
            break
        else:
            print("No se pudo establecer la conexión bluetooth.")

# Define the ArUco detector
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
arucoParams =  cv2.aruco.DetectorParameters()
arucoDetector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)

# Load the data from the pickle file
with open('config/charuco_calibration_640x480.pckl', 'rb') as file:
    loadedData = pickle.load(file)

# Default values obtained through calibration are specific to the Q-BOX camera
cameraMatrix = loadedData[0]
distCoeffs = loadedData[1]

# Marker size in centimeters:
markerSizeInCM = 12

# Window name for displaying the video
winName = 'CAM'
cv2.namedWindow(winName, cv2.WINDOW_AUTOSIZE)

# Main loop
while True:
    # Open the url before capturing the frame
    cap.open(url)

    # Read a frame from the video stream
    ret, frame = cap.read()

    # Check if the frame was successfully captured
    if ret:
        # Rotate the frame 180 degrees
        frame = cv2.rotate(frame, cv2.ROTATE_180)

        frameCopy = frame.copy()

        # Convert to gray scale
        gray = cv2.cvtColor(frameCopy, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers in the video frame
        corners, ids, rejected = arucoDetector.detectMarkers(gray)

        # Draw ArUco markers detected
        cv2.aruco.drawDetectedMarkers(frameCopy, corners)

        if ids is not None:

            # print(len(corners)) # n
            # print(corners[0].shape) # (n,1,4,2) -> n is in the tuple layer (in this case, n = 0 --> corners[0].shape)
            # print(ids.shape) # (n,1)

            # Flatten the ArUco IDs and make it a list
            ids = ids.flatten().tolist()
            # print(ids.shape) # (n,)

            # Display how many ArUco markers are being detected
            markersFoundString = f'{len(ids)} marcadores encontrados.'
            cv2.putText(frameCopy,markersFoundString,(20,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(0,255,0),2)

            if (5 in ids): # This is done to avoid false detections (for example, [38])
                
                # Subpixel corner detection
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
                markerCornerUnmodified = cv2.cornerSubPix(cv2.cvtColor(frameCopy, cv2.COLOR_BGR2GRAY), corners[0], winSize = (3,3), zeroZone = (-1,-1), criteria = criteria)

                # Reshape the array
                markerCorner2 = markerCornerUnmodified.reshape((4, 2))

                # Classify each corner of the marker
                (topLeft, topRight, bottomRight, bottomLeft) = markerCorner2

                cv2.putText(frameCopy, str(ids[0]), (int(bottomRight[0]+10), int(bottomRight[1]+10)),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                # Find the pose of each marker respect to the camera
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(markerCornerUnmodified, markerSizeInCM, cameraMatrix, distCoeffs)

                # Flatten the position from the camera to the marker
                tvec = tvec.flatten()

                # Draw frame axis:
                cv2.drawFrameAxes(frameCopy, cameraMatrix, distCoeffs, rvec, tvec, length=8, thickness=4)

                distanceToMarker = tvec[2]

                cv2.putText(frameCopy,f"Distancia: {round(distanceToMarker,2)}cm",(640 - 210, 480 - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(0,0,255),2)

                if (distanceToMarker > 10):
                    # Tell the robot to move forward

                    if bluetoothCommunication.is_open:  # Si usas pyserial, por ejemplo
                        bluetoothCommunication.write("f\n".encode("utf-8"))
                        print("Mandando comando de avance...")
                    else:
                        print("Conexión no establecida.")
            
            else:
                bluetoothCommunication.write("s\n".encode("utf-8"))
                print("Mandando comando de parada...")

        else:
            # Display that no ArUco markers were detected
            markersnotFoundString = 'No se han encontrado marcadores'
            cv2.putText(frameCopy,markersnotFoundString,(20,30), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(0,255,0),2)
            bluetoothCommunication.write("s\n".encode("utf-8"))
            print("Mandando comando de parada...")

        # Display the video frame in the named window
        cv2.imshow(winName, frameCopy)
    else:
        print("No se pudo leer un frame.")

    # Break the loop if the 'ESC' key is pressed
    tecla = cv2.waitKey(1) & 0xFF
    if tecla == 27:  # ASCII for 'ESC' key
        break

# Release the video capture object and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()

bluetoothCommunication.close()
