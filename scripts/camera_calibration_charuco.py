import cv2
from cv2 import aruco
import pickle
import glob

# Create your own ChArUco board:
# https://docs.opencv.org/4.x/da/d0d/tutorial_camera_calibration_pattern.html
# https://calib.io/pages/camera-calibration-pattern-generator (check if they have the legacy pattern)

# https://docs.opencv.org/4.x/df/d4a/tutorial_charuco_detection.html
# https://github.com/kyle-bersani/opencv-examples/blob/master/CalibrationByCharucoBoard/CalibrateCamera.py
# https://github.com/opencv/opencv/issues/23873
# https://docs.opencv.org/4.7.0/d9/df5/classcv_1_1aruco_1_1CharucoDetector.html#aacbea601612a3a0feaa45ebb7fb255fd
# https://github.com/pettod/charuco-corner-detection
# https://docs.opencv.org/4.x/da/d13/tutorial_aruco_calibration.html
# https://github.com/opencv/opencv_contrib/blob/70f870600addc014a50798d7599950f36e6551c4/modules/aruco/samples/calibrate_camera_charuco.cpp
# https://github.com/opencv/opencv/blob/4.x/doc/pattern_tools/test_charuco_board.py
# https://medium.com/@ed.twomey1/using-charuco-boards-in-opencv-237d8bc9e40d
# https://gist.github.com/naoki-mizuno/c80e909be82434ddae202ff52ea1f80a
# https://www.kaggle.com/code/photunix/camera-calibration-with-charuco-board
# https://medium.com/@nflorent7/a-comprehensive-guide-to-camera-calibration-using-charuco-boards-and-opencv-for-perspective-9a0fa71ada5f

# ChAruco board variables
CHARUCOBOARD_ROWCOUNT = 9
CHARUCOBOARD_COLCOUNT = 12
ARUCO_DICT = aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)

# Create constants to be passed into OpenCV and Aruco methods
charucoBoard = aruco.CharucoBoard(
        (CHARUCOBOARD_COLCOUNT,CHARUCOBOARD_ROWCOUNT),
        squareLength=2, # The measurement is in cm
        markerLength=1.5, # The measurement is in cm
        dictionary=ARUCO_DICT)

# Termination criteria
CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Charuco templates was updated in opencv/opencv_contrib#3174. The behavior has changed only for templates with an even number of rows. This was done to make the ChArUco patterns compatible with the chessboard.
# In this case we don't have any problems because the number of rows is odd

# charucoBoard.setLegacyPattern(True)

# from_cv_img = charucoBoard.generateImage((CHARUCOBOARD_COLCOUNT*50, CHARUCOBOARD_ROWCOUNT*50))
# cv2.imshow("board", from_cv_img)
# cv2.waitKey(0)

# Create the arrays and variables we'll use to store info like corners and IDs from images processed
corners_all = [] # Corners discovered in all images processed
ids_all = [] # Aruco ids corresponding to corners discovered
image_size = None # Determined at runtime

# This requires a set of images or a video taken with the camera you want to calibrate
# All images used should be the same size, which if taken with the same camera shouldn't be a problem
images = glob.glob('scripts/calibration_images/*.png')

# Create charuco detector
charucoParams =  cv2.aruco.CharucoParameters()
detectorParams = cv2.aruco.DetectorParameters()
charucoDetector = cv2.aruco.CharucoDetector(charucoBoard, charucoParams, detectorParams)

# Loop through images glob'ed
for iname in images:
    # Open the image
    img = cv2.imread(iname)
    # Grayscale the image
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # https://stackoverflow.com/questions/76781469/attributeerror-module-cv2-aruco-has-no-attribute-interpolatecornerscharuco

    # Find aruco markers in the query image
    charuco_corners, charuco_ids, marker_corners, marker_ids = charucoDetector.detectBoard(gray)
    print(iname)
    if charuco_ids.shape[0] > 0:
        charuco_corners = cv2.cornerSubPix(gray, charuco_corners, (3,3), (-1,-1), CRITERIA) # The function improves the accuracy of the corner points detected by the cv2.goodFeaturesToTrack or cv2.findChessboardCorners functions

    # Outline the aruco markers found in our query image
    img = aruco.drawDetectedMarkers(image=img, corners=marker_corners)

    # If a Charuco board was found, let's collect image/corner points
    # Requiring at least 20 squares
    if charuco_ids.shape[0] > 30:
        # Add these corners and ids to our calibration arrays
        corners_all.append(charuco_corners)
        ids_all.append(charuco_ids)
        
        # Draw the Charuco board we've detected to show our calibrator the board was properly detected
        img = aruco.drawDetectedCornersCharuco(
                image=img,
                charucoCorners=charuco_corners,
                charucoIds=charuco_ids)
       
        # If our image size is unknown, set it now
        if not image_size:
            image_size = gray.shape[::-1]
    
        # Reproportion the image, maxing width or height at 1000
        # proportion = max(img.shape) / 1000.0
        # img = cv2.resize(img, (int(img.shape[1]/proportion), int(img.shape[0]/proportion)))
        
        # Pause to display each image, waiting for key press
        # cv2.imshow('Charuco board', img)
        # cv2.waitKey(0)
    else:
        print("Not able to detect a charuco board in image: {}".format(iname))

# Destroy any open CV windows
cv2.destroyAllWindows()

# Make sure at least one image was found
if len(images) < 1:
    # Calibration failed because there were no images, warn the user
    print("Calibration was unsuccessful. No images of charucoboards were found. Add images of charucoboards and use or alter the naming conventions used in this file.")
    # Exit for failure
    exit()

# Make sure we were able to calibrate on at least one charucoboard by checking
# if we ever determined the image size
if not image_size:
    # Calibration failed because we didn't see any charucoboards of the PatternSize used
    print("Calibration was unsuccessful. We couldn't detect charucoboards in any of the images supplied. Try changing the patternSize passed into Charucoboard_create(), or try different pictures of charucoboards.")
    # Exit for failure
    exit()

# Now that we've seen all of our images, perform the camera calibration
# based on the set of points we've discovered

# https://docs.opencv.org/4.10.0/d9/d6a/group__aruco.html#gaa7357017aa9da857b487e447c7b13f11
# https://github.com/opencv/opencv/commit/b4b35cff15041c9a2795c749a8dfcffc317acefc
# https://github.com/opencv/opencv/issues/23493

calibration, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
        charucoCorners=corners_all,
        charucoIds=ids_all,
        board=charucoBoard,
        imageSize=image_size,
        cameraMatrix=None,
        distCoeffs=None)
    
# Print matrix and distortion coefficient to the console
print(cameraMatrix)
print(distCoeffs)

# Save values to be used where matrix+dist is required, for instance for posture estimation
# I save files in a pickle file, but you can use yaml or whatever works for you
f = open('scripts\calibration_output/charuco_calibration_640x480.pckl', 'wb')
pickle.dump((cameraMatrix, distCoeffs, rvecs, tvecs), f)
f.close()

# Print to console our success
print('Calibration successful. Calibration file used: {}'.format('charuco_calibration_640x480.pckl'))
print(f'Final re-projection error: {calibration}.')

# Code base (outdated):
# https://github.com/kyle-bersani/opencv-examples/blob/master/CalibrationByCharucoBoard/CalibrateCamera.py


# Relevant links:
# https://docs.opencv.org/4.10.0/d9/d6a/group__aruco.html#gaa7357017aa9da857b487e447c7b13f11
# https://docs.opencv.org/4.10.0/d4/d17/namespacecv_1_1aruco.html
# https://docs.opencv.org/4.x/df/d4a/tutorial_charuco_detection.html
# https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#gab3ab7bb2bdfe7d5d9745bb92d13f9564
# https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
# https://docs.opencv.org/3.4/da/d13/tutorial_aruco_calibration.html