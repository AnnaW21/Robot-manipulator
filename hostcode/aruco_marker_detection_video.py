import cv2

# Define camera
camera = cv2.VideoCapture(0)
while True:
    # Read image
    good, img = camera.read()

    # Define ArUco detection parameters
    arucoParams = cv2.aruco.DetectorParameters()
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)

    # Perform ArUco marker detection
    detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)
    (corners, ids, rejected) = detector.detectMarkers(img)
    # print(corners)
    if len(corners) != 0:
        # Display corners of marker
        # cv2.aruco.drawDetectedMarkers(img, corners, ids)
        cv2.aruco.drawDetectedMarkers(img, corners)

        # Display center of marker
        x_sum = corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0]
        y_sum = corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1]
        x_centerPixel = x_sum * .25
        y_centerPixel = y_sum * .25
        print("x: ", x_centerPixel, "y: ", y_centerPixel)
        cv2.circle(img, (int(x_centerPixel), int(y_centerPixel)),
                   radius=int((corners[0][0][1][0] - corners[0][0][0][0]) * 0.05), color=(0, 0, 255), thickness=1)

        # Display size of the image
        # print("width: ", img.shape[1], "height: ", img.shape[0])
        # Display size of the marker
        # print(corners[0][0][1][0] - corners[0][0][0][0])
        # Display angles of joints
        font = cv2.FONT_HERSHEY_COMPLEX
        bottomLeftCornerOfText = (img.shape[1] // 8, img.shape[0] // 10)
        fontScale = 1
        fontColor = (147, 20, 255)
        thickness = 2
        lineType = 1

        angle_joint_1 = 0
        angle_joint_2 = 0
        angle_joint_3 = 0
        angle_joint_4 = 0
        angle_joint_5 = 0
        angle_joint_6 = 0

        text = (f"angle joint_1: {angle_joint_1} \n"
                f"angle joint_2: {angle_joint_2}\n"
                f"angle joint_3: {angle_joint_3}\n"
                f"angle joint_4: {angle_joint_4}\n"
                f"angle joint_5: {angle_joint_5}\n"
                f"angle joint_6: {angle_joint_6}\n"
                )
        y0, dy = 50, 50
        for i, line in enumerate(text.split('\n')):
            y = y0 + i * dy
            cv2.putText(img, line, (50, y),
                        font,
                        fontScale,
                        fontColor,
                        thickness,
                        lineType)
    # Display image
    cv2.namedWindow("Object_detection", cv2.WINDOW_NORMAL)
    # cv2.resizeWindow("Object_detection", width=1248, height=702)
    cv2.imshow("Object_detection", img)
    # Wait for the key press
    if cv2.waitKey(1) == ord('q'):
        break
